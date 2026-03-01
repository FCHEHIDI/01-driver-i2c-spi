//! # MPU6050 – Driver accéléromètre / gyroscope (I2C)
//!
//! Implémente la communication avec l'IMU InvenSense MPU6050 via I2C,
//! en utilisant le driver bas niveau [`crate::i2c::I2c1`].
//!
//! ## Adresse I2C
//! | Broche AD0 | Adresse 7 bits |
//! |-----------|----------------|
//! | GND       | `0x68`         |
//! | VDD       | `0x69`         |
//!
//! ## Registres clés (MPU6050 Register Map §3)
//! | Adresse | Nom           | Usage                         |
//! |---------|---------------|-------------------------------|
//! | `0x75`  | WHO_AM_I      | Identifiant (doit être `0x68`)|
//! | `0x6B`  | PWR_MGMT_1    | Réveil du composant           |
//! | `0x1A`  | CONFIG        | DLPF (Low-Pass Filter)        |
//! | `0x1B`  | GYRO_CONFIG   | Plage gyro (FS_SEL)           |
//! | `0x1C`  | ACCEL_CONFIG  | Plage accél (AFS_SEL)         |
//! | `0x3B`  | ACCEL_XOUT_H  | Début bloc données (14 octets)|
//!
//! ## Sensibilités par défaut
//! - Accéléromètre ±2 g    → 1 LSB = 1/16384 g    → diviser par 16384
//! - Gyroscope     ±250°/s → 1 LSB = 1/131 °/s   → diviser par 131
//! - Température   °C      → `(raw / 340) + 36.53`
//!
//! ## Représentation des résultats
//! - Accélération : **millièmes de g** (ex. 1000 →  1.000 g)
//! - Rotation     : **centièmes de °/s** (ex. 12345 → 123.45 °/s)
//! - Température  : **centièmes de °C** (ex. 2153 → 21.53 °C)

use crate::bus::{I2cAddr, I2cBus, I2cError, RegAddr};

// --------------------------------------------------------------------------- //
// Registres (MPU6050 Register Map rev 4.2)
// --------------------------------------------------------------------------- //

const REG_WHO_AM_I:    u8 = 0x75;
const REG_PWR_MGMT_1:  u8 = 0x6B;
const REG_CONFIG:      u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
/// Premier registre du bloc de mesures (ACCEL_XOUT_H).
/// Le bloc contient 14 octets : 6 accél + 2 temp + 6 gyro.
const REG_ACCEL_XOUT_H: u8 = 0x3B;

const WHO_AM_I_VAL: u8 = 0x68;

// --------------------------------------------------------------------------- //
// Erreurs
// --------------------------------------------------------------------------- //

/// Erreurs propres au driver MPU6050.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mpu6050Error {
    /// Erreur de bus I2C sous-jacente.
    Bus(I2cError),
    /// WHO_AM_I inattendu : mauvais composant ou câblage.
    InvalidWhoAmI(u8),
}

impl From<I2cError> for Mpu6050Error {
    fn from(e: I2cError) -> Self {
        Mpu6050Error::Bus(e)
    }
}

// --------------------------------------------------------------------------- //
// Plages de mesure
// --------------------------------------------------------------------------- //

/// Plage de l'accéléromètre (AFS_SEL dans ACCEL_CONFIG).
#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    G2  = 0b00, // ±2 g,   LSB = 16384
    G4  = 0b01, // ±4 g,   LSB = 8192
    G8  = 0b10, // ±8 g,   LSB = 4096
    G16 = 0b11, // ±16 g,  LSB = 2048
}

impl AccelRange {
    /// Facteur de conversion : diviseur pour obtenir des millièmes de g.
    ///
    /// raw / lsb_per_g * 1000 → on retourne lsb_per_g pour calculer :
    /// `mg = (raw * 1000) / lsb_per_g`
    fn lsb_per_g(self) -> i32 {
        match self {
            AccelRange::G2  => 16_384,
            AccelRange::G4  => 8_192,
            AccelRange::G8  => 4_096,
            AccelRange::G16 => 2_048,
        }
    }
    fn config_bits(self) -> u8 {
        (self as u8) << 3 // AFS_SEL[4:3]
    }
}

/// Plage du gyroscope (FS_SEL dans GYRO_CONFIG).
#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    Dps250  = 0b00, // ±250 °/s,  LSB = 131.0
    Dps500  = 0b01, // ±500 °/s,  LSB = 65.5
    Dps1000 = 0b10, // ±1000 °/s, LSB = 32.8
    Dps2000 = 0b11, // ±2000 °/s, LSB = 16.4
}

impl GyroRange {
    /// Facteur × 100 pour garder 2 décimales : LSB * 100 / (raw * 100) = raw / lsb
    /// On retourne lsb × 10 pour éviter les flottants :
    /// `centidps = (raw * 1000) / lsb_x10`
    fn lsb_x10(self) -> i32 {
        match self {
            GyroRange::Dps250  => 1_310, // 131.0 × 10
            GyroRange::Dps500  =>   655, //  65.5 × 10
            GyroRange::Dps1000 =>   328, //  32.8 × 10
            GyroRange::Dps2000 =>   164, //  16.4 × 10
        }
    }
    fn config_bits(self) -> u8 {
        (self as u8) << 3 // FS_SEL[4:3]
    }
}

// --------------------------------------------------------------------------- //
// Résultats
// --------------------------------------------------------------------------- //

/// Mesures brutes + converties du MPU6050.
#[derive(Debug, Clone, Copy)]
pub struct Mpu6050Measurement {
    /// Accélération X en **millièmes de g** (1000 = 1 g)
    pub accel_x_mg: i32,
    /// Accélération Y en **millièmes de g**
    pub accel_y_mg: i32,
    /// Accélération Z en **millièmes de g**
    pub accel_z_mg: i32,
    /// Rotation X en **centièmes de °/s** (12345 = 123.45 °/s)
    pub gyro_x_cdps: i32,
    /// Rotation Y en **centièmes de °/s**
    pub gyro_y_cdps: i32,
    /// Rotation Z en **centièmes de °/s**
    pub gyro_z_cdps: i32,
    /// Température interne en **centièmes de °C** (2153 = 21.53 °C)
    pub temp_cdeg: i32,
}

// --------------------------------------------------------------------------- //
// Driver
// --------------------------------------------------------------------------- //

/// Driver niveau applicatif pour le MPU6050.
#[derive(Debug)]
pub struct Mpu6050 {
    addr: I2cAddr,
    accel_range: AccelRange,
    gyro_range:  GyroRange,
}

impl Mpu6050 {
    /// Initialise le MPU6050 sur le bus I2C fourni.
    ///
    /// - `addr`        : adresse 7 bits (`0x68` ou `0x69`)
    /// - `i2c`         : driver I2C1 déjà initialisé
    /// - `accel_range` : plage de l'accéléromètre
    /// - `gyro_range`  : plage du gyroscope
    ///
    /// # Errors
    /// [`Mpu6050Error::InvalidWhoAmI`] si le registre WHO_AM_I ≠ `0x68`.
    pub fn init(
        i2c: &mut impl I2cBus,
        addr: I2cAddr,
        accel_range: AccelRange,
        gyro_range: GyroRange,
    ) -> Result<Self, Mpu6050Error> {
        // --- 1. Vérifier WHO_AM_I ---
        let mut who = [0u8; 1];
        i2c.write_read(addr, RegAddr(REG_WHO_AM_I), &mut who)?;
        if who[0] != WHO_AM_I_VAL {
            return Err(Mpu6050Error::InvalidWhoAmI(who[0]));
        }

        // --- 2. Réveiller le MPU6050 (SLEEP=0 dans PWR_MGMT_1) ---
        // Par défaut après la mise sous tension, SLEEP=1
        // On sélectionne aussi CLKSEL=1 (PLL sur gyro X) pour stabilité
        i2c.write(addr, &[REG_PWR_MGMT_1, 0x01])?;

        // --- 3. DLPF : ~44 Hz bande passante (CONFIG[2:0] = 3) ---
        i2c.write(addr, &[REG_CONFIG, 0x03])?;

        // --- 4. Configurer la plage de l'accéléromètre ---
        i2c.write(addr, &[REG_ACCEL_CONFIG, accel_range.config_bits()])?;

        // --- 5. Configurer la plage du gyroscope ---
        i2c.write(addr, &[REG_GYRO_CONFIG, gyro_range.config_bits()])?;

        #[cfg(target_arch = "arm")]
        defmt::info!(
            "MPU6050 @ 0x{:02x} : accel=±{}g gyro=±{}°/s",
            addr.0,
            match accel_range { AccelRange::G2 => 2, AccelRange::G4 => 4,
                                 AccelRange::G8 => 8, AccelRange::G16 => 16 },
            match gyro_range {  GyroRange::Dps250 => 250, GyroRange::Dps500 => 500,
                                GyroRange::Dps1000 => 1000, GyroRange::Dps2000 => 2000 },
        );

        Ok(Mpu6050 { addr, accel_range, gyro_range })
    }

    /// Lit les 14 octets de mesure et retourne les valeurs converties.
    ///
    /// L'ordre dans le bloc est (RM §3.1) :
    /// `ACCEL_X H/L · ACCEL_Y H/L · ACCEL_Z H/L · TEMP H/L · GYRO_X H/L · GYRO_Y H/L · GYRO_Z H/L`
    ///
    /// # Errors
    /// [`Mpu6050Error::Bus`] si une erreur I2C survient.
    pub fn read(&mut self, i2c: &mut impl I2cBus) -> Result<Mpu6050Measurement, Mpu6050Error> {
        let mut raw = [0u8; 14];
        i2c.write_read(self.addr, RegAddr(REG_ACCEL_XOUT_H), &mut raw)?;

        // Reconstruction des valeurs i16 (big-endian)
        let ax = i16::from_be_bytes([raw[0],  raw[1]]);
        let ay = i16::from_be_bytes([raw[2],  raw[3]]);
        let az = i16::from_be_bytes([raw[4],  raw[5]]);
        let t  = i16::from_be_bytes([raw[6],  raw[7]]);
        let gx = i16::from_be_bytes([raw[8],  raw[9]]);
        let gy = i16::from_be_bytes([raw[10], raw[11]]);
        let gz = i16::from_be_bytes([raw[12], raw[13]]);

        // --- Conversion accéléromètre : mg = raw * 1000 / LSB ---
        let lsb = self.accel_range.lsb_per_g();
        let accel_x_mg = (ax as i32 * 1_000) / lsb;
        let accel_y_mg = (ay as i32 * 1_000) / lsb;
        let accel_z_mg = (az as i32 * 1_000) / lsb;

        // --- Conversion gyroscope : centidps = raw * 1000 / lsb_x10 ---
        let lsb_g = self.gyro_range.lsb_x10();
        let gyro_x_cdps = (gx as i32 * 1_000) / lsb_g;
        let gyro_y_cdps = (gy as i32 * 1_000) / lsb_g;
        let gyro_z_cdps = (gz as i32 * 1_000) / lsb_g;

        // --- Conversion température : centideg = (raw * 100) / 340 + 3653 ---
        // Formule datasheet : T = raw/340 + 36.53 °C
        let temp_cdeg = (t as i32 * 100) / 340 + 3_653;

        Ok(Mpu6050Measurement {
            accel_x_mg, accel_y_mg, accel_z_mg,
            gyro_x_cdps, gyro_y_cdps, gyro_z_cdps,
            temp_cdeg,
        })
    }
}
