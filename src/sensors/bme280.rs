//! # BME280 – Driver capteur Température / Pression / Humidité (I2C)
//!
//! Implémente la communication avec le capteur Bosch BME280 via I2C,
//! en utilisant le driver bas niveau [`crate::i2c::I2c1`].
//!
//! ## Adresse I2C
//! | Broche SDO | Adresse 7 bits |
//! |-----------|----------------|
//! | GND       | `0x76`         |
//! | VDD       | `0x77`         |
//!
//! ## Séquence d'initialisation (BME280 datasheet §4.2)
//! 1. Vérifier `chip_id` (registre `0xD0`) == `0x60`
//! 2. Lancer un soft-reset (`0xE0` ← `0xB6`)
//! 3. Attendre la fin du chargement NVM (`status.im_update == 0`)
//! 4. Lire les 26 coefficients de calibration (registres `0x88` et `0xE1`)
//! 5. Configurer oversampling T × 2, P × 16, H × 1 ; mode Normal
//!
//! ## Représentation des résultats
//! - Température : **centièmes de °C**  (ex. 2345 → 23.45 °C)
//! - Pression    : **centièmes de hPa** (ex. 101325 → 1013.25 hPa)
//! - Humidité    : **centièmes de %RH** (ex. 4863 → 48.63 %)

use crate::i2c::{I2cAddr, I2cError, I2cFreq, RegAddr, I2c1, Ready};

// --------------------------------------------------------------------------- //
// Constantes de registres (BME280 datasheet §4.2.2)
// --------------------------------------------------------------------------- //

const REG_CHIP_ID:    u8 = 0xD0;
const REG_RESET:      u8 = 0xE0;
const SOFT_RESET_CMD: u8 = 0xB6;
const REG_STATUS:     u8 = 0xF3;
const REG_CTRL_HUM:   u8 = 0xF2;
const REG_CTRL_MEAS:  u8 = 0xF4;
const REG_CONFIG:     u8 = 0xF5;
const REG_PRESS_MSB:  u8 = 0xF7; // début du bloc de données brutes (6 + 2 octets)
const REG_CALIB_00:   u8 = 0x88; // début des calibrations T + P (26 octets)
const REG_CALIB_H1:   u8 = 0xA1; // dig_H1 (1 octet)
const REG_CALIB_H2:   u8 = 0xE1; // début des calibrations H (7 octets)

const CHIP_ID_BME280: u8 = 0x60;

// Nombre de tentatives max pour attendre la fin du NVM load
const NVM_WAIT_RETRIES: u32 = 100;

// --------------------------------------------------------------------------- //
// Erreurs
// --------------------------------------------------------------------------- //

/// Erreurs propres au driver BME280.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bme280Error {
    /// Erreur de bus I2C sous-jacente.
    Bus(I2cError),
    /// Le chip_id lu n'est pas `0x60` : mauvais composant ou câblage.
    InvalidChipId(u8),
    /// Timeout lors du chargement NVM après reset.
    NvmTimeout,
}

impl From<I2cError> for Bme280Error {
    fn from(e: I2cError) -> Self {
        Bme280Error::Bus(e)
    }
}

// --------------------------------------------------------------------------- //
// Données de calibration (noms selon datasheet Bosch)
// --------------------------------------------------------------------------- //

/// Coefficients de calibration lus dans la Flash du BME280.
///
/// Tous sont signés/non-signés selon la datasheet §4.2.2 (trim parameters).
#[allow(non_snake_case)]
struct Calib {
    // Temperature
    dig_T1: u16,
    dig_T2: i16,
    dig_T3: i16,
    // Pressure
    dig_P1: u16,
    dig_P2: i16,
    dig_P3: i16,
    dig_P4: i16,
    dig_P5: i16,
    dig_P6: i16,
    dig_P7: i16,
    dig_P8: i16,
    dig_P9: i16,
    // Humidity
    dig_H1: u8,
    dig_H2: i16,
    dig_H3: u8,
    dig_H4: i16,
    dig_H5: i16,
    dig_H6: i8,
}

// --------------------------------------------------------------------------- //
// Résultats compensés
// --------------------------------------------------------------------------- //

/// Mesures compensées du BME280.
#[derive(Debug, Clone, Copy)]
pub struct Bme280Measurement {
    /// Température en **centièmes de °C** (2345 → 23.45 °C)
    pub temperature_cdeg: i32,
    /// Pression en **centièmes de hPa** (101325 → 1013.25 hPa)
    pub pressure_chpa: u32,
    /// Humidité relative en **centièmes de %RH** (4863 → 48.63 %)
    pub humidity_cpct: u32,
}

// --------------------------------------------------------------------------- //
// Driver
// --------------------------------------------------------------------------- //

/// Driver niveau applicatif pour le capteur BME280.
pub struct Bme280 {
    addr: I2cAddr,
    calib: Calib,
}

impl Bme280 {
    /// Initialise le BME280 sur le bus I2C fourni.
    ///
    /// - `addr` : adresse 7 bits (`0x76` ou `0x77`)
    /// - `i2c`  : driver I2C1 déjà initialisé
    ///
    /// # Errors
    /// [`Bme280Error::InvalidChipId`] si le composant ne répond pas avec `0x60`.
    pub fn init(i2c: &mut I2c1<Ready>, addr: I2cAddr) -> Result<Self, Bme280Error> {
        // --- 1. Vérifier chip_id ---
        let mut id = [0u8; 1];
        i2c.write_read(addr, RegAddr(REG_CHIP_ID), &mut id)?;
        if id[0] != CHIP_ID_BME280 {
            return Err(Bme280Error::InvalidChipId(id[0]));
        }

        // --- 2. Soft reset ---
        i2c.write(addr, &[REG_RESET, SOFT_RESET_CMD])?;

        // --- 3. Attendre fin chargement NVM (status.im_update) ---
        let mut retries = NVM_WAIT_RETRIES;
        loop {
            let mut status = [0u8; 1];
            i2c.write_read(addr, RegAddr(REG_STATUS), &mut status)?;
            if status[0] & 0x01 == 0 {
                break;
            }
            retries = retries.saturating_sub(1);
            if retries == 0 {
                return Err(Bme280Error::NvmTimeout);
            }
            // Petite attente
            for _ in 0..1000 {
                cortex_m::asm::nop();
            }
        }

        // --- 4. Lire les coefficients de calibration ---
        let calib = Self::read_calib(i2c, addr)?;

        // --- 5. Configurer le capteur ---
        // ctrl_hum : oversampling humidité × 1 (osrs_h = 001)
        // DOIT être écrit AVANT ctrl_meas (contrainte BME280)
        i2c.write(addr, &[REG_CTRL_HUM, 0x01])?;

        // config   : t_sb=0.5ms, filter=off, spi3w=0
        i2c.write(addr, &[REG_CONFIG, 0x00])?;

        // ctrl_meas: osrs_t=010 (×2), osrs_p=101 (×16), mode=11 (Normal)
        // [7:5]=010 [4:2]=101 [1:0]=11 → 0b0101_0111 = 0x57
        i2c.write(addr, &[REG_CTRL_MEAS, 0x57])?;

        defmt::info!("BME280 initialisé @ addr=0x{:02x}", addr.0);

        Ok(Bme280 { addr, calib })
    }

    /// Lit les coefficients de calibration depuis les registres internes.
    #[allow(non_snake_case)]
    fn read_calib(i2c: &mut I2c1<Ready>, addr: I2cAddr) -> Result<Calib, Bme280Error> {
        // Bloc 1 : 0x88..0x9F – 24 octets (T et P)
        let mut b = [0u8; 24];
        i2c.write_read(addr, RegAddr(REG_CALIB_00), &mut b)?;

        let dig_T1 = u16::from_le_bytes([b[0],  b[1]]);
        let dig_T2 = i16::from_le_bytes([b[2],  b[3]]);
        let dig_T3 = i16::from_le_bytes([b[4],  b[5]]);
        let dig_P1 = u16::from_le_bytes([b[6],  b[7]]);
        let dig_P2 = i16::from_le_bytes([b[8],  b[9]]);
        let dig_P3 = i16::from_le_bytes([b[10], b[11]]);
        let dig_P4 = i16::from_le_bytes([b[12], b[13]]);
        let dig_P5 = i16::from_le_bytes([b[14], b[15]]);
        let dig_P6 = i16::from_le_bytes([b[16], b[17]]);
        let dig_P7 = i16::from_le_bytes([b[18], b[19]]);
        let dig_P8 = i16::from_le_bytes([b[20], b[21]]);
        let dig_P9 = i16::from_le_bytes([b[22], b[23]]);

        // dig_H1 : registre 0xA1 (1 octet isolé)
        let mut h1 = [0u8; 1];
        i2c.write_read(addr, RegAddr(REG_CALIB_H1), &mut h1)?;
        let dig_H1 = h1[0];

        // Bloc humidité : 0xE1..0xE7 (7 octets)
        let mut hb = [0u8; 7];
        i2c.write_read(addr, RegAddr(REG_CALIB_H2), &mut hb)?;

        let dig_H2 = i16::from_le_bytes([hb[0], hb[1]]);
        let dig_H3 = hb[2];
        // dig_H4 = [11:4] hb[3] | [3:0] hb[4][3:0]
        let dig_H4 = ((hb[3] as i16) << 4) | ((hb[4] as i16) & 0x0F);
        // dig_H5 = [11:4] hb[5] | [3:0] hb[4][7:4]
        let dig_H5 = ((hb[5] as i16) << 4) | (((hb[4] as i16) >> 4) & 0x0F);
        let dig_H6 = hb[6] as i8;

        Ok(Calib {
            dig_T1, dig_T2, dig_T3,
            dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6,
            dig_P7, dig_P8, dig_P9,
            dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6,
        })
    }

    /// Lit les données brutes et retourne les valeurs compensées.
    ///
    /// # Errors
    /// [`Bme280Error::Bus`] si une erreur I2C survient.
    pub fn read(&mut self, i2c: &mut I2c1<Ready>) -> Result<Bme280Measurement, Bme280Error> {
        // Lire 8 octets depuis 0xF7 :
        // [0..2] = press (msb, lsb, xlsb)   [3..5] = temp   [6..7] = hum
        let mut raw = [0u8; 8];
        i2c.write_read(self.addr, RegAddr(REG_PRESS_MSB), &mut raw)?;

        let adc_P = ((raw[0] as u32) << 12)
                  | ((raw[1] as u32) << 4)
                  | ((raw[2] as u32) >> 4);
        let adc_T = ((raw[3] as u32) << 12)
                  | ((raw[4] as u32) << 4)
                  | ((raw[5] as u32) >> 4);
        let adc_H = ((raw[6] as u32) << 8) | (raw[7] as u32);

        // --- Compensation (formules entières BME280 datasheet §4.2.3) ---
        let (temperature_cdeg, t_fine) = self.compensate_temp(adc_T);
        let pressure_chpa              = self.compensate_press(adc_P, t_fine);
        let humidity_cpct              = self.compensate_hum(adc_H, t_fine);

        Ok(Bme280Measurement { temperature_cdeg, pressure_chpa, humidity_cpct })
    }

    // ----------------------------------------------------------------------- //
    // Compensation – formules entières de la datasheet Bosch (§4.2.3)
    // ----------------------------------------------------------------------- //

    /// Retourne (température en centièmes de °C, t_fine).
    ///
    /// `t_fine` est nécessaire pour compenser pression et humidité.
    #[allow(non_snake_case)]
    fn compensate_temp(&self, adc_T: u32) -> (i32, i32) {
        let c = &self.calib;
        let adc_T = adc_T as i32;

        let var1 = ((adc_T >> 3) - ((c.dig_T1 as i32) << 1))
                   * (c.dig_T2 as i32)
                   >> 11;
        let tmp  = (adc_T >> 4) - (c.dig_T1 as i32);
        let var2 = (((tmp * tmp) >> 12) * (c.dig_T3 as i32)) >> 14;

        let t_fine = var1 + var2;
        // Température en centièmes de °C : t_fine × 5 + 128 >> 8
        let temp = (t_fine * 5 + 128) >> 8;
        (temp, t_fine)
    }

    /// Retourne la pression en centièmes de hPa.
    #[allow(non_snake_case)]
    fn compensate_press(&self, adc_P: u32, t_fine: i32) -> u32 {
        let c = &self.calib;
        let adc_P = adc_P as i64;

        let var1: i64 = (t_fine as i64) - 128_000;
        let var2: i64 = var1 * var1 * (c.dig_P6 as i64);
        let var2        = var2 + ((var1 * (c.dig_P5 as i64)) << 17);
        let var2        = var2 + ((c.dig_P4 as i64) << 35);
        let var1        = ((var1 * var1 * (c.dig_P3 as i64)) >> 8)
                        + ((var1 * (c.dig_P2 as i64)) << 12);
        let var1        = (((1_i64 << 47) + var1) * (c.dig_P1 as i64)) >> 33;

        if var1 == 0 {
            return 0; // éviter la division par zéro
        }

        let p: i64 = 1_048_576 - adc_P;
        let p      = (((p << 31) - var2) * 3_125) / var1;
        let var1   = ((c.dig_P9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        let var2   = ((c.dig_P8 as i64) * p) >> 19;
        let p      = ((p + var1 + var2) >> 8) + ((c.dig_P7 as i64) << 4);

        // p est en Pa × 256 ; convertir en centièmes de hPa
        // Pa × 256 / 256 = Pa → Pa / 100 = hPa → × 100 = centièmes de hPa
        // Raccourci : p / 256 * 100 / 100 = p / 256 mais on garde centièmes :
        // p_cent_hpa = p * 100 / 256 / 100 = p / 256
        // On veut des centièmes de hPa = Pa / 100 * 100... 
        // Détail : p/256 est en Pa, /100 = hPa, ×100 = centièmes hPa → reste p/256
        (p as u32) / 256
    }

    /// Retourne l'humidité en centièmes de %RH.
    #[allow(non_snake_case)]
    fn compensate_hum(&self, adc_H: u32, t_fine: i32) -> u32 {
        let c = &self.calib;
        let adc_H = adc_H as i32;

        let x: i32 = t_fine - 76_800;
        if x == 0 {
            return 0;
        }

        let x = (adc_H << 14)
              - ((c.dig_H4 as i32) << 20)
              - ((c.dig_H5 as i32) * x);
        let x = (x + 16_384) >> 15;
        let x = x * ((((
                    ((x * (c.dig_H6 as i32)) >> 10)
                    * (((x * (c.dig_H3 as i32)) >> 11) + 32_768)
                   ) >> 10)
                   + 2_097_152) * (c.dig_H2 as i32)
                   + 8_192)
                  >> 14;
        let x = x - (((((x >> 15) * (x >> 15)) >> 7) * (c.dig_H1 as i32)) >> 4);
        let x = x.clamp(0, 419_430_400);

        // Résultat en Q22.10 → diviser par 1024 donne %RH × 1000
        // On ramène en centièmes : × 100 / 1000 = / 10
        ((x >> 12) as u32) / 10
    }
}
