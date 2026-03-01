//! # bus – Types partagés et trait d'abstraction des bus
//!
//! Ce module ne contient **aucune dépendance au PAC** (stm32f4) ni à
//! `cortex-m`. Il peut donc être compilé à la fois sur la cible embedded
//! et sur la machine hôte (tests).
//!
//! Les drivers bas niveau (`i2c.rs`, SPI futur) implémentent les traits
//! définis ici. Les drivers capteurs et les tests unitaires n'importent
//! que ce module.

// --------------------------------------------------------------------------- //
// Newtypes communs
// --------------------------------------------------------------------------- //

/// Fréquence d'horloge en Hz.
///
/// Exemple : `ClockHz(16_000_000)` pour 16 MHz HSI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockHz(pub u32);

/// Calcule la valeur du registre BRR (Baud Rate Register) pour l'USART.
///
/// Formule : `BRR = apb_hz / baud`
/// - Les 4 bits bas encodent la fraction (× 16).
/// - Les bits supérieurs encodent la mantisse entière.
pub fn compute_brr(apb1_hz: ClockHz, baud: u32) -> u32 {
    // USARTDIV × 16 pour garder la précision entière (arrondi au plus proche)
    let div_x16 = (apb1_hz.0 + baud / 2) / baud;
    let mantissa = div_x16 >> 4;
    let fraction = div_x16 & 0xF;
    (mantissa << 4) | fraction
}

/// Adresse 7 bits d'un périphérique I2C (non décalée).
///
/// Exemple : `I2cAddr(0x76)` pour le BME280 avec SDO à GND.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cAddr(pub u8);

/// Adresse d'un registre interne d'un composant I2C.
///
/// Exemple : `RegAddr(0x3B)` pour le premier registre de mesure du MPU6050.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegAddr(pub u8);

// --------------------------------------------------------------------------- //
// Erreurs I2C
// --------------------------------------------------------------------------- //

/// Erreurs possibles lors d'une transaction I2C.
///
/// Pas d'`unwrap()` dans le code de production – toujours propagé avec `?`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// L'esclave n'a pas répondu (NACK sur l'adresse ou sur une donnée).
    Nack,
    /// Timeout : un flag matériel n'est pas levé dans le délai imparti.
    Timeout,
    /// Erreur d'arbitrage : un autre maître a gagné le bus.
    ArbitrationLost,
    /// Erreur de bus détectée (START/STOP invalide).
    BusError,
}

// --------------------------------------------------------------------------- //
// Types SPI partagés (pas de dépendance PAC)
// --------------------------------------------------------------------------- //

/// Erreurs possibles lors d'une transaction SPI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    /// Timeout : TXE ou RXNE non levé dans le délai imparti.
    Timeout,
    /// Overrun : un octet reçu n'a pas été lu avant d'en recevoir un autre.
    Overrun,
    /// Mode Fault détecté (conflit de maîtres).
    ModeFault,
}

/// Diviseur de fréquence SPI (BR[2:0] dans CR1).
///
/// La fréquence résultante est `APB2 / diviseur`.
#[derive(Debug, Clone, Copy)]
pub enum SpiDiv {
    Div2   = 0b000,
    Div4   = 0b001,
    Div8   = 0b010,
    Div16  = 0b011,
    Div32  = 0b100,
    Div64  = 0b101,
    Div128 = 0b110,
    Div256 = 0b111,
}

// --------------------------------------------------------------------------- //
// Trait I2cBus
// --------------------------------------------------------------------------- //

/// Abstraction d'un bus I2C maître.
///
/// Implémenté par :
/// - [`crate::i2c::I2c1<Ready>`] → hardware STM32F411
/// - [`crate::hal_mock::MockI2c`]  → stub pour les tests host
///
/// Les drivers capteurs ([`crate::sensors`]) prennent `&mut impl I2cBus`
/// pour être indépendants du hardware concret.
pub trait I2cBus {
    /// Écrit `data` vers l'esclave à l'adresse `addr`.
    ///
    /// Séquence : START → SLA+W → data[0..n] → STOP
    fn write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError>;

    /// Lit `buf.len()` octets depuis l'esclave à `addr`.
    ///
    /// Séquence : START → SLA+R → buf[0..n] (ACK/NACK) → STOP
    fn read(&mut self, addr: I2cAddr, buf: &mut [u8]) -> Result<(), I2cError>;

    /// Écrit l'adresse du registre `reg` puis lit `buf.len()` octets.
    ///
    /// Séquence : START → SLA+W → reg → RESTART → SLA+R → buf[0..n] → STOP
    fn write_read(
        &mut self,
        addr: I2cAddr,
        reg: RegAddr,
        buf: &mut [u8],
    ) -> Result<(), I2cError>;
}
