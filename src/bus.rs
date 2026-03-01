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
