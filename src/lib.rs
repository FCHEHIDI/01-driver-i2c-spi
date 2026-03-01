//! # driver-i2c-spi
//!
//! Drivers bare-metal I2C / SPI / UART pour le STM32F411 (Nucleo-F411RE).
//!
//! Écrits directement depuis les registres PAC (`stm32f4` crate),
//! sans HAL, en `#![no_std]`.
//!
//! ## Organisation des modules
//! ```text
//! lib.rs
//! ├── uart     → USART2 @ PA2/PA3  – printf-style via fmt::Write
//! ├── i2c      → I2C1   @ PB6/PB7  – mode maître 100 / 400 kHz
//! ├── spi      → SPI1   @ PA5/PA6/PA7 – mode maître CPOL=0 CPHA=0
//! ├── sensors/
//! │   ├── bme280   → Température / Pression / Humidité via I2C
//! │   └── mpu6050  → Accéléromètre + Gyroscope via I2C
//! └── hal_mock → Stubs pour les tests unitaires (hôte)
//! ```

#![no_std]

pub mod i2c;
pub mod spi;
pub mod uart;

pub mod sensors {
    pub mod bme280;
    pub mod mpu6050;
}

// hal_mock est compilé seulement lors des tests (cfg(test))
// ou explicitement demandé – il n'a pas de code hardware
#[cfg(any(test, feature = "mock"))]
pub mod hal_mock;
