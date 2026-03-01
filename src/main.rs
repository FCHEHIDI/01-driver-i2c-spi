//! # main.rs – Point d'entrée du firmware bare-metal
//!
//! Câble tous les modules ensemble sur le Nucleo-F411RE :
//! - UART2  : sortie série via ST-LINK Virtual COM Port (115 200 baud)
//! - I2C1   : bus capteurs à 100 kHz (PB6/PB7)
//! - SPI1   : bus capteur alternatif à 1 MHz (PA5/PA6/PA7, CS=PA4)
//! - BME280 : température, pression, humidité (adresse I2C 0x76)
//! - MPU6050: accéléromètre + gyroscope (adresse I2C 0x68)
//!
//! Les sorties sont affichées via :
//! - `defmt::info!` → probe-rs / `cargo embed`
//! - `core::fmt::Write` sur UART2 → minicom / screen @ 115200 8N1
//!
//! ## Câblage résumé
//! ```
//! STM32F411  │  Signal       │  Composant
//! ───────────┼───────────────┼──────────────────────
//! PA2  (AF7) │  USART2_TX    │  ST-LINK VCP
//! PA3  (AF7) │  USART2_RX    │  ST-LINK VCP
//! PB6  (AF4) │  I2C1_SCL     │  BME280 + MPU6050
//! PB7  (AF4) │  I2C1_SDA     │  BME280 + MPU6050
//! PA5  (AF5) │  SPI1_SCK     │  (disponible)
//! PA6  (AF5) │  SPI1_MISO    │  (disponible)
//! PA7  (AF5) │  SPI1_MOSI    │  (disponible)
//! PA4  (GPIO)│  SPI1_CS      │  actif bas
//! ```

#![no_std]
#![no_main]

use core::fmt::Write;

use cortex_m_rt::entry;
use defmt_rtt as _; // transport RTT pour defmt
use panic_probe as _; // gestion des panics → defmt
use stm32f4; // PAC – accès aux périphériques matériels

use driver_i2c_spi::{
    i2c::{I2cFreq, I2c1},
    spi::{Spi1, SpiDiv},
    uart::{ClockHz, Uart2},
    sensors::{
        bme280::Bme280,
        mpu6050::{AccelRange, GyroRange, Mpu6050},
    },
    bus::I2cAddr,
};

// --------------------------------------------------------------------------- //
// Constantes de configuration
// --------------------------------------------------------------------------- //

/// Adresse I2C du BME280 : SDO → GND.
const BME280_ADDR: I2cAddr = I2cAddr(0x76);

/// Adresse I2C du MPU6050 : AD0 → GND.
const MPU6050_ADDR: I2cAddr = I2cAddr(0x68);

/// Fréquence APB1/APB2 par défaut sur le STM32F411 (HSI 16 MHz, pas de PLL).
const APB_HZ: ClockHz = ClockHz(16_000_000);

/// Baud rate UART cible.
const BAUD: u32 = 115_200;

/// Délai approximatif entre deux mesures (boucle ~160 k cycles @ 16 MHz = ~10 ms).
const LOOP_DELAY_CYCLES: u32 = 160_000;

// --------------------------------------------------------------------------- //
// Point d'entrée
// --------------------------------------------------------------------------- //

#[entry]
fn main() -> ! {
    defmt::info!("=== driver-i2c-spi boot ===");

    // --------------------------------------------------------------------- //
    // 1. Prendre les périphériques (singleton PAC)
    // --------------------------------------------------------------------- //
    let dp = stm32f4::stm32f411::Peripherals::take().unwrap();
    // SAFETY: unwrap dans main() est acceptable – premier appel garanti unique

    let rcc   = &dp.RCC;
    let gpioa = &dp.GPIOA;
    let gpiob = &dp.GPIOB;
    let usart2 = dp.USART2;
    let i2c1   = dp.I2C1;
    let spi1   = dp.SPI1;

    // --------------------------------------------------------------------- //
    // 2. UART2 – sortie de débogage
    // --------------------------------------------------------------------- //
    let mut uart = Uart2::init(rcc, gpioa, usart2, APB_HZ, BAUD);
    writeln!(uart, "\r\n--- boot firmware ---").ok();

    // --------------------------------------------------------------------- //
    // 3. I2C1 – bus capteurs 100 kHz
    // --------------------------------------------------------------------- //
    let mut i2c = I2c1::init(rcc, gpiob, i2c1, I2cFreq::Standard);

    // --------------------------------------------------------------------- //
    // 4. SPI1 – disponible pour ajout de composant SPI (ex: SD card)
    // --------------------------------------------------------------------- //
    // Diviseur Div16 → 1 MHz @ 16 MHz APB2
    let _spi = Spi1::init(rcc, gpioa, spi1, SpiDiv::Div16);

    // --------------------------------------------------------------------- //
    // 5. BME280 – capteur ambiant
    // --------------------------------------------------------------------- //
    let mut bme280 = match Bme280::init(&mut i2c, BME280_ADDR) {
        Ok(d) => {
            defmt::info!("BME280 OK");
            writeln!(uart, "BME280 OK").ok();
            d
        }
        Err(e) => {
            defmt::error!("BME280 init échoué : {:?}", defmt::Debug2Format(&e));
            writeln!(uart, "BME280 ERREUR - vérifier câblage").ok();
            // Bloque ici si le capteur est absent – safe en bare-metal
            loop {
                cortex_m::asm::nop();
            }
        }
    };

    // --------------------------------------------------------------------- //
    // 6. MPU6050 – IMU
    // --------------------------------------------------------------------- //
    let mut mpu6050 = match Mpu6050::init(&mut i2c, MPU6050_ADDR, AccelRange::G2, GyroRange::Dps250) {
        Ok(d) => {
            defmt::info!("MPU6050 OK");
            writeln!(uart, "MPU6050 OK").ok();
            d
        }
        Err(e) => {
            defmt::error!("MPU6050 init échoué : {:?}", defmt::Debug2Format(&e));
            writeln!(uart, "MPU6050 ERREUR - vérifier câblage").ok();
            loop {
                cortex_m::asm::nop();
            }
        }
    };

    // --------------------------------------------------------------------- //
    // 7. Boucle de mesure principale
    // --------------------------------------------------------------------- //
    writeln!(uart, "--- boucle de mesure ---").ok();

    let mut tick: u32 = 0;
    loop {
        tick = tick.wrapping_add(1);

        // ----------------------------------------------------------------- //
        // Lecture BME280
        // ----------------------------------------------------------------- //
        match bme280.read(&mut i2c) {
            Ok(m) => {
                let t_int   = m.temperature_cdeg / 100;
                let t_frac  = (m.temperature_cdeg.unsigned_abs()) % 100;
                let p_int   = m.pressure_chpa / 100;
                let p_frac  = m.pressure_chpa % 100;
                let h_int   = m.humidity_cpct / 100;
                let h_frac  = m.humidity_cpct % 100;

                defmt::info!(
                    "[{}] BME280: T={}.{:02}°C P={}.{:02}hPa H={}.{:02}%",
                    tick, t_int, t_frac, p_int, p_frac, h_int, h_frac
                );
                writeln!(
                    uart,
                    "[{}] T={}.{:02}C P={}.{:02}hPa H={}.{:02}%",
                    tick, t_int, t_frac, p_int, p_frac, h_int, h_frac
                ).ok();
            }
            Err(e) => {
                defmt::warn!("BME280 read err: {:?}", defmt::Debug2Format(&e));
            }
        }

        // ----------------------------------------------------------------- //
        // Lecture MPU6050
        // ----------------------------------------------------------------- //
        match mpu6050.read(&mut i2c) {
            Ok(m) => {
                defmt::info!(
                    "[{}] MPU6050: ax={} ay={} az={} mg | gx={} gy={} gz={} cdps | T={}.{:02}C",
                    tick,
                    m.accel_x_mg, m.accel_y_mg, m.accel_z_mg,
                    m.gyro_x_cdps, m.gyro_y_cdps, m.gyro_z_cdps,
                    m.temp_cdeg / 100, (m.temp_cdeg.unsigned_abs()) % 100
                );
                writeln!(
                    uart,
                    "[{}] ax={} ay={} az={} mg gx={} gy={} gz={} cdps",
                    tick,
                    m.accel_x_mg, m.accel_y_mg, m.accel_z_mg,
                    m.gyro_x_cdps, m.gyro_y_cdps, m.gyro_z_cdps
                ).ok();
            }
            Err(e) => {
                defmt::warn!("MPU6050 read err: {:?}", defmt::Debug2Format(&e));
            }
        }

        // ----------------------------------------------------------------- //
        // Attente (compteur de cycles – pas de SysTick ni de timer)
        // ----------------------------------------------------------------- //
        for _ in 0..LOOP_DELAY_CYCLES {
            cortex_m::asm::nop();
        }
    }
}
