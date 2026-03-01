//! # I2C1 – Driver bas niveau
//!
//! Implémente le maître I2C1 du STM32F411 directement sur les registres PAC,
//! et expose le trait [`crate::bus::I2cBus`] pour les capteurs et les tests.
//!
//! ## Câblage (Nucleo-F411RE)
//! ```
//! PB6  →  I2C1_SCL  (AF4, open-drain, pull-up externe 4.7 kΩ)
//! PB7  →  I2C1_SDA  (AF4, open-drain, pull-up externe 4.7 kΩ)
//! ```
//!
//! ## Calculs de timing (APB1 = 16 MHz HSI)
//!
//! ### Standard mode (Sm, 100 kHz)
//! ```
//! FREQ   = 16  (bits CR2[5:0])
//! CCR    = 80  = 16_000_000 / (2 × 100_000)
//! TRISE  = 17  = floor(1000 ns / 62.5 ns) + 1  (max rise time Sm = 1000 ns)
//! ```
//!
//! ### Fast mode (Fm, 400 kHz, duty = 0 → Tlow/Thigh = 2)
//! ```
//! CCR    = 13  = 16_000_000 / (3 × 400_000)
//! TRISE  = 6   = floor(300 ns / 62.5 ns) + 1  (max rise time Fm = 300 ns)
//! ```

use core::marker::PhantomData;
use stm32f4::stm32f411::{GPIOB, I2C1, RCC};

// Réexports pratiques depuis le module bus (évite d'importer deux modules)
pub use crate::bus::{I2cAddr, I2cBus, I2cError, RegAddr};

// --------------------------------------------------------------------------- //
// Fréquence du bus
// --------------------------------------------------------------------------- //

/// Fréquence de l'horloge I2C.
#[derive(Debug, Clone, Copy)]
pub enum I2cFreq {
    /// Mode standard – 100 kHz (RM0383 tableau 80)
    Standard,
    /// Mode rapide   – 400 kHz (RM0383 tableau 81)
    Fast,
}

// --------------------------------------------------------------------------- //
// Timeout
// --------------------------------------------------------------------------- //

/// Nombre de boucles d'attente avant de déclarer un timeout.
///
/// Calibré empiriquement pour ~5 ms à 16 MHz.
const TIMEOUT_CYCLES: u32 = 10_000;

// --------------------------------------------------------------------------- //
// Typestate
// --------------------------------------------------------------------------- //

/// Marqueur : I2C non configuré.
pub struct Uninitialized;
/// Marqueur : I2C prêt.
pub struct Ready;

// --------------------------------------------------------------------------- //
// Driver
// --------------------------------------------------------------------------- //

/// Driver I2C1 côté maître.
///
/// ```rust,ignore
/// let i2c = I2c1::init(&rcc, &gpiob, i2c1_periph, I2cFreq::Standard);
/// i2c.write(I2cAddr(0x68), &[RegAddr(0x6B).0, 0x00])?;
/// ```
pub struct I2c1<STATE> {
    i2c: I2C1,
    _state: PhantomData<STATE>,
}

impl I2c1<Uninitialized> {
    /// Initialise l'I2C1 avec la fréquence demandée.
    ///
    /// ### Séquence d'init (RM0383 §18.3.3)
    /// 1. Reset logiciel du périphérique (PE=0)
    /// 2. Horloge GPIOB + configuration PB6/PB7 AF4 open-drain
    /// 3. Horloge I2C1 (APB1) + reset peripheral
    /// 4. CR2 : fréquence APB1 en MHz
    /// 5. CCR et TRISE selon le mode
    /// 6. Activer le périphérique (PE=1)
    ///
    /// # Safety
    /// Accès exclusif aux registres I2C1, GPIOB, RCC.
    pub fn init(rcc: &RCC, gpiob: &GPIOB, i2c: I2C1, freq: I2cFreq) -> I2c1<Ready> {
        // ------------------------------------------------------------------- //
        // 1. Horloge GPIOB
        // ------------------------------------------------------------------- //
        rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit());
        cortex_m::asm::dsb();

        // ------------------------------------------------------------------- //
        // 2. PB6 (SCL) et PB7 (SDA) en AF4, open-drain, pull-up
        // ------------------------------------------------------------------- //
        gpiob.moder.modify(|_, w| {
            w.moder6().alternate()
             .moder7().alternate()
        });
        // Open-drain obligatoire pour I2C (RM0383 §18.3.3)
        gpiob.otyper.modify(|_, w| {
            w.ot6().open_drain()
             .ot7().open_drain()
        });
        gpiob.ospeedr.modify(|_, w| {
            w.ospeedr6().fast_speed()
             .ospeedr7().fast_speed()
        });
        gpiob.pupdr.modify(|_, w| {
            w.pupdr6().pull_up()
             .pupdr7().pull_up()
        });
        // AF4 = 0b0100 sur AFRL (pins 0..7)
        gpiob.afrl.modify(|_, w| {
            w.afrl6().af4()
             .afrl7().af4()
        });

        // ------------------------------------------------------------------- //
        // 3. Horloge I2C1 + reset périphérique
        // ------------------------------------------------------------------- //
        rcc.apb1enr.modify(|_, w| w.i2c1en().set_bit());
        // Reset logiciel pour s'assurer d'un état propre
        rcc.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());
        cortex_m::asm::dsb();

        // ------------------------------------------------------------------- //
        // 4. CR2 : fréquence APB1 (doit être en MHz, ici 16)
        // ------------------------------------------------------------------- //
        // SAFETY: PE=0 à ce stade (pas encore activé)
        unsafe { i2c.cr2.modify(|_, w| w.freq().bits(16)) };

        // ------------------------------------------------------------------- //
        // 5. CCR et TRISE
        // ------------------------------------------------------------------- //
        match freq {
            I2cFreq::Standard => {
                // Sm : CCR = APB1 / (2 × SCL) = 80, TRISE = 17
                unsafe {
                    i2c.ccr.write(|w| {
                        w.f_s().clear_bit() // mode standard
                         .duty().clear_bit()
                         .ccr().bits(80)
                    });
                    i2c.trise.write(|w| w.trise().bits(17));
                }
            }
            I2cFreq::Fast => {
                // Fm duty=0 : CCR = APB1 / (3 × SCL) = 13, TRISE = 6
                unsafe {
                    i2c.ccr.write(|w| {
                        w.f_s().set_bit()   // mode rapide
                         .duty().clear_bit() // rapport 2:1 (Tlow = 2 × Thigh)
                         .ccr().bits(13)
                    });
                    i2c.trise.write(|w| w.trise().bits(6));
                }
            }
        }

        // ------------------------------------------------------------------- //
        // 6. Activer le périphérique
        // ------------------------------------------------------------------- //
        i2c.cr1.modify(|_, w| w.pe().set_bit());

        defmt::info!("I2C1 initialisé en mode {:?}", freq);

        I2c1 { i2c, _state: PhantomData }
    }
}

// --------------------------------------------------------------------------- //
// Fonctions internes (helpers de flag)
// --------------------------------------------------------------------------- //

impl I2c1<Ready> {
    /// Attend qu'un flag soit à 1, avec timeout.
    fn wait_flag<F>(&self, mut flag: F) -> Result<(), I2cError>
    where
        F: FnMut() -> bool,
    {
        let mut count = TIMEOUT_CYCLES;
        while !flag() {
            // Vérifie les erreurs de bus à chaque itération
            let sr1 = self.i2c.sr1.read();
            if sr1.af().bit_is_set() {
                // Acknowledge Failure → NACK
                self.i2c.sr1.modify(|_, w| w.af().clear_bit());
                self.generate_stop();
                return Err(I2cError::Nack);
            }
            if sr1.arlo().bit_is_set() {
                self.i2c.sr1.modify(|_, w| w.arlo().clear_bit());
                return Err(I2cError::ArbitrationLost);
            }
            if sr1.berr().bit_is_set() {
                self.i2c.sr1.modify(|_, w| w.berr().clear_bit());
                return Err(I2cError::BusError);
            }
            count = count.saturating_sub(1);
            if count == 0 {
                self.generate_stop();
                return Err(I2cError::Timeout);
            }
            cortex_m::asm::nop();
        }
        Ok(())
    }

    /// Génère une condition START.
    fn generate_start(&self) -> Result<(), I2cError> {
        self.i2c.cr1.modify(|_, w| w.start().set_bit());
        // Attendre SB (Start Bit généré)
        self.wait_flag(|| self.i2c.sr1.read().sb().bit_is_set())
    }

    /// Génère une condition STOP.
    fn generate_stop(&self) {
        self.i2c.cr1.modify(|_, w| w.stop().set_bit());
    }

    /// Envoie l'adresse 7 bits avec le bit R/W.
    ///
    /// `rw = false` → write (0), `rw = true` → read (1).
    fn send_addr(&self, addr: I2cAddr, rw: bool) -> Result<(), I2cError> {
        let byte = (addr.0 << 1) | (rw as u8);
        // SAFETY: DR accessible après SB=1
        unsafe { self.i2c.dr.write(|w| w.dr().bits(byte as u16)) };
        // Attendre ADDR (adresse envoyée + ACK reçu)
        self.wait_flag(|| self.i2c.sr1.read().addr().bit_is_set())?;
        // Effacer ADDR en lisant SR1 puis SR2
        let _ = self.i2c.sr1.read();
        let _ = self.i2c.sr2.read();
        Ok(())
    }

    /// Envoie un octet et attend TXE ou BTF.
    fn send_byte(&self, byte: u8) -> Result<(), I2cError> {
        // Attendre TXE (Data Register Empty)
        self.wait_flag(|| self.i2c.sr1.read().tx_e().bit_is_set())?;
        // SAFETY: TXE=1, DR prêt
        unsafe { self.i2c.dr.write(|w| w.dr().bits(byte as u16)) };
        Ok(())
    }
}

// --------------------------------------------------------------------------- //
// API publique
// --------------------------------------------------------------------------- //

impl I2c1<Ready> {
    /// Écrit `data` vers l'esclave à l'adresse `addr`.
    ///
    /// Séquence : START → SLA+W → data[0] → … → data[n-1] → STOP
    ///
    /// # Erreurs
    /// - [`I2cError::Nack`] si aucun esclave ne répond
    /// - [`I2cError::Timeout`] si un flag n'est pas levé dans les délais
    pub fn write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError> {
        self.generate_start()?;
        self.send_addr(addr, false)?;

        for &byte in data {
            self.send_byte(byte)?;
        }

        // Attendre BTF (Byte Transfer Finished) avant le STOP
        self.wait_flag(|| self.i2c.sr1.read().btf().bit_is_set())?;
        self.generate_stop();
        Ok(())
    }

    /// Lit `buf.len()` octets depuis l'esclave à l'adresse `addr`.
    ///
    /// Séquence : START → SLA+R → data[0] → … → data[n-2] (ACK) → data[n-1] (NACK) → STOP
    ///
    /// # Erreurs
    /// - [`I2cError::Nack`] si l'esclave répond NACK
    /// - [`I2cError::Timeout`]
    pub fn read(&mut self, addr: I2cAddr, buf: &mut [u8]) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }

        // Activer ACK avant le START
        self.i2c.cr1.modify(|_, w| w.ack().set_bit());

        self.generate_start()?;
        self.send_addr(addr, true)?;

        let len = buf.len();
        for (i, slot) in buf.iter_mut().enumerate() {
            // Avant le dernier octet : désactiver ACK + générer STOP après
            if i == len - 1 {
                self.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                self.generate_stop();
            }
            // Attendre RXNE (Receive Data Register Not Empty)
            self.wait_flag(|| self.i2c.sr1.read().rx_ne().bit_is_set())?;
            // SAFETY: RXNE=1, DR contains new byte
            *slot = (self.i2c.dr.read().dr().bits() & 0xFF) as u8;
        }

        Ok(())
    }

    /// Écrit l'adresse du registre `reg` puis lit `buf.len()` octets.
    ///
    /// Séquence : START → SLA+W → reg → RESTART → SLA+R → buf[0..n] → STOP
    ///
    /// Utilisé par les capteurs pour lire un registre interne (ex. BME280).
    ///
    /// # Erreurs
    /// - [`I2cError::Nack`], [`I2cError::Timeout`], [`I2cError::BusError`]
    pub fn write_read(
        &mut self,
        addr: I2cAddr,
        reg: RegAddr,
        buf: &mut [u8],
    ) -> Result<(), I2cError> {
        // Phase WRITE : START → SLA+W → reg
        self.generate_start()?;
        self.send_addr(addr, false)?;
        self.send_byte(reg.0)?;
        self.wait_flag(|| self.i2c.sr1.read().btf().bit_is_set())?;

        // Phase READ : RESTART → SLA+R → octets
        self.read(addr, buf)
    }
}
// --------------------------------------------------------------------------- //
// impl I2cBus for I2c1<Ready>
// --------------------------------------------------------------------------- //

impl I2cBus for I2c1<Ready> {
    fn write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError> {
        I2c1::write(self, addr, data)
    }
    fn read(&mut self, addr: I2cAddr, buf: &mut [u8]) -> Result<(), I2cError> {
        I2c1::read(self, addr, buf)
    }
    fn write_read(
        &mut self,
        addr: I2cAddr,
        reg: RegAddr,
        buf: &mut [u8],
    ) -> Result<(), I2cError> {
        I2c1::write_read(self, addr, reg, buf)
    }
}