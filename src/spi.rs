//! # SPI1 – Driver bas niveau
//!
//! Implémente le maître SPI1 du STM32F411 directement sur les registres PAC.
//!
//! ## Câblage (Nucleo-F411RE)
//! ```
//! PA5  →  SPI1_SCK   (AF5, push-pull)
//! PA6  ←  SPI1_MISO  (AF5, pull-up)
//! PA7  →  SPI1_MOSI  (AF5, push-pull)
//! PA4  →  NSS (CS)   géré manuellement en GPIO — actif bas
//! ```
//!
//! ## Configuration par défaut
//! - Mode maître, full-duplex
//! - CPOL = 0, CPHA = 0  (mode 0 : capture sur front montant)
//! - Frame 8 bits, MSB en premier
//! - NSS géré par logiciel (SSM=1, SSI=1)
//! - Diviseur d'horloge configurable via [`SpiDiv`]
//!
//! ## Fréquences disponibles (APB2 = 16 MHz HSI)
//! | [`SpiDiv`]  | Fréquence SPI |
//! |-------------|--------------|
//! | Div2        | 8 MHz        |
//! | Div4        | 4 MHz        |
//! | Div8        | 2 MHz        |
//! | Div16       | 1 MHz        |
//! | Div32       | 500 kHz      |
//! | Div64       | 250 kHz      |
//! | Div128      | 125 kHz      |
//! | Div256      | 62.5 kHz     |

use core::marker::PhantomData;
use stm32f4::stm32f411::{GPIOA, RCC, SPI1};

// SpiError et SpiDiv sont définis dans bus.rs (sans dépendance PAC)
// et réexportés ici pour la compatibilité des imports existants.
pub use crate::bus::{SpiDiv, SpiError};

// --------------------------------------------------------------------------- //
// Timeout
// --------------------------------------------------------------------------- //

/// Nombre de boucles avant de déclarer un timeout (~5 ms @ 16 MHz).
const TIMEOUT_CYCLES: u32 = 10_000;

// --------------------------------------------------------------------------- //
// Typestate
// --------------------------------------------------------------------------- //

/// Marqueur : SPI non configuré.
pub struct Uninitialized;
/// Marqueur : SPI prêt.
pub struct Ready;

// --------------------------------------------------------------------------- //
// Driver
// --------------------------------------------------------------------------- //

/// Driver SPI1 côté maître.
///
/// Le NSS (Chip Select) est géré manuellement par l'appelant via
/// la broche PA4 en GPIO – cela permet de contrôler le CS fin-grain.
///
/// ```rust,ignore
/// let spi = Spi1::init(&rcc, &gpioa, spi1_periph, SpiDiv::Div16);
/// // Abaisser CS manuellement
/// let buf = [0x80, 0x00]; // commande lecture + dummy
/// let received = spi.transfer(buf)?;
/// ```
pub struct Spi1<STATE> {
    spi: SPI1,
    _state: PhantomData<STATE>,
}

impl Spi1<Uninitialized> {
    /// Initialise SPI1 côté maître, CPOL=0, CPHA=0, 8 bits, MSB first.
    ///
    /// ### Séquence d'init (RM0383 §20.3.3)
    /// 1. Horloge GPIOA + configuration PA5/PA6/PA7 en AF5
    /// 2. (Optionnel) PA4 en GPIO sortie pour NSS
    /// 3. Horloge SPI1 (APB2)
    /// 4. Configurer CR1 : maître, SSM, CPOL, CPHA, BR
    /// 5. Activer SPI (SPE=1)
    ///
    /// # Safety
    /// Accès exclusif aux registres SPI1, GPIOA, RCC.
    pub fn init(rcc: &RCC, gpioa: &GPIOA, spi: SPI1, div: SpiDiv) -> Spi1<Ready> {
        // ------------------------------------------------------------------- //
        // 1. Horloge GPIOA (peut déjà être active si UART2 utilisé)
        // ------------------------------------------------------------------- //
        rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
        cortex_m::asm::dsb();

        // ------------------------------------------------------------------- //
        // 2. PA5 (SCK), PA6 (MISO), PA7 (MOSI) en AF5
        // ------------------------------------------------------------------- //
        gpioa.moder.modify(|_, w| {
            w.moder5().alternate()
             .moder6().alternate()
             .moder7().alternate()
        });
        // SCK + MOSI push-pull ; MISO input (mode AF suffit)
        gpioa.otyper.modify(|_, w| {
            w.ot5().push_pull()
             .ot7().push_pull()
        });
        gpioa.ospeedr.modify(|_, w| {
            w.ospeedr5().very_high_speed()
             .ospeedr6().very_high_speed()
             .ospeedr7().very_high_speed()
        });
        // Pull-up sur MISO pour état stable quand aucun esclave n'attaque
        gpioa.pupdr.modify(|_, w| w.pupdr6().pull_up());
        // AF5 = 0b0101 sur AFRL (pins 0..7)
        gpioa.afrl.modify(|_, w| {
            w.afrl5().af5()
             .afrl6().af5()
             .afrl7().af5()
        });

        // ------------------------------------------------------------------- //
        // 3. PA4 en GPIO sortie push-pull (NSS software)
        // ------------------------------------------------------------------- //
        gpioa.moder.modify(|_, w| w.moder4().output());
        gpioa.otyper.modify(|_, w| w.ot4().push_pull());
        gpioa.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
        // Désactiver le CS par défaut (actif bas → mettre à 1)
        gpioa.bsrr.write(|w| w.bs4().set_bit());

        // ------------------------------------------------------------------- //
        // 4. Horloge SPI1 (APB2)
        // ------------------------------------------------------------------- //
        rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());
        // Reset pour état propre
        rcc.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
        cortex_m::asm::dsb();

        // ------------------------------------------------------------------- //
        // 5. Configurer CR1
        //   MSTR=1    : mode maître
        //   SSM=1     : NSS géré par logiciel
        //   SSI=1     : NSS interne à 1 (évite le Mode Fault)
        //   CPOL=0    : horloge basse au repos
        //   CPHA=0    : capture sur le 1er front (montant)
        //   DFF=0     : trame 8 bits
        //   LSBFIRST=0: MSB en premier
        //   BR[2:0]   : diviseur
        // ------------------------------------------------------------------- //
        spi.cr1.write(|w| {
            w.mstr()    .set_bit()           // maître
             .ssm()     .set_bit()           // NSS software
             .ssi()     .set_bit()           // NSS interne = 1
             .cpol()    .clear_bit()         // CPOL = 0
             .cpha()    .clear_bit()         // CPHA = 0
             .dff()     .clear_bit()         // 8 bits
             .lsbfirst().clear_bit()         // MSB first
             // SAFETY: la valeur provient de l'enum SpiDiv
             .br()      .bits(div as u8)
        });

        // CR2 : pas d'interruption, pas de DMA
        spi.cr2.reset();

        // ------------------------------------------------------------------- //
        // 6. Activer SPI (SPE=1)
        // ------------------------------------------------------------------- //
        spi.cr1.modify(|_, w| w.spe().set_bit());

        defmt::info!("SPI1 initialisé, diviseur BR={}", div as u8);

        Spi1 { spi, _state: PhantomData }
    }
}

// --------------------------------------------------------------------------- //
// Helpers internes
// --------------------------------------------------------------------------- //

impl Spi1<Ready> {
    /// Attend qu'un flag soit à 1, avec timeout.
    fn wait_flag<F>(&self, mut f: F) -> Result<(), SpiError>
    where
        F: FnMut() -> bool,
    {
        let mut count = TIMEOUT_CYCLES;
        while !f() {
            let sr = self.spi.sr.read();
            if sr.ovr().bit_is_set() {
                return Err(SpiError::Overrun);
            }
            if sr.modf().bit_is_set() {
                return Err(SpiError::ModeFault);
            }
            count = count.saturating_sub(1);
            if count == 0 {
                return Err(SpiError::Timeout);
            }
            cortex_m::asm::nop();
        }
        Ok(())
    }
}

// --------------------------------------------------------------------------- //
// API publique
// --------------------------------------------------------------------------- //

impl Spi1<Ready> {
    /// Transfert full-duplex : envoie et reçoit simultanément.
    ///
    /// Chaque octet de `buf` est transmis **et** remplacé par l'octet reçu.
    ///
    /// Le Chip Select (PA4) doit être abaissé par l'appelant avant cet appel
    /// et relevé ensuite.
    ///
    /// ```rust,ignore
    /// cs_low(&gpioa);
    /// spi.transfer(&mut [0x80, 0x00])?;
    /// cs_high(&gpioa);
    /// ```
    ///
    /// # Erreurs
    /// - [`SpiError::Timeout`]  → BSY/TXE/RXNE jamais prêts
    /// - [`SpiError::Overrun`]  → octet reçu perdu
    /// - [`SpiError::ModeFault`]
    pub fn transfer(&mut self, buf: &mut [u8]) -> Result<(), SpiError> {
        for byte in buf.iter_mut() {
            // Attendre TXE (Transmit Buffer Empty)
            self.wait_flag(|| self.spi.sr.read().txe().bit_is_set())?;
            // SAFETY: TXE=1, on peut écrire DR
            unsafe { self.spi.dr.write(|w| w.dr().bits(*byte as u16)) };

            // Attendre RXNE (Receive Buffer Not Empty)
            self.wait_flag(|| self.spi.sr.read().rxne().bit_is_set())?;
            // SAFETY: RXNE=1, DR contient l'octet reçu
            *byte = (self.spi.dr.read().dr().bits() & 0xFF) as u8;
        }

        // Attendre la fin de transmission (BSY=0) avant de relâcher CS
        self.wait_flag(|| self.spi.sr.read().bsy().bit_is_clear())?;

        Ok(())
    }

    /// Envoie des octets sans lire les données reçues (write-only).
    ///
    /// Utile pour écrire dans un registre d'un capteur SPI.
    pub fn write(&mut self, data: &[u8]) -> Result<(), SpiError> {
        let mut buf = [0u8; 1];
        for &b in data {
            buf[0] = b;
            self.transfer(&mut buf)?;
        }
        Ok(())
    }

    /// Abaisse le Chip Select PA4 (actif bas).
    ///
    /// Doit être appelé avant tout transfert pour activer l'esclave SPI.
    ///
    /// # Safety
    /// Accès direct au registre BSRR de GPIOA.
    pub fn cs_low(&self, gpioa: &GPIOA) {
        // BR4 = reset bit 4 = PA4 → 0
        gpioa.bsrr.write(|w| w.br4().set_bit());
    }

    /// Relève le Chip Select PA4.
    ///
    /// Doit être appelé après tout transfert pour libérer l'esclave SPI.
    pub fn cs_high(&self, gpioa: &GPIOA) {
        // BS4 = set bit 4 = PA4 → 1
        gpioa.bsrr.write(|w| w.bs4().set_bit());
    }
}
