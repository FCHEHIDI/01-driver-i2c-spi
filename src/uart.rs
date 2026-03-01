//! # UART2 – Driver bas niveau
//!
//! Implémente le driver USART2 du STM32F411 depuis les registres PAC,
//! sans couche HAL.
//!
//! ## Câblage (Nucleo-F411RE)
//! ```
//! PA2  →  USART2_TX  →  ST-LINK Virtual COM Port (TX)
//! PA3  ←  USART2_RX  ←  ST-LINK Virtual COM Port (RX)
//! ```
//!
//! ## Calcul du baud rate (BRR)
//! ```
//! USARTDIV = f_CK / baud
//! f_CK     = 16 MHz (HSI par défaut après reset)
//! baud     = 115 200
//! USARTDIV = 16_000_000 / 115_200 = 138.88…
//!   → mantisse  = 138  = 0x8A
//!   → fraction  = round(0.88 × 16) = 14  = 0xE
//!   → BRR       = (138 << 4) | 14  = 0x08AE
//! ```

use core::fmt;
use stm32f4::stm32f411::{GPIOA, RCC, USART2};

// --------------------------------------------------------------------------- //
// Newtype partagé
// --------------------------------------------------------------------------- //

/// Fréquence d'horloge en Hz – newtype pour éviter les confusions d'unité.
///
/// Exemple : `ClockHz(16_000_000)` pour 16 MHz HSI.
#[derive(Debug, Clone, Copy)]
pub struct ClockHz(pub u32);

// --------------------------------------------------------------------------- //
// Constantes
// --------------------------------------------------------------------------- //

/// BRR pour 115 200 baud @ 16 MHz HSI (oversampling × 16)
const BRR_115200_16MHZ: u32 = 0x008A_000E; // mantisse=0x8A, fraction=0xE

/// Calcule dynamiquement le BRR depuis la fréquence APB1 et le baud souhaité.
///
/// # Paramètres
/// - `apb1_hz` : fréquence de l'horloge APB1 en Hz
/// - `baud`    : vitesse en bauds cible
///
/// Le résultat est encodé en `[15:4] mantisse | [3:0] fraction/16`.
pub fn compute_brr(apb1_hz: ClockHz, baud: u32) -> u32 {
    // USARTDIV × 16 pour garder la précision entière
    let div_x16 = (apb1_hz.0 + baud / 2) / baud; // arrondi au plus proche
    let mantissa = div_x16 >> 4;
    let fraction = div_x16 & 0xF;
    (mantissa << 4) | fraction
}

// --------------------------------------------------------------------------- //
// Type d'état (typestate pattern)
// --------------------------------------------------------------------------- //

/// Marqueur : UART non encore configuré
pub struct Uninitialized;

/// Marqueur : UART prêt à transmettre/recevoir
pub struct Ready;

// --------------------------------------------------------------------------- //
// Structure principale
// --------------------------------------------------------------------------- //

/// Driver USART2.
///
/// Le paramètre de type `STATE` empêche d'appeler `write_byte` avant `init`.
///
/// ```rust,ignore
/// let uart = Uart2::init(&rcc, &gpioa, &usart2, ClockHz(16_000_000), 115_200);
/// writeln!(uart, "Hello embedded Rust!").ok();
/// ```
pub struct Uart2<STATE> {
    /// Accès direct au bloc USART2 (PAC)
    usart: USART2,
    _state: core::marker::PhantomData<STATE>,
}

impl Uart2<Uninitialized> {
    /// Initialise USART2 à la vitesse demandée.
    ///
    /// ### Séquence d'init (RM0383 §19.3)
    /// 1. Activer l'horloge GPIOA (AHB1ENR.GPIOAEN)
    /// 2. Configurer PA2/PA3 en mode AF, open-drain, pull-up, AF7
    /// 3. Activer l'horloge USART2 (APB1ENR.USART2EN)
    /// 4. Écrire BRR
    /// 5. Activer UE + TE + RE dans CR1
    ///
    /// # Safety
    /// Accès aux registres matériels via `write()` non-atomic – à appeler
    /// une seule fois avant tout autre code qui traite ces périphériques.
    pub fn init(
        rcc: &RCC,
        gpioa: &GPIOA,
        usart: USART2,
        apb1_hz: ClockHz,
        baud: u32,
    ) -> Uart2<Ready> {
        // ------------------------------------------------------------------- //
        // 1. Horloge GPIOA
        // ------------------------------------------------------------------- //
        // SAFETY: modification d'un registre RCC – seul appelant à ce stade
        rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
        cortex_m::asm::dsb(); // barrière de données

        // ------------------------------------------------------------------- //
        // 2. Configurer PA2 (TX) et PA3 (RX)
        //    MODER : 10 (Alternate Function)
        //    OTYPER: 0  (push-pull)
        //    OSPEEDR: 10 (fast)
        //    PUPDR : 01 (pull-up)
        //    AFRL  : AF7 (USART2) pour bits 2 et 3
        // ------------------------------------------------------------------- //
        gpioa.moder.modify(|_, w| {
            w.moder2().alternate() // PA2 = AF
             .moder3().alternate() // PA3 = AF
        });
        gpioa.otyper.modify(|_, w| {
            w.ot2().push_pull()
             .ot3().push_pull()
        });
        gpioa.ospeedr.modify(|_, w| {
            w.ospeedr2().fast_speed()
             .ospeedr3().fast_speed()
        });
        gpioa.pupdr.modify(|_, w| {
            w.pupdr2().pull_up()
             .pupdr3().pull_up()
        });
        // AF7 = 0b0111 sur AFRL (pins 0..7)
        gpioa.afrl.modify(|_, w| {
            w.afrl2().af7()
             .afrl3().af7()
        });

        // ------------------------------------------------------------------- //
        // 3. Horloge USART2 (APB1)
        // ------------------------------------------------------------------- //
        rcc.apb1enr.modify(|_, w| w.usart2en().set_bit());
        cortex_m::asm::dsb();

        // ------------------------------------------------------------------- //
        // 4. Baud rate
        // ------------------------------------------------------------------- //
        let brr = compute_brr(apb1_hz, baud);
        // SAFETY: registre BRR – USART désactivé à ce stade
        usart.brr.write(|w| unsafe { w.bits(brr) });

        // ------------------------------------------------------------------- //
        // 5. Activer USART, émetteur, récepteur
        // ------------------------------------------------------------------- //
        usart.cr1.write(|w| {
            w.ue()  // USART Enable
             .set_bit()
             .te()  // Transmitter Enable
             .set_bit()
             .re()  // Receiver Enable
             .set_bit()
        });

        defmt::info!("UART2 initialisé @ {} baud, BRR=0x{:04x}", baud, brr);

        Uart2 { usart, _state: core::marker::PhantomData }
    }
}

impl Uart2<Ready> {
    /// Envoie un octet et attend que le registre de transmission soit vide.
    ///
    /// Bloque jusqu'à ce que TXE (Transmit Data Register Empty) soit à 1.
    ///
    /// # Safety
    /// Écriture dans DR du USART – aucune protection d'accès concurrent.
    pub fn write_byte(&mut self, byte: u8) {
        // Attendre TXE : registre DR prêt à recevoir un nouvel octet
        while self.usart.sr.read().txe().bit_is_clear() {
            cortex_m::asm::nop();
        }
        // SAFETY: DR est disponible (TXE=1)
        unsafe { self.usart.dr.write(|w| w.dr().bits(byte as u16)) };
    }

    /// Envoie une tranche d'octets.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_byte(b);
        }
    }

    /// Attend que la transmission soit complète (TC=1).
    /// Utile avant de désactiver le UART ou d'entrer en veille.
    pub fn flush(&mut self) {
        while self.usart.sr.read().tc().bit_is_clear() {
            cortex_m::asm::nop();
        }
    }
}

// --------------------------------------------------------------------------- //
// impl fmt::Write → permet d'utiliser write!() / writeln!()
// --------------------------------------------------------------------------- //

/// Implémente `core::fmt::Write` pour pouvoir écrire des chaînes formatées.
///
/// ```rust,ignore
/// use core::fmt::Write;
/// writeln!(uart, "temp = {}.{} °C", t / 100, t % 100).ok();
/// ```
impl fmt::Write for Uart2<Ready> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_bytes(s.as_bytes());
        Ok(())
    }
}
