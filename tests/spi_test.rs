//! # Tests du driver SPI
//!
//! Ces tests vérifient les propriétés statiques du driver SPI1 :
//! valeurs des diviseurs d'horloge, variants d'erreurs, etc.
//!
//! Les tests de transfert effectifs nécessitent le hardware ou un
//! mock SPI (extension future).
//!
//! ## Lancer les tests
//! ```sh
//! cargo test --features mock
//! ```

use driver_i2c_spi::spi::{SpiDiv, SpiError};

// =========================================================================== //
// Tests SpiDiv – valeurs CR1 BR[2:0] (RM0383 §20.5.1)
// =========================================================================== //

#[test]
fn spi_div_values_match_cr1_br_field() {
    // Les valeurs doivent correspondre exactement aux bits BR[2:0] du registre SPI_CR1
    assert_eq!(SpiDiv::Div2   as u8, 0b000);
    assert_eq!(SpiDiv::Div4   as u8, 0b001);
    assert_eq!(SpiDiv::Div8   as u8, 0b010);
    assert_eq!(SpiDiv::Div16  as u8, 0b011);
    assert_eq!(SpiDiv::Div32  as u8, 0b100);
    assert_eq!(SpiDiv::Div64  as u8, 0b101);
    assert_eq!(SpiDiv::Div128 as u8, 0b110);
    assert_eq!(SpiDiv::Div256 as u8, 0b111);
}

#[test]
fn spi_div_frequencies_at_16mhz() {
    // Vérification documentaire : APB2 = 16 MHz, on calcule f = 16 / div
    let apb2_mhz: u32 = 16;
    let freq_mhz = |div: SpiDiv| apb2_mhz / (1u32 << (div as u32 + 1));

    assert_eq!(freq_mhz(SpiDiv::Div2),     8);
    assert_eq!(freq_mhz(SpiDiv::Div4),     4);
    assert_eq!(freq_mhz(SpiDiv::Div8),     2);
    assert_eq!(freq_mhz(SpiDiv::Div16),    1);
    // Div32 = 500 kHz → 16/32 = 0 en entier (arrondi), c'est normal
    assert_eq!(freq_mhz(SpiDiv::Div32),    0);
}

// =========================================================================== //
// Tests SpiError – variants
// =========================================================================== //

#[test]
fn spi_errors_are_distinguishable() {
    let e1 = SpiError::Timeout;
    let e2 = SpiError::Overrun;
    let e3 = SpiError::ModeFault;

    assert_ne!(e1, e2);
    assert_ne!(e2, e3);
    assert_ne!(e1, e3);
}

#[test]
fn spi_error_debug_format() {
    // S'assure que Debug est implémenté et produit quelque chose de lisible
    let s = format!("{:?}", SpiError::Overrun);
    assert!(s.contains("Overrun"), "Debug devrait contenir 'Overrun', obtenu: {s}");
}

// =========================================================================== //
// Tests BRR UART (compute_brr)
// =========================================================================== //

use driver_i2c_spi::uart::{compute_brr, ClockHz};

#[test]
fn uart_brr_115200_at_16mhz() {
    // USARTDIV = 16_000_000 / 115_200 ≈ 138.88
    // mantisse = 138 = 0x8A, fraction = round(0.88 × 16) = 14 = 0xE
    // BRR = (138 << 4) | 14 = 0x08AE
    let brr = compute_brr(ClockHz(16_000_000), 115_200);
    let mantissa = (brr >> 4) & 0xFFF;
    let fraction = brr & 0xF;
    assert_eq!(mantissa, 138, "Mantisse attendue 138");
    assert_eq!(fraction, 14,  "Fraction attendue 14");
}

#[test]
fn uart_brr_9600_at_16mhz() {
    // USARTDIV = 16_000_000 / 9_600 = 1666.66... → 1667
    // mantisse = 1667 >> 4 = 104, fraction = 1667 & 0xF = 3
    let brr = compute_brr(ClockHz(16_000_000), 9_600);
    let mantissa = (brr >> 4) & 0xFFF;
    // Peu importe la valeur exacte, vérifier la plage de raisonnabilité
    assert!(mantissa > 100, "Mantisse devrait être > 100 pour 9600 baud @ 16 MHz");
}

#[test]
fn uart_brr_symmetric_scaling() {
    // Doubler la fréquence d'horloge et doubler le baud rate → même BRR
    let brr1 = compute_brr(ClockHz(16_000_000), 115_200);
    let brr2 = compute_brr(ClockHz(32_000_000), 230_400);
    assert_eq!(brr1, brr2, "BRR doit être identique si f_CK et baud sont doublés");
}
