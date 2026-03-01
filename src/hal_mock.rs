//! # hal_mock – Stubs de bus pour les tests unitaires
//!
//! Ce module fournit [`MockI2c`], une implémentation de [`crate::bus::I2cBus`]
//! qui **n'accède à aucun hardware**. Il est utilisé dans les tests
//! d'intégration (`tests/`) qui s'exécutent sur la machine hôte.
//!
//! ## Principe
//! Avant d'appeler le code sous test, on charge :
//! - Une file de **réponses** (`expect_read`) que le mock renverra lors
//!   des appels à `read()` / `write_read()`.
//! - Après l'appel, on vérifie les **octets écrits** avec `assert_wrote()`.
//!
//! ## Exemple
//! ```rust
//! use driver_i2c_spi::hal_mock::MockI2c;
//! use driver_i2c_spi::bus::{I2cAddr, RegAddr};
//!
//! let mut mock = MockI2c::new();
//! // WHO_AM_I renvoie 0x68
//! mock.expect_read(&[0x68]);
//! // Réponse NVM ready (im_update = 0) pour le BME280
//! mock.expect_read(&[0x00]);
//!
//! // … appel à du code qui utilise &mut mock en tant qu'impl I2cBus …
//! ```
//!
//! ## Contraintes no_std
//! Pas de `Vec` ni `Box` : les queues sont des tableaux fixes.
//! Taille maximale : `MAX_TRANSACTIONS` transactions, `MAX_BUF` octets par
//! transaction.

use crate::bus::{I2cAddr, I2cBus, I2cError, RegAddr};

// --------------------------------------------------------------------------- //
// Constantes de capacité
// --------------------------------------------------------------------------- //

/// Nombre maximum de transactions pré-programmées dans la file.
pub const MAX_TRANSACTIONS: usize = 32;

/// Taille maximum d'un tampon de réponse ou d'écriture.
pub const MAX_BUF: usize = 64;

// --------------------------------------------------------------------------- //
// Transaction enregistrée
// --------------------------------------------------------------------------- //

/// Une transaction I2C capturée par le mock.
#[derive(Clone, Copy, Debug)]
pub struct CapturedWrite {
    /// Adresse I2C de l'esclave ciblé
    pub addr: I2cAddr,
    /// Octets écrits
    pub data: [u8; MAX_BUF],
    /// Nombre d'octets valides dans `data`
    pub len: usize,
}

impl CapturedWrite {
    /// Retourne les octets effectivement écrits.
    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

// --------------------------------------------------------------------------- //
// MockI2c
// --------------------------------------------------------------------------- //

/// Stub du bus I2C sans hardware.
///
/// ### Utilisation
/// 1. Appeler [`MockI2c::expect_read`] pour chaque `read` / `write_read`
///    que le code sous test va effectuer, **dans l'ordre**.
/// 2. Appeler le code sous test en passant `&mut mock`.
/// 3. Utiliser [`MockI2c::writes`] pour inspecter ce qui a été écrit.
pub struct MockI2c {
    // ------------------------------------------------------------------- //
    // Réponses pré-programmées (file FIFO)
    // ------------------------------------------------------------------- //
    read_queue:  [[u8; MAX_BUF]; MAX_TRANSACTIONS],
    read_lens:   [usize; MAX_TRANSACTIONS],
    read_head:   usize,  // prochain à consommer
    read_tail:   usize,  // prochain emplacement libre

    // ------------------------------------------------------------------- //
    // Journal des écritures
    // ------------------------------------------------------------------- //
    write_log:   [CapturedWrite; MAX_TRANSACTIONS],
    write_count: usize,

    /// Si `Some(err)`, le prochain appel I2C retournera cette erreur.
    pub inject_error: Option<I2cError>,
}

impl MockI2c {
    /// Crée un mock vide.
    pub fn new() -> Self {
        MockI2c {
            read_queue:   [[0u8; MAX_BUF]; MAX_TRANSACTIONS],
            read_lens:    [0; MAX_TRANSACTIONS],
            read_head:    0,
            read_tail:    0,
            write_log: [CapturedWrite {
                addr: I2cAddr(0),
                data: [0u8; MAX_BUF],
                len: 0,
            }; MAX_TRANSACTIONS],
            write_count:  0,
            inject_error: None,
        }
    }

    /// Enfile une réponse qui sera retournée lors du prochain `read` ou
    /// `write_read`.
    ///
    /// # Panics
    /// Panique si la file est pleine ou si `data.len() > MAX_BUF`.
    pub fn expect_read(&mut self, data: &[u8]) {
        assert!(data.len() <= MAX_BUF, "MockI2c: tampon trop grand");
        let tail = self.read_tail;
        assert!(
            (tail + 1) % MAX_TRANSACTIONS != self.read_head,
            "MockI2c: file de réponses pleine"
        );
        let slot = &mut self.read_queue[tail];
        slot[..data.len()].copy_from_slice(data);
        self.read_lens[tail] = data.len();
        self.read_tail = (tail + 1) % MAX_TRANSACTIONS;
    }

    /// Retourne la slice de toutes les transactions capturées (écriture).
    pub fn writes(&self) -> &[CapturedWrite] {
        &self.write_log[..self.write_count]
    }

    /// Retourne la n-ième transaction écrite (base 0).
    pub fn write_at(&self, n: usize) -> &CapturedWrite {
        assert!(n < self.write_count, "MockI2c: index d'écriture hors limite");
        &self.write_log[n]
    }

    /// Vérifie que la n-ième écriture correspond aux octets attendus.
    ///
    /// # Panics
    /// Panique si les données ne correspondent pas.
    pub fn assert_wrote(&self, n: usize, expected: &[u8]) {
        let got = self.write_at(n).bytes();
        assert_eq!(
            got, expected,
            "MockI2c: écriture #{n} inattendue\n  attendu : {expected:?}\n  obtenu  : {got:?}"
        );
    }

    /// Remise à zéro du mock (file + journal).
    pub fn reset(&mut self) {
        self.read_head  = 0;
        self.read_tail  = 0;
        self.write_count = 0;
        self.inject_error = None;
    }

    /// Dépile une réponse pré-programmée et la copie dans `buf`.
    fn pop_read(&mut self, buf: &mut [u8]) -> Result<(), I2cError> {
        if let Some(err) = self.inject_error.take() {
            return Err(err);
        }
        if self.read_head == self.read_tail {
            // Aucune réponse pré-programmée → panic en test pour signal clair
            panic!("MockI2c: aucune réponse pré-programmée pour cette lecture");
        }
        let head = self.read_head;
        let src  = &self.read_queue[head];
        let len  = self.read_lens[head].min(buf.len());
        buf[..len].copy_from_slice(&src[..len]);
        // Si le buf est plus grand que la réponse, remplir le reste de 0
        for b in &mut buf[len..] { *b = 0; }
        self.read_head = (head + 1) % MAX_TRANSACTIONS;
        Ok(())
    }

    /// Enregistre une écriture dans le journal.
    fn push_write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError> {
        if let Some(err) = self.inject_error.take() {
            return Err(err);
        }
        assert!(
            self.write_count < MAX_TRANSACTIONS,
            "MockI2c: journal d'écritures plein"
        );
        let mut buf = [0u8; MAX_BUF];
        let len = data.len().min(MAX_BUF);
        buf[..len].copy_from_slice(&data[..len]);
        self.write_log[self.write_count] = CapturedWrite { addr, data: buf, len };
        self.write_count += 1;
        Ok(())
    }
}

// --------------------------------------------------------------------------- //
// impl I2cBus
// --------------------------------------------------------------------------- //

impl I2cBus for MockI2c {
    fn write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError> {
        self.push_write(addr, data)
    }

    fn read(&mut self, _addr: I2cAddr, buf: &mut [u8]) -> Result<(), I2cError> {
        self.pop_read(buf)
    }

    fn write_read(
        &mut self,
        addr: I2cAddr,
        reg: RegAddr,
        buf: &mut [u8],
    ) -> Result<(), I2cError> {
        // Enregistre l'écriture du registre (comme un write d'1 octet)
        self.push_write(addr, &[reg.0])?;
        // Retourne la réponse pré-programmée
        self.pop_read(buf)
    }
}

// --------------------------------------------------------------------------- //
// ClockHz (type utilitaire partagé)
// --------------------------------------------------------------------------- //

/// Réexport de [`crate::bus::ClockHz`] pour éviter de l'importer séparément
/// dans les tests.
pub use crate::bus::ClockHz;

// --------------------------------------------------------------------------- //
// Tests internes du mock lui-même
// --------------------------------------------------------------------------- //

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_write_is_logged() {
        let mut m = MockI2c::new();
        m.write(I2cAddr(0x68), &[0x6B, 0x00]).unwrap();
        m.assert_wrote(0, &[0x6B, 0x00]);
    }

    #[test]
    fn mock_read_returns_preset() {
        let mut m = MockI2c::new();
        m.expect_read(&[0xAB, 0xCD]);
        let mut buf = [0u8; 2];
        m.read(I2cAddr(0x68), &mut buf).unwrap();
        assert_eq!(buf, [0xAB, 0xCD]);
    }

    #[test]
    fn mock_write_read_logs_reg_then_returns_data() {
        let mut m = MockI2c::new();
        m.expect_read(&[0x60]); // chip_id BME280
        m.write_read(I2cAddr(0x76), RegAddr(0xD0), &mut [0u8; 1]).unwrap();
        m.assert_wrote(0, &[0xD0]);
    }

    #[test]
    fn mock_inject_error() {
        let mut m = MockI2c::new();
        m.inject_error = Some(I2cError::Nack);
        let res = m.write(I2cAddr(0x68), &[0x00]);
        assert_eq!(res, Err(I2cError::Nack));
    }
}
