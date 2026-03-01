# Projet 01 — Driver I2C/SPI from scratch (sans HAL)

## Contexte

Tu es un développeur firmware Rust qui implémente des drivers de communication série directement sur les registres matériels,
sans utiliser de couche HAL (Hardware Abstraction Layer) existante.  
L'objectif est de prouver la maîtrise complète du bus I2C et SPI en bare-metal, du bit de contrôle jusqu'à la lecture d'un capteur réel.

**Cible matérielle** : STM32F411 (Nucleo-F411RE) — ARM Cortex-M4  
**Toolchain** : `thumbv7em-none-eabihf`  
**Environnement** : `#![no_std]` `#![no_main]`, pas de `std`, pas de `alloc`

---

## Objectifs du projet

1. Implémenter un driver **I2C1** en mode maître (standard 100 kHz, fast 400 kHz) depuis les registres SVD
2. Implémenter un driver **SPI1** en mode maître (CPOL=0 CPHA=0, full-duplex)
3. Écrire un driver haut niveau pour le capteur **BME280** (température, pression, humidité) via I2C
4. Écrire un driver haut niveau pour le capteur **MPU6050** (accéléromètre + gyroscope) via I2C
5. Publier les sorties via **UART2** (115200 baud) lisibles dans `probe-rs` ou minicom
6. Couvrir chaque module avec des **tests unitaires** via des stubs/mocks du bus

---

## Spécifications techniques

### Structure du projet

```
driver-i2c-spi/
├── Cargo.toml
├── Embed.toml
├── memory.x
├── src/
│   ├── main.rs
│   ├── i2c.rs          ← driver I2C bas niveau
│   ├── spi.rs          ← driver SPI bas niveau
│   ├── uart.rs         ← sortie debug
│   ├── sensors/
│   │   ├── bme280.rs
│   │   └── mpu6050.rs
│   └── hal_mock.rs     ← stubs pour les tests
└── tests/
    ├── i2c_test.rs
    └── spi_test.rs
```

### Contraintes Rust

- Utiliser les crates `stm32f4` (PAC généré par svd2rust) uniquement — pas de `stm32f4xx-hal`
- Typage fort : utiliser des newtypes pour les adresses I2C (`I2cAddr(u8)`), les registres (`RegAddr(u8)`)
- Gestion d'erreurs : `enum I2cError { Nack, Timeout, ArbitrationLost, BusError }` — pas de `unwrap()` dans le code de production
- Timeout par compteur de cycles, pas de `std::time`
- Mode `embedded-hal` trait compatible en option (bonus)

### API cible du driver I2C

```rust
pub struct I2c1<STATE> { /* registres PAC */ }

impl I2c1<Uninitialized> {
    pub fn init(rcc: &RCC, gpiob: &GPIOB, freq: I2cFreq) -> I2c1<Ready>;
}

impl I2c1<Ready> {
    pub fn write(&mut self, addr: I2cAddr, data: &[u8]) -> Result<(), I2cError>;
    pub fn read(&mut self, addr: I2cAddr, buf: &mut [u8]) -> Result<(), I2cError>;
    pub fn write_read(&mut self, addr: I2cAddr, reg: RegAddr, buf: &mut [u8]) -> Result<(), I2cError>;
}
```

---

## Livrables attendus

- [ ] `i2c.rs` complet avec les 3 méthodes et gestion d'erreurs
- [ ] `spi.rs` complet avec `transfer(&mut [u8])` full-duplex
- [ ] `bme280.rs` : lecture compensée température/pression/humidité
- [ ] `mpu6050.rs` : lecture brute accél + gyro, conversion en g et °/s
- [ ] `uart.rs` : implémentation `core::fmt::Write` pour `defmt`
- [ ] Tests unitaires passants avec mocks
- [ ] `README.md` du projet avec schéma de câblage (texte ASCII)

---

## Critères de qualité

- `cargo clippy -- -D warnings` : zéro warning
- Pas de `unsafe` non justifié (chaque bloc `unsafe` commenté)
- Toutes les erreurs propagées, pas de `panic!` en dehors du `main`
- Code lisible par quelqu'un qui ne connaît pas le projet
- `defmt::info!` aux points clés de l'initialisation

---

## Ressources à fournir à Claude

- Reference Manual STM32F411 (RM0383) sections I2C et SPI si disponible
- Datasheet BME280 / MPU6050 (registres et séquences d'init)
- Fichier SVD du STM32F411 si la crate PAC n'est pas suffisante
