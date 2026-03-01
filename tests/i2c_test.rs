//! # Tests du driver I2C et des capteurs via MockI2c
//!
//! Ces tests s'exécutent sur la machine **hôte** (x86_64) et n'ont pas
//! besoin de carte Nucleo-F411RE. Ils utilisent [`driver_i2c_spi::hal_mock::MockI2c`]
//! pour simuler les réponses du bus hardware.
//!
//! ## Lancer les tests
//! ```sh
//! cargo test --features mock
//! ```

use driver_i2c_spi::{
    bus::{I2cAddr, I2cBus, I2cError, RegAddr},
    hal_mock::MockI2c,
    sensors::{bme280::Bme280, mpu6050::{AccelRange, GyroRange, Mpu6050}},
};

// =========================================================================== //
// Tests MockI2c
// =========================================================================== //

#[test]
fn mock_write_logs_bytes() {
    let mut m = MockI2c::new();
    m.write(I2cAddr(0x68), &[0x6B, 0x01]).unwrap();
    m.assert_wrote(0, &[0x6B, 0x01]);
}

#[test]
fn mock_read_returns_preset_data() {
    let mut m = MockI2c::new();
    m.expect_read(&[0xDE, 0xAD]);
    let mut buf = [0u8; 2];
    m.read(I2cAddr(0x42), &mut buf).unwrap();
    assert_eq!(buf, [0xDE, 0xAD]);
}

#[test]
fn mock_read_partial_response_zero_pads() {
    // Si la réponse pré-programmée est plus courte que le buf, le reste = 0
    let mut m = MockI2c::new();
    m.expect_read(&[0xAB]);
    let mut buf = [0xFF; 4];
    m.read(I2cAddr(0x10), &mut buf).unwrap();
    assert_eq!(buf, [0xAB, 0x00, 0x00, 0x00]);
}

#[test]
fn mock_write_read_logs_register_and_returns_data() {
    let mut m = MockI2c::new();
    m.expect_read(&[0x60]); // réponse simulée
    m.write_read(I2cAddr(0x76), RegAddr(0xD0), &mut [0u8; 1]).unwrap();
    // write_read doit logger l'écriture du registre
    m.assert_wrote(0, &[0xD0]);
}

#[test]
fn mock_multiple_reads_consumed_in_order() {
    let mut m = MockI2c::new();
    m.expect_read(&[0x01]);
    m.expect_read(&[0x02]);
    m.expect_read(&[0x03]);

    let mut buf = [0u8; 1];
    m.read(I2cAddr(0), &mut buf).unwrap();
    assert_eq!(buf[0], 0x01);
    m.read(I2cAddr(0), &mut buf).unwrap();
    assert_eq!(buf[0], 0x02);
    m.read(I2cAddr(0), &mut buf).unwrap();
    assert_eq!(buf[0], 0x03);
}

#[test]
fn mock_inject_nack_error() {
    let mut m = MockI2c::new();
    m.inject_error = Some(I2cError::Nack);
    let res = m.write(I2cAddr(0x68), &[0x00]);
    assert_eq!(res, Err(I2cError::Nack));
}

#[test]
fn mock_inject_timeout_error() {
    let mut m = MockI2c::new();
    m.inject_error = Some(I2cError::Timeout);
    let mut buf = [0u8; 1];
    let res = m.read(I2cAddr(0x00), &mut buf);
    assert_eq!(res, Err(I2cError::Timeout));
}

#[test]
fn mock_reset_clears_state() {
    let mut m = MockI2c::new();
    m.write(I2cAddr(0x10), &[0x01]).unwrap();
    m.expect_read(&[0xFF]);
    m.reset();
    // Après reset : plus d'écritures loguées, plus de réponses en queue
    assert_eq!(m.writes().len(), 0);
}

// =========================================================================== //
// Tests BME280
// =========================================================================== //

/// Construit le mock avec les réponses attendues lors de `Bme280::init`.
///
/// Séquence d'init BME280 :
/// 1. write_read(0xD0)  → chip_id = 0x60
/// 2. write(reset)      → [0xE0, 0xB6]
/// 3. write_read(0xF3)  → status = 0x00 (NVM ready)
/// 4. write_read(0x88)  → 24 octets de calibration T+P
/// 5. write_read(0xA1)  → 1 octet dig_H1
/// 6. write_read(0xE1)  → 7 octets calib humidité
/// 7. write(ctrl_hum)   → [0xF2, 0x01]
/// 8. write(config)     → [0xF5, 0x00]
/// 9. write(ctrl_meas)  → [0xF4, 0x57]
fn mock_with_bme280_init_responses() -> MockI2c {
    let mut m = MockI2c::new();

    // 1. chip_id
    m.expect_read(&[0x60]);

    // 3. Status NVM = 0x00 (prêt)
    m.expect_read(&[0x00]);

    // 4. Calibration T+P (24 octets, valeurs quelconques non nulles)
    //    dig_T1=27504, dig_T2=26435, dig_T3=-1000 (exemples datasheet)
    let mut calib_tp = [0u8; 24];
    // dig_T1 = 27504 = 0x6B90 → LE = [0x90, 0x6B]
    calib_tp[0] = 0x90; calib_tp[1] = 0x6B;
    // dig_T2 = 26435 = 0x6743 → LE = [0x43, 0x67]
    calib_tp[2] = 0x43; calib_tp[3] = 0x67;
    // dig_T3 = -1000 = 0xFC18 → LE = [0x18, 0xFC]
    calib_tp[4] = 0x18; calib_tp[5] = 0xFC;
    // dig_P1 = 36477 (0x8E7D) → [0x7D, 0x8E]
    calib_tp[6] = 0x7D; calib_tp[7] = 0x8E;
    // Autres Pn = 0 pour simplifier le test
    m.expect_read(&calib_tp);

    // 5. dig_H1 = 75
    m.expect_read(&[75]);

    // 6. Calib humidité (7 octets) – valeurs quelconques
    m.expect_read(&[0x70, 0x01, 0x00, 0x12, 0xD4, 0x2C, 0x1E]);

    m
}

#[test]
fn bme280_init_checks_chip_id() {
    let mut m = MockI2c::new();
    m.expect_read(&[0x42]); // mauvais chip_id

    let result = Bme280::init(&mut m, I2cAddr(0x76));
    assert!(
        matches!(result, Err(driver_i2c_spi::sensors::bme280::Bme280Error::InvalidChipId(0x42))),
        "Devrait retourner InvalidChipId(0x42), obtenu : {result:?}"
    );
}

#[test]
fn bme280_init_succeeds_with_valid_chip_id() {
    let mut m = mock_with_bme280_init_responses();
    let result = Bme280::init(&mut m, I2cAddr(0x76));
    assert!(result.is_ok(), "Init BME280 devrait réussir : {result:?}");
}

#[test]
fn bme280_init_sends_soft_reset() {
    let mut m = mock_with_bme280_init_responses();
    Bme280::init(&mut m, I2cAddr(0x76)).unwrap();
    // write #0 = reset (après les write_read du chip_id, il y a des write)
    // On cherche [0xE0, 0xB6] dans les écritures
    let found = m.writes().iter().any(|w| w.bytes() == [0xE0, 0xB6]);
    assert!(found, "Soft-reset 0xB6 attendu parmi les écritures : {:?}",
        m.writes().iter().map(|w| w.bytes().to_vec()).collect::<std::vec::Vec<_>>()
    );
}

#[test]
fn bme280_init_configures_oversampling() {
    let mut m = mock_with_bme280_init_responses();
    Bme280::init(&mut m, I2cAddr(0x76)).unwrap();

    // ctrl_meas attendu : 0x57 = 0b0101_0111 (T×2, P×16, Normal)
    let found_ctrl_meas = m.writes().iter().any(|w| w.bytes() == [0xF4, 0x57]);
    assert!(found_ctrl_meas, "ctrl_meas 0x57 attendu, écritures : {:?}",
        m.writes().iter().map(|w| w.bytes().to_vec()).collect::<std::vec::Vec<_>>()
    );

    // ctrl_hum attendu : osrs_h=1
    let found_ctrl_hum = m.writes().iter().any(|w| w.bytes() == [0xF2, 0x01]);
    assert!(found_ctrl_hum, "ctrl_hum 0x01 attendu");
}

#[test]
fn bme280_read_returns_compensated_values() {
    let mut m = mock_with_bme280_init_responses();
    let mut bme = Bme280::init(&mut m, I2cAddr(0x76)).unwrap();

    // Valeurs brutes réalistes pour T≈25°C, P≈1013 hPa, H≈50%
    // La reconstruction : adc_T = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
    //
    // adc_T = 519888 = 0x7EED0
    //   MSB  = 0x7EED0 >> 12     = 0x7E
    //   LSB  = (0x7EED0 >> 4) & 0xFF = 0xED
    //   XLSB = (0x7EED0 & 0xF) << 4  = 0x00
    let raw_t_msb = 0x7Eu8; let raw_t_lsb = 0xEDu8; let raw_t_xlsb = 0x00u8;

    // adc_P = 415148 = 0x655AC
    //   MSB  = 0x65, LSB = 0x5A, XLSB = 0xC0
    let raw_p_msb = 0x65u8; let raw_p_lsb = 0x5Au8; let raw_p_xlsb = 0xC0u8;

    // adc_H = 29317 = 0x7285 (16 bits)
    let raw_h_msb = 0x72u8; let raw_h_lsb = 0x85u8;
    m.expect_read(&[raw_p_msb, raw_p_lsb, raw_p_xlsb,
                    raw_t_msb, raw_t_lsb, raw_t_xlsb,
                    raw_h_msb, raw_h_lsb]);

    let result = bme.read(&mut m);
    assert!(result.is_ok(), "Lecture BME280 devrait réussir : {result:?}");
    let meas = result.unwrap();

    // La température doit être dans une plage raisonnable [-40..85 °C]
    let temp_deg_x100 = meas.temperature_cdeg;
    assert!(
        (-4000..=8500).contains(&temp_deg_x100),
        "Température hors plage : {} (×100 °C)", temp_deg_x100
    );
}

// =========================================================================== //
// Tests MPU6050
// =========================================================================== //

/// Construit le mock avec les réponses d'init du MPU6050.
///
/// Séquence d'init :
/// 1. write_read(0x75) → WHO_AM_I = 0x68
/// 2. write(PWR_MGMT_1) → [0x6B, 0x01]
/// 3. write(CONFIG)     → [0x1A, 0x03]
/// 4. write(ACCEL_CONFIG) → [0x1C, AFS_SEL]
/// 5. write(GYRO_CONFIG)  → [0x1B, FS_SEL]
fn mock_with_mpu6050_init_responses() -> MockI2c {
    let mut m = MockI2c::new();
    m.expect_read(&[0x68]); // WHO_AM_I
    m
}

#[test]
fn mpu6050_init_checks_who_am_i() {
    let mut m = MockI2c::new();
    m.expect_read(&[0xFF]); // mauvais who_am_i

    let result = Mpu6050::init(
        &mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250
    );
    assert!(
        matches!(result, Err(driver_i2c_spi::sensors::mpu6050::Mpu6050Error::InvalidWhoAmI(0xFF))),
        "Devrait retourner InvalidWhoAmI(0xFF)"
    );
}

#[test]
fn mpu6050_init_succeeds() {
    let mut m = mock_with_mpu6050_init_responses();
    let result = Mpu6050::init(
        &mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250
    );
    assert!(result.is_ok(), "Init MPU6050 devrait réussir : {result:?}");
}

#[test]
fn mpu6050_init_wakes_up_chip() {
    let mut m = mock_with_mpu6050_init_responses();
    Mpu6050::init(&mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250).unwrap();

    // PWR_MGMT_1 : doit écrire CLKSEL=1 (SLEEP=0)
    let found = m.writes().iter().any(|w| w.bytes() == [0x6B, 0x01]);
    assert!(found, "PWR_MGMT_1 = 0x01 attendu, écritures : {:?}",
        m.writes().iter().map(|w| w.bytes().to_vec()).collect::<std::vec::Vec<_>>()
    );
}

#[test]
fn mpu6050_init_sets_accel_and_gyro_range() {
    let mut m = mock_with_mpu6050_init_responses();
    Mpu6050::init(&mut m, I2cAddr(0x68), AccelRange::G4, GyroRange::Dps500).unwrap();

    // ACCEL_CONFIG : AFS_SEL = 01 → bits [4:3] → 0b00001000 = 0x08
    let found_accel = m.writes().iter().any(|w| w.bytes() == [0x1C, 0x08]);
    assert!(found_accel, "ACCEL_CONFIG G4 attendu");

    // GYRO_CONFIG : FS_SEL = 01 → bits [4:3] → 0b00001000 = 0x08
    let found_gyro = m.writes().iter().any(|w| w.bytes() == [0x1B, 0x08]);
    assert!(found_gyro, "GYRO_CONFIG 500dps attendu");
}

#[test]
fn mpu6050_read_converts_accel_correctly() {
    let mut m = mock_with_mpu6050_init_responses();
    let mut mpu = Mpu6050::init(&mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250)
        .unwrap();

    // Simuler ax=+1g : raw = 16384 = 0x4000 (big-endian)
    // ay=0, az=0, temp=0, gx=0, gy=0, gz=0
    let raw = [
        0x40, 0x00, // ax = 16384  → 1000 mg
        0x00, 0x00, // ay = 0
        0x00, 0x00, // az = 0
        0x00, 0x00, // temp = 0    → (0*100/340) + 3653 = 3653 cdeg
        0x00, 0x00, // gx = 0
        0x00, 0x00, // gy = 0
        0x00, 0x00, // gz = 0
    ];
    m.expect_read(&raw);

    let meas = mpu.read(&mut m).unwrap();
    assert_eq!(meas.accel_x_mg, 1000, "ax=1g devrait donner 1000 mg");
    assert_eq!(meas.accel_y_mg, 0);
    assert_eq!(meas.accel_z_mg, 0);
    assert_eq!(meas.temp_cdeg, 3653, "Température au repos devrait être ~36.53°C");
}

#[test]
fn mpu6050_read_converts_negative_accel() {
    let mut m = mock_with_mpu6050_init_responses();
    let mut mpu = Mpu6050::init(&mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250)
        .unwrap();

    // Simuler az=-1g : raw = -16384 = 0xC000 big-endian
    let raw = [
        0x00, 0x00,  // ax = 0
        0x00, 0x00,  // ay = 0
        0xC0, 0x00,  // az = -16384 → -1000 mg
        0x00, 0x00,  // temp
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];
    m.expect_read(&raw);

    let meas = mpu.read(&mut m).unwrap();
    assert_eq!(meas.accel_z_mg, -1000, "az=-1g devrait donner -1000 mg");
}

#[test]
fn mpu6050_read_converts_gyro_correctly() {
    let mut m = mock_with_mpu6050_init_responses();
    let mut mpu = Mpu6050::init(&mut m, I2cAddr(0x68), AccelRange::G2, GyroRange::Dps250)
        .unwrap();

    // Simuler gx = 131 LSB @ ±250°/s → 1.00 °/s → 100 centidps
    // 131 = 0x0083 big-endian
    let raw = [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // accel
        0x00, 0x00,                           // temp
        0x00, 0x83,                           // gx = 131 → 100 centidps
        0x00, 0x00, 0x00, 0x00,               // gy, gz
    ];
    m.expect_read(&raw);

    let meas = mpu.read(&mut m).unwrap();
    assert_eq!(meas.gyro_x_cdps, 100, "gx=131 LSB @ ±250°/s devrait donner 100 centidps");
}
