mod support;

use embedded_hal::i2c::ErrorKind;
use is31fl3741::{Error, IS31FL3741};
use support::MockI2c;

fn driver() -> IS31FL3741<MockI2c> {
    IS31FL3741 {
        i2c: MockI2c::default(),
        address: 0x30,
        width: 9,
        height: 34,
        calc_pixel: |x, y| (x, u8::from(y >= 30)),
    }
}

#[test]
fn fill_writes_both_complete_pwm_pages() {
    let mut device = driver();
    device.fill(123).unwrap();
    let writes = &device.i2c.writes;
    assert_eq!(writes.len(), 6);
    assert_eq!(writes[0], [0xfe, 0xc5]);
    assert_eq!(writes[1], [0xfd, 0]);
    assert_eq!(writes[2].len(), 181);
    assert_eq!(writes[2][0], 0);
    assert!(writes[2][1..].iter().all(|&value| value == 123));
    assert_eq!(writes[3], [0xfe, 0xc5]);
    assert_eq!(writes[4], [0xfd, 1]);
    assert_eq!(writes[5].len(), 172);
    assert_eq!(writes[5][0], 0);
    assert!(writes[5][1..].iter().all(|&value| value == 123));
}

#[test]
fn pixel_selects_the_mapped_page() {
    let mut device = driver();
    device.pixel(8, 33, 42).unwrap();
    assert_eq!(
        device.i2c.writes,
        [vec![0xfe, 0xc5], vec![0xfd, 1], vec![8, 42]]
    );
}

#[test]
fn pixel_propagates_bus_errors() {
    let mut device = driver();
    device.i2c.fail = true;
    assert!(matches!(
        device.pixel(0, 0, 42),
        Err(Error::I2cError(ErrorKind::Other))
    ));
    assert!(device.i2c.writes.is_empty());
}

#[test]
fn pixel_rejects_dimension_boundaries_without_bus_access() {
    let mut device = driver();
    for (x, y) in [(9, 0), (0, 34), (255, 255)] {
        assert!(matches!(
            device.pixel(x, y, 42),
            Err(Error::InvalidLocation(_))
        ));
    }
    assert!(device.i2c.writes.is_empty());
}

#[test]
fn fill_matrix_sends_every_channel_in_order() {
    let mut device = driver();
    let values: Vec<u8> = (0..351).map(|n| (n % 251) as u8).collect();
    device.fill_matrix(&values).unwrap();
    assert_eq!(device.i2c.writes[1], [0xfd, 0]);
    assert_eq!(device.i2c.writes[2][0], 0);
    assert_eq!(device.i2c.writes[2][1..], values[..180]);
    assert_eq!(device.i2c.writes[4], [0xfd, 1]);
    assert_eq!(device.i2c.writes[5][0], 0);
    assert_eq!(device.i2c.writes[5][1..], values[180..]);
}

#[test]
fn short_frame_panics_before_bus_access() {
    for length in [0, 179, 180, 350] {
        let mut device = driver();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            device.fill_matrix(&vec![0; length]).unwrap();
        }));
        assert!(result.is_err());
        assert!(device.i2c.writes.is_empty());
    }
}

#[test]
fn shutdown_preserves_all_other_configuration_bits() {
    for config in 0..=255 {
        for shutdown in [true, false] {
            let mut device = driver();
            device.i2c.read_value = config;
            device.shutdown(shutdown).unwrap();
            assert_eq!(
                device.i2c.writes,
                [
                    vec![0xfe, 0xc5],
                    vec![0xfd, 4],
                    vec![0],
                    vec![0xfe, 0xc5],
                    vec![0xfd, 4],
                    vec![0, (config & !1) | u8::from(!shutdown)],
                ]
            );
        }
    }
}

#[test]
fn shutdown_stops_when_configuration_read_fails() {
    let mut device = driver();
    device.i2c.fail_at = Some(3); // Fail the read, after page selection and register address.
    assert_eq!(device.shutdown(true), Err(ErrorKind::Other));
    assert_eq!(
        device.i2c.writes,
        [vec![0xfe, 0xc5], vec![0xfd, 4], vec![0]]
    );
}

#[test]
fn frame_write_stops_at_each_failed_transaction() {
    for failed in 0..6 {
        let mut device = driver();
        device.i2c.fail_at = Some(failed);
        assert_eq!(device.fill_matrix(&[42; 351]), Err(ErrorKind::Other));
        assert_eq!(device.i2c.transactions, failed + 1);
        assert_eq!(device.i2c.writes.len(), failed);
    }
}

#[test]
fn extra_frame_values_are_ignored() {
    let mut device = driver();
    let mut values = [42; 352];
    values[351] = 99;
    device.fill_matrix(&values).unwrap();
    assert_eq!(device.i2c.writes[5].len(), 172);
    assert!(device.i2c.writes[5][1..].iter().all(|&v| v == 42));
}

#[test]
fn address_change_targets_subsequent_transactions() {
    let mut device = driver();
    device.set_address(0x33);
    device.i2c.expected_address = Some(0x33);
    device.pixel(0, 0, 42).unwrap();
    assert_eq!(device.i2c.transactions, 3);
}

#[test]
fn global_current_regression() {
    let mut device = driver();
    device.set_global_current(128).unwrap();
    assert_eq!(device.i2c.writes.last().unwrap(), &[1, 128]);
}

#[test]
fn deghost_resistors_regression() {
    let mut device = driver();
    device
        .set_deghost_resistors(
            is31fl3741::Resistor::KiloOhms32,
            is31fl3741::Resistor::Ohms500,
        )
        .unwrap();
    assert_eq!(device.i2c.writes.last().unwrap(), &[2, 0x71]);
}

#[test]
fn interrupt_config_regression() {
    let mut device = driver();
    device.set_interrupt_config(true, true, true).unwrap();
    assert_eq!(device.i2c.writes, [vec![0xf0, 0x13]]);
}
