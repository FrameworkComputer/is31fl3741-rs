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

#[test]
fn interrupt_status_regression() {
    let mut device = driver();
    device.i2c.read_value = 0xfd;
    let status = device.read_interrupt_status().unwrap();
    assert!(status.open);
    assert!(!status.short);
    assert_eq!(device.i2c.writes, [vec![0xf1]]);
}

#[test]
fn logic_threshold_regression() {
    let mut device = driver();
    device.i2c.read_value = 0x11;
    device
        .set_logic_threshold(is31fl3741::LogicThreshold::High2V4Low0V6)
        .unwrap();
    assert_eq!(device.i2c.writes.last().unwrap(), &[0, 0x19]);
}

#[test]
fn all_global_current_values_have_exact_transactions() {
    for value in 0..=255 {
        let mut d = driver();
        d.set_global_current(value).unwrap();
        assert_eq!(
            d.i2c.writes,
            [vec![0xfe, 0xc5], vec![0xfd, 4], vec![1, value]]
        );
    }
}

#[test]
fn all_resistor_combinations_encode_only_documented_bits() {
    use is31fl3741::Resistor::*;
    let values = [
        Disabled, Ohms500, KiloOhms1, KiloOhms2, KiloOhms4, KiloOhms8, KiloOhms16, KiloOhms32,
    ];
    for (down, &d) in values.iter().enumerate() {
        for (up, &u) in values.iter().enumerate() {
            let mut device = driver();
            device.set_deghost_resistors(d, u).unwrap();
            assert_eq!(
                device.i2c.writes,
                [
                    vec![0xfe, 0xc5],
                    vec![0xfd, 4],
                    vec![2, (down as u8) << 4 | up as u8]
                ]
            );
        }
    }
}

#[test]
fn all_interrupt_masks_and_status_bytes() {
    for open in [false, true] {
        for short in [false, true] {
            for clear in [false, true] {
                let mut d = driver();
                d.set_interrupt_config(open, short, clear).unwrap();
                assert_eq!(
                    d.i2c.writes,
                    [vec![
                        0xf0,
                        u8::from(open) | u8::from(short) << 1 | u8::from(clear) << 4
                    ]]
                );
            }
        }
    }
    for byte in 0..=255 {
        let mut d = driver();
        d.i2c.read_value = byte;
        let status = d.read_interrupt_status().unwrap();
        assert_eq!((status.open, status.short), (byte & 1 != 0, byte & 2 != 0));
        assert_eq!(d.i2c.writes, [vec![0xf1]]);
    }
}

#[test]
fn logic_selection_preserves_every_other_bit() {
    use is31fl3741::LogicThreshold::*;
    for config in 0..=255 {
        for (threshold, bit) in [(High1V4Low0V4, 0), (High2V4Low0V6, 8)] {
            let mut d = driver();
            d.i2c.read_value = config;
            d.set_logic_threshold(threshold).unwrap();
            assert_eq!(
                d.i2c.writes,
                [
                    vec![0xfe, 0xc5],
                    vec![0xfd, 4],
                    vec![0],
                    vec![0xfe, 0xc5],
                    vec![0xfd, 4],
                    vec![0, (config & !8) | bit]
                ]
            );
        }
    }
}

#[test]
fn register_apis_stop_at_every_bus_failure() {
    // Successful transaction counts include reads; each failing operation must be the last.
    for (api, count) in [(0, 3), (1, 3), (2, 1), (3, 2), (4, 7)] {
        for failed in 0..count {
            let mut d = driver();
            d.i2c.fail_at = Some(failed);
            let result = match api {
                0 => d.set_global_current(32),
                1 => d.set_deghost_resistors(
                    is31fl3741::Resistor::Disabled,
                    is31fl3741::Resistor::KiloOhms32,
                ),
                2 => d.set_interrupt_config(true, false, true),
                3 => d.read_interrupt_status().map(|_| ()),
                _ => d.set_logic_threshold(is31fl3741::LogicThreshold::High2V4Low0V6),
            };
            assert_eq!(
                result,
                Err(ErrorKind::Other),
                "API {api}, transaction {failed}"
            );
            assert_eq!(d.i2c.transactions, failed + 1);
        }
    }
}

#[test]
fn invalid_mapping_pages_are_rejected_without_bus_access() {
    use std::sync::atomic::{AtomicU8, Ordering};
    static PAGE: AtomicU8 = AtomicU8::new(2);
    let mut d = driver();
    d.calc_pixel = |_, _| (0, PAGE.load(Ordering::Relaxed));
    for page in 2..=255 {
        PAGE.store(page, Ordering::Relaxed);
        assert!(
            matches!(d.pixel(0, 0, 123), Err(Error::InvalidFrame(value)) if value == page),
            "accepted invalid mapping page {page}"
        );
    }
    assert_eq!(d.i2c.transactions, 0);
}
