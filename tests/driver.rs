use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use is31fl3741::{Error, IS31FL3741};

#[derive(Default)]
struct MockI2c {
    writes: Vec<Vec<u8>>,
    read_value: u8,
    fail: bool,
}

impl ErrorType for MockI2c {
    type Error = ErrorKind;
}

impl I2c for MockI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(address, 0x30);
        if self.fail {
            return Err(ErrorKind::Other);
        }
        for operation in operations {
            match operation {
                Operation::Write(bytes) => self.writes.push(bytes.to_vec()),
                Operation::Read(bytes) => bytes.fill(self.read_value),
            }
        }
        Ok(())
    }
}

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

#[cfg(feature = "adafruit_rgb_13x9")]
#[test]
fn rgb_rejects_invalid_coordinates_before_flattening() {
    let mut device = is31fl3741::devices::AdafruitRGB13x9::configure(MockI2c::default());
    for (x, y) in [(13, 0), (0, 9), (255, 255)] {
        assert!(matches!(
            device.pixel_rgb(x, y, 1, 2, 3),
            Err(Error::InvalidLocation(_))
        ));
    }
    assert!(device.device.i2c.writes.is_empty());
    device.pixel_rgb(12, 8, 1, 2, 3).unwrap();
}

#[cfg(all(feature = "adafruit_rgb_13x9", feature = "embedded_graphics"))]
#[test]
fn graphics_discards_out_of_bounds_pixels() {
    use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*};
    let mut device = is31fl3741::devices::AdafruitRGB13x9::configure(MockI2c::default());
    device
        .draw_iter(
            [(-1, 0), (0, -1), (13, 0), (0, 9), (13, 9)]
                .map(|(x, y)| Pixel(Point::new(x, y), Rgb888::new(1, 2, 3))),
        )
        .unwrap();
    assert!(device.device.i2c.writes.is_empty());
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
