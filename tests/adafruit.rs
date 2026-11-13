#![cfg(feature = "adafruit_rgb_13x9")]

mod support;

use is31fl3741::Error;
use support::MockI2c;

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
