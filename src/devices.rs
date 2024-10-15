// #[cfg_attr(docsrs, doc(cfg(feature = "adafruit_rgb_13x9")))]
#[allow(unused_imports)]
use crate::{Error, IS31FL3741};
#[allow(unused_imports)]
use core::convert::TryFrom;
#[allow(unused_imports)]
use embedded_hal::blocking::delay::DelayMs;
#[allow(unused_imports)]
use embedded_hal::blocking::i2c::Read;
#[allow(unused_imports)]
use embedded_hal::blocking::i2c::Write;

#[cfg(feature = "adafruit_rgb_13x9")]
pub struct AdafruitRGB13x9<I2C> {
    pub device: IS31FL3741<I2C>,
}

#[cfg(all(feature = "adafruit_rgb_13x9", feature = "embedded_graphics"))]
use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*, primitives::Rectangle};

#[cfg(all(feature = "adafruit_rgb_13x9", feature = "embedded_graphics"))]
impl<I2C, I2cError> Dimensions for AdafruitRGB13x9<I2C>
where
    I2C: Write<Error = I2cError>,
    I2C: Read<Error = I2cError>,
{
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), Size::new(13, 9))
    }
}

#[cfg(all(feature = "adafruit_rgb_13x9", feature = "embedded_graphics"))]
impl<I2C, I2cError> DrawTarget for AdafruitRGB13x9<I2C>
where
    I2C: Write<Error = I2cError>,
    I2C: Read<Error = I2cError>,
    I2cError:,
{
    type Color = Rgb888;
    type Error = Error<I2cError>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds (negative or greater than
            // (63,63)). `DrawTarget` implementation are required to discard any out of bounds
            // pixels without returning an error or causing a panic.
            if let Ok((x @ 0..=13, y @ 0..=9)) = coord.try_into() {
                // Calculate the index in the framebuffer.
                self.pixel_rgb(x as u8, y as u8, color.r(), color.g(), color.b())?
            }
        }

        Ok(())
    }
}

#[cfg(feature = "adafruit_rgb_13x9")]
impl<I2C, I2cError> AdafruitRGB13x9<I2C>
where
    I2C: Write<Error = I2cError>,
    I2C: Read<Error = I2cError>,
{
    pub fn unwrap(self) -> I2C {
        self.device.i2c
    }

    pub fn set_scaling(&mut self, scale: u8) -> Result<(), I2cError> {
        self.device.set_scaling(scale)
    }

    pub fn configure(i2c: I2C) -> AdafruitRGB13x9<I2C> {
        AdafruitRGB13x9 {
            device: IS31FL3741 {
                i2c,
                address: 0x30,
                width: 13 * 9,
                height: 3,
                calc_pixel: |x: u8, y: u8| -> (u8, u8) {
                    let lookup: [[u16; 3]; 13 * 9] = [
                        [240, 241, 242],
                        [245, 243, 244],
                        [246, 247, 248],
                        [251, 249, 250],
                        [252, 253, 254],
                        [257, 255, 256],
                        [258, 259, 260],
                        [263, 261, 262],
                        [264, 265, 266],
                        [269, 267, 268],
                        [342, 343, 344],
                        [347, 345, 346],
                        [350, 348, 349],
                        [150, 151, 152],
                        [155, 153, 154],
                        [156, 157, 158],
                        [161, 159, 160],
                        [162, 163, 164],
                        [167, 165, 166],
                        [168, 169, 170],
                        [173, 171, 172],
                        [174, 175, 176],
                        [179, 177, 178],
                        [315, 316, 317],
                        [320, 318, 319],
                        [323, 321, 322],
                        [120, 121, 122],
                        [125, 123, 124],
                        [126, 127, 128],
                        [131, 129, 130],
                        [132, 133, 134],
                        [137, 135, 136],
                        [138, 139, 140],
                        [143, 141, 142],
                        [144, 145, 146],
                        [149, 147, 148],
                        [306, 307, 308],
                        [311, 309, 310],
                        [314, 312, 313],
                        [90, 91, 92],
                        [95, 93, 94],
                        [96, 97, 98],
                        [101, 99, 100],
                        [102, 103, 104],
                        [107, 105, 106],
                        [108, 109, 110],
                        [113, 111, 112],
                        [114, 115, 116],
                        [119, 117, 118],
                        [297, 298, 299],
                        [302, 300, 301],
                        [305, 303, 304],
                        [60, 61, 62],
                        [65, 63, 64],
                        [66, 67, 68],
                        [71, 69, 70],
                        [72, 73, 74],
                        [77, 75, 76],
                        [78, 79, 80],
                        [83, 81, 82],
                        [84, 85, 86],
                        [89, 87, 88],
                        [288, 289, 290],
                        [293, 291, 292],
                        [296, 294, 295],
                        [30, 31, 32],
                        [35, 33, 34],
                        [36, 37, 38],
                        [41, 39, 40],
                        [42, 43, 44],
                        [47, 45, 46],
                        [48, 49, 50],
                        [53, 51, 52],
                        [54, 55, 56],
                        [59, 57, 58],
                        [279, 280, 281],
                        [284, 282, 283],
                        [287, 285, 286],
                        [0, 1, 2],
                        [5, 3, 4],
                        [6, 7, 8],
                        [11, 9, 10],
                        [12, 13, 14],
                        [17, 15, 16],
                        [18, 19, 20],
                        [23, 21, 22],
                        [24, 25, 26],
                        [29, 27, 28],
                        [270, 271, 272],
                        [275, 273, 274],
                        [278, 276, 277],
                        [210, 211, 212],
                        [215, 213, 214],
                        [216, 217, 218],
                        [221, 219, 220],
                        [222, 223, 224],
                        [227, 225, 226],
                        [228, 229, 230],
                        [233, 231, 232],
                        [234, 235, 236],
                        [239, 237, 238],
                        [333, 334, 335],
                        [338, 336, 337],
                        [341, 339, 340],
                        [180, 181, 182],
                        [185, 183, 184],
                        [186, 187, 188],
                        [191, 189, 190],
                        [192, 193, 194],
                        [197, 195, 196],
                        [198, 199, 200],
                        [203, 201, 202],
                        [204, 205, 206],
                        [209, 207, 208],
                        [324, 325, 326],
                        [329, 327, 328],
                        [332, 330, 331],
                    ];
                    let addr = lookup[x as usize][y as usize];
                    if addr < 180 {
                        (addr as u8, 0)
                    } else {
                        ((addr - 180) as u8, 1)
                    }
                },
            },
        }
    }

    pub fn pixel_rgb(&mut self, x: u8, y: u8, r: u8, g: u8, b: u8) -> Result<(), Error<I2cError>> {
        let x = x + y * 13;
        self.device.pixel(x, 2, r)?;
        self.device.pixel(x, 1, g)?;
        self.device.pixel(x, 0, b)?;
        Ok(())
    }

    pub fn setup<DEL: DelayMs<u8>>(&mut self, delay: &mut DEL) -> Result<(), Error<I2cError>> {
        self.device.setup(delay)
    }

    pub fn fill_rgb(&mut self, r: u8, g: u8, b: u8) -> Result<(), Error<I2cError>> {
        for x in 0..13 {
            for y in 0..9 {
                self.pixel_rgb(x, y, r, g, b)?;
            }
        }
        Ok(())
    }
}

#[cfg(feature = "framework_ledmatrix")]
pub const CALC_PIXEL: fn(x: u8, y: u8) -> (u8, u8) = |x: u8, y: u8| -> (u8, u8) {
    // Generated by led-matrix.py
    let lookup: [(u8, u8); 34 * 9] = [
        (0x00, 0), // x: 1, y: 1, sw: 1, cs: 1, id:  1
        (0x1e, 0), // x: 2, y: 1, sw: 2, cs: 1, id:  2
        (0x3c, 0), // x: 3, y: 1, sw: 3, cs: 1, id:  3
        (0x5a, 0), // x: 4, y: 1, sw: 4, cs: 1, id:  4
        (0x78, 0), // x: 5, y: 1, sw: 5, cs: 1, id:  5
        (0x96, 0), // x: 6, y: 1, sw: 6, cs: 1, id:  6
        (0x00, 1), // x: 7, y: 1, sw: 7, cs: 1, id:  7
        (0x1e, 1), // x: 8, y: 1, sw: 8, cs: 1, id:  8
        (0x5f, 1), // x: 9, y: 1, sw: 1, cs:36, id:  9
        (0x01, 0), // x: 1, y: 2, sw: 1, cs: 2, id: 10
        (0x1f, 0), // x: 2, y: 2, sw: 2, cs: 2, id: 11
        (0x3d, 0), // x: 3, y: 2, sw: 3, cs: 2, id: 12
        (0x5b, 0), // x: 4, y: 2, sw: 4, cs: 2, id: 13
        (0x79, 0), // x: 5, y: 2, sw: 5, cs: 2, id: 14
        (0x97, 0), // x: 6, y: 2, sw: 6, cs: 2, id: 15
        (0x01, 1), // x: 7, y: 2, sw: 7, cs: 2, id: 16
        (0x1f, 1), // x: 8, y: 2, sw: 8, cs: 2, id: 17
        (0x60, 1), // x: 9, y: 2, sw: 1, cs:37, id: 18
        (0x02, 0), // x: 1, y: 3, sw: 1, cs: 3, id: 19
        (0x20, 0), // x: 2, y: 3, sw: 2, cs: 3, id: 20
        (0x3e, 0), // x: 3, y: 3, sw: 3, cs: 3, id: 21
        (0x5c, 0), // x: 4, y: 3, sw: 4, cs: 3, id: 22
        (0x7a, 0), // x: 5, y: 3, sw: 5, cs: 3, id: 23
        (0x98, 0), // x: 6, y: 3, sw: 6, cs: 3, id: 24
        (0x02, 1), // x: 7, y: 3, sw: 7, cs: 3, id: 25
        (0x20, 1), // x: 8, y: 3, sw: 8, cs: 3, id: 26
        (0x61, 1), // x: 9, y: 3, sw: 1, cs:38, id: 27
        (0x03, 0), // x: 1, y: 4, sw: 1, cs: 4, id: 28
        (0x21, 0), // x: 2, y: 4, sw: 2, cs: 4, id: 29
        (0x3f, 0), // x: 3, y: 4, sw: 3, cs: 4, id: 30
        (0x5d, 0), // x: 4, y: 4, sw: 4, cs: 4, id: 31
        (0x7b, 0), // x: 5, y: 4, sw: 5, cs: 4, id: 32
        (0x99, 0), // x: 6, y: 4, sw: 6, cs: 4, id: 33
        (0x03, 1), // x: 7, y: 4, sw: 7, cs: 4, id: 34
        (0x21, 1), // x: 8, y: 4, sw: 8, cs: 4, id: 35
        (0x62, 1), // x: 9, y: 4, sw: 1, cs:39, id: 36
        (0x04, 0), // x: 1, y: 5, sw: 1, cs: 5, id: 37
        (0x22, 0), // x: 2, y: 5, sw: 2, cs: 5, id: 41
        (0x40, 0), // x: 3, y: 5, sw: 3, cs: 5, id: 45
        (0x5e, 0), // x: 4, y: 5, sw: 4, cs: 5, id: 49
        (0x7c, 0), // x: 5, y: 5, sw: 5, cs: 5, id: 53
        (0x9a, 0), // x: 6, y: 5, sw: 6, cs: 5, id: 57
        (0x04, 1), // x: 7, y: 5, sw: 7, cs: 5, id: 61
        (0x22, 1), // x: 8, y: 5, sw: 8, cs: 5, id: 65
        (0x5e, 1), // x: 9, y: 5, sw: 1, cs:35, id: 69
        (0x05, 0), // x: 1, y: 6, sw: 1, cs: 6, id: 38
        (0x23, 0), // x: 2, y: 6, sw: 2, cs: 6, id: 42
        (0x41, 0), // x: 3, y: 6, sw: 3, cs: 6, id: 46
        (0x5f, 0), // x: 4, y: 6, sw: 4, cs: 6, id: 50
        (0x7d, 0), // x: 5, y: 6, sw: 5, cs: 6, id: 54
        (0x9b, 0), // x: 6, y: 6, sw: 6, cs: 6, id: 58
        (0x05, 1), // x: 7, y: 6, sw: 7, cs: 6, id: 62
        (0x23, 1), // x: 8, y: 6, sw: 8, cs: 6, id: 66
        (0x68, 1), // x: 9, y: 6, sw: 2, cs:36, id: 70
        (0x06, 0), // x: 1, y: 7, sw: 1, cs: 7, id: 39
        (0x24, 0), // x: 2, y: 7, sw: 2, cs: 7, id: 43
        (0x42, 0), // x: 3, y: 7, sw: 3, cs: 7, id: 47
        (0x60, 0), // x: 4, y: 7, sw: 4, cs: 7, id: 51
        (0x7e, 0), // x: 5, y: 7, sw: 5, cs: 7, id: 55
        (0x9c, 0), // x: 6, y: 7, sw: 6, cs: 7, id: 59
        (0x06, 1), // x: 7, y: 7, sw: 7, cs: 7, id: 63
        (0x24, 1), // x: 8, y: 7, sw: 8, cs: 7, id: 67
        (0x69, 1), // x: 9, y: 7, sw: 2, cs:37, id: 71
        (0x07, 0), // x: 1, y: 8, sw: 1, cs: 8, id: 40
        (0x25, 0), // x: 2, y: 8, sw: 2, cs: 8, id: 44
        (0x43, 0), // x: 3, y: 8, sw: 3, cs: 8, id: 48
        (0x61, 0), // x: 4, y: 8, sw: 4, cs: 8, id: 52
        (0x7f, 0), // x: 5, y: 8, sw: 5, cs: 8, id: 56
        (0x9d, 0), // x: 6, y: 8, sw: 6, cs: 8, id: 60
        (0x07, 1), // x: 7, y: 8, sw: 7, cs: 8, id: 64
        (0x25, 1), // x: 8, y: 8, sw: 8, cs: 8, id: 68
        (0x6a, 1), // x: 9, y: 8, sw: 2, cs:38, id: 72
        (0x08, 0), // x: 1, y: 9, sw: 1, cs: 9, id: 73
        (0x26, 0), // x: 2, y: 9, sw: 2, cs: 9, id: 81
        (0x44, 0), // x: 3, y: 9, sw: 3, cs: 9, id: 89
        (0x62, 0), // x: 4, y: 9, sw: 4, cs: 9, id: 97
        (0x80, 0), // x: 5, y: 9, sw: 5, cs: 9, id:105
        (0x9e, 0), // x: 6, y: 9, sw: 6, cs: 9, id:113
        (0x08, 1), // x: 7, y: 9, sw: 7, cs: 9, id:121
        (0x26, 1), // x: 8, y: 9, sw: 8, cs: 9, id:129
        (0x6b, 1), // x: 9, y: 9, sw: 2, cs:39, id:137
        (0x09, 0), // x: 1, y:10, sw: 1, cs:10, id: 74
        (0x27, 0), // x: 2, y:10, sw: 2, cs:10, id: 82
        (0x45, 0), // x: 3, y:10, sw: 3, cs:10, id: 90
        (0x63, 0), // x: 4, y:10, sw: 4, cs:10, id: 98
        (0x81, 0), // x: 5, y:10, sw: 5, cs:10, id:106
        (0x9f, 0), // x: 6, y:10, sw: 6, cs:10, id:114
        (0x09, 1), // x: 7, y:10, sw: 7, cs:10, id:122
        (0x27, 1), // x: 8, y:10, sw: 8, cs:10, id:130
        (0x67, 1), // x: 9, y:10, sw: 2, cs:35, id:138
        (0x0a, 0), // x: 1, y:11, sw: 1, cs:11, id: 75
        (0x28, 0), // x: 2, y:11, sw: 2, cs:11, id: 83
        (0x46, 0), // x: 3, y:11, sw: 3, cs:11, id: 91
        (0x64, 0), // x: 4, y:11, sw: 4, cs:11, id: 99
        (0x82, 0), // x: 5, y:11, sw: 5, cs:11, id:107
        (0xa0, 0), // x: 6, y:11, sw: 6, cs:11, id:115
        (0x0a, 1), // x: 7, y:11, sw: 7, cs:11, id:123
        (0x28, 1), // x: 8, y:11, sw: 8, cs:11, id:131
        (0x71, 1), // x: 9, y:11, sw: 3, cs:36, id:139
        (0x0b, 0), // x: 1, y:12, sw: 1, cs:12, id: 76
        (0x29, 0), // x: 2, y:12, sw: 2, cs:12, id: 84
        (0x47, 0), // x: 3, y:12, sw: 3, cs:12, id: 92
        (0x65, 0), // x: 4, y:12, sw: 4, cs:12, id:100
        (0x83, 0), // x: 5, y:12, sw: 5, cs:12, id:108
        (0xa1, 0), // x: 6, y:12, sw: 6, cs:12, id:116
        (0x0b, 1), // x: 7, y:12, sw: 7, cs:12, id:124
        (0x29, 1), // x: 8, y:12, sw: 8, cs:12, id:132
        (0x72, 1), // x: 9, y:12, sw: 3, cs:37, id:140
        (0x0c, 0), // x: 1, y:13, sw: 1, cs:13, id: 77
        (0x2a, 0), // x: 2, y:13, sw: 2, cs:13, id: 85
        (0x48, 0), // x: 3, y:13, sw: 3, cs:13, id: 93
        (0x66, 0), // x: 4, y:13, sw: 4, cs:13, id:101
        (0x84, 0), // x: 5, y:13, sw: 5, cs:13, id:109
        (0xa2, 0), // x: 6, y:13, sw: 6, cs:13, id:117
        (0x0c, 1), // x: 7, y:13, sw: 7, cs:13, id:125
        (0x2a, 1), // x: 8, y:13, sw: 8, cs:13, id:133
        (0x73, 1), // x: 9, y:13, sw: 3, cs:38, id:141
        (0x0d, 0), // x: 1, y:14, sw: 1, cs:14, id: 78
        (0x2b, 0), // x: 2, y:14, sw: 2, cs:14, id: 86
        (0x49, 0), // x: 3, y:14, sw: 3, cs:14, id: 94
        (0x67, 0), // x: 4, y:14, sw: 4, cs:14, id:102
        (0x85, 0), // x: 5, y:14, sw: 5, cs:14, id:110
        (0xa3, 0), // x: 6, y:14, sw: 6, cs:14, id:118
        (0x0d, 1), // x: 7, y:14, sw: 7, cs:14, id:126
        (0x2b, 1), // x: 8, y:14, sw: 8, cs:14, id:134
        (0x70, 1), // x: 9, y:14, sw: 3, cs:35, id:142
        (0x0e, 0), // x: 1, y:15, sw: 1, cs:15, id: 79
        (0x2c, 0), // x: 2, y:15, sw: 2, cs:15, id: 87
        (0x4a, 0), // x: 3, y:15, sw: 3, cs:15, id: 95
        (0x68, 0), // x: 4, y:15, sw: 4, cs:15, id:103
        (0x86, 0), // x: 5, y:15, sw: 5, cs:15, id:111
        (0xa4, 0), // x: 6, y:15, sw: 6, cs:15, id:119
        (0x0e, 1), // x: 7, y:15, sw: 7, cs:15, id:127
        (0x2c, 1), // x: 8, y:15, sw: 8, cs:15, id:135
        (0x7a, 1), // x: 9, y:15, sw: 4, cs:36, id:143
        (0x0f, 0), // x: 1, y:16, sw: 1, cs:16, id: 80
        (0x2d, 0), // x: 2, y:16, sw: 2, cs:16, id: 88
        (0x4b, 0), // x: 3, y:16, sw: 3, cs:16, id: 96
        (0x69, 0), // x: 4, y:16, sw: 4, cs:16, id:104
        (0x87, 0), // x: 5, y:16, sw: 5, cs:16, id:112
        (0xa5, 0), // x: 6, y:16, sw: 6, cs:16, id:120
        (0x0f, 1), // x: 7, y:16, sw: 7, cs:16, id:128
        (0x2d, 1), // x: 8, y:16, sw: 8, cs:16, id:136
        (0x7b, 1), // x: 9, y:16, sw: 4, cs:37, id:144
        (0x10, 0), // x: 1, y:17, sw: 1, cs:17, id:145
        (0x2e, 0), // x: 2, y:17, sw: 2, cs:17, id:161
        (0x4c, 0), // x: 3, y:17, sw: 3, cs:17, id:177
        (0x6a, 0), // x: 4, y:17, sw: 4, cs:17, id:193
        (0x88, 0), // x: 5, y:17, sw: 5, cs:17, id:209
        (0xa6, 0), // x: 6, y:17, sw: 6, cs:17, id:225
        (0x10, 1), // x: 7, y:17, sw: 7, cs:17, id:241
        (0x2e, 1), // x: 8, y:17, sw: 8, cs:17, id:257
        (0x7c, 1), // x: 9, y:17, sw: 4, cs:38, id:273
        (0x11, 0), // x: 1, y:18, sw: 1, cs:18, id:146
        (0x2f, 0), // x: 2, y:18, sw: 2, cs:18, id:162
        (0x4d, 0), // x: 3, y:18, sw: 3, cs:18, id:178
        (0x6b, 0), // x: 4, y:18, sw: 4, cs:18, id:194
        (0x89, 0), // x: 5, y:18, sw: 5, cs:18, id:210
        (0xa7, 0), // x: 6, y:18, sw: 6, cs:18, id:226
        (0x11, 1), // x: 7, y:18, sw: 7, cs:18, id:242
        (0x2f, 1), // x: 8, y:18, sw: 8, cs:18, id:258
        (0x79, 1), // x: 9, y:18, sw: 4, cs:35, id:274
        (0x12, 0), // x: 1, y:19, sw: 1, cs:19, id:147
        (0x30, 0), // x: 2, y:19, sw: 2, cs:19, id:163
        (0x4e, 0), // x: 3, y:19, sw: 3, cs:19, id:179
        (0x6c, 0), // x: 4, y:19, sw: 4, cs:19, id:195
        (0x8a, 0), // x: 5, y:19, sw: 5, cs:19, id:211
        (0xa8, 0), // x: 6, y:19, sw: 6, cs:19, id:227
        (0x12, 1), // x: 7, y:19, sw: 7, cs:19, id:243
        (0x30, 1), // x: 8, y:19, sw: 8, cs:19, id:259
        (0x83, 1), // x: 9, y:19, sw: 5, cs:36, id:275
        (0x13, 0), // x: 1, y:20, sw: 1, cs:20, id:148
        (0x31, 0), // x: 2, y:20, sw: 2, cs:20, id:164
        (0x4f, 0), // x: 3, y:20, sw: 3, cs:20, id:180
        (0x6d, 0), // x: 4, y:20, sw: 4, cs:20, id:196
        (0x8b, 0), // x: 5, y:20, sw: 5, cs:20, id:212
        (0xa9, 0), // x: 6, y:20, sw: 6, cs:20, id:228
        (0x13, 1), // x: 7, y:20, sw: 7, cs:20, id:244
        (0x31, 1), // x: 8, y:20, sw: 8, cs:20, id:260
        (0x84, 1), // x: 9, y:20, sw: 5, cs:37, id:276
        (0x14, 0), // x: 1, y:21, sw: 1, cs:21, id:149
        (0x32, 0), // x: 2, y:21, sw: 2, cs:21, id:165
        (0x50, 0), // x: 3, y:21, sw: 3, cs:21, id:181
        (0x6e, 0), // x: 4, y:21, sw: 4, cs:21, id:197
        (0x8c, 0), // x: 5, y:21, sw: 5, cs:21, id:213
        (0xaa, 0), // x: 6, y:21, sw: 6, cs:21, id:229
        (0x14, 1), // x: 7, y:21, sw: 7, cs:21, id:245
        (0x32, 1), // x: 8, y:21, sw: 8, cs:21, id:261
        (0x85, 1), // x: 9, y:21, sw: 5, cs:38, id:277
        (0x15, 0), // x: 1, y:22, sw: 1, cs:22, id:150
        (0x33, 0), // x: 2, y:22, sw: 2, cs:22, id:166
        (0x51, 0), // x: 3, y:22, sw: 3, cs:22, id:182
        (0x6f, 0), // x: 4, y:22, sw: 4, cs:22, id:198
        (0x8d, 0), // x: 5, y:22, sw: 5, cs:22, id:214
        (0xab, 0), // x: 6, y:22, sw: 6, cs:22, id:230
        (0x15, 1), // x: 7, y:22, sw: 7, cs:22, id:246
        (0x33, 1), // x: 8, y:22, sw: 8, cs:22, id:262
        (0x82, 1), // x: 9, y:22, sw: 5, cs:35, id:278
        (0x16, 0), // x: 1, y:23, sw: 1, cs:23, id:151
        (0x34, 0), // x: 2, y:23, sw: 2, cs:23, id:167
        (0x52, 0), // x: 3, y:23, sw: 3, cs:23, id:183
        (0x70, 0), // x: 4, y:23, sw: 4, cs:23, id:199
        (0x8e, 0), // x: 5, y:23, sw: 5, cs:23, id:215
        (0xac, 0), // x: 6, y:23, sw: 6, cs:23, id:231
        (0x16, 1), // x: 7, y:23, sw: 7, cs:23, id:247
        (0x34, 1), // x: 8, y:23, sw: 8, cs:23, id:263
        (0x8c, 1), // x: 9, y:23, sw: 6, cs:36, id:279
        (0x17, 0), // x: 1, y:24, sw: 1, cs:24, id:152
        (0x35, 0), // x: 2, y:24, sw: 2, cs:24, id:168
        (0x53, 0), // x: 3, y:24, sw: 3, cs:24, id:184
        (0x71, 0), // x: 4, y:24, sw: 4, cs:24, id:200
        (0x8f, 0), // x: 5, y:24, sw: 5, cs:24, id:216
        (0xad, 0), // x: 6, y:24, sw: 6, cs:24, id:232
        (0x17, 1), // x: 7, y:24, sw: 7, cs:24, id:248
        (0x35, 1), // x: 8, y:24, sw: 8, cs:24, id:264
        (0x8d, 1), // x: 9, y:24, sw: 6, cs:37, id:280
        (0x18, 0), // x: 1, y:25, sw: 1, cs:25, id:153
        (0x36, 0), // x: 2, y:25, sw: 2, cs:25, id:169
        (0x54, 0), // x: 3, y:25, sw: 3, cs:25, id:185
        (0x72, 0), // x: 4, y:25, sw: 4, cs:25, id:201
        (0x90, 0), // x: 5, y:25, sw: 5, cs:25, id:217
        (0xae, 0), // x: 6, y:25, sw: 6, cs:25, id:233
        (0x18, 1), // x: 7, y:25, sw: 7, cs:25, id:249
        (0x36, 1), // x: 8, y:25, sw: 8, cs:25, id:265
        (0x8e, 1), // x: 9, y:25, sw: 6, cs:38, id:281
        (0x19, 0), // x: 1, y:26, sw: 1, cs:26, id:154
        (0x37, 0), // x: 2, y:26, sw: 2, cs:26, id:170
        (0x55, 0), // x: 3, y:26, sw: 3, cs:26, id:186
        (0x73, 0), // x: 4, y:26, sw: 4, cs:26, id:202
        (0x91, 0), // x: 5, y:26, sw: 5, cs:26, id:218
        (0xaf, 0), // x: 6, y:26, sw: 6, cs:26, id:234
        (0x19, 1), // x: 7, y:26, sw: 7, cs:26, id:250
        (0x37, 1), // x: 8, y:26, sw: 8, cs:26, id:266
        (0x8b, 1), // x: 9, y:26, sw: 6, cs:35, id:282
        (0x1a, 0), // x: 1, y:27, sw: 1, cs:27, id:155
        (0x38, 0), // x: 2, y:27, sw: 2, cs:27, id:171
        (0x56, 0), // x: 3, y:27, sw: 3, cs:27, id:187
        (0x74, 0), // x: 4, y:27, sw: 4, cs:27, id:203
        (0x92, 0), // x: 5, y:27, sw: 5, cs:27, id:219
        (0xb0, 0), // x: 6, y:27, sw: 6, cs:27, id:235
        (0x1a, 1), // x: 7, y:27, sw: 7, cs:27, id:251
        (0x38, 1), // x: 8, y:27, sw: 8, cs:27, id:267
        (0x95, 1), // x: 9, y:27, sw: 7, cs:36, id:283
        (0x1b, 0), // x: 1, y:28, sw: 1, cs:28, id:156
        (0x39, 0), // x: 2, y:28, sw: 2, cs:28, id:172
        (0x57, 0), // x: 3, y:28, sw: 3, cs:28, id:188
        (0x75, 0), // x: 4, y:28, sw: 4, cs:28, id:204
        (0x93, 0), // x: 5, y:28, sw: 5, cs:28, id:220
        (0xb1, 0), // x: 6, y:28, sw: 6, cs:28, id:236
        (0x1b, 1), // x: 7, y:28, sw: 7, cs:28, id:252
        (0x39, 1), // x: 8, y:28, sw: 8, cs:28, id:268
        (0x96, 1), // x: 9, y:28, sw: 7, cs:37, id:284
        (0x1c, 0), // x: 1, y:29, sw: 1, cs:29, id:157
        (0x3a, 0), // x: 2, y:29, sw: 2, cs:29, id:173
        (0x58, 0), // x: 3, y:29, sw: 3, cs:29, id:189
        (0x76, 0), // x: 4, y:29, sw: 4, cs:29, id:205
        (0x94, 0), // x: 5, y:29, sw: 5, cs:29, id:221
        (0xb2, 0), // x: 6, y:29, sw: 6, cs:29, id:237
        (0x1c, 1), // x: 7, y:29, sw: 7, cs:29, id:253
        (0x3a, 1), // x: 8, y:29, sw: 8, cs:29, id:269
        (0x97, 1), // x: 9, y:29, sw: 7, cs:38, id:285
        (0x1d, 0), // x: 1, y:30, sw: 1, cs:30, id:158
        (0x3b, 0), // x: 2, y:30, sw: 2, cs:30, id:174
        (0x59, 0), // x: 3, y:30, sw: 3, cs:30, id:190
        (0x77, 0), // x: 4, y:30, sw: 4, cs:30, id:206
        (0x95, 0), // x: 5, y:30, sw: 5, cs:30, id:222
        (0xb3, 0), // x: 6, y:30, sw: 6, cs:30, id:238
        (0x1d, 1), // x: 7, y:30, sw: 7, cs:30, id:254
        (0x3b, 1), // x: 8, y:30, sw: 8, cs:30, id:270
        (0x94, 1), // x: 9, y:30, sw: 7, cs:35, id:286
        (0x5a, 1), // x: 1, y:31, sw: 1, cs:31, id:159
        (0x63, 1), // x: 2, y:31, sw: 2, cs:31, id:175
        (0x6c, 1), // x: 3, y:31, sw: 3, cs:31, id:191
        (0x75, 1), // x: 4, y:31, sw: 4, cs:31, id:207
        (0x7e, 1), // x: 5, y:31, sw: 5, cs:31, id:223
        (0x87, 1), // x: 6, y:31, sw: 6, cs:31, id:239
        (0x90, 1), // x: 7, y:31, sw: 7, cs:31, id:255
        (0x99, 1), // x: 8, y:31, sw: 8, cs:31, id:271
        (0x9e, 1), // x: 9, y:31, sw: 8, cs:36, id:287
        (0x5b, 1), // x: 1, y:32, sw: 1, cs:32, id:160
        (0x64, 1), // x: 2, y:32, sw: 2, cs:32, id:176
        (0x6d, 1), // x: 3, y:32, sw: 3, cs:32, id:192
        (0x76, 1), // x: 4, y:32, sw: 4, cs:32, id:208
        (0x7f, 1), // x: 5, y:32, sw: 5, cs:32, id:224
        (0x88, 1), // x: 6, y:32, sw: 6, cs:32, id:240
        (0x91, 1), // x: 7, y:32, sw: 7, cs:32, id:256
        (0x9a, 1), // x: 8, y:32, sw: 8, cs:32, id:272
        (0x9f, 1), // x: 9, y:32, sw: 8, cs:37, id:288
        (0x5c, 1), // x: 1, y:33, sw: 1, cs:33, id:289
        (0x65, 1), // x: 2, y:33, sw: 2, cs:33, id:290
        (0x6e, 1), // x: 3, y:33, sw: 3, cs:33, id:291
        (0x77, 1), // x: 4, y:33, sw: 4, cs:33, id:292
        (0x80, 1), // x: 5, y:33, sw: 5, cs:33, id:293
        (0x89, 1), // x: 6, y:33, sw: 6, cs:33, id:294
        (0x92, 1), // x: 7, y:33, sw: 7, cs:33, id:295
        (0x9b, 1), // x: 8, y:33, sw: 8, cs:33, id:296
        (0xa0, 1), // x: 9, y:33, sw: 8, cs:38, id:297
        (0x5d, 1), // x: 1, y:34, sw: 1, cs:34, id:298
        (0x66, 1), // x: 2, y:34, sw: 2, cs:34, id:299
        (0x6f, 1), // x: 3, y:34, sw: 3, cs:34, id:300
        (0x78, 1), // x: 4, y:34, sw: 4, cs:34, id:301
        (0x81, 1), // x: 5, y:34, sw: 5, cs:34, id:302
        (0x8a, 1), // x: 6, y:34, sw: 6, cs:34, id:303
        (0x93, 1), // x: 7, y:34, sw: 7, cs:34, id:304
        (0x9c, 1), // x: 8, y:34, sw: 8, cs:34, id:305
        (0x9d, 1), // x: 9, y:34, sw: 8, cs:35, id:306
    ];
    let index: usize = (x as usize) + (y as usize) * 9;
    if index < lookup.len() {
        lookup[index]
    } else {
        (0x00, 0)
    }
};

#[cfg(feature = "framework_ledmatrix")]
pub struct LedMatrix<I2C> {
    pub device: IS31FL3741<I2C>,
}

#[cfg(feature = "framework_ledmatrix")]
impl<I2C, I2cError> LedMatrix<I2C>
where
    I2C: Write<Error = I2cError>,
    I2C: Read<Error = I2cError>,
{
    pub fn unwrap(self) -> I2C {
        self.device.i2c
    }

    // TODO: Maybe make this private and set it once in the constructor
    pub fn set_scaling(&mut self, scale: u8) -> Result<(), I2cError> {
        self.device.set_scaling(scale)
    }

    pub fn new(i2c: I2C, calc_pixel: fn(x: u8, y: u8) -> (u8, u8)) -> LedMatrix<I2C> {
        LedMatrix {
            device: IS31FL3741 {
                i2c,
                address: 0x30,
                width: 9,
                height: 34,
                calc_pixel,
            },
        }
    }

    pub fn setup<DEL: DelayMs<u8>>(&mut self, delay: &mut DEL) -> Result<(), Error<I2cError>> {
        self.device.setup(delay)?;
        Ok(())
    }

    /// Fills the matrix with a _raw_ brightness value, i.e. without gamma
    /// correction, to show the native PWM values.
    pub fn fill_brightness(&mut self, brightness: u8) -> Result<(), Error<I2cError>> {
        for x in 0..self.device.width {
            for y in 0..self.device.height {
                self.device.pixel(x, y, brightness)?;
            }
        }
        Ok(())
    }
}
