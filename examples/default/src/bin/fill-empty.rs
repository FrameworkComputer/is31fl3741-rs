#![no_std]
#![no_main]

/// This example demonstrates how to use the IS31FL3741 LED matrix driver with the Embassy framework on a Raspberry Pi Pico. 
/// It uses this [adafruit module board](https://www.adafruit.com/product/5201) for the IS31FL3741 LED matrix.
/// It initializes the LED matrix, sets all pixels to white one by one, clears them one by one, and then does the same for all pixels at once.

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    i2c::{self, I2c},
};
use embassy_time::{Duration, Timer};
use embedded_hal::delay::DelayNs;
use is31fl3741::IS31FL3741;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let peri = embassy_rp::init(Default::default());
    let _status_led = Output::new(peri.PIN_25, Level::High);

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = 400_000; // Set I2C frequency to 400 kHz for faster communication
    let i2c_peri = I2c::new_blocking(peri.I2C1, peri.PIN_3, peri.PIN_2, i2c_config);

    // This configuration was taken from the Adafruit library provided by this driver. If you're using a different module, you may need to make changes.
    let mut led_matrix = IS31FL3741 {
        i2c: i2c_peri,
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
    };

    // Initialize the LED matrix
    let mut delay = embassy_time::Delay;
    led_matrix.setup(&mut delay).unwrap();

    // Set scaling (current limiting) to maximum
    led_matrix.set_scaling(0xFF).unwrap();

    defmt::info!("Matrix initialized, setting all pixels to white...");

    loop {
        defmt::info!("Setting pixels to white one by one...");
        for x in 0..13 * 9 {
            // This being keep because it illustrates how the row and column logic work.
            // if x < 13 {
            //     for y in 0..3 {
            //         led_matrix.pixel(x, y, 0xFF / 10).unwrap();
            //     }
            //     delay.delay_ns(100);
            // }

            // if x % 13 == 0 {
            //     for y in 0..3 {
            //         led_matrix.pixel(x, y, 0xFF / 10).unwrap();
            //     }
            //     delay.delay_ns(100);
            // }

            for y in 0..3 {
                led_matrix.pixel(x, y, 0xFF / 10).unwrap();
            }
            delay.delay_ns(100);
        }
        Timer::after(Duration::from_secs(5)).await;
        defmt::info!("Clearing all pixels one by one...");
        for x in 0..13 * 9 {
            for y in 0..3 {
                led_matrix.pixel(x, y, 0).unwrap();
            }
            delay.delay_ns(100);
        }
        Timer::after(Duration::from_secs(5)).await;

        defmt::info!("Setting all pixels to white at the same time...");
        led_matrix.fill(0xFF/10).unwrap();
        Timer::after(Duration::from_secs(5)).await;
        defmt::info!("Clearing all pixels at the same time...");
        led_matrix.fill(0).unwrap();
        Timer::after(Duration::from_secs(5)).await;
    }
}
