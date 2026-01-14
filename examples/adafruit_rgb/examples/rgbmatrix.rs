//! Adafruit RGB 13x9 Matrix Example for QT Py RP2040
//!
//! Lights up each LED one by one on the Adafruit IS31FL3741 RGB matrix.
//! Connect the matrix to the STEMMA QT connector on the QT Py RP2040.
//!
//! STEMMA QT pinout on QT Py RP2040:
//! - SDA: GPIO22
//! - SCL: GPIO23
#![no_std]
#![no_main]

use panic_halt as _;

use embedded_hal::delay::DelayNs;
use fugit::RateExtU32;
use rp2040_hal::{
    self as hal,
    clocks::init_clocks_and_plls,
    entry,
    gpio::{FunctionI2C, PullUp},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Timer, I2C,
};

use is31fl3741::devices::AdafruitRGB13x9;

/// External crystal frequency (QT Py RP2040 uses 12MHz)
const XTAL_FREQ_HZ: u32 = 12_000_000;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = timer;

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // STEMMA QT connector I2C1 pins on QT Py RP2040
    let sda: hal::gpio::Pin<_, FunctionI2C, PullUp> = pins.gpio22.reconfigure();
    let scl: hal::gpio::Pin<_, FunctionI2C, PullUp> = pins.gpio23.reconfigure();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let mut matrix = AdafruitRGB13x9::configure(i2c);
    matrix
        .setup(&mut delay)
        .expect("failed to setup RGB controller");

    matrix.set_scaling(0xFF).expect("failed to set scaling");

    loop {
        // Light up each LED one by one
        for y in 0..9 {
            for x in 0..13 {
                matrix
                    .pixel_rgb(x, y, 0x1E, 0x90, 0xFF)
                    .expect("couldn't turn on");
                delay.delay_ms(100);
                matrix.pixel_rgb(x, y, 0, 0, 0).expect("couldn't turn off");
            }
        }
    }
}
