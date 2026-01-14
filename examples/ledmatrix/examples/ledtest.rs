//! LED Matrix Module
//!
//! Goes into bootloader mode when the host is asleep. This is to make it easy to reflash your
//! firmware - the regular bootloader mechanism using the DIP switch still works.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use rp2040_panic_usb_boot as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use fugit::RateExtU32;
use rp2040_hal::{
    self as hal,
    clocks::init_clocks_and_plls,
    entry,
    gpio::{FunctionI2C, PullUp},
    pac,
    rom_data::reset_to_usb_boot,
    sio::Sio,
    watchdog::Watchdog,
    Timer, I2C,
};

use is31fl3741::devices::{LedMatrix, CALC_PIXEL};
use is31fl3741::PwmFreq;

/// Maximum brightness out of 255
///
/// 100/255 results in 250mA current draw and is plenty bright.
///  50/255 results in 160mA current draw and is plenty bright.
const MAX_BRIGHTNESS: u8 = 50;

/// External crystal frequency (12MHz on RP Pico and Framework LED Matrix)
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

    // Enable LED controller
    // SDB - Gpio29
    let mut led_enable = pins.gpio29.into_push_pull_output();
    led_enable.set_high().unwrap();
    // INTB. Currently ignoring
    let _intb = pins.gpio28.into_floating_input();

    // I2C1 pins for the LED matrix
    let sda: hal::gpio::Pin<_, FunctionI2C, PullUp> = pins.gpio26.reconfigure();
    let scl: hal::gpio::Pin<_, FunctionI2C, PullUp> = pins.gpio27.reconfigure();

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        1000.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Gpio25 (LED on rp-pico, DIP switch on Framework LED Matrix)
    let mut dip1 = pins.gpio25.into_pull_up_input();
    let _dip1_state = dip1.is_low().unwrap();

    // Detect whether the sleep pin is connected
    // Early revisions of the hardware didn't have it wired up, if that is the
    // case we have to ignore its state.
    let mut sleep_present = false;
    let mut sleep = pins.gpio0.into_pull_up_input();
    if sleep.is_low().unwrap() {
        sleep_present = true;
    }
    let mut sleep = sleep.into_pull_down_input();
    if sleep.is_high().unwrap() {
        sleep_present = true;
    }

    let mut matrix = LedMatrix::new(i2c, CALC_PIXEL);
    matrix
        .setup(&mut delay)
        .expect("failed to setup RGB controller");

    // Enable only the SW pins that we're using.
    // Otherwise driving the unused pins might result in audible noise.
    matrix
        .device
        .sw_enablement(is31fl3741::SwSetting::Sw1Sw8)
        .unwrap();

    matrix
        .set_scaling(MAX_BRIGHTNESS)
        .expect("failed to set scaling");

    matrix.device.set_pwm_freq(PwmFreq::P29k).unwrap();

    loop {
        // Light up each LED, one by one
        for y in 0..matrix.device.height {
            for x in 0..matrix.device.width {
                matrix.device.pixel(x, y, 0xFF).expect("couldn't turn on");
                delay.delay_ms(100);
                matrix.device.pixel(x, y, 0).expect("couldn't turn off");

                // Reset into bootloader if system asleep
                if sleep_present && sleep.is_low().unwrap() {
                    reset_to_usb_boot(0, 0);
                }
            }
        }
    }
}
