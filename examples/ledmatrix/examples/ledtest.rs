//! LED Matrix Module
//!
//!
//!
//! Goes into bootloader mode when the host is asleep. This is to make it easy to reflash your
//! firmware - the regular bootloader mechanism using the DIP switch still works.
#![no_std]
#![no_main]
#![allow(clippy::needless_range_loop)]

use embedded_hal::digital::{InputPin, OutputPin};
use rp2040_hal::rom_data::reset_to_usb_boot;
use rp2040_panic_usb_boot as _;

/// Maximum brightness out of 255
///
/// 100/255 results in 250mA current draw and is plenty bright.
///  50/255 results in 160mA current draw and is plenty bright.
const MAX_BRIGHTNESS: u8 = 50;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use bsp::entry;
use is31fl3741::devices::{LedMatrix, CALC_PIXEL};
use is31fl3741::{PwmFreq};
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};
use fugit::RateExtU32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Enable LED controller
    // SDB - Gpio29
    let mut led_enable = pins.voltage_monitor.into_push_pull_output();
    led_enable.set_high().unwrap();
    // INTB. Currently ignoring
    pins.gpio28.into_floating_input();

    let i2c = bsp::hal::I2C::i2c1(
        pac.I2C1,
        pins.gpio26.into_mode::<gpio::FunctionI2C>(),
        pins.gpio27.into_mode::<gpio::FunctionI2C>(),
        // 1000,
        1000.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // Gpio25 (LED on rp-pico)
    let dip1 = pins.led.into_pull_up_input();
    let _dip1_state = dip1.is_low().unwrap();

    // Detect whether the sleep pin is connected
    // Early revisions of the hardware didn't have it wired up, if that is the
    // case we have to ignore its state.
    let mut sleep_present = false;
    let sleep = pins.gpio0.into_pull_up_input();
    if sleep.is_low().unwrap() {
        sleep_present = true;
    }
    let sleep = sleep.into_pull_down_input();
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
                matrix
                    .device
                    .pixel(x, y, 0xFF)
                    .expect("couldn't turn on");
                delay.delay_ms(100);
                matrix.device.pixel(x, y, 0).expect("couldn't turn off");

                // Reset into bootloader if system asleep
                if sleep_present && sleep.is_low().unwrap(){
                    reset_to_usb_boot(0, 0);
                }
            }
        }
    }
}
