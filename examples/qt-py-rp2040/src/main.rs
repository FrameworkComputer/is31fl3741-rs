#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _;

use fugit::RateExtU32;
use adafruit_qt_py_rp2040::entry;
use adafruit_qt_py_rp2040::{
    hal::{
        i2c::I2C,
        clocks::{init_clocks_and_plls, Clock},
        pac,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use is31fl3741::devices::AdafruitRGB13x9;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
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
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Using STEMMA QT connector
    let sda = pins.sda1.into_mode(); // gpio22
    let scl = pins.scl1.into_mode(); // gpio23

    //let sda = pins.sda.into_mode(); // gpio24
    //let scl = pins.scl.into_mode(); // gpio25

    let i2c = I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );

    // // https://github.com/adafruit/Adafruit_CircuitPython_IS31FL3741/blob/main/adafruit_is31fl3 741/adafruit_rgbmatrixqt.py#L53-L65

    let mut matrix = AdafruitRGB13x9::configure(i2c);
    matrix
        .setup(&mut delay)
        .expect("failed to setup rgb controller");

    matrix.set_scaling(0xFF).expect("failed to set scaling");

    loop {
        for y in 0..9 {
            for x in 0..13 {
                matrix
                    .pixel_rgb(x, y, 0x1E, 0x90, 0xFF)
                    .expect("couldn't turn on");
                delay.delay_ms(100u32);
                matrix.pixel_rgb(x, y, 0, 0, 0).expect("couldn't turn off");
            }
        }
    }
}
