//! On-device regression checks: send ASCII T over USB serial for a PASS/FAIL report.
//! The inputmodule bootloader command remains available for automated reflashing.
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
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Maximum brightness out of 255
///
/// 100/255 results in 250mA current draw and is plenty bright.
///  50/255 results in 160mA current draw and is plenty bright.
const MAX_BRIGHTNESS: u8 = 50;

/// External crystal frequency (12MHz on RP Pico and Framework LED Matrix)
const XTAL_FREQ_HZ: u32 = 12_000_000;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

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

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x32ac, 0x0020))
        .strings(&[StringDescriptors::default()
            .manufacturer("Framework Computer Inc")
            .product("LED Matrix ledtest")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();
    // Match the inputmodule bootloader command across USB packet boundaries.
    let mut boot_command_prefix = 0;

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
        usb_dev.poll(&mut [&mut serial]);
        let mut buf = [0; 64];
        if let Ok(count) = serial.read(&mut buf) {
            for &byte in &buf[..count] {
                if matches!(byte, b'T' | b'F' | b'C' | b'S' | b'A') {
                    let report: &[u8] = match if byte == b'A' {
                        check_apis(&mut matrix.device)
                    } else {
                        check(&mut matrix.device, byte)
                    } {
                        Ok(()) => match byte {
                            b'A' => b"PASS: GCC resistor logic readback; IRQ ACK/status only\r\n",
                            b'F' => b"PASS: PWM351\r\n",
                            b'C' => b"PASS: bounds\r\n",
                            _ => b"PASS: PWM351 bounds shutdown resume\r\n",
                        },
                        Err(e) => e.as_bytes(),
                    };
                    let mut sent = 0;
                    for _ in 0..2000 {
                        usb_dev.poll(&mut [&mut serial]);
                        if let Ok(n) = serial.write(&report[sent..]) {
                            sent += n;
                        }
                        if sent == report.len() {
                            break;
                        }
                        delay.delay_ms(1);
                    }
                }
                boot_command_prefix = match (boot_command_prefix, byte) {
                    (0, 0x32) => 1,
                    (1, 0xac) => 2,
                    (2, 2) => {
                        reset_to_usb_boot(0, 0);
                        0
                    }
                    (_, 0x32) => 1,
                    _ => 0,
                };
            }
        }
        delay.delay_ms(1);
    }
}

fn read<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    p: u8,
    out: &mut [u8],
) -> Result<(), &'static str> {
    for bytes in [&[0xfe, 0xc5][..], &[0xfd, p], &[0]] {
        d.i2c
            .write(d.address, bytes)
            .map_err(|_| "FAIL: select\r\n")?;
    }
    d.i2c.read(d.address, out).map_err(|_| "FAIL: read\r\n")
}
fn check<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    mode: u8,
) -> Result<(), &'static str> {
    let mut expected = [0; 351];
    let mut actual = [0; 351];
    for (i, v) in expected.iter_mut().enumerate() {
        *v = (i % 251) as u8;
    }
    if mode != b'C' {
        d.fill(0).map_err(|_| "FAIL: clear\r\n")?;
        d.fill_matrix(&expected)
            .map_err(|_| "FAIL: frame write\r\n")?;
        read(d, 0, &mut actual[..180])?;
        read(d, 1, &mut actual[180..])?;
        if actual != expected {
            return Err("FAIL: PWM351 readback\r\n");
        }
    }
    if mode == b'F' {
        return Ok(());
    }
    d.fill(0).map_err(|_| "FAIL: clear\r\n")?;
    expected.fill(0);
    for (x, y) in [(0, 0), (8, 0), (0, 33), (8, 33)] {
        d.pixel(x, y, 128).map_err(|_| "FAIL: valid corner\r\n")?;
        let (r, p) = (d.calc_pixel)(x, y);
        expected[r as usize + if p == 0 { 0 } else { 180 }] = 128;
    }
    for (x, y) in [(9, 0), (0, 34), (9, 34), (255, 255)] {
        if !matches!(
            d.pixel(x, y, 255),
            Err(is31fl3741::Error::InvalidLocation(_))
        ) {
            return Err("FAIL: invalid coordinate\r\n");
        }
    }
    read(d, 0, &mut actual[..180])?;
    read(d, 1, &mut actual[180..])?;
    if actual != expected {
        return Err("FAIL: bounds readback\r\n");
    }
    if mode == b'C' {
        return Ok(());
    }
    d.sw_enablement(is31fl3741::SwSetting::Sw1Sw8)
        .map_err(|_| "FAIL: rows\r\n")?;
    let mut config = [0];
    read(d, 4, &mut config)?;
    let before = config[0];
    if before & 0xf1 != 0x11 {
        return Err("FAIL: initial config\r\n");
    }
    for shutdown in [true, false] {
        d.shutdown(shutdown)
            .map_err(|_| "FAIL: shutdown write\r\n")?;
        read(d, 4, &mut config)?;
        if config[0] != (before & !1) | u8::from(!shutdown) {
            return Err("FAIL: config preservation\r\n");
        }
    }
    read(d, 0, &mut actual[..180])?;
    read(d, 1, &mut actual[180..])?;
    if actual != expected {
        return Err("FAIL: resume PWM\r\n");
    }
    Ok(())
}

fn config_byte<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    register: u8,
) -> Result<u8, &'static str> {
    for bytes in [&[0xfe, 0xc5][..], &[0xfd, 4], &[register]] {
        d.i2c
            .write(d.address, bytes)
            .map_err(|_| "FAIL: API select\r\n")?;
    }
    let mut value = [0];
    d.i2c
        .read(d.address, &mut value)
        .map_err(|_| "FAIL: API read\r\n")?;
    Ok(value[0])
}
fn check_apis<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
) -> Result<(), &'static str> {
    use is31fl3741::{LogicThreshold, Resistor};
    let gcc = config_byte(d, 1)?;
    for v in [0, 1, 32, 128, 255] {
        d.set_global_current(v).map_err(|_| "FAIL: GCC write\r\n")?;
        if config_byte(d, 1)? != v {
            return Err("FAIL: GCC readback\r\n");
        }
    }
    d.set_global_current(gcc)
        .map_err(|_| "FAIL: GCC restore\r\n")?;
    let resistors = [
        Resistor::Disabled,
        Resistor::Ohms500,
        Resistor::KiloOhms1,
        Resistor::KiloOhms2,
        Resistor::KiloOhms4,
        Resistor::KiloOhms8,
        Resistor::KiloOhms16,
        Resistor::KiloOhms32,
    ];
    for down in resistors {
        for up in resistors {
            d.set_deghost_resistors(down, up)
                .map_err(|_| "FAIL: resistor write\r\n")?;
            if config_byte(d, 2)? != (down as u8) << 4 | up as u8 {
                return Err("FAIL: resistor readback\r\n");
            }
        }
    }
    d.set_deghost_resistors(Resistor::KiloOhms8, Resistor::KiloOhms8)
        .map_err(|_| "FAIL: resistor restore\r\n")?;
    let config = config_byte(d, 0)?;
    for threshold in [LogicThreshold::High2V4Low0V6, LogicThreshold::High1V4Low0V4] {
        d.set_logic_threshold(threshold)
            .map_err(|_| "FAIL: logic write\r\n")?;
        if config_byte(d, 0)? != (config & !8) | threshold as u8 {
            return Err("FAIL: logic readback\r\n");
        }
    }
    for open in [false, true] {
        for short in [false, true] {
            for clear in [false, true] {
                d.set_interrupt_config(open, short, clear)
                    .map_err(|_| "FAIL: IRQ ACK\r\n")?;
            }
        }
    }
    d.set_interrupt_config(false, false, false)
        .map_err(|_| "FAIL: IRQ restore\r\n")?;
    let _status = d
        .read_interrupt_status()
        .map_err(|_| "FAIL: IRQ status read\r\n")?;
    Ok(())
}
