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
    let mut intb = pins.gpio28.into_pull_up_input();

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

    let mut optical_saved: Option<Snapshot> = None;
    loop {
        usb_dev.poll(&mut [&mut serial]);
        let mut buf = [0; 64];
        if let Ok(count) = serial.read(&mut buf) {
            for &byte in &buf[..count] {
                if matches!(
                    byte,
                    b'T' | b'F'
                        | b'C'
                        | b'S'
                        | b'A'
                        | b'G'
                        | b'R'
                        | b'L'
                        | b'M'
                        | b'I'
                        | b'X'
                        | b'0'
                        | b'1'
                        | b'2'
                        | b'3'
                        | b'4'
                        | b'5'
                        | b'6'
                        | b'Z'
                        | b'P'
                        | b'E'
                        | b'Q'
                ) {
                    let commands: &[u8] = if byte == b'A' {
                        b"GRLMI"
                    } else if byte == b'E' {
                        b"efgh"
                    } else if byte == b'Q' {
                        b"jkl"
                    } else {
                        core::slice::from_ref(&byte)
                    };
                    for &command in commands {
                        let mut timed_report = TextReport::new();
                        let result =
                            if matches!(command, b'e' | b'f' | b'g' | b'h' | b'j' | b'k' | b'l') {
                                observe_interrupt(
                                    &mut matrix.device,
                                    command,
                                    &mut delay,
                                    &mut intb,
                                    &mut timed_report,
                                )
                            } else if command == b'P' {
                                let start = delay.get_counter().ticks();
                                let mut checksum = 0u32;
                                for _ in 0..100 {
                                    for y in 0..34 {
                                        for x in 0..9 {
                                            let (register, page) =
                                                core::hint::black_box(CALC_PIXEL)(
                                                    core::hint::black_box(x),
                                                    core::hint::black_box(y),
                                                );
                                            checksum = checksum.wrapping_add(
                                                u32::from(register) + u32::from(page) * 180,
                                            );
                                        }
                                    }
                                }
                                let elapsed = delay.get_counter().ticks() - start;
                                use core::fmt::Write;
                                let _ = write!(
                                    &mut timed_report,
                                    "PASS: mapping30600; us={}; checksum={}\r\n",
                                    elapsed, checksum
                                );
                                Ok(())
                            } else if command.is_ascii_digit() {
                                if optical_saved.is_none() {
                                    optical_saved = Snapshot::capture(&mut matrix.device).ok();
                                }
                                if optical_saved.is_none() {
                                    Err("FAIL: optical snapshot\r\n")
                                } else {
                                    optical_pattern(&mut matrix.device, command)
                                }
                            } else if command == b'Z' {
                                match optical_saved.as_ref() {
                                    Some(saved) => {
                                        let result = saved.restore(&mut matrix.device);
                                        if result.is_ok() {
                                            optical_saved = None;
                                        }
                                        result
                                    }
                                    None => Ok(()),
                                }
                            } else {
                                isolated_check(&mut matrix.device, command)
                            };
                        let report: &[u8] = match result {
                            Ok(())
                                if command == b'P'
                                    || matches!(
                                        command,
                                        b'e' | b'f' | b'g' | b'h' | b'j' | b'k' | b'l'
                                    ) =>
                            {
                                timed_report.as_bytes()
                            }
                            Ok(()) => match command {
                                b'0' => b"PASS: optical uniform GCC0; held\r\n",
                                b'1' => b"PASS: optical uniform GCC16; held\r\n",
                                b'2' => b"PASS: optical uniform GCC64; held\r\n",
                                b'3' => b"PASS: optical uniform GCC255; held\r\n",
                                b'4' => b"PASS: optical sparse resistor disabled; held\r\n",
                                b'5' => b"PASS: optical sparse resistor 8k; held\r\n",
                                b'6' => b"PASS: optical sparse resistor 32k; held\r\n",
                                b'Z' => b"PASS: optical state restored\r\n",
                                b'G' => b"PASS: GCC; state restored\r\n",
                                b'R' => b"PASS: resistors; state restored\r\n",
                                b'L' => b"PASS: logic; state restored\r\n",
                                b'M' => b"PASS: IRQ mask ACK only; state restored\r\n",
                                b'I' => b"PASS: IRQ status access only; state restored\r\n",
                                b'F' => b"PASS: PWM351; state restored\r\n",
                                b'C' => b"PASS: bounds; state restored\r\n",
                                _ => b"PASS: PWM351 bounds shutdown resume; state restored\r\n",
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
    mode: u8,
) -> Result<(), &'static str> {
    use is31fl3741::{LogicThreshold, Resistor};
    if mode == b'G' {
        let gcc = config_byte(d, 1)?;
        for v in [0, 1, 32, 128, 255] {
            d.set_global_current(v).map_err(|_| "FAIL: GCC write\r\n")?;
            if config_byte(d, 1)? != v {
                return Err("FAIL: GCC readback\r\n");
            }
        }
        d.set_global_current(gcc)
            .map_err(|_| "FAIL: GCC restore\r\n")?;
    }
    if mode == b'R' {
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
    }
    if mode == b'L' {
        let config = config_byte(d, 0)?;
        for threshold in [LogicThreshold::High2V4Low0V6, LogicThreshold::High1V4Low0V4] {
            d.set_logic_threshold(threshold)
                .map_err(|_| "FAIL: logic write\r\n")?;
            if config_byte(d, 0)? != (config & !8) | threshold as u8 {
                return Err("FAIL: logic readback\r\n");
            }
        }
    }
    if mode == b'M' {
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
    }
    if mode == b'I' {
        let _status = d
            .read_interrupt_status()
            .map_err(|_| "FAIL: IRQ status read\r\n")?;
    }
    Ok(())
}

// Snapshot only registers touched by these diagnostics. Scaling remains unchanged.
struct Snapshot {
    pwm: [u8; 351],
    scaling: [u8; 351],
    config: u8,
    gcc: u8,
    resistors: u8,
    frequency: u8,
}
fn raw_write<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    register: u8,
    value: u8,
) -> Result<(), &'static str> {
    for bytes in [&[0xfe, 0xc5][..], &[0xfd, 4], &[register, value]] {
        d.i2c
            .write(d.address, bytes)
            .map_err(|_| "FAIL: restore write\r\n")?;
    }
    Ok(())
}
impl Snapshot {
    fn capture<I: embedded_hal::i2c::I2c>(
        d: &mut is31fl3741::IS31FL3741<I>,
    ) -> Result<Self, &'static str> {
        let mut pwm = [0; 351];
        read(d, 0, &mut pwm[..180])?;
        read(d, 1, &mut pwm[180..])?;
        let mut scaling = [0; 351];
        read(d, 2, &mut scaling[..180])?;
        read(d, 3, &mut scaling[180..])?;
        Ok(Self {
            pwm,
            scaling,
            config: config_byte(d, 0)?,
            gcc: config_byte(d, 1)?,
            resistors: config_byte(d, 2)?,
            frequency: config_byte(d, 0x36)?,
        })
    }
    fn restore<I: embedded_hal::i2c::I2c>(
        &self,
        d: &mut is31fl3741::IS31FL3741<I>,
    ) -> Result<(), &'static str> {
        // Attempt every cleanup operation even if an earlier one fails. Interrupt mask
        // is write-only: this diagnostic owns it and always returns it to disabled.
        let results = [
            raw_write(d, 1, 0),
            raw_write(d, 2, self.resistors),
            raw_write(d, 0x36, self.frequency),
            raw_write(d, 0, self.config),
            d.fill_matrix(&self.pwm)
                .map_err(|_| "FAIL: restore PWM\r\n"),
            restore_scaling(d, &self.scaling),
            raw_write(d, 1, self.gcc),
            d.set_interrupt_config(false, false, false)
                .map_err(|_| "FAIL: restore mask\r\n"),
        ];
        if results.iter().any(Result::is_err) {
            return Err("FAIL: cleanup bus error; state uncertain\r\n");
        }
        let actual = Self::capture(d)
            .map_err(|_| "FAIL: cleanup verification bus error; state uncertain\r\n")?;
        if actual.pwm != self.pwm
            || actual.scaling != self.scaling
            || actual.config != self.config
            || actual.gcc != self.gcc
            || actual.resistors != self.resistors
            || actual.frequency != self.frequency
        {
            return Err("FAIL: cleanup readback; state uncertain\r\n");
        }
        Ok(())
    }
}
fn isolated_check<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    mode: u8,
) -> Result<(), &'static str> {
    let saved = Snapshot::capture(d)?;
    let result = if mode == b'X' {
        (|| {
            d.set_global_current(0)
                .map_err(|_| "FAIL: injection write\r\n")?;
            raw_write(d, 2, 0)?;
            raw_write(d, 0, 0x10)?;
            d.fill(42).map_err(|_| "FAIL: injection frame\r\n")?;
            Err("FAIL: deliberate injection; state restored\r\n")
        })()
    } else if matches!(mode, b'G' | b'R' | b'L' | b'M' | b'I') {
        check_apis(d, mode)
    } else {
        check(d, mode)
    };
    saved.restore(d)?;
    result
}

// Optical patterns intentionally remain displayed until the next pattern or Z.
fn optical_pattern<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    mode: u8,
) -> Result<(), &'static str> {
    use is31fl3741::Resistor;
    d.fill(0).map_err(|_| "FAIL: optical clear\r\n")?;
    d.set_deghost_resistors(Resistor::KiloOhms8, Resistor::KiloOhms8)
        .map_err(|_| "FAIL: optical resistor\r\n")?;
    if mode <= b'3' {
        let gcc = [0, 16, 64, 255][(mode - b'0') as usize];
        d.set_global_current(gcc)
            .map_err(|_| "FAIL: optical GCC\r\n")?;
        d.fill(128).map_err(|_| "FAIL: optical fill\r\n")?;
    } else {
        let resistor = match mode {
            b'4' => Resistor::Disabled,
            b'5' => Resistor::KiloOhms8,
            _ => Resistor::KiloOhms32,
        };
        d.set_global_current(255)
            .map_err(|_| "FAIL: optical GCC\r\n")?;
        d.set_deghost_resistors(resistor, resistor)
            .map_err(|_| "FAIL: optical resistor\r\n")?;
        for y in (1..34).step_by(4) {
            d.pixel(4, y, 255).map_err(|_| "FAIL: optical pixel\r\n")?;
        }
    }
    Ok(())
}

struct TextReport {
    bytes: [u8; 384],
    length: usize,
}
impl TextReport {
    fn new() -> Self {
        Self {
            bytes: [0; 384],
            length: 0,
        }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
}
impl core::fmt::Write for TextReport {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if self.length + s.len() > self.bytes.len() {
            return Err(core::fmt::Error);
        }
        self.bytes[self.length..self.length + s.len()].copy_from_slice(s.as_bytes());
        self.length += s.len();
        Ok(())
    }
}

// Observations only: does not assume detection completion timing or a known short.
fn observe_interrupt<I: embedded_hal::i2c::I2c, D: DelayNs, P: InputPin>(
    d: &mut is31fl3741::IS31FL3741<I>,
    mode: u8,
    delay: &mut D,
    pin: &mut P,
    report: &mut TextReport,
) -> Result<(), &'static str> {
    use core::fmt::Write;
    let saved = Snapshot::capture(d)?;
    let result = (|| {
        let short = mode == b'h';
        let enabled = mode != b'e';
        let auto = mode == b'g';
        d.set_interrupt_config(false, false, false)
            .map_err(|_| "FAIL: IRQ baseline mask\r\n")?;
        let initial = d
            .read_interrupt_status()
            .map_err(|_| "FAIL: IRQ initial status\r\n")?;
        let initial_low = pin.is_low().map_err(|_| "FAIL: INTB input\r\n")?;
        let scaling = if matches!(mode, b'j' | b'l') { 50 } else { 255 };
        let pwm = if matches!(mode, b'k' | b'l') {
            128
        } else {
            255
        };
        d.set_scaling(scaling)
            .map_err(|_| "FAIL: detect scaling\r\n")?;
        d.fill(pwm).map_err(|_| "FAIL: detect PWM\r\n")?;
        d.set_global_current(1)
            .map_err(|_| "FAIL: detect GCC\r\n")?;
        let config = config_byte(d, 0)? & !6;
        raw_write(d, 0, config)?;
        d.set_interrupt_config(enabled && !short, enabled && short, auto)
            .map_err(|_| "FAIL: detect mask\r\n")?;
        raw_write(d, 0, config | if short { 4 } else { 2 })?;
        delay.delay_ms(1);
        let low1 = pin.is_low().map_err(|_| "FAIL: INTB input\r\n")?;
        delay.delay_ms(11);
        let low12 = pin.is_low().map_err(|_| "FAIL: INTB input\r\n")?;
        let mut bitmap = [0; 45];
        for bytes in [&[0xfe, 0xc5][..], &[0xfd, 4], &[3]] {
            d.i2c
                .write(d.address, bytes)
                .map_err(|_| "FAIL: bitmap select\r\n")?;
        }
        d.i2c
            .read(d.address, &mut bitmap)
            .map_err(|_| "FAIL: bitmap read\r\n")?;
        delay.delay_ms(88);
        let low100 = pin.is_low().map_err(|_| "FAIL: INTB input\r\n")?;
        let status = d
            .read_interrupt_status()
            .map_err(|_| "FAIL: detect status\r\n")?;
        let low_read = pin.is_low().map_err(|_| "FAIL: INTB input\r\n")?;
        let status2 = d
            .read_interrupt_status()
            .map_err(|_| "FAIL: detect second status\r\n")?;
        let count: u32 = bitmap
            .iter()
            .enumerate()
            .map(|(i, v)| (v & if i % 5 == 4 { 0x7f } else { 0xff }).count_ones())
            .sum();
        let bits = |v: is31fl3741::InterruptStatus| u8::from(v.open) | u8::from(v.short) << 1;
        write!(report,"OBS: {} enabled={} auto={} initial={}/{} scaling={} pwm={} low1={} low12={} low100={} status={}/{} low_after_read={} faults={} bitmap=",if short {"short"} else {"open"},u8::from(enabled),u8::from(auto),bits(initial),u8::from(initial_low),scaling,pwm,u8::from(low1),u8::from(low12),u8::from(low100),bits(status),bits(status2),u8::from(low_read),count).map_err(|_| "FAIL: report size\r\n")?;
        for v in bitmap {
            write!(report, "{:02x}", v).map_err(|_| "FAIL: report size\r\n")?;
        }
        report
            .write_str("; state restored\r\n")
            .map_err(|_| "FAIL: report size\r\n")?;
        Ok(())
    })();
    saved.restore(d)?;
    result
}

fn restore_scaling<I: embedded_hal::i2c::I2c>(
    d: &mut is31fl3741::IS31FL3741<I>,
    values: &[u8; 351],
) -> Result<(), &'static str> {
    let mut payload = [0; 181];
    for (page, start, length) in [(2, 0, 180), (3, 180, 171)] {
        payload[1..=length].copy_from_slice(&values[start..start + length]);
        for bytes in [&[0xfe, 0xc5][..], &[0xfd, page], &payload[..=length]] {
            d.i2c
                .write(d.address, bytes)
                .map_err(|_| "FAIL: restore scaling\r\n")?;
        }
    }
    Ok(())
}
