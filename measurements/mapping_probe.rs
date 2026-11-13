#![no_std]
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
struct UnusedBus;
impl ErrorType for UnusedBus { type Error = ErrorKind; }
impl I2c for UnusedBus {
    fn transaction(&mut self, _:u8, _:&mut [Operation<'_>]) -> Result<(),ErrorKind> { panic!("mapping probe must not access I2C") }
}
#[no_mangle]
#[inline(never)]
pub fn probe_framework(x:u8,y:u8) -> (u8,u8) { (is31fl3741::devices::CALC_PIXEL)(x,y) }
#[no_mangle]
#[inline(never)]
pub fn probe_adafruit(pixel:u8,channel:u8) -> (u8,u8) {
    let d=is31fl3741::devices::AdafruitRGB13x9::configure(UnusedBus);
    (d.device.calc_pixel)(pixel,channel)
}
