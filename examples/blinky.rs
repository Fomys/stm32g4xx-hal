#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::blocking::OutputPin;
use stm32g4xx_hal::gpio::{GpioExt, Output, Pin, PushPull};
use stm32g4xx_hal::pac;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa1.into_output_push_pull();
    loop {
        for _ in 0..100_000 {
            led.set_low().unwrap();
        }
        for _ in 0..100_000 {
            led.set_high().unwrap();
        }
    }
}
