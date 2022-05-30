#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32g4xx_hal::gpio::GpioExt;
use stm32g4xx_hal::pac;
use stm32g4xx_hal::rcc::RccExt;
//use stm32g4xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa1.into_push_pull_output();
    loop {
        for _ in 0..100_000 {
            led.set_low();
        }
        for _ in 0..100_000 {
            led.set_high();
        }
    }
}
