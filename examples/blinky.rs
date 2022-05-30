#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use stm32g4xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac

    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa2.into_push_pull_output();

    loop {
        info!("Set Led low");
        for _ in 0..100_000 {
            led.set_low().unwrap();
        }
        info!("Set Led High");
        for _ in 0..100_000 {
            led.set_high().unwrap();
        }
    }
}
