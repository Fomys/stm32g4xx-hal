#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::blocking::OutputPin;
use stm32g4xx_hal::gpio::GpioExt;
use stm32g4xx_hal::pac;
use stm32g4xx_hal::timer::{Direction, TimerExt};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut counter = dp.TIM2.counter();
    counter.write_auto_reload(10000000);
    counter.enable();
    counter.set_direction(Direction::Down);
    let gpioa = dp.GPIOA.split();
    let mut led = gpioa.pa1.into_output_push_pull();

    loop {
        if counter.read_count() < 10000000 / 2 {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
    }
}
