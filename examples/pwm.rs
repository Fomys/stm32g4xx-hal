#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use hal::gpio::Alternate;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::gpio::{GpioExt, Pin, PushPull};
use stm32g4xx_hal::pac;
use stm32g4xx_hal::rcc::RccExt;
use stm32g4xx_hal::time::U32Ext;
use stm32g4xx_hal::timer::pwm::{PwmExt, PwmPin};

extern crate cortex_m_rt as rt;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split();

    let pin: Pin<'A', 8, Alternate<6, PushPull>> = gpioa.pa8.into_alternate_push_pull();

    let mut pwm = dp.TIM1.pwm(pin, 100.hz(), &mut rcc);

    pwm.enable();

    loop {
        for i in 0..pwm.get_max_duty() {
            pwm.set_duty(i);
        }
        for i in (0..pwm.get_max_duty()).rev() {
            pwm.set_duty(i);
        }
        cortex_m::asm::nop()
    }
}
