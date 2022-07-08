pub mod counter;

use crate::pac;
use crate::rcc::{Enable, Reset};
use crate::timer::counter::Counter;

pub struct Timer<TIM> {
    tim: TIM,
    //pub clk
}

impl<TIM> Timer<TIM>
where
    TIM: Enable + Reset,
{
    fn new(tim: TIM) -> Self {
        let rcc = unsafe { &(*pac::RCC::ptr()) };
        <TIM as Enable>::enable(rcc);
        TIM::reset(rcc);
        Self { tim }
    }
}

impl<TIM> Timer<TIM> {
    fn release(self) -> TIM {
        self.tim
    }
}

pub trait TimerExt<TIM>
where
    TIM: sealed::General,
{
    fn counter(self) -> Counter<TIM>;
    fn timer(self) -> Timer<TIM>;
}

impl<TIM> TimerExt<TIM> for TIM
where
    TIM: sealed::General + Enable + Reset,
{
    fn counter(self) -> Counter<TIM> {
        Counter::new(self)
    }
    fn timer(self) -> Timer<TIM> {
        Timer::new(self)
    }
}

mod sealed {
    use crate::timer::Direction;

    pub trait General {
        type Width;

        fn enable_counter(&mut self);
        fn read_count(&self) -> Self::Width;
        fn write_auto_reload(&mut self, value: Self::Width);
        fn read_auto_reload(&self) -> Self::Width;
        fn enable(&mut self);
    }
    pub trait SettableDirection {
        fn write_direction(&mut self, dir: Direction);
    }
}

pub enum Direction {
    Up,
    Down,
}

// impl General for pac::TIM2 {
//     fn enable_counter(&mut self) {
//         self.cr1.modify(|_, w| w.cen().set_bit());
//     }
//     fn read_count(&self) -> u32 {
//         self.cnt.read().bits() as u32
//     }
//     fn write_auto_reload(&mut self, value: u32) {
//         unsafe { self.arr.write(|w| w.bits(value)) }
//     }
//     fn enable(&mut self) {
//         self.cr1.modify(|_, w| w.cen().set_bit())
//     }
//     fn read_auto_reload(&self) -> u32 {
//         self.arr.read().bits()
//     }
// }

macro_rules! hal {
    ($TIM:ty, $bits:ty) => {
        impl sealed::General for $TIM {
            type Width = $bits;

            fn enable_counter(&mut self) {
                self.cr1.modify(|_, w| w.cen().set_bit());
            }
            fn read_count(&self) -> Self::Width {
                self.cnt.read().bits() as Self::Width
            }
            fn write_auto_reload(&mut self, value: Self::Width) {
                unsafe { self.arr.write(|w| w.bits(value as u32)) }
            }
            fn enable(&mut self) {
                self.cr1.modify(|_, w| w.cen().set_bit())
            }
            fn read_auto_reload(&self) -> Self::Width {
                self.arr.read().bits() as Self::Width
            }
        }
    };
}
macro_rules! settable_direction {
    ($TIM:ty) => {
        impl sealed::SettableDirection for $TIM {
            // Set the counting direction of the counter
            // Up => 0..reload_value
            // Down => reload_value..0
            fn write_direction(&mut self, dir: Direction) {
                match dir {
                    Direction::Down => self.cr1.modify(|_, w| w.dir().set_bit()),
                    Direction::Up => self.cr1.modify(|_, w| w.dir().clear_bit()),
                }
            }
        }
    };
}

hal!(pac::TIM2, u32);
settable_direction!(pac::TIM2);
hal!(pac::TIM3, u16);
settable_direction!(pac::TIM3);
hal!(pac::TIM4, u16);
settable_direction!(pac::TIM4);
#[cfg(not(feature = "stm32g431"))]
hal!(pac::TIM5, u32);

hal!(pac::TIM6, u16);
hal!(pac::TIM7, u16);

hal!(pac::TIM15, u16);
hal!(pac::TIM16, u16);
hal!(pac::TIM17, u16);

// TODO impl low power timer with counter
//hal!(pac::LPTIMER1, u16);
