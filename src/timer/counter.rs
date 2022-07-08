use crate::pac;
use crate::rcc::{Enable, Reset};
use crate::timer::sealed::{General, SettableDirection};
use crate::timer::Direction;

pub struct Counter<TIM: General> {
    tim: TIM,
}

impl<TIM> Counter<TIM>
where
    TIM: General + SettableDirection,
{
    /// Set the counting direction of the counter
    /// Up => 0..reload_value
    /// Down => reload_value..0
    pub fn set_direction(&mut self, dir: Direction) {
        self.tim.write_direction(dir)
    }
}

impl<TIM> Counter<TIM>
where
    TIM: General + Enable + Reset,
{
    pub fn new(tim: TIM) -> Self {
        let rcc = unsafe { &(*pac::RCC::ptr()) };
        <TIM as Enable>::enable(rcc);
        TIM::reset(rcc);
        Self { tim }
    }

    pub fn release(self) -> TIM {
        self.tim
    }

    pub fn enable(&mut self) {
        self.tim.enable();
    }

    pub fn write_auto_reload(&mut self, value: TIM::Width) {
        self.tim.write_auto_reload(value);
    }

    pub fn read_auto_reload(&self) -> TIM::Width {
        self.tim.read_auto_reload()
    }

    pub fn read_count(&self) -> TIM::Width {
        self.tim.read_count()
    }
}
