use crate::gpio::sealed::PinMode;
use crate::pac::RCC;
use crate::rcc::{Enable, Reset};
use core::marker::PhantomData;
use embedded_hal::digital::v2::PinState;

pub trait GpioExt {
    type Parts;

    fn split(self) -> Self::Parts;
}

pub enum IOOutputSpeed {
    Low = 0b00,
    Medium = 0b01,
    High = 0b10,
    VeryHigh = 0b11,
}

pub trait OutputSpeed {
    fn set_speed(&mut self, speed: IOOutputSpeed);
}

#[derive(Default)]
pub struct OpenDrain;
#[derive(Default)]
pub struct PushPull;

#[derive(Default)]
pub struct Floating;
#[derive(Default)]
pub struct PullUp;
#[derive(Default)]
pub struct PullDown;

// (GPIOx_MODER) MODE
#[derive(Default)]
pub struct Analog;
#[derive(Default)]
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}
#[derive(Default)]
pub struct Output<MODE = PushPull> {
    _mode: PhantomData<MODE>,
}
#[derive(Default)]
pub struct Input<MODE = Floating> {
    _mode: PhantomData<MODE>,
}

mod sealed {
    pub trait PinMode {
        // (GPIOx_MODER)
        const MODE: u32;
        // (GPIOx_OTYPER)
        const TYPE: u32;
        // (GPIOx_PUPDR)
        const PULL: u32;
    }
}

impl PinMode for Analog {
    const MODE: u32 = 0b11;
    const TYPE: u32 = 0b00;
    const PULL: u32 = 0b00;
}

impl PinMode for Input<Floating> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b00;
    const PULL: u32 = 0b00;
}

impl PinMode for Input<PullUp> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b00;
    const PULL: u32 = 0b01;
}

impl PinMode for Input<PullDown> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b00;
    const PULL: u32 = 0b10;
}

impl PinMode for Output<PushPull> {
    const MODE: u32 = 0b01;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b00;
}

impl PinMode for Output<OpenDrain> {
    const MODE: u32 = 0b01;
    const TYPE: u32 = 0b1;
    const PULL: u32 = 0b00;
}

pub struct Pin<const P: char, const N: u8, MODE = Analog> {
    mode: MODE,
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    fn mode<NewMode: PinMode>(&mut self) {
        let gpio = unsafe { &(*Gpio::<P>::ptr()) };
        unsafe {
            // Set the mode in the register N*2 as there is two bits to configure
            gpio.moder
                .modify(|r, w| w.bits((r.bits() & !(0b11 << N * 2)) | (NewMode::MODE << N * 2)));
            // Write the config into the type register
            gpio.otyper
                .modify(|r, w| w.bits((r.bits() & !(0b1 << N)) | (NewMode::TYPE << N)));
            // Write the config into the pull-up/pull-down register
            gpio.pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << N * 2)) | (NewMode::PULL << N * 2)));
        };
    }

    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }

    #[inline]
    fn _set_high(&mut self) {
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) }
    }

    #[inline]
    fn _set_low(&mut self) {
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << 16 + N)) }
    }

    #[inline]
    pub fn into_push_pull_output(self) -> Pin<P, N, Output<PushPull>> {
        self.into_push_pull_output_with_state(PinState::Low)
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    #[inline]
    pub fn into_push_pull_output_with_state(
        mut self,
        initial_state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_state(initial_state);
        self.mode::<Output<PushPull>>();
        Pin::new()
    }
}

// TODO: try to make this less ugly
struct Gpio<const P: char>;
impl<const P: char> Gpio<P> {
    const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
        match P {
            'A' => crate::pac::GPIOA::ptr(),
            'B' => crate::pac::GPIOB::ptr() as _,
            'C' => crate::pac::GPIOC::ptr() as _,
            'D' => crate::pac::GPIOD::ptr() as _,
            'E' => crate::pac::GPIOE::ptr() as _,
            'F' => crate::pac::GPIOF::ptr() as _,
            'G' => crate::pac::GPIOG::ptr() as _,
            _ => crate::pac::GPIOA::ptr(),
        }
    }
}

pub type PA1<MODE = Output<PushPull>> = Pin<'A', 1, MODE>;

/// GPIO parts
pub struct Parts {
    /// Pin
    pub pa1: PA1,
}

impl GpioExt for crate::pac::GPIOA {
    type Parts = Parts;

    fn split(self) -> Parts {
        let rcc = unsafe { &(*RCC::ptr()) };
        crate::pac::GPIOA::enable(rcc);
        crate::pac::GPIOA::reset(rcc);

        Parts { pa1: PA1::new() }
    }
}

impl<const P: char, const N: u8, MODE: Default> Pin<P, N, MODE> {
    fn new() -> Self {
        Self {
            mode: Default::default(),
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline]
    pub fn set_high(&mut self) {
        self._set_high()
    }
    #[inline]
    pub fn set_low(&mut self) {
        self._set_low()
    }
}
