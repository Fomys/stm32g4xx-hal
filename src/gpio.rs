use crate::gpio::sealed::PinMode;
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::blocking::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::digital::{ErrorType, PinState};

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
pub struct Alternate<const A: u8, MODE = PushPull> {
    _mode: PhantomData<MODE>,
}

/// JTAG/SWD mote (type state)
pub type Debugger = Alternate<0, PushPull>;

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
        // (GPIOx_AFRL)
        const AFRL: u32;
        // (GPIOx_AFRH)
        const AFRH: u32;
    }
}

impl PinMode for Analog {
    const MODE: u32 = 0b11;
    const TYPE: u32 = 0b00;
    const PULL: u32 = 0b0;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl PinMode for Input<Floating> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b00;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl PinMode for Input<PullUp> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b01;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl PinMode for Input<PullDown> {
    const MODE: u32 = 0b00;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b10;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl PinMode for Output<PushPull> {
    const MODE: u32 = 0b01;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b00;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl PinMode for Output<OpenDrain> {
    const MODE: u32 = 0b01;
    const TYPE: u32 = 0b1;
    const PULL: u32 = 0b00;
    const AFRL: u32 = 0b0000;
    const AFRH: u32 = 0b0000;
}

impl<const I: u8> PinMode for Alternate<I, PushPull> {
    const MODE: u32 = 0b10;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b00;
    const AFRL: u32 = I as u32;
    const AFRH: u32 = I as u32;
}

impl<const I: u8> PinMode for Alternate<I, OpenDrain> {
    const MODE: u32 = 0b10;
    const TYPE: u32 = 0b1;
    const PULL: u32 = 0b00;
    const AFRL: u32 = I as u32;
    const AFRH: u32 = I as u32;
}

impl<const I: u8> PinMode for Alternate<I, PullUp> {
    const MODE: u32 = 0b10;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b01;
    const AFRL: u32 = I as u32;
    const AFRH: u32 = I as u32;
}

impl<const I: u8> PinMode for Alternate<I, PullDown> {
    const MODE: u32 = 0b10;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b10;
    const AFRL: u32 = I as u32;
    const AFRH: u32 = I as u32;
}

impl<const I: u8> PinMode for Alternate<I, Floating> {
    const MODE: u32 = 0b10;
    const TYPE: u32 = 0b0;
    const PULL: u32 = 0b00;
    const AFRL: u32 = I as u32;
    const AFRH: u32 = I as u32;
}

pub struct Pin<const P: char, const N: u8, MODE = Analog> {
    _mode: PhantomData<MODE>,
}

// impl<const P: char, const N: u8> From<Pin<P, N, Analog>> for Pin<P, N, Output<PushPull>> {
//     fn from(p: Pin<P, N, Analog>) -> Self {
//         let gpio = unsafe { &(*Gpio::<P>::ptr()) };
//         unsafe {
//             // Set the mode in the register N*2 as there is two bits to configure
//             gpio.moder.modify(|r, w| {
//                 w.bits((r.bits() & !(0b11 << N * 2)) | (Output::<PushPull>::MODE << N * 2))
//             });
//             // Write the config into the type register
//             gpio.otyper
//                 .modify(|r, w| w.bits((r.bits() & !(0b1 << N)) | (Output::<PushPull>::TYPE << N)));
//             // Write the config into the pull-up/pull-down register
//             gpio.pupdr.modify(|r, w| {
//                 w.bits((r.bits() & !(0b11 << N * 2)) | (Output::<PushPull>::PULL << N * 2))
//             });
//         };
//         Self {
//             _mode: Default::default(),
//         }
//     }
// }

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    // This should be done with From/Into trait so this implementation is due to change at some point
    fn mode<NewMode: PinMode>(self) -> Pin<P, N, NewMode> {
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

            // TODO: to test
            if N < 8 {
                // Write the config into the alternate function low register
                gpio.afrl.modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << N * 4)) | (NewMode::AFRL << N * 4))
                });
            } else {
                // Write the config into the alternate function high register
                gpio.afrl.modify(|r, w| {
                    w.bits((r.bits() & !(0b1111 << (N - 8) * 4)) | (NewMode::AFRH << (N - 8) * 4))
                });
            }
        };
        Pin {
            _mode: Default::default(),
        }
    }

    pub fn into_analog(self) -> Pin<P, N, Analog> {
        self.mode()
    }

    pub fn into_input_floating(self) -> Pin<P, N, Input<Floating>> {
        self.mode()
    }
    pub fn into_input_pull_up(self) -> Pin<P, N, Input<PullUp>> {
        self.mode()
    }
    pub fn into_input_pull_down(self) -> Pin<P, N, Input<PullDown>> {
        self.mode()
    }
    pub fn into_output_push_pull(self) -> Pin<P, N, Output<PushPull>> {
        self.mode()
    }
    pub fn into_output_open_drain(self) -> Pin<P, N, Output<OpenDrain>> {
        self.mode()
    }

    /// This implementation is subject to change because all pins can not be assigned to all alternate functions
    pub fn into_alternate_floating<const I: u8>(self) -> Pin<P, N, Alternate<I, Floating>> {
        self.mode()
    }

    /// This implementation is subject to change because all pins can not be assigned to all alternate functions
    pub fn into_alternate_pull_down<const I: u8>(self) -> Pin<P, N, Alternate<I, PullDown>> {
        self.mode()
    }

    /// This implementation is subject to change because all pins can not be assigned to all alternate functions
    pub fn into_alternate_pull_up<const I: u8>(self) -> Pin<P, N, Alternate<I, PullUp>> {
        self.mode()
    }

    /// This implementation is subject to change because all pins can not be assigned to all alternate functions
    pub fn into_alternate_push_pull<const I: u8>(self) -> Pin<P, N, Alternate<I, PushPull>> {
        self.mode()
    }

    /// This implementation is subject to change because all pins can not be assigned to all alternate functions
    pub fn into_alternate_open_drain<const I: u8>(self) -> Pin<P, N, Alternate<I, OpenDrain>> {
        self.mode()
    }

    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }

    #[inline(always)]
    fn _set_high(&mut self) {
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) }
    }

    #[inline(always)]
    fn _set_low(&mut self) {
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << 16 + N)) }
    }

    #[inline(always)]
    fn _is_set_high(&self) -> bool {
        unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << N) == 1 << N }
    }

    #[inline(always)]
    fn _is_high(&self) -> bool {
        unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << N) == 1 << N }
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

// /// GPIO parts
// pub struct Parts {
//     /// Pin
//     pub pa1: PA1,
// }
//
// impl GpioExt for crate::pac::GPIOA {
//     type Parts = Parts;
//
//     fn split(self) -> Parts {
//         let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
//         crate::pac::GPIOA::enable(rcc);
//         crate::pac::GPIOA::reset(rcc);
//
//         Parts { pa1: PA1::new() }
//     }
// }
macro_rules! from_pin {
    ($port_id: expr, $pin_number: expr, $head: ty, $($tail: ty, )+) => {
        $(
            impl core::convert::From<super::Pin<$port_id, $pin_number, $head>> for super::Pin<$port_id, $pin_number, $tail> {
                fn from(p: super::Pin<$port_id, $pin_number, $head>) -> Self {
                    p.mode()
                }
            }

            impl core::convert::From<super::Pin<$port_id, $pin_number, $tail>> for super::Pin<$port_id, $pin_number, $head> {
                fn from(p: super::Pin<$port_id, $pin_number, $tail>) -> Self {
                    p.mode()
                }
            }
        )+

        from_pin!($port_id, $pin_number, $($tail, )+);
    };
    ($port_id: expr, $pin_number: expr, $head: ty, ) => {};
}

// TODO: disable RCC on demand, the "inverse" of split
macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $port_id:expr, $PXn:ident, [
        $($PXi:ident: ($pxi:ident, $pin_number:expr, [$($alternate_id: expr ,)*] $(, $MODE:ty)?),)+
    ]) => {
        mod $gpiox {
             use crate::rcc::{Reset, Enable};
             pub struct Parts {
                    $(
                        /// Pin
                        pub $pxi: $PXi $(<$MODE>)?,
                    )+
                }

            $(
                pub type $PXi<MODE = super::Analog> = super::Pin<$port_id, $pin_number, MODE>;

                from_pin!($port_id, $pin_number,
                    super::Analog,
                    super::Output<super::PushPull>,
                    super::Output<super::OpenDrain>,
                    super::Input<super::Floating>,
                    super::Input<super::PullUp>,
                    super::Input<super::PullDown>,
                    $(
                        super::Alternate<$alternate_id, super::PushPull>,
                        super::Alternate<$alternate_id, super::OpenDrain>,
                        super::Alternate<$alternate_id, super::PullUp>,
                        super::Alternate<$alternate_id, super::PullDown>,
                        super::Alternate<$alternate_id, super::Floating>,
                    )*
                );
            )+

            impl super::GpioExt for crate::pac::$GPIOX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
                    crate::pac::$GPIOX::enable(rcc);
                    crate::pac::$GPIOX::reset(rcc);

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }
        }
    };
}
#[cfg(all(feature = "stm32g431"))] // PA0-PA15 are available on all packages
gpio!(GPIOA, gpioa, 'A', PAn, [
    PA0: (pa0, 0, [1, 7, 8, 9, 10, 14, 15, ]),
    PA1: (pa1, 1, [0, 1, 7, 9, 15, ]),
    PA2: (pa2, 2, [1, 7, 8, 9, 12, 14, 15, ]),
    PA3: (pa3, 3, [1, 3, 7, 9, 12, 13, 15, ]),
    PA4: (pa4, 4, [2, 5, 6, 7, 13, 15, ]),
    PA5: (pa5, 5, [1, 2, 5, 14, ]),
    PA6: (pa6, 6, [1, 2, 5, 6, 8, 12, 15, ]),
    PA7: (pa7, 7, [1, 2, 4, 5, 6, 8, 14, 15, ]),
    PA8: (pa8, 8, [0, 2, 4, 5, 6, 7, 10, 12, 14, 15, ]),
    PA9: (pa9, 9, [2, 4, 5, 6, 7, 9, 10, 14, 15, ]),
    PA10: (pa10, 10, [1, 3, 4, 5,6, 7, 10, 11, 12, 14, 15, ]),
    PA11: (pa11, 11, [5, 6, 7, 8, 9, 10, 11, 12, 15, ]),
    PA12: (pa12, 12, [1, 5, 6, 7, 8, 9, 10, 11, 15, ]),
    PA13: (pa13, 13, [0, 1, 4, 5, 7, 10, 13, 15, ], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 1, 4, 5, 6, 7, 13, 15, ], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 2, 4, 5, 6, 7, 8, 9, 14, ], super::Debugger), // JTDI, PullUp
]);

#[cfg(all(feature = "stm32g431", any(feature = "lqfp32", feature = "ufqfpn32")))]
gpio!(GPIOB, gpiob, 'B', PBn, [
    PB0: (pb0, 0, [2, 4, 6, 14, 15, ]),
    PB3: (pb3, 3, [0, 1, 2, 3, 4, 5, 6, 7, 10, 14, 15, ], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 4, 5, 6, 7, 10, 14, 15, ], super::Debugger), // JTRST, PullUp-l
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, ]),
    PB6: (pb6, 6, [1, 2, 5, 6, 7, 8, 10, 11, 14, 15, ]),
    PB7: (pb7, 7, [1, 2, 4, 5, 7, 8, 10, 11, 14, 15, ]),
    PB8: (pb8, 8, [1, 2, 3, 4, 7, 8, 9, 10, 12, 14, 15, ]),
]);

#[cfg(all(feature = "stm32g431", any(feature = "ufqfpn48", feature = "lqfp48", feature = "wlcsp49", feature = "lqfp64", feature = "ufbga64", feature = "lqfp80", feature = "lqfp100")))]
gpio!(GPIOB, gpiob, 'B', PBn, [
    PB0: (pb0, 0, [2, 4, 6, 14, 15, ]),
    PB1: (pb1, 0, [2, 4, 6, 8, 12, 15, ]),
    PB2: (pb2, 0, [0, 1, 4, 15, ]),
    PB3: (pb3, 3, [0, 1, 2, 3, 4, 5, 6, 7, 10, 14, 15, ], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 4, 5, 6, 7, 10, 14, 15, ], super::Debugger), // JTRST, PullUp-l
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, ]),
    PB6: (pb6, 6, [1, 2, 5, 6, 7, 8, 10, 11, 14, 15, ]),
    PB7: (pb7, 7, [1, 2, 4, 5, 7, 8, 10, 11, 14, 15, ]),
    PB8: (pb8, 8, [1, 2, 3, 4, 7, 8, 9, 10, 12, 14, 15, ]),
    PB9: (pb9, 9, [1, 2, 3, 4, 6, 7, 8, 9, 10, 12, 14, 15, ]),
    PB10: (pb10, 10, [1, 7, 8, 12, 14, 15, ]),
    PB11: (pb11, 11, [1, 7, 8, 15, ]),
    PB12: (pb12, 12, [4, 5, 6, 7, 8, 15, ]),
    PB13: (pb13, 13, [5, 6, 7, 8, 15, ]),
    PB14: (pb14, 14, [1, 5, 6, 7, 8, 15, ]),
    PB15: (pb15, 15, [0, 1, 2, 3, 4, 5, 15, ]),
]);


// gpio!(GPIOC, gpioc, 'C', PCn, [
//     PC0: (pc0, 0),
//     PC1: (pc1, 1),
//     PC2: (pc2, 2),
//     PC3: (pc3, 3),
//     PC4: (pc4, 4),
//     PC5: (pc5, 5),
//     PC6: (pc6, 6),
//     PC7: (pc7, 7),
//     PC8: (pc8, 8),
//     PC9: (pc9, 9),
//     PC10: (pc10, 10),
//     PC11: (pc11, 11),
//     PC12: (pc12, 12),
//     PC13: (pc13, 13),
//     PC14: (pc14, 14),
//     PC15: (pc15, 15),
// ]);
//
// gpio!(GPIOD, gpiod, 'D', PDn, [
//     PD0: (pd0, 0),
//     PD1: (pd1, 1),
//     PD2: (pd2, 2),
//     PD3: (pd3, 3),
//     PD4: (pd4, 4),
//     PD5: (pd5, 5),
//     PD6: (pd6, 6),
//     PD7: (pd7, 7),
//     PD8: (pd8, 8),
//     PD9: (pd9, 9),
//     PD10: (pd10, 10),
//     PD11: (pd11, 11),
//     PD12: (pd12, 12),
//     PD13: (pd13, 13),
//     PD14: (pd14, 14),
//     PD15: (pd15, 15),
// ]);
//
// gpio!(GPIOE, gpioe, 'E', PEn, [
//     PE0: (pe0, 0),
//     PE1: (pe1, 1),
//     PE2: (pe2, 2),
//     PE3: (pe3, 3),
//     PE4: (pe4, 4),
//     PE5: (pe5, 5),
//     PE6: (pe6, 6),
//     PE7: (pe7, 7),
//     PE8: (pe8, 8),
//     PE9: (pe9, 9),
//     PE10: (pe10, 10),
//     PE11: (pe11, 11),
//     PE12: (pe12, 12),
//     PE13: (pe13, 13),
//     PE14: (pe14, 14),
//     PE15: (pe15, 15),
// ]);
//
// gpio!(GPIOF, gpiof, 'F', PFn, [
//     PF0: (pf0, 0),
//     PF1: (pf1, 1),
//     PF2: (pf2, 2),
//     PF3: (pf3, 3),
//     PF4: (pf4, 4),
//     PF5: (pf5, 5),
//     PF6: (pf6, 6),
//     PF7: (pf7, 7),
//     PF8: (pf8, 8),
//     PF9: (pf9, 9),
//     PF10: (pf10, 10),
//     PF11: (pf11, 11),
//     PF12: (pf12, 12),
//     PF13: (pf13, 13),
//     PF14: (pf14, 14),
//     PF15: (pf15, 15),
// ]);
//
// gpio!(GPIOG, gpiog, 'G', PGn, [
//     PG0: (pg0, 0),
//     PG1: (pg1, 1),
//     PG2: (pg2, 2),
//     PG3: (pg3, 3),
//     PG4: (pg4, 4),
//     PG5: (pg5, 5),
//     PG6: (pg6, 6),
//     PG7: (pg7, 7),
//     PG8: (pg8, 8),
//     PG9: (pg9, 9),
//     PG10: (pg10, 10),
//     PG11: (pg11, 11),
//     PG12: (pg12, 12),
//     PG13: (pg13, 13),
//     PG14: (pg14, 14),
//     PG15: (pg15, 15),
// ]);

impl<const P: char, const N: u8, MODE: Default> Pin<P, N, MODE> {
    fn new() -> Self {
        Self {
            _mode: Default::default(),
        }
    }
}

impl<const P: char, const N: u8, MODE> ErrorType for Pin<P, N, MODE> {
    type Error = Infallible;
}

impl<const P: char, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    #[inline]
    fn set_low(&mut self) -> Result<(), Infallible> {
        self._set_low();
        Ok(())
    }
    #[inline]
    fn set_high(&mut self) -> Result<(), Infallible> {
        self._set_high();
        Ok(())
    }
}

// Able to read its state
impl<const P: char, const N: u8> StatefulOutputPin for Pin<P, N, Output<PushPull>> {
    #[inline(always)]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self._is_set_high())
    }
}

// Able to read its state
impl<const P: char, const N: u8, MODE> InputPin for Pin<P, N, Input<MODE>> {
    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self._is_high())
    }
}