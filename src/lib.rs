#![no_std]

// If no target specified, print error message.
#[cfg(not(feature="device-selected"))]
compile_error!("Target not found. A `--features <device>` is required. It must be one of stm32g431, stm32g441, stm32g471, stm32g473, stm32g474, stm32g483, stm32g484, stm32g491 or stm32g4a1.");
#[cfg(not(feature="package-selected"))]
compile_error!("No package selected. A `--features <package>` is required. It must be one of ufqfpn32, lqfp32, ufqfpn48, lqfp48, wlcsp49, lqfp64, ufbga64, lqfp80 or lqfp100.");


// If any two or more targets are specified, print error message.
#[cfg(any(
    all(feature = "stm32g431", feature = "stm32g441"),
    all(feature = "stm32g431", feature = "stm32g471"),
    all(feature = "stm32g431", feature = "stm32g473"),
    all(feature = "stm32g431", feature = "stm32g474"),
    all(feature = "stm32g431", feature = "stm32g483"),
    all(feature = "stm32g431", feature = "stm32g484"),
    all(feature = "stm32g431", feature = "stm32g491"),
    all(feature = "stm32g431", feature = "stm32g4a1"),
    all(feature = "stm32g441", feature = "stm32g471"),
    all(feature = "stm32g441", feature = "stm32g473"),
    all(feature = "stm32g441", feature = "stm32g474"),
    all(feature = "stm32g441", feature = "stm32g483"),
    all(feature = "stm32g441", feature = "stm32g484"),
    all(feature = "stm32g441", feature = "stm32g491"),
    all(feature = "stm32g441", feature = "stm32g4a1"),
    all(feature = "stm32g471", feature = "stm32g473"),
    all(feature = "stm32g471", feature = "stm32g474"),
    all(feature = "stm32g471", feature = "stm32g483"),
    all(feature = "stm32g471", feature = "stm32g484"),
    all(feature = "stm32g471", feature = "stm32g491"),
    all(feature = "stm32g471", feature = "stm32g4a1"),
    all(feature = "stm32g473", feature = "stm32g474"),
    all(feature = "stm32g473", feature = "stm32g483"),
    all(feature = "stm32g473", feature = "stm32g484"),
    all(feature = "stm32g473", feature = "stm32g491"),
    all(feature = "stm32g473", feature = "stm32g4a1"),
    all(feature = "stm32g474", feature = "stm32g483"),
    all(feature = "stm32g474", feature = "stm32g484"),
    all(feature = "stm32g474", feature = "stm32g491"),
    all(feature = "stm32g474", feature = "stm32g4a1"),
    all(feature = "stm32g483", feature = "stm32g484"),
    all(feature = "stm32g483", feature = "stm32g491"),
    all(feature = "stm32g483", feature = "stm32g4a1"),
    all(feature = "stm32g484", feature = "stm32g491"),
    all(feature = "stm32g484", feature = "stm32g4a1"),
    all(feature = "stm32g491", feature = "stm32g4a1"),
))]
compile_error!(
    "Multiple devices specified. Only a single `--features <device>` can be specified."
);

#[cfg(any(
    all(feature="ufqfpn32", feature = "lqfp32"),
    all(feature="ufqfpn32", feature = "ufqfpn48"),
    all(feature="ufqfpn32", feature = "lqfp48"),
    all(feature="ufqfpn32", feature = "wlcsp49"),
    all(feature="ufqfpn32", feature = "lqfp64"),
    all(feature="ufqfpn32", feature = "ufbga64"),
    all(feature="ufqfpn32", feature = "lqfp80"),
    all(feature="ufqfpn32", feature = "lqfp100"),
    all(feature="lqfp32", feature = "ufqfpn48"),
    all(feature="lqfp32", feature = "lqfp48"),
    all(feature="lqfp32", feature = "wlcsp49"),
    all(feature="lqfp32", feature = "lqfp64"),
    all(feature="lqfp32", feature = "ufbga64"),
    all(feature="lqfp32", feature = "lqfp80"),
    all(feature="lqfp32", feature = "lqfp100"),
    all(feature="ufqfpn48", feature = "lqfp48"),
    all(feature="ufqfpn48", feature = "wlcsp49"),
    all(feature="ufqfpn48", feature = "lqfp64"),
    all(feature="ufqfpn48", feature = "ufbga64"),
    all(feature="ufqfpn48", feature = "lqfp80"),
    all(feature="ufqfpn48", feature = "lqfp100"),
    all(feature="lqfp48", feature = "wlcsp49"),
    all(feature="lqfp48", feature = "lqfp64"),
    all(feature="lqfp48", feature = "ufbga64"),
    all(feature="lqfp48", feature = "lqfp80"),
    all(feature="lqfp48", feature = "lqfp100"),
    all(feature="wlcsp49", feature = "lqfp64"),
    all(feature="wlcsp49", feature = "ufbga64"),
    all(feature="wlcsp49", feature = "lqfp80"),
    all(feature="wlcsp49", feature = "lqfp100"),
    all(feature="lqfp64", feature = "ufbga64"),
    all(feature="lqfp64", feature = "lqfp80"),
    all(feature="lqfp64", feature = "lqfp100"),
    all(feature="ufbga64", feature = "lqfp80"),
    all(feature="ufbga64", feature = "lqfp100"),
    all(feature="lqfp80", feature = "lqfp100"),
))]
compile_error!(
    "Multiple packages specified. Only a single `--features <package>` can be specified."
);

#[cfg(feature = "device-selected")]
pub use embedded_hal as hal;

pub extern crate stm32g4;

#[cfg(feature = "stm32g431")]
pub use stm32g4::stm32g431 as pac;

/*#[cfg(feature = "stm32g441")]
pub use stm32g4::stm32g441 as pac;

#[cfg(feature = "stm32g471")]
pub use stm32g4::stm32g471 as pac;

#[cfg(feature = "stm32g473")]
pub use stm32g4::stm32g473 as pac;

#[cfg(feature = "stm32g474")]
pub use stm32g4::stm32g474 as pac;

#[cfg(feature = "stm32g483")]
pub use stm32g4::stm32g483 as pac;

#[cfg(feature = "stm32g484")]
pub use stm32g4::stm32g484 as pac;

#[cfg(feature = "stm32g491")]
pub use stm32g4::stm32g491 as pac;

#[cfg(feature = "stm32g4a1")]
pub use stm32g4::stm32g4a1 as pac;*/

pub mod gpio;
pub mod timer;
// TODO: everything
pub mod bb;
pub mod rcc;
pub mod time;

mod sealed {
    pub trait Sealed {}
}
use sealed::Sealed;

/*
mod pwm;
mod serial;
mod can;
mod qei;
mod exti;
mod i2c;
mod spi;
mod adc;
mod flash;
mod rtc;
mod usb
mod crc;
mod watchdog;
mod time;

mod afio;
mod bb;
mod dma;

 */
