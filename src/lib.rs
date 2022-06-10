#![no_std]

// If no target specified, print error message.
#[cfg(not(any(
    feature = "stm32g431",
    feature = "stm32g441",
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4a1"
)))]
compile_error!("Target not found. A `--features <target-name>` is required.");

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
    "Multiple targets specified. Only a single `--features <target-name>` can be specified."
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
// TODO: everything
pub mod bb;
pub mod rcc;
pub mod time;
mod sealed {
    pub trait Sealed {}
}
use sealed::Sealed;

/*
mod timer;
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
