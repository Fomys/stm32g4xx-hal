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
use embedded_hal as hal;

/*
mod gpio;
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
