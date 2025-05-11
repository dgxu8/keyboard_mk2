#![no_std]
#![no_main]

extern crate panic_halt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PB6, Level::High, Speed::Low);

    led.set_high();
    loop {
        Timer::after_millis(500).await;
        led.toggle();
    }
}
