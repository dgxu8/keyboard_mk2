use defmt;
use embassy_stm32::{peripherals::USB, usb::Driver};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer, WithTimeout};
use embassy_usb::class::cdc_acm::Sender;

use util::logger::BUFFER;

use crate::usb::UsbSerial;

pub static OUTPUT_DEFMT: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static BRIDGE_RB_DEFMT: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
pub async fn run(mut class: Sender<'static, Driver<'static, USB>>) {
    let mut buf = [0; 256];

    class.wait_connection().await;

    // So we don't start send log data to early wait for byte
    // class.read_packet(&mut buf).await.unwrap();
    OUTPUT_DEFMT.wait().await;
    // Delay a bit so we don't send the data
    Timer::after_millis(100).await;
    defmt::info!("Buffer size: {}", BUFFER.len());
    loop {
        let len = BUFFER.read(&mut buf).await;
        class.write_packets(&buf[..len]).await.unwrap();
    }
}

#[inline(always)]
pub async fn bridge_rb<'a>(sender: &mut Sender<'a, Driver<'a, USB>>, buf: &[u8]) {
    if BRIDGE_RB_DEFMT.signaled() {
        if sender.write_packet(buf)
            .with_timeout(Duration::from_millis(50)).await
            .is_err() {
            // If we timeout, consume the signal so we stop trying to send out USB
            defmt::info!("Rightboard defmt bridge timed out, closing defmt forwarding");
            BRIDGE_RB_DEFMT.try_take();
        }
    }
}
