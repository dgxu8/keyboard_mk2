use defmt;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::{mode::Async, peripherals::USB, usart::Uart, usb::Driver};
use embassy_futures::block_on;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::CdcAcmClass;

use crate::usb::UsbSerial;


pub static BUFFER: Pipe<CriticalSectionRawMutex, 256> = Pipe::new();

const START_SIZE: usize = 1;
const END_SIZE: usize = 2;
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut RESTORE_STATE: critical_section::RestoreState = critical_section::RestoreState::invalid();
static mut ENCODER: defmt::Encoder = defmt::Encoder::new();


#[defmt::global_logger]
struct GlobalLogger;

#[allow(static_mut_refs)]
unsafe impl defmt::Logger for GlobalLogger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };
        if TAKEN.load(Ordering::Relaxed) {
            panic!("Bad");
        }
        TAKEN.store(true, Ordering::Relaxed);

        unsafe { RESTORE_STATE = restore; }
        if BUFFER.free_capacity() >= START_SIZE {
            unsafe { ENCODER.start_frame(|x| { block_on(BUFFER.write_all(x)) }) }
        }
    }
    unsafe fn release() {
        if BUFFER.free_capacity() >= END_SIZE {
            ENCODER.end_frame(|x| { block_on(BUFFER.write_all(x)) });
        }
        TAKEN.store(false, Ordering::Relaxed);
        let restore = RESTORE_STATE;
        critical_section::release(restore);
    }

    unsafe fn write(bytes: &[u8]) {
        if BUFFER.free_capacity() >= bytes.len() {
            ENCODER.write(bytes, |x| { block_on(BUFFER.write_all(x)) });
        }
    }

    unsafe fn flush() {
        // This never seems to be called
    }
}

#[embassy_executor::task]
pub async fn run(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    let mut buf = [0; 256];
    loop {
        class.wait_connection().await;
        // So we don't start send log data to early wait for byte
        class.read_packet(&mut buf).await.unwrap();
        // Delay a bit so we don't send the data
        Timer::after_millis(100).await;
        loop {
            let len = BUFFER.read(&mut buf).await;
            class.write_all(&buf[..len]).await.unwrap();
        }
    }
}
