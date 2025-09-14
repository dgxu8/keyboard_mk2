use defmt;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::block_on;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};

pub static BUFFER: Pipe<CriticalSectionRawMutex, 256> = Pipe::new();

// Start can be 1 byte and end can be 2 bytes so don't even attempt to write if we are < 5 bytes
const MIN_BUFFER_SPACE: usize = 5;

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

        unsafe { RESTORE_STATE = restore };
        // It just stuffs a zero for the first message so drop
        unsafe { ENCODER.start_frame(|_| { }) };
    }
    unsafe fn release() {
        if BUFFER.free_capacity() > MIN_BUFFER_SPACE {
            unsafe { ENCODER.end_frame(|x| { block_on(BUFFER.write_all(x)) }) };
        }
        TAKEN.store(false, Ordering::Relaxed);
        unsafe { critical_section::release(RESTORE_STATE) };
    }

    unsafe fn write(bytes: &[u8]) {
        if BUFFER.free_capacity() > bytes.len() + MIN_BUFFER_SPACE {
            unsafe { ENCODER.write(bytes, |x| { block_on(BUFFER.write_all(x)) }) };
        }
    }

    unsafe fn flush() {
        // This never seems to be called
    }
}
