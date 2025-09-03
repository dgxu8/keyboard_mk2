use embassy_futures::select::select;
use embassy_stm32::exti::ExtiInput;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{with_deadline, Duration, Instant};
use rotary_encoder_hal::{DefaultPhase, Direction, Rotary};

use crate::{cobs_uart::{self, CobsTx}, UartAsyncMutex};

pub static ENCODER_STATE: Signal<CriticalSectionRawMutex, EncoderState> = Signal::new();

pub struct EncoderState {
    pub pos: i8,
    pub interrupts: u32,
}

type Encoder = Rotary<ExtiInput<'static>, ExtiInput<'static>, DefaultPhase>;

#[embassy_executor::task]
pub async fn encoder_monitor(pin_a: ExtiInput<'static>, pin_b: ExtiInput<'static>, uart_tx: &'static UartAsyncMutex) {
    let mut enc: Encoder = Rotary::new(pin_a, pin_b);

    let mut pos: i8 = 0;
    let mut interrupts = 0;
    let mut timeout = Instant::now();
    loop {
        let (pin_a, pin_b) = enc.pins();

        let change_fut = select(pin_a.wait_for_any_edge(), pin_b.wait_for_any_edge());
        if pos == 0 {
            change_fut.await;
            timeout = Instant::now() + Duration::from_millis(30);
        } else if let Err(_) = with_deadline(timeout, change_fut).await {
            let mut uart_tx = uart_tx.lock().await;
            uart_tx.write_cobs(cobs_uart::ROTARY_CHANGE, pos.to_le_bytes().as_slice()).await.unwrap();
            pos = 0;
        }

        interrupts += 1;
        match enc.update().unwrap() {
            Direction::Clockwise => {
                pos += 1;
            }
            Direction::CounterClockwise => {
                pos -= 1;
            }
            Direction::None => {
                continue;
            }
        }

        let state = EncoderState {pos, interrupts};
        ENCODER_STATE.signal(state);
    }
}
