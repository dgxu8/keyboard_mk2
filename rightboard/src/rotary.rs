use embassy_futures::select::select;
use embassy_stm32::exti::ExtiInput;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use rotary_encoder_hal::{Direction, Rotary};

pub static ENCODER_STATE: Signal<CriticalSectionRawMutex, EncoderState> = Signal::new();

pub struct EncoderState {
    pub pos: i32,
    pub interrupts: u32,
}

#[embassy_executor::task]
pub async fn encoder_monitor(pin_a: ExtiInput<'static>, pin_b: ExtiInput<'static>) {
    let mut enc = Rotary::new(pin_a, pin_b);

    let mut pos = 0;
    let mut interrupts = 0;
    loop {
        let (pin_a, pin_b) = enc.pins();
        select(pin_a.wait_for_any_edge(), pin_b.wait_for_any_edge()).await;
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
