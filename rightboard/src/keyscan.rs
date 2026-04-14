use embassy_time::{Duration, Ticker};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::pac;
use static_cell::StaticCell;
use util::debounce::{self, RB_COL_LEN, RB_ROW_LEN};

use util::cobs_uart::{CobsTx, RspnId, UartTxMutex, encode_cobs};

use crate::display::{Draw, DISPLAY_DRAW};
use crate::keymap::{Keymap, KeymapMutex};
use crate::state_machine::{KeyScan, Layer, REPORT_FULL, StateMachine};

pub static KEYS: StaticCell<KeyMatrix> = StaticCell::new();

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn run(
    keys: &'static mut KeyMatrix<'static>,
    uart_tx: &'static UartTxMutex,
    keymap: &'static KeymapMutex,
    state: &'static mut StateMachine<'static>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let oled = DISPLAY_DRAW.sender();
    let mut scan = KeyScan::new();
    loop {
        task_profiler::set!();

        state.pre_scan_check(&mut scan);
        keymap.lock(|keymap| {
            keys.scan(&mut scan, state, keymap);
        });
        state.check_key_events(&mut scan);

        // TODO: evaluate not needing to send full state if layer changed
        let packet = if REPORT_FULL.try_take().is_some() || state.layer_changed {
            Some(scan.encode_state())
        } else if scan.len() > 0 {
            Some(encode_cobs(RspnId::KeyChange as u8, scan.as_slice()).unwrap())
        } else {
            None
        };

        scan.clear();
        if let Some(code) = state.take_pulse() {
            // Key up on next scan to imitate a pulse
            scan.update_force(code, false);
        }

        // Send data
        if let Some(packet) = packet {
            let mut uart_tx = uart_tx.lock().await;
            uart_tx.write_slice(packet.as_slice()).await;
            task_profiler::print!();
        }
        // state.defmt_state();

        if state.take_changed() {
            oled.send(Draw::Numlock(state.layer == Layer::Numpad)).await;
        }
        ticker.next().await;
    }
}

pub struct KeyMatrix<'a> {
    // While we use the PAC to read/write these pins hold on to these pins so nothing
    // else can use them
    _select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    _input_pins: [Input<'a>; RB_ROW_LEN],
    enc_btn: Input<'a>,
    state: [[u8; RB_ROW_LEN]; RB_COL_LEN],
}

impl<'a> KeyMatrix<'a> {
    pub fn new(
        select_pins: [Output<'a>; 3],
        enable: Output<'a>,
        input_pins: [Input<'a>; RB_ROW_LEN],
        enc_btn: Input<'a>) -> Self
    {
        Self {
            _select_pins: select_pins,
            _enable: enable,
            _input_pins: input_pins,
            enc_btn,
            state: [[0u8; RB_ROW_LEN]; RB_COL_LEN],
        }
    }

    #[inline(always)]
    fn set_raw(&mut self, val: u32) {
        assert!(val < 8);
        let mut reg: u32 = ((val & 0b100) >> 2) | (val & 0b010) | ((val & 0b001) << 2);
        reg <<= 5;
        pac::GPIOA.bsrr().write(|w| w.0 = reg);
        reg = (!reg & (0b111 << 5)) << 16;
        pac::GPIOA.bsrr().write(|w| w.0 = reg);
    }

    #[inline(always)]
    fn read_raw(&mut self) -> u32 {
        (pac::GPIOA.idr().read().0 >> 8) & 0xFF
    }

    fn scan(&mut self, key_state: &mut KeyScan, state: &mut StateMachine, keymap: &Keymap) {
        for col in 0..RB_COL_LEN {
            self.set_raw(col as _);
            let reg = self.read_raw();
            let end = if col < 3 {RB_ROW_LEN-1} else {RB_ROW_LEN};
            let notify =  |row, pressed, changed| {
                key_state.update(state, keymap, col as usize, row as usize, pressed, changed);
            };
            debounce::debounce(&mut self.state[col][..end], reg, notify);
        }
        let notify =  |pressed, changed| {
            key_state.update(state, keymap, 0, 7, pressed, changed);
        };
        debounce::integrate(&mut self.state[0][7], self.enc_btn.is_low(), notify);
    }
}
