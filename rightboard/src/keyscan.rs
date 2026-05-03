use core::marker::PhantomData;
use core::sync::atomic::Ordering;

use embassy_time::{Duration, Ticker};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::pac;
use static_cell::StaticCell;
use util::debounce::{self, RB_COL_LEN, RB_ROW_LEN};

use util::cobs_uart::{CobsTx, RspnId, UartTxMutex, encode_cobs};
use util::keycom::KeyType;

use crate::display::{Draw, DISPLAY_DRAW};
use crate::keymap::{Keymap, KeymapMutex};
use crate::state_machine::{ALT_HOLD, KeyScan, Layer, REPORT_FULL, StateMachine};

pub static KEYS: StaticCell<KeyMatrix> = StaticCell::new();

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn run(
    keys: &'static mut KeyMatrix<'static>,
    uart_tx: &'static UartTxMutex,
    keymap: &'static KeymapMutex,
    mut state: StateMachine<'static>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let oled = DISPLAY_DRAW.sender();
    let mut scan = KeyScan::new();
    loop {
        task_profiler::set!();

        state.pre_scan_check(&mut scan);
        let fullscan = REPORT_FULL.try_take().is_some();

        let keymap = keymap.borrow();
        match (fullscan, state.lockout) {
            (false, false) => {
                let normal = UpdateBounce::new(keymap, &mut scan, NormalParser);
                keys.scan(&mut state, normal);
            }
            (false, true) => {
                let lockout = UpdateBounce::new(keymap, &mut scan, LockoutParser);
                keys.scan(&mut state, lockout);
            }
            (true, false)=> {
                let normal = FullBounce::new(keymap, &mut scan, NormalParser);
                keys.scan(&mut state, normal);
            }
            (true, true) => {
                let lockout = FullBounce::new(keymap, &mut scan, LockoutParser);
                keys.scan(&mut state, lockout);
            }
        }
        let _ = keymap;

        state.check_key_events(&mut scan);

        let packet = if fullscan {
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

trait Parser {
    fn parse(
        keymap: &Keymap,
        state: &mut StateMachine,
        col: usize, row: usize,
        pressed: bool, changed: bool,
    ) -> Option<KeyType>;
}

struct NormalParser;
impl Parser for NormalParser {
    #[inline(always)]
    fn parse(
        keymap: &Keymap,
        state: &mut StateMachine,
        col: usize, row: usize,
        pressed: bool, changed: bool,
    ) -> Option<KeyType> {
        let keycode = keymap.map(col, row, state.layer);
        match keycode {
            KeyType::Keycode(_)|KeyType::Mediacode(_) => return Some(keycode),
            KeyType::HoldEnableNum(_) if changed => {
                let info = keymap.get_info(col, row);
                state.set_hold_event(info, pressed);
            },
            KeyType::TapDanceDisableNum(_) if changed => {
                let info = keymap.get_info(col, row);
                state.set_tap_event(info, pressed);
            },
            KeyType::NoCode => (),
            KeyType::EnableNum => {
                ALT_HOLD.fetch_or(pressed, Ordering::Relaxed);
            },
            _ => (),
        }
        return None;
    }
}

struct LockoutParser;
impl Parser for LockoutParser {
    #[inline(always)]
    fn parse(
        keymap: &Keymap,
        state: &mut StateMachine,
        col: usize, row: usize,
        pressed: bool, changed: bool,
    ) -> Option<KeyType> {
        let keycode = keymap.map(col, row, state.layer);
        match keycode {
            KeyType::Keycode(_)|KeyType::Mediacode(_) => return Some(keycode),
            KeyType::HoldEnableNum(code)|KeyType::TapDanceDisableNum(code) if !changed => {
                return Some(KeyType::Keycode(code));
            },
            KeyType::HoldEnableNum(code) => {
                let info = keymap.get_info(col, row);
                state.set_hold_event(info, pressed);
                return Some(KeyType::Keycode(code));
            },
            KeyType::TapDanceDisableNum(code) => {
                let info = keymap.get_info(col, row);
                state.set_tap_event(info, pressed);
                return Some(KeyType::Keycode(code));
            },
            KeyType::NoCode => (),
            KeyType::EnableNum => {
                ALT_HOLD.fetch_or(pressed, Ordering::Relaxed);
            },
        }
        return None;
    }
}

trait Debounce {
    fn debounce(&mut self, state: &mut StateMachine, val: &mut [u8], col: usize, reg: u32);
    fn integrate(&mut self, state: &mut StateMachine, val: &mut u8, col: usize, row: usize, pressed: bool);
}

struct FullBounce<'a, 'b, T> {
    keymap: &'a Keymap<'a>,
    scan: &'b mut KeyScan,
    _parser: PhantomData<T>,
}
impl<'a, 'b, T> FullBounce<'a, 'b, T>
where T: Parser
{
    fn new(keymap: &'a Keymap<'a>, scan: &'b mut KeyScan, _parser: T) -> Self {
        Self {keymap, scan, _parser: PhantomData}
    }
    // Grouping the parser w/ the "update" function is the quickest for some reason. But it breaks if
    // we try to generalize the update funtion as well. Why? idfk
    fn notify(&mut self, state: &mut StateMachine, col: usize, row: usize, changed: bool) {
        let keycode = T::parse(self.keymap, state, col, row, true, changed);
        if let Some(keycode) = keycode {
            self.scan.state.set(keycode);
        }
    }
}
impl<T> Debounce for FullBounce<'_, '_, T>
where T: Parser
{
    fn debounce(&mut self, state: &mut StateMachine, val: &mut [u8], col: usize, reg: u32) {
        let notify = |row, changed| {
            self.notify(state, col, row, changed);
        };
        debounce::debounce_full(val, reg, notify);
    }
    fn integrate(&mut self, state: &mut StateMachine, val: &mut u8, col: usize, row: usize, pressed: bool) {
        let notify = |changed| {
            self.notify(state, col, row, changed);
        };
        debounce::integrate_full(val, pressed, notify);
    }
}

struct UpdateBounce<'a, 'b, T> {
    keymap: &'a Keymap<'a>,
    scan: &'b mut KeyScan,
    _parser: PhantomData<T>,
}
impl<'a, 'b, T> UpdateBounce<'a, 'b, T>
where T: Parser
{
    fn new(keymap: &'a Keymap<'a>, scan: &'b mut KeyScan, _parser: T) -> Self {
        Self {keymap, scan, _parser: PhantomData}
    }
    fn notify(&mut self, state: &mut StateMachine, col: usize, row: usize, pressed: bool) {
        let keycode = T::parse(self.keymap, state, col, row, pressed, true);
        if let Some(keycode) = keycode && !REPORT_FULL.signaled() {
            self.scan.update_vec(keycode, pressed);
        }
    }
}
impl<T> Debounce for UpdateBounce<'_, '_, T>
where T: Parser
{
    fn debounce(&mut self, state: &mut StateMachine, val: &mut [u8], col: usize, reg: u32) {
        let notify = |row, pressed| {
            self.notify(state, col, row, pressed);
        };
        debounce::debounce_update(val, reg, notify);
    }
    fn integrate(&mut self, state: &mut StateMachine, val: &mut u8, col: usize, row: usize, pressed: bool) {
        let notify = |pressed| {
            self.notify(state, col, row, pressed);
        };
        debounce::integrate_update(val, pressed, notify);
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

    fn scan(&mut self, state: &mut StateMachine, mut notify: impl Debounce)
    {
        for col in 0..RB_COL_LEN {
            self.set_raw(col as _);
            let reg = self.read_raw();
            let end = if col < 3 {RB_ROW_LEN-1} else {RB_ROW_LEN};
            notify.debounce(state, &mut self.state[col][..end], col, reg);
        }
        notify.integrate(state, &mut self.state[0][7], 0, 7, self.enc_btn.is_low());
    }
}
