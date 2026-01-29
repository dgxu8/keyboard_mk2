use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::pac;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::signal::Signal;
use heapless::Vec;
use portable_atomic::AtomicBool;
use static_cell::StaticCell;
use util::debounce::{self, RB_COL_LEN, RB_ROW_LEN};
use util::keymap::{FullState, KeyType};

use core::ops::Deref;
use core::sync::atomic::Ordering;

use util::cobs_uart::{CobsTx, RspnId, SerialBuffer, UartTxMutex, encode_cobs};

use crate::display::{Draw, DISPLAY_DRAW};

pub static ALT_HOLD: AtomicBool = AtomicBool::new(false);
pub static REPORT_FULL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static KEYS: StaticCell<KeyMatrix> = StaticCell::new();

pub type KeyStateMutex = Mutex<ThreadModeRawMutex, KeyScan<'static>>;
pub static KEY_STATE: StaticCell<KeyScan<'static>> = StaticCell::new();

// Do full report in the next scan
pub static REPORT_NEXT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(PartialEq, Eq, Clone, Copy)]
enum KeyEvent {
    HoldPress(u8),
    HoldRelease(u8),
    DancePress(u8),
    DanceRelease(u8),
    MultiEventReset,
    MultiEventPause,
}

trait IntoKeyEvent {
    fn into_event(&self, pressed: bool) -> Option<(KeyEvent, u8)>;
}

impl IntoKeyEvent for KeyType {
    #[inline(always)]
    fn into_event(&self, pressed: bool) -> Option<(KeyEvent, u8)> {
        match *self {
            KeyType::HoldEnableNum(code) if pressed => {
                Some((KeyEvent::HoldPress(code), code))
            },
            KeyType::HoldEnableNum(code) => {
                Some((KeyEvent::HoldRelease(code), code))
            },
            KeyType::TapDanceDisableNum(code) if pressed => {
                Some((KeyEvent::DancePress(code), code))
            },
            KeyType::TapDanceDisableNum(code) => {
                Some((KeyEvent::DanceRelease(code), code))
            },
            _ => None,
        }
    }
}

static LOCKOUT: Duration = Duration::from_secs(3);

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
enum State {
    Normal,
    Hold((u8, Instant)),
    TapDown((u8, Instant)),
    TapUp((u8, Instant)),
    LockoutTimed(Option<Instant>),
    LockoutHold,
}

struct KeyState {
    pub state: State,
    pub changed: bool,
    pub pulse: Option<KeyType>,
}

impl KeyState {
    pub fn new() -> Self {
        Self {
            state: State::Normal,
            changed: false,
            pulse: None,
        }
    }
    #[inline(always)]
    pub fn get(&self) -> State {
        self.state
    }
    #[inline(always)]
    pub fn set(&mut self, state: State) {
        self.state = state;
        self.changed = true;
    }
    #[inline(always)]
    pub fn set_normal(&mut self) {
        self.set(State::Normal);
    }
    #[inline(always)]
    pub fn set_hold(&mut self, key_code: u8) {
        self.set(State::Hold(
            (key_code, Instant::now() + Duration::from_millis(500))
        ));
    }
    #[inline(always)]
    pub fn set_tap_down(&mut self, key_code: u8) {
        self.set(State::TapDown(
            (key_code, Instant::now() + Duration::from_millis(200))
        ));
    }
    #[inline(always)]
    pub fn set_lockout_timed(&mut self, timeout: bool) {
        if timeout {
            self.set(State::LockoutTimed(Some(Instant::now() + LOCKOUT)));
        } else {
            self.set(State::LockoutTimed(None));
        }
    }
    #[inline(always)]
    pub fn set_lockout_hold(&mut self) {
        self.set(State::LockoutHold); }
    #[inline(always)]
    pub fn set_pulse(&mut self, key: KeyType) {
        self.pulse = Some(key);
    }
    #[inline(always)]
    pub fn take_pulse(&mut self) -> Option<KeyType> {
        core::mem::take(&mut self.pulse)
    }
}

fn check_timeouts(scan: &mut KeyScan, state: &mut KeyState, toggle: bool) -> bool {
    let time = Instant::now();
    match state.get() {
        State::Hold((_, timeout)) if timeout < time => {
            scan.lockout = true;
            scan.alt_en = true;
            state.set_lockout_timed(true);
            return true;
        },
        State::TapDown((code, timeout)) if timeout < time => {
            scan.lockout = true;
            scan.update_force(KeyType::Keycode(code), true);
            state.set_lockout_timed(false);
        },
        State::TapUp((code, timeout)) if timeout < time => {
            scan.lockout = true;
            let key_code = KeyType::Keycode(code);
            scan.update_force(key_code, true);
            state.set_pulse(key_code);
            state.set_lockout_timed(true);
        },
        State::LockoutTimed(Some(timeout)) if timeout < time => {
            state.set_normal();
            scan.lockout = false;
            if scan.alt_en != toggle {
                scan.alt_en = toggle;
                REPORT_FULL.signal(());
                return true;
            }
        },
        _ => (),
    }
    false
}

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn run(
    keys: &'static mut KeyMatrix<'static>,
    uart_tx: &'static UartTxMutex,
    keymap: &'static KeymapMutex,
    key_scan: &'static mut KeyScan<'static>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let oled = DISPLAY_DRAW.sender();
    let mut state = KeyState::new();
    loop {
        task_profiler::set!();
        if REPORT_NEXT.try_take().is_some() {
            REPORT_FULL.signal(());
        }

        // Maybe move into check_alt
        let mut alt_changed = key_scan.check_alt_and_timeouts(&mut state);

        keymap.lock(|keymap| keys.scan(key_scan, keymap));
        key_scan.check_signal(&mut state, &mut alt_changed);

        // TODO: evaluate not needing to send full state if alt mode changed
        let packet = if REPORT_FULL.try_take().is_some() || alt_changed {
            Some(key_scan.encode_state())
        } else if key_scan.populate_update() {
            Some(encode_cobs(RspnId::KeyChange as u8, key_scan.as_slice()).unwrap())
        } else {
            None
        };

        key_scan.clear();
        if let Some(code) = state.take_pulse() {
            key_scan.update_force(code, false);
        }

        if state.changed {
            defmt::info!("Switched to {}", state.get());
            state.changed = false;
        }

        // Send data
        if let Some(packet) = packet {
            let mut uart_tx = uart_tx.lock().await;
            uart_tx.write_slice(packet.as_slice()).await;
            // task_profiler::print!();
        }
        if alt_changed {
            oled.send(Draw::Numlock(key_scan.alt_en)).await;
        }
        ticker.next().await;
    }
}

enum Layer {
    Arrow,
    Numpad,
}

pub struct KeyScan<'a> {
    pub state: FullState,
    update: Vec<u8, 8>,
    inter: Vec<(KeyType, bool), 8>,
    toggle: Input<'a>,
    toggle_state: bool,
    pub alt_en: bool,  // TODO: Replace w/ an enum
    lockout: bool,
    event: Option<KeyEvent>,
}

impl<'a> KeyScan<'a> {
    #[inline(always)]
    pub fn new(toggle: Input<'a>) -> Self {
        let alt_en = toggle.is_high();
        KeyScan {
            state: FullState::new(),
            update: Vec::new(),
            inter: Vec::new(),
            toggle,
            alt_en,
            toggle_state: alt_en,
            lockout: false,
            event: None,
        }
    }

    #[inline(always)]
    fn clear(&mut self) {
        self.state.clear();
        self.update.clear();
        self.inter.clear();
    }

    fn check_alt_and_timeouts(&mut self, state: &mut KeyState) -> bool {
        let hold_pressed = ALT_HOLD.load(Ordering::Relaxed);

        let toggle_state = self.toggle.is_high();
        let toggle_change = toggle_state != self.toggle_state;
        // TODO: Maybe wrap this into a if toggle_change
        self.toggle_state = toggle_state;

        match state.get() {
            State::Hold(_) if hold_pressed => {
                self.alt_en = true;
                self.lockout = true;
                state.set_lockout_hold();
                REPORT_NEXT.signal(());
                true
            },
            // User toggled into numpad while in hold state so assume meant to send numpad key.
            State::Hold(_) if toggle_change && toggle_state => {
                self.alt_en = true;
                self.lockout = true;
                state.set_lockout_timed(false);
                REPORT_NEXT.signal(());
                // Don't need to add key since we will trigger full keystate send
                true
            },
            State::TapDown(_) if toggle_change && !toggle_state => {
                self.alt_en = false;
                self.lockout = true;
                state.set_lockout_timed(false);
                REPORT_NEXT.signal(());
                true
            },
            // User taped a numkey then switched to arrow mapping
            State::TapUp((code, _)) if toggle_change && !toggle_state => {
                self.alt_en = false;
                self.lockout = true;

                let key_code = KeyType::Keycode(code);
                self.update_force(key_code, true);
                state.set_pulse(key_code);

                state.set_lockout_timed(true);
                true
            },
            State::LockoutHold if !hold_pressed => {
                let changed = toggle_state != self.alt_en; self.alt_en = toggle_state;
                self.lockout = true;
                state.set_lockout_timed(true);
                if changed {
                    REPORT_FULL.signal(());
                    REPORT_NEXT.signal(());
                    true
                } else {
                    false
                }
            },
            State::Normal | State::LockoutTimed(_) if hold_pressed && !toggle_state => {
                let changed = !self.alt_en;
                self.alt_en = true;
                self.lockout = true;
                state.set_lockout_hold();
                if changed {
                    REPORT_FULL.signal(());
                    true
                } else {
                    false
                }
            },
            State::Normal | State::LockoutTimed(_) if toggle_change => {
                let changed = toggle_state != self.alt_en;
                self.alt_en = toggle_state;
                self.lockout = true;
                state.set_lockout_timed(true);
                if changed {
                    REPORT_FULL.signal(());
                    true
                } else {
                    false
                }
            },
            _ => check_timeouts(self, state, toggle_state),
        }
    }

    #[inline(always)]
    fn check_signal(&mut self, state: &mut KeyState, alt_changed: &mut bool){
        let Some(event) = core::mem::take(&mut self.event) else {
            return;
        };
        match (state.get(), event) {
            (State::Normal, KeyEvent::HoldPress(code)) => {
                state.set_hold(code);
            },
            (State::Normal, KeyEvent::DancePress(code)) => {
                state.set_tap_down(code);
            },
            (State::Hold((hold_key, _)), KeyEvent::HoldRelease(code)) => {
                if hold_key != code {
                    defmt::warn!("Press key is different from the release key: ({}, {})", hold_key, code);
                }
                let key_code = KeyType::Keycode(hold_key);
                self.lockout = true;
                self.update_force(key_code, true);
                state.set_lockout_timed(true);
                state.set_pulse(key_code);
            },
            (State::Hold((prev_code, _)), KeyEvent::HoldPress(code))
            | (State::TapDown((prev_code, _)), KeyEvent::DancePress(code)) =>
            {
                self.update_force(KeyType::Keycode(prev_code), true);
                self.update_force(KeyType::Keycode(code), true);
                state.set_lockout_timed(true);
                self.lockout = true;
            },
            (State::TapDown(info), KeyEvent::DanceRelease(_)) => {
                state.set(State::TapUp(info));
            },
            (State::TapUp((tap_key, _)), KeyEvent::DancePress(code)) if tap_key == code => {
                self.lockout = true;
                self.alt_en = false;
                *alt_changed = true;
                state.set_lockout_timed(true);
                REPORT_NEXT.signal(());
            },
            (State::TapUp((tap_key, _)), KeyEvent::DancePress(code)) => {
                self.lockout = true;
                state.set_lockout_timed(true);

                // Need to initiate this key down
                let pulsed_key = KeyType::Keycode(tap_key);
                self.update_force(pulsed_key, true);
                state.set_pulse(pulsed_key);

                // Also key down this
                self.update_force(KeyType::Keycode(code), true);
            },
            (State::TapUp((tap_key, _)), KeyEvent::MultiEventReset|KeyEvent::MultiEventPause) => {
                self.lockout = true;
                state.set_lockout_timed(true);

                let pulsed_key = KeyType::Keycode(tap_key);
                self.update_force(pulsed_key, true);
                state.set_pulse(pulsed_key);
                REPORT_FULL.signal(());
            },
            (State::LockoutTimed(_),
             KeyEvent::HoldPress(_)
             | KeyEvent::DancePress(_)
             | KeyEvent::MultiEventPause) =>
            {
                state.set_lockout_timed(false);
                self.lockout = true;
            },
            (State::LockoutTimed(_),
             KeyEvent::HoldRelease(_)
             | KeyEvent::DanceRelease(_)
             | KeyEvent::MultiEventReset) =>
            {
                state.set_lockout_timed(true);
                self.lockout = true;
            },
            (_, KeyEvent::MultiEventReset|KeyEvent::MultiEventPause) => {
                REPORT_FULL.signal(());
                state.set_lockout_timed(true);
                self.lockout = true;
            },
            _ => (),
        }
    }

    #[inline(always)]
    fn set_event(&mut self, event: KeyEvent) {
        match (self.event, event) {
            (None, event) => {
                self.event = Some(event);
            },
            (_, KeyEvent::HoldRelease(_)|KeyEvent::DanceRelease(_)) => {
                self.event = Some(KeyEvent::MultiEventReset);
            },
            _ => {
                self.event = Some(KeyEvent::MultiEventPause);
            },
        }
    }

    #[inline(always)]
    fn update(&mut self, mut keycode: KeyType, pressed: bool, changed: bool) {
        match keycode.into_event(pressed) {
            None => (),
            Some((KeyEvent::HoldPress(_)|KeyEvent::DancePress(_), code)) if self.lockout && !changed => {
                keycode = KeyType::Keycode(code);
            },
            Some((event, code)) if self.lockout => {
                self.set_event(event);
                keycode = KeyType::Keycode(code);
            },
            Some((event, _)) if changed => {
                self.set_event(event);
                return;  // Exit early
            },
            _ => (),
        }

        if pressed {
            self.state.set(keycode);
        }

        if !changed {
            return;
        }

        match keycode {
            // I'm not keybinding EnableNum on the right board anyways ¯\_(ツ)_/¯
            KeyType::EnableNum => { ALT_HOLD.fetch_or(pressed, Ordering::Relaxed); },
            KeyType::NoCode => (),
            _ if !REPORT_FULL.signaled() => {
                // If we are going to send a full report don't update inter
                if self.inter.push((keycode, pressed)).is_err() {
                    REPORT_FULL.signal(());
                }
            },
            _ => (),
        }
    }

    #[inline(always)]
    fn update_force(&mut self, keycode: KeyType, pressed: bool) {
        if self.inter.push((keycode, pressed)).is_err() {
            REPORT_FULL.signal(());
        }
        if pressed {
            self.state.set(keycode);
        }
    }

    /// Take the intermediary and populated the update vector
    ///
    /// Returns true if an update is available
    #[inline(always)]
    fn populate_update(&mut self) -> bool {
        self.update = self.inter.iter().filter_map(|(key, pressed)| {
            let is_pressed = self.state.is_set(*key);
            if *pressed == is_pressed
                && let Ok(k) = key.encode_update(*pressed)
            {
                Some(k)
            } else {
                None
            }
        }).collect();
        !self.update.is_empty()
    }

    #[inline(always)]
    fn encode_state(&mut self) -> SerialBuffer {
        encode_cobs(RspnId::FullState as u8, self.state.serialize_slice()).unwrap()
    }
}

impl<'a> Deref for KeyScan<'a> {
    type Target = Vec<u8, 8>;
    fn deref(&self) -> &Self::Target {
        &self.update
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

    fn scan(&mut self, key_state: &mut KeyScan, keymap: &Keymap) {
        let alt_en = key_state.alt_en;
        for col in 0..RB_COL_LEN {
            self.set_raw(col as _);
            let reg = self.read_raw();
            let end = if col < 3 {RB_ROW_LEN-1} else {RB_ROW_LEN};
            let notify =  |row, pressed, changed| {
                let key = keymap.map(col as usize, row as usize, alt_en);
                key_state.update(key, pressed, changed);
            };
            debounce::debounce(&mut self.state[col][..end], reg, notify);
        }
        let notify =  |pressed, changed| {
            let key = keymap.map(0, 7, alt_en);
            key_state.update(key, pressed, changed);
        };
        debounce::integrate(&mut self.state[0][7], self.enc_btn.is_low(), notify);
    }
}

const NUM_ROW_BOT: usize = 0;
const NUM_ROW_LEN: usize = 7;
const NUM_COL_LEN: usize = 3;

pub type KeymapMutex = Mutex<ThreadModeRawMutex, Keymap<'static>>;
pub static KEYMAP: StaticCell<KeymapMutex> = StaticCell::new();

macro_rules! push_keymap {
    ($matrix:expr, $buff:expr) => {
        if let Ok(bind) = KeyType::try_from(&$buff[2..]) {
            $matrix[$buff[0] as usize][$buff[1] as usize] = bind;
        } else {
            defmt::error!("Invalid keybind: {:?}", $buff);
        }
    };
}

#[repr(C)]
#[derive(Default)]
pub struct Map {
    pub right: [[KeyType; RB_ROW_LEN]; RB_COL_LEN],
    pub numpad: [[KeyType; NUM_ROW_LEN]; NUM_COL_LEN],
    pub ccw: KeyType,
    pub cw: KeyType,
}

pub struct Keymap<'a> {
    pub keymap: Map,
    flash: Flash<'a, Blocking>,
    start_addr: u32,
}

impl<'a> Keymap<'a> {
    pub fn new(flash: Flash<'a, Blocking>, start_addr: u32) -> Self {
        let mut data = [0; size_of::<Map>()];
        flash.eeprom_read_slice(start_addr, &mut data).unwrap();
        Keymap {
            keymap: unsafe { core::mem::transmute(data) },
            flash,
            start_addr,
        }
    }
    pub fn save(&mut self) {
        let buf: [u8; size_of::<Map>()] = unsafe { core::mem::transmute_copy(&self.keymap) };
        self.flash.eeprom_write_slice(self.start_addr, &buf).unwrap();
    }
    pub fn clear(&mut self) {
        self.keymap = Map::default();
    }
    pub fn update_right(&mut self, buff: &[u8]) {
        push_keymap!(self.keymap.right, buff);
    }
    pub fn update_numpad(&mut self, buff: &[u8]) {
        push_keymap!(self.keymap.numpad, buff);
    }
    pub fn update_rotary(&mut self, buff: &[u8]) -> Result<(), ()> {
        self.keymap.ccw = KeyType::try_from(&buff[..2])?;
        self.keymap.cw = KeyType::try_from(&buff[2..])?;
        Ok(())
    }
    #[inline(always)]
    pub fn map(&self, col: usize, row: usize, alt_en: bool) -> KeyType {
        if alt_en && col < NUM_COL_LEN &&  row > NUM_ROW_BOT && row < NUM_ROW_LEN {
            self.keymap.numpad[col][row]
        } else {
            self.keymap.right[col][row]
        }
    }
}
