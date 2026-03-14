use embassy_time::{Duration, Instant};
use embassy_stm32::gpio::Input;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use heapless::Vec;
use portable_atomic::AtomicBool;
use static_cell::StaticCell;
use util::keycom::{FullState, KeyType};

use core::ops::Deref;
use core::sync::atomic::Ordering;

use util::cobs_uart::{RspnId, SerialBuffer, encode_cobs};

use crate::keymap::{KeyBinds, Keymap};

const HOLD_TIMEOUT: Duration = Duration::from_millis(500);
const TAP_TIMEOUT: Duration = Duration::from_millis(200);
const LOCKOUT: Duration = Duration::from_secs(3);

pub static ALT_HOLD: AtomicBool = AtomicBool::new(false);
pub static REPORT_FULL: Signal<ThreadModeRawMutex, ()> = Signal::new();
pub static STATE_MACHINE: StaticCell<StateMachine<'static>> = StaticCell::new();

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum KeyEvent {
    HoldPress(KeyBinds),
    HoldRelease(KeyBinds),
    DancePress(KeyBinds),
    DanceRelease(KeyBinds),
    MultiEventReset,
    MultiEventPause,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, defmt::Format)]
pub enum Layer {
    Arrow,
    Numpad,
}

impl From<bool> for Layer {
    fn from(value: bool) -> Self {
        match value {
            true => Layer::Numpad,
            false => Layer::Arrow,
        }
    }
}

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum State {
    Normal,
    Hold(KeyBinds),
    TapDown(KeyBinds),
    TapUp(KeyBinds),
    LockoutTimed(Option<Instant>),
    LockoutHold,
}

impl State {
    pub fn is_lockout(&self) -> bool {
        match *self {
            State::LockoutHold|State::LockoutTimed(_) => true,
            _ => false,
        }
    }
}

pub struct KeyState {
    state: State,
    pub changed: bool,  // Only used to trigger a defmt print
}

impl KeyState {
    pub fn new() -> Self {
        Self {
            state: State::Normal,
            changed: false,
        }
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
    pub fn set_hold(&mut self, mut binds: KeyBinds) {
        binds.timeout += HOLD_TIMEOUT;
        self.set(State::Hold(binds));
    }
    #[inline(always)]
    pub fn set_tap_down(&mut self, mut binds: KeyBinds) {
        binds.timeout += TAP_TIMEOUT;
        self.set(State::TapDown(binds));
    }
    #[inline(always)]
    pub fn set_tap_up(&mut self, binds: KeyBinds) {
        // Don't reset timeout, carry it on from the down
        self.set(State::TapUp(binds));
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
        self.set(State::LockoutHold);
    }
}

impl Deref for KeyState {
    type Target = State;
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

pub struct StateMachine<'a> {
    state: KeyState,
    pub layer: Layer,
    toggle: Input<'a>,
    toggle_state: Layer,

    // Temporary values use for caching and reporting scan info
    pulse: Option<KeyType>,
    pub layer_changed: bool,
    event: Option<KeyEvent>,
    lockout: bool,
}

impl<'a> StateMachine<'a> {
    pub fn new(toggle: Input<'a>) -> Self {
        let layer = Layer::from(toggle.is_high());
        Self {
            state: KeyState::new(),
            layer,
            toggle,
            toggle_state: layer,

            pulse: None,
            layer_changed: false,
            event: None,
            lockout: false,
        }
    }

    #[inline(always)]
    pub fn take_pulse(&mut self) -> Option<KeyType> {
        core::mem::take(&mut self.pulse)
    }

    #[inline(always)]
    pub fn take_changed(&mut self) -> bool {
        core::mem::take(&mut self.layer_changed)
    }

    #[inline(always)]
    pub fn defmt_state(&mut self) {
        if core::mem::take(&mut self.state.changed) {
            defmt::info!("Switched to {}", *self.state);
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
    fn set_hold_event(&mut self, binds: KeyBinds, pressed: bool) {
        if pressed {
            self.set_event(KeyEvent::HoldPress(binds));
        } else {
            self.set_event(KeyEvent::HoldRelease(binds));
        }
    }

    #[inline(always)]
    fn set_tap_event(&mut self, binds: KeyBinds, pressed: bool) {
        if pressed {
            self.set_event(KeyEvent::DancePress(binds));
        } else {
            self.set_event(KeyEvent::DanceRelease(binds));
        }
    }

    /// Run pre-key scan checks and cache lockout information.
    ///
    /// Check and handle toggle/hold changes alongside timeouts.
    pub fn pre_scan_check(&mut self, scan: &mut KeyScan) {
        let hold_pressed = ALT_HOLD.load(Ordering::Relaxed);

        let toggle_layer = Layer::from(self.toggle.is_high());
        let toggle_change = toggle_layer != self.toggle_state;
        self.toggle_state = toggle_layer;

        match *self.state {
            State::Hold(_) if hold_pressed => {
                self.layer = Layer::Numpad;
                self.layer_changed = true;
                self.state.set_lockout_hold();
            },
            // User toggled into numpad while in hold state so assume meant to send numpad key.
            State::Hold(_) if toggle_change && toggle_layer == Layer::Numpad => {
                self.layer = Layer::Numpad;
                // Don't need to add key since we will trigger full keystate send
                self.layer_changed = true;
                self.state.set_lockout_timed(false);
            },
            State::TapDown(_) if toggle_change && toggle_layer == Layer::Arrow => {
                self.layer = Layer::Arrow;
                self.layer_changed = true;
                self.state.set_lockout_timed(false);
            },
            // User taped a numkey then switched to arrow mapping
            State::TapUp(binds) if toggle_change && toggle_layer == Layer::Arrow => {
                self.layer = Layer::Arrow;
                self.layer_changed = true;
                self.state.set_lockout_timed(true);

                let key_code = binds.get(Layer::Numpad);
                scan.update_force(key_code, true);
                self.pulse = Some(key_code);
            },
            State::LockoutHold if !hold_pressed => {
                if self.layer != toggle_layer {
                    self.layer = toggle_layer;
                    self.layer_changed = true;
                    REPORT_FULL.signal(());
                }
                self.state.set_lockout_timed(true);
            },
            State::Normal | State::LockoutTimed(_) if hold_pressed && toggle_layer == Layer::Arrow => {
                if self.layer != Layer::Numpad {
                    self.layer = Layer::Numpad;
                    self.layer_changed = true;
                    REPORT_FULL.signal(());
                }
                self.state.set_lockout_hold();
            },
            State::Normal | State::LockoutTimed(_) if toggle_change => {
                if self.layer != toggle_layer {
                    self.layer = toggle_layer;
                    self.layer_changed = true;
                    REPORT_FULL.signal(());
                }
                self.state.set_lockout_timed(true);
            },
            _ => self.check_timeouts(scan, toggle_layer),
        }
        // Cache the lockout state so it isn't calculated every key scan
        self.lockout = self.state.is_lockout()
    }

    fn check_timeouts(&mut self, scan: &mut KeyScan, layer: Layer) {
        let time = Instant::now();
        match *self.state {
            State::Hold(key) if key.timeout < time => {
                self.layer = Layer::Numpad;
                self.layer_changed = true;
                self.state.set_lockout_timed(true);
            },
            State::TapDown(key) if key.timeout < time => {
                let key_code = key.get(Layer::Numpad);
                scan.update_force(key_code, true);
                self.state.set_lockout_timed(false);
            },
            State::TapUp(key) if key.timeout < time => {
                let key_code = key.get(Layer::Numpad);
                scan.update_force(key_code, true);

                self.pulse = Some(key_code);
                self.state.set_lockout_timed(true);
            },
            State::LockoutTimed(Some(timeout)) if timeout < time => {
                if self.layer != layer {
                    self.layer = layer;
                    self.layer_changed = true;
                    REPORT_FULL.signal(());
                }
                self.state.set_normal();
            },
            _ => (),
        }
    }

    /// Check and handle key events from the key scan.
    pub fn check_key_events(&mut self, scan: &mut KeyScan) {
        let Some(event) = core::mem::take(&mut self.event) else {
            return;
        };
        match (*self.state, event) {
            (State::Normal, KeyEvent::HoldPress(code)) => {
                self.state.set_hold(code);
            },
            (State::Normal, KeyEvent::DancePress(coord)) => {
                self.state.set_tap_down(coord);
            },
            (State::Hold(prev_key), KeyEvent::HoldRelease(key)) => {
                if prev_key != key {
                    defmt::warn!("Press key is different from the release key: ({}, {})", prev_key, key);
                }
                let key_code = prev_key.get(Layer::Arrow);
                scan.update_force(key_code, true);
                self.pulse = Some(key_code);
                self.state.set_lockout_timed(true);
            },
            (State::Hold(prev_key), KeyEvent::HoldPress(key)) => {
                scan.update_force(prev_key.get(Layer::Arrow), true);
                scan.update_force(key.get(Layer::Arrow), true);
                self.state.set_lockout_timed(true);
            },
            (State::TapDown(prev_key), KeyEvent::DancePress(key)) => {
                let prev_code = prev_key.get(self.layer);
                let code = key.get(self.layer);
                scan.update_force(prev_code, true);
                scan.update_force(code, true);
                self.state.set_lockout_timed(true);
            },
            (State::TapDown(key), KeyEvent::DanceRelease(_)) => {
                self.state.set_tap_up(key);
            },
            (State::TapUp(prev_key), KeyEvent::DancePress(key)) if prev_key == key => {
                self.layer = Layer::Arrow;
                self.layer_changed = true;

                scan.update_force(key.get(Layer::Arrow), true);

                self.state.set_lockout_timed(true);
            },
            (State::TapUp(prev_key), KeyEvent::DancePress(key)) => {
                // Key down both keys
                let pulsed_key = prev_key.get(Layer::Numpad);
                let new = key.get(Layer::Numpad);
                scan.update_force(pulsed_key, true);
                scan.update_force(new, true);

                self.pulse = Some(pulsed_key);
                self.state.set_lockout_timed(true);
            },
            (State::TapUp(prev_key), KeyEvent::MultiEventReset|KeyEvent::MultiEventPause) => {
                let pulsed_key = prev_key.get(Layer::Numpad);
                scan.update_force(pulsed_key, true);
                self.pulse = Some(pulsed_key);

                self.state.set_lockout_timed(true);
                REPORT_FULL.signal(());
            },
            (State::LockoutTimed(_),
             KeyEvent::HoldPress(_)
             | KeyEvent::DancePress(_)
             | KeyEvent::MultiEventPause) =>
            {
                self.state.set_lockout_timed(false);
            },
            (State::LockoutTimed(_),
             KeyEvent::HoldRelease(_)
             | KeyEvent::DanceRelease(_)
             | KeyEvent::MultiEventReset) =>
            {
                self.state.set_lockout_timed(true);
            },
            (_, KeyEvent::MultiEventReset|KeyEvent::MultiEventPause) => {
                self.state.set_lockout_timed(true);
                REPORT_FULL.signal(());
            },
            _ => (),
        }
    }
}

pub struct KeyScan {
    pub state: FullState,
    update: Vec<u8, 8>,
    inter: Vec<(KeyType, bool), 8>,
}

impl KeyScan {
    pub fn new() -> Self {
        KeyScan {
            state: FullState::new(),
            update: Vec::new(),
            inter: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.state.clear();
        self.update.clear();
        self.inter.clear();
    }

    pub fn update(
        &mut self,
        state: &mut StateMachine,
        map: &Keymap,
        col: usize, row: usize,
        pressed: bool, changed: bool,
    ) {
        let mut keycode = map.map(col, row, state.layer);
        match keycode {
            KeyType::Keycode(_)|KeyType::Mediacode(_) => (),
            KeyType::HoldEnableNum(code)|KeyType::TapDanceDisableNum(code) if state.lockout && !changed => {
                if pressed {
                    self.state.set(KeyType::Keycode(code));
                }
                return;
            },
            KeyType::HoldEnableNum(code) if state.lockout => {
                let info = map.get_info(col, row);
                state.set_hold_event(info, pressed);
                keycode = KeyType::Keycode(code);
            },
            KeyType::HoldEnableNum(_) if changed => {
                let info = map.get_info(col, row);
                state.set_hold_event(info, pressed);
                return;
            },
            KeyType::TapDanceDisableNum(code) if state.lockout => {
                let info = map.get_info(col, row);
                state.set_tap_event(info, pressed);
                keycode = KeyType::Keycode(code);
            },
            KeyType::TapDanceDisableNum(_) if changed => {
                let info = map.get_info(col, row);
                state.set_tap_event(info, pressed);
                return;
            },
            KeyType::NoCode => return,
            KeyType::EnableNum => {
                ALT_HOLD.fetch_or(pressed, Ordering::Relaxed);
                return;
            },
            _ => return,
        }

        if pressed {
            self.state.set(keycode);
        }

        if !changed {
            return;
        }

        if !REPORT_FULL.signaled() {
            // If we are going to send a full report don't update inter
            if self.inter.push((keycode, pressed)).is_err() {
                REPORT_FULL.signal(());
            }
        }
    }

    #[inline(always)]
    pub fn update_force(&mut self, keycode: KeyType, pressed: bool) {
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
    pub fn populate_update(&mut self) -> bool {
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
    pub fn encode_state(&mut self) -> SerialBuffer {
        encode_cobs(RspnId::FullState as u8, self.state.serialize_slice()).unwrap()
    }
}

impl Deref for KeyScan {
    type Target = Vec<u8, 8>;
    fn deref(&self) -> &Self::Target {
        &self.update
    }
}
