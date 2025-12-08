use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::pac;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::Vec;
use static_cell::StaticCell;
use util::debounce::{self, RB_COL_LEN, RB_ROW_LEN};
use util::keymap::{FullState, KeyType};

use core::ops::Deref;

use util::cobs_uart::{CobsTx, RspnId, UartTxMutex};

use crate::display::{Draw, DISPLAY_DRAW};

pub static ALT_EN: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static REPORT_FULL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static KEYS: StaticCell<Keyscan> = StaticCell::new();

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn key_scan(keys: &'static mut Keyscan<'static>, uart_tx: &'static UartTxMutex, keymap: &'static KeymapMutex) {

    let oled = DISPLAY_DRAW.sender();
    let mut ticker = Ticker::every(Duration::from_millis(1));
    //let mut ticker = Ticker::every(Duration::from_micros(10));
    let mut alt_en = false;
    let mut prev_alt = alt_en;
    let mut update = KeyUpdate::new();
    loop {
        // If we get more than 8 changes send a full keystate scan
        let start: Instant = Instant::now();
        task_profiler::set!();
        let mut full_state = FullState::new();

        alt_en = ALT_EN.try_take().unwrap_or(alt_en);  // Handle tap dance/hold to enable alt state
        {
            let keymap = keymap.lock().await;
            keys.scan(&mut full_state, &mut update, alt_en, &keymap);
        }

        if REPORT_FULL.try_take().is_some() || update.overflow || keys.alt_en != prev_alt {
            // Send full key state if we switch alt state in case a numkey is being pressed
            let mut uart_tx = uart_tx.lock().await;
            let full_state = full_state.serialize();
            uart_tx.send(RspnId::FullState, full_state.as_slice()).await.unwrap();
            task_profiler::print!();
        } else if update.populate_update(&full_state) {
            let mut uart_tx = uart_tx.lock().await;
            uart_tx.send(RspnId::Timestamp, start.as_micros().to_le_bytes().as_slice()).await.unwrap();
            uart_tx.send(RspnId::KeyChange, update.as_slice()).await.unwrap();
            task_profiler::print!();
        }
        if keys.alt_en != prev_alt {
            oled.send(Draw::Numlock(keys.alt_en)).await;
            prev_alt = keys.alt_en;
        }
        update.clear();
        ticker.next().await;
    }
}

struct KeyUpdate {
    update: Vec<u8, 8>,
    inter: Vec<(KeyType, bool), 8>,
    overflow: bool,
}

impl KeyUpdate {
    #[inline(always)]
    fn new() -> Self {
        KeyUpdate {
            update: Vec::new(),
            inter: Vec::new(),
            overflow: false,
        }
    }
    #[inline(always)]
    fn clear(&mut self) {
        self.update.clear();
        self.inter.clear();
        self.overflow = false;
    }

    #[inline(always)]
    fn push(&mut self, keymap: &Keymap, alt_en: bool, col: usize, row: usize, state: bool) -> KeyType {
        if self.overflow {
            return KeyType::NoCode;
        }

        let val = keymap.map(col, row, alt_en);
        if val != KeyType::NoCode {
            if self.inter.push((val, state)).is_err() {
                self.overflow = true;
            }
            val
        } else {
            KeyType::NoCode
        }
    }

    /// Take the intermediary and populated the update vector
    ///
    /// Returns true if an update is available
    #[inline(always)]
    fn populate_update(&mut self, full_state: &FullState) -> bool {
        self.update = self.inter.iter().filter_map(|(key, pressed)| {
            let is_pressed = full_state.is_set(*key);
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
}

impl Deref for KeyUpdate {
    type Target = Vec<u8, 8>;
    fn deref(&self) -> &Self::Target {
        &self.update
    }
}

pub struct Keyscan<'a> {
    // While we use the PAC to read/write these pins hold on to these pins so nothing
    // else can use them
    _select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    _input_pins: [Input<'a>; RB_ROW_LEN],
    enc_btn: Input<'a>,
    toggle: Input<'a>,
    state: [[u8; RB_ROW_LEN]; RB_COL_LEN],
    alt_en: bool,
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; RB_ROW_LEN], enc_btn: Input<'a>, toggle: Input<'a>) -> Self {
        let alt_en = toggle.is_high();
        Self {
            _select_pins: select_pins,
            _enable: enable,
            _input_pins: input_pins,
            enc_btn,
            toggle,
            state: [[0u8; RB_ROW_LEN]; RB_COL_LEN],
            alt_en,
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

    fn scan(&mut self, full_state: &mut FullState, update: &mut KeyUpdate, alt_en: bool, keymap: &Keymap) {
        self.alt_en = alt_en || self.toggle.is_high();
        let alt_en = self.alt_en;
        for col in 0..RB_COL_LEN {
            self.set_raw(col as _);
            let reg = self.read_raw();
            let end = if col < 3 {RB_ROW_LEN-1} else {RB_ROW_LEN};
            let notify_change = |row, state| {
                match update.push(keymap, self.alt_en, col as usize, row as usize, state) {
                    KeyType::EnableNum => self.alt_en |= state,
                    _ => (),
                };
            };
            let notify_pressed = |row| full_state.set(keymap.map(col as _, row as _, alt_en));
            debounce::debounce(&mut self.state[col][..end], reg, notify_change, notify_pressed);
        }
        let notify_change = |state| {update.push(keymap, self.alt_en, 0, 7, state as bool);};
        let notify_pressed = || full_state.set(keymap.map(0, 7, self.alt_en));
        debounce::integrate(&mut self.state[0][7], self.enc_btn.is_low(), notify_change, notify_pressed);
    }
}

const NUM_ROW_BOT: usize = 0;
const NUM_ROW_LEN: usize = 7;
const NUM_COL_LEN: usize = 3;

pub type KeymapMutex = Mutex<CriticalSectionRawMutex, Keymap<'static>>;
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
