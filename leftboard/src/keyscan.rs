use core::ops::{Deref, DerefMut};

use embassy_stm32::{flash::{Bank1Region, Blocking}, gpio::{Input, Output}, pac};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use static_cell::StaticCell;
use util::{debounce, keymap::{FullState, KeyType}};
use util::cobs_uart::{CmdId, CobsTx, UartTxMutex};

use crate::{KeyStateMutex, KEY_CHANGE};

static ROW_LEN: usize = 5;
static COL_LEN: usize = 8;

pub static LB_KEYSTATE: StaticCell<KeyStateMutex> = StaticCell::new();

#[embassy_executor::task]
pub async fn run(mut keys: Keyscan<'static>, keymap: &'static KeymapMutex, key_state: &'static KeyStateMutex, uart_tx: &'static UartTxMutex) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        let start = Instant::now();
        let (change, enable_num) =  {
            let mut state = key_state.lock().await;
            state.clear();
            let mut keymap = keymap.lock().await;
            keys.scan(&mut state, &mut keymap)
        };
        let dur = Instant::now() - start;
        if let Some(alt_en) = enable_num {
            let mut uart_tx = uart_tx.lock().await;
            uart_tx.send(CmdId::AltEnable, &[alt_en as u8]).await.unwrap();
        }
        if change {
            KEY_CHANGE.signal(true);
            defmt::info!("Keyscan time: {}", dur.as_micros());
        }
        ticker.next().await;
    }
}

pub struct Keyscan<'a> {
    _select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    _input_pins: [Input<'a>; ROW_LEN],
    state: [[u8; ROW_LEN]; COL_LEN],
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; 5]) -> Self {
        Self {
            _select_pins: select_pins,
            _enable: enable,
            _input_pins: input_pins,
            state: [[0; ROW_LEN]; COL_LEN],
        }
    }

    #[inline(always)]
    fn set_raw(&self, val: u32) {
        let mut reg = (val & 0x7) << 4;
        pac::GPIOA.bsrr().write(|w| w.0 = reg);
        reg = (!reg & (0b111 << 4)) << 16;
        pac::GPIOA.bsrr().write(|w| w.0 = reg);
    }

    #[inline(always)]
    fn read_raw(&self) -> u32 {
        (pac::GPIOB.idr().read().0 >> 3) & 0xFF
    }

    #[inline(always)]
    fn scan(&mut self, key_state: &mut FullState, keymap: &mut Keymap) -> (bool, Option<bool>) {
        let mut change = false;
        let mut enable_num = None;
        // We enter here w/ decoder set to col zero
        for col in 0..8 {
            let notify = |row, pressed, changed| {
                let key = keymap.map[col as usize][row as usize];
                if changed {
                    match key {
                        // Maybe just remove this? who cares if we trigger a read?
                        KeyType::NoCode => (),
                        // Bug here if multiple keys are EnableNum. If both are pressed this could
                        // flip if one is released (depends on scan order).
                        KeyType::EnableNum => enable_num = Some(pressed),
                        _ => change = true,
                    }
                }
                if pressed {
                    key_state.set(key);
                }
            };
            let reg = self.read_raw();
            // Set next decoder for next column so it has time to propagate
            self.set_raw((col+1) as u32);
            debounce::debounce(&mut self.state[col], reg, notify);
        }

        (change, enable_num)
    }
}

pub type KeymapMutex = Mutex<CriticalSectionRawMutex, Keymap<'static>>;
pub static KEYMAP: StaticCell<KeymapMutex> = StaticCell::new();

#[repr(C, align(8))]
#[derive(Default)]
pub struct Map([[KeyType; ROW_LEN]; COL_LEN]);

impl Map {
    pub const fn len() -> usize {
        size_of::<Map>()
    }
}

impl Deref for Map {
    type Target = [[KeyType; ROW_LEN]; COL_LEN];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Map {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub struct Keymap<'a> {
    pub map: Map,
    flash: Bank1Region<'a, Blocking>,
    start_addr: u32,
    end_addr: u32,
}

impl<'a> Keymap<'a> {
    pub fn new(mut flash: Bank1Region<'a, Blocking>, start_addr: u32, end_addr: u32) -> Self {
        let mut data = [0; Map::len()];
        flash.blocking_read(start_addr, &mut data).unwrap();
        Keymap {
            map: unsafe{ core::mem::transmute(data) },
            flash,
            start_addr,
            end_addr,
        }
    }
    pub fn save(&mut self) {
        self.flash.blocking_erase(self.start_addr, self.end_addr).unwrap();
        let buf: [u8; Map::len()] = unsafe { core::mem::transmute_copy(&self.map) };
        self.flash.blocking_write(self.start_addr, &buf).unwrap();
    }
    pub fn clear(&mut self) {
        self.map = Map::default();
    }
    pub fn update(&mut self, buff: &[u8]) {
        // buff.chunks(16);
        if let Ok(bind) = KeyType::try_from(&buff[2..]) {
            self.map[buff[0] as usize][buff[1] as usize] = bind;
        } else {
            defmt::error!("Invalid keybind: {:?}", buff);
        }
    }
}
