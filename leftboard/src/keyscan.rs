use core::ops::{Deref, DerefMut};

use embassy_stm32::{flash::{Bank1Region, Blocking}, gpio::{Input, Output}, pac};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use static_cell::StaticCell;
use util::{debounce, keymap::KeyType};

static ROW_LEN: usize = 5;
static COL_LEN: usize = 8;

#[embassy_executor::task]
pub async fn run(mut keys: Keyscan<'static>, keymap: &'static KeymapMutex) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        let start = Instant::now();
        let change =  {
            let mut keymap = keymap.lock().await;
            keys.scan(&mut keymap)
        };
        let dur = Instant::now() - start;
        if change {
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

    // fn scan(&mut self, update: &mut Vec<u8, 8>) {
    #[inline(always)]
    fn scan(&mut self, keymap: &mut Keymap) -> bool {
        let mut change = false;
        // We enter here w/ decoder set to col zero
        for col in 0..8 {
            let notify = |row, state| {
                defmt::info!("{}: {}", keymap.map[col as usize][row as usize], state);
                change = true;
            };
            let reg = self.read_raw();
            // Set next decoder for next column so it has time to propagate
            self.set_raw((col+1) as u32);
            debounce::debounce(&mut self.state[col], reg, notify);
        }
        change
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
