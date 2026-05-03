use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::{Mutex, raw::{NoopRawMutex}};
use embassy_time::Instant;
use static_cell::StaticCell;
use util::{debounce::{RB_COL_LEN, RB_ROW_LEN}, keycom::KeyType};

use crate::state_machine::Layer;

const NUM_ROW_BOT: usize = 0;
const NUM_ROW_LEN: usize = 7;
const NUM_COL_LEN: usize = 3;

pub type KeymapMutex = Mutex<NoopRawMutex, Keymap<'static>>;
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

#[derive(Clone, Copy, defmt::Format)]
pub struct KeyBinds {
    id: usize,
    arrow_code: KeyType,
    numpad_code: KeyType,
    pub timeout: Instant,
}

impl PartialEq for KeyBinds {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl KeyBinds {
    pub fn new(col: usize, row: usize, arrow_code: KeyType, numpad_code: KeyType) -> Self {
        Self {
            id: col << 4 | row,
            arrow_code,
            numpad_code,
            timeout: Instant::now(),
        }
    }
    #[inline]
    pub fn get(self, layer: Layer) -> KeyType {
        match layer {
            Layer::Arrow => self.arrow_code,
            Layer::Numpad => self.numpad_code,
        }
    }
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
    pub fn map(&self, col: usize, row: usize, layer: Layer) -> KeyType {
        if layer == Layer::Numpad && col < NUM_COL_LEN &&  row > NUM_ROW_BOT && row < NUM_ROW_LEN {
            self.keymap.numpad[col][row]
        } else {
            self.keymap.right[col][row]
        }
    }
    pub fn get_info(&self, col: usize, row: usize) -> KeyBinds {
        assert!(col < NUM_COL_LEN && row > NUM_ROW_BOT && row < NUM_ROW_LEN);
        KeyBinds::new(
            col, row,
            self.keymap.right[col][row].dance_to(),
            self.keymap.numpad[col][row].dance_to(),
        )
    }
}

/*
 *   +-------+-------+-------+-------+   +-------+-------+-------+-------+
 *   | (7,0) | (6,0) | (5,0) | (4,0) |   | (3,0) | (2,0) | (1,0) | (0,0) |
 *   +-------+-------+-------+-------+   +-------+-------+-------+-------+
 *   +-------+-------+-------+-------+-------+-------+-------+-----------+
 *   | (7,1) | (6,2) | (6,1) | (5,2) | (5,1) | (4,2) | (4,1) |   (3,1)   |
 *   +-------+-------+-------+-------+-------+-------+-------+-----------+
 *   | (7,2) | (6,3) | (5,3) | (5,4) | (4,4) | (4,3) | (3,3) |   (3,2)   |
 *   +-------+-------+-------+-------+-------+-------+-------+---------+-+
 *   | (7,3) | (6,4) | (6,5) | (5,5) | (4,5) | (3,5) |      (3,4)      |
 *   +-------+-------+-------+-------+-------+-------+-------+---------+
 *   | (7,4) | (7,5) | (7,6) | (6,6) | (5,6) | (4,6) | (3,6) |
 *   +-------+-------+-------+-------+-------+-------+-------+
 *           | (7,7) |     (6,7)     | (5,7) | (4,7) | (3,7) |
 *           +-------+---------------+-------+-------+-------+
 */

/*      2       1       0
 *   +-------+-------+-------+-------+
 * 0 | (2,2) | (2,1) | (1,1) | (0,1) |
 *   +-------+-------+-------+-------+
 * 1 | (2,3) | (1,3) | (1,2) |       |
 *   +-------+-------+-------+ (0,2) |
 * 2 | (2,4) | (1,4) | (0,3) |       |
 *   +-------+-------+-------+-------+
 * 3 | (2,5) | (1,5) | (0,5) |       |
 *   +-------+-------+-------+ (0,4) |
 * 4 | (2,6) | (1,6) | (0,6) |       |
 *   +-------+-------+-------+-------+
 */

/*
 *   +-------+-------+-------+-------+   +-------+-------+-------+-------+
 *   | (7,0) | (6,0) | (5,0) | (4,0) |   | (3,0) | (2,0) | (1,0) | (0,0) |    (0,7)
 *   +-------+-------+-------+-------+   +-------+-------+-------+-------+
 *   +-------+-------+-------+-------+-------+-------+-------+-----------+  +-------+-------+-------+-------+
 *   | (7,1) | (6,2) | (6,1) | (5,2) | (5,1) | (4,2) | (4,1) |   (3,1)   |  | (2,2) | (2,1) | (1,1) | (0,1) |
 *   +-------+-------+-------+-------+-------+-------+-------+-----------+  +-------+-------+-------+-------+
 *   | (7,2) | (6,3) | (5,3) | (5,4) | (4,4) | (4,3) | (3,3) |   (3,2)   |  | (2,3) | (1,3) | (1,2) |       |
 *   +-------+-------+-------+-------+-------+-------+-------+---------+-+  +-------+-------+-------+ (0,2) |
 *   | (7,3) | (6,4) | (6,5) | (5,5) | (4,5) | (3,5) |      (3,4)      |    | (2,4) | (1,4) | (0,3) |       |
 *   +-------+-------+-------+-------+-------+-------+-------+---------+    +-------+-------+-------+-------+
 *   | (7,4) | (7,5) | (7,6) | (6,6) | (5,6) | (4,6) | (3,6) |              | (2,5) | (1,5) | (0,5) |       |
 *   +-------+-------+-------+-------+-------+-------+-------+              +-------+-------+-------+ (0,4) |
 *           | (7,7) |     (6,7)     | (5,7) | (4,7) | (3,7) |              | (2,6) | (1,6) | (0,6) |       |
 *           +-------+---------------+-------+-------+-------+              +-------+-------+-------+-------+
 *                                                                          Not in keypad: (2,7) (1,7) (0,7)
 *                                                                          Used for rotary button: (0,7)
 */
