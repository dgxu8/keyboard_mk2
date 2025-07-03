use embassy_time::{Duration, Instant, Ticker};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::pac;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use heapless::{String, Vec};
use static_cell::StaticCell;
use core::fmt::Write;
use core::ops::{DerefMut, Range};

use core::hint::cold_path;

use crate::serial::{self, CobsTx};
use crate::{DisplayAsyncMutex, UartAsyncMutex};

pub static ALT_EN: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static REPORT_FULL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static KEYS: StaticCell<Keyscan> = StaticCell::new();

const ROW_LEN: usize = 8;
const COL_LEN: usize = 8;

const FLIP: u8 = 5;
const MAX: u8 = 10;
const MIN: u8 = 0;
const OVERSHOOT: u8 = 25;

#[derive(Debug)]
enum Error {
    UpdateOverflow,
}

type KeyUpdate = Vec<u8, 8>;
type BoardState = [u8; COL_LEN];

#[embassy_executor::task]
pub async fn key_scan(keys: &'static mut Keyscan<'static>, uart_tx: &'static UartAsyncMutex, display: &'static DisplayAsyncMutex) {
    let mut str: String<16> = String::new();
    let text_style = MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build();
    let mut max = Duration::default();

    let mut ticker = Ticker::every(Duration::from_millis(1));
    //let mut ticker = Ticker::every(Duration::from_micros(10));
    let mut alt_en = false;
    loop {
        // If we get more than 8 changes send a full keystate scan
        let start: Instant = Instant::now();
        alt_en = ALT_EN.try_take().unwrap_or(alt_en);
        let scan = keys.scan(alt_en);

        if REPORT_FULL.try_take().is_some() || scan.is_err() {
            cold_path();
            let mut uart_tx = uart_tx.lock().await;
            let state_id = if keys.alt_en {serial::ALT_STATE} else {serial::FULL_STATE};
            uart_tx.write_cobs(serial::TIMESTAMP, start.as_micros().to_le_bytes().as_slice()).await.unwrap();
            uart_tx.write_cobs(state_id, keys.get_full_state().as_slice()).await.unwrap();
        } else if let Ok(update) = scan {
            if !update.is_empty() {
                let mut uart_tx = uart_tx.lock().await;
                uart_tx.write_cobs(serial::TIMESTAMP, start.as_micros().to_le_bytes().as_slice()).await.unwrap();
                uart_tx.write_cobs(serial::KEY_CHANGE, update.as_slice()).await.unwrap();
            }
        }
        let elapsed = Instant::now() - start;
        if elapsed > max {
            let mut display = display.lock().await;
            display.clear(BinaryColor::Off).unwrap();
            str.clear();
            core::write!(&mut str, "{}", elapsed.as_micros()).unwrap();
            Text::with_baseline(&str, Point::zero(), text_style, Baseline::Top).draw(display.deref_mut()).unwrap();
            display.flush().await.unwrap();
            max = elapsed;
        }
        ticker.next().await;
    }
}

pub struct Keyscan<'a> {
    // While we use the PAC to read/write these pins hold on to these pins so nothing
    // else can use them
    _select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    _input_pins: [Input<'a>; ROW_LEN],
    enc_btn: Input<'a>,
    toggle: Input<'a>,
    state: [[u8; ROW_LEN]; COL_LEN],
    alt_en: bool,
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; 8], enc_btn: Input<'a>, toggle: Input<'a>) -> Self {
        let alt_en = toggle.is_high();
        Self {
            _select_pins: select_pins,
            _enable: enable,
            _input_pins: input_pins,
            enc_btn,
            toggle,
            state: [[0u8; ROW_LEN]; COL_LEN],
            alt_en,
        }
    }

    //    7     6-4     3-1     0
    // +-----+--------+-----+-------+
    // | Alt | Column | Row | State |
    // +-----+--------+-----+-------+
    #[inline(always)]
    pub fn push(&mut self, col: u8, row: u8, state: u8, update: &mut KeyUpdate) -> bool {
        update.push((self.alt_en as u8) << 7 | col << 4 | row << 1 | state).is_err()
    }

    #[allow(dead_code)]
    #[inline(always)]
    fn set(&mut self, val: u8) {
        for i in 0..self._select_pins.len() {
            self._select_pins[i].set_level((val & (1 << i) != 0).into());
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

    fn get_full_state(&self) -> BoardState {
        let mut state: BoardState = [0; COL_LEN];
        for (i, col) in self.state.iter().enumerate() {
            for (j, val) in col.iter().enumerate() {
                let val = (*val > FLIP) as u8;
                state[i] |= val << j;
            }
        }
        state
    }

    #[allow(dead_code)]
    fn scan(&mut self, alt_en: bool) -> Result<KeyUpdate, Error> {
        let mut update: KeyUpdate = Vec::new();
        self.alt_en = alt_en || self.toggle.is_high();
        for col in 0..COL_LEN {
            self.set_raw(col as _);

            // The last row isn't full (missing col 0-2) so skip those and start at 3
            let reg = self.read_raw();
            let end = if col < 3 {ROW_LEN-1} else {ROW_LEN};
            self.read_int(reg, 0..end, col, &mut update);
            // self.read_shift(reg, 0..end, col, &mut update);
            // self.read_no_debounce(reg, 0..end, col, &mut update);
        }
        if self.read_int(self.enc_btn.is_low() as _, 7..8, 0, &mut update) {
            Err(Error::UpdateOverflow)
        } else {
            Ok(update)
        }
    }

    #[allow(dead_code)]
    #[inline(always)]
    fn read_no_debounce(&mut self, reg: u32, range: Range<usize>, col: usize, update: &mut KeyUpdate) -> bool {
        let mut state = reg;
        let mut overflow = false;
        for row in range {
            let val = (state & 0x1) as u8;
            if self.state[col][row] != val {
                self.state[col][row] = val;
                overflow = self.push(col as _, row as _, val, update);
            }
            state >>= 1;
        }
        overflow
    }

    #[allow(dead_code)]
    #[inline(always)]
    fn read_shift(&mut self, reg: u32, range: Range<usize>, col: usize, update: &mut KeyUpdate) -> bool {
        let mut state = reg;
        let mut overflow = false;
        for row in range {
            let mut val = self.state[col][row] & 0b1000_0000;
            val |= (self.state[col][row] << 1) & 0b0001_1111;
            val |= (state & 0x1) as u8;
            if val == 0b0001_1111 {
                self.state[col][row] = 0b1001_1111;
                overflow = self.push(col as _, row as _, 1, update);
            } else if val == 0b1000_0000 {
                self.state[col][row] = 0;
                overflow = self.push(col as _, row as _, 0, update);
            } else {
                self.state[col][row] = val;
            }
            state >>= 1;
        }
        overflow
    }

    #[allow(dead_code)]
    #[inline(always)]
    fn read_int(&mut self, reg: u32, range: Range<usize>, col: usize, update: &mut KeyUpdate) -> bool {
        let mut state = reg;
        let mut overflow = false;
        for row in range {
            if state & 0x1 == 1 {
                if self.state[col][row] < MAX {
                    self.state[col][row] += 1;
                    if self.state[col][row] == FLIP {
                        // Add overshoot factor so we can adjust the minimum press time
                        self.state[col][row] = MAX + OVERSHOOT;
                        overflow = self.push(col as _, row as _, 1, update);
                    }
                }
            } else {
                if self.state[col][row] > MIN {
                    self.state[col][row] -= 1;
                    if self.state[col][row] == FLIP {
                        self.state[col][row] = MIN;
                        overflow = self.push(col as _, row as _, 0, update);
                    }
                }
            }
            state >>= 1;
        }
        overflow
    }
}
