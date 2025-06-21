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
use core::ops::DerefMut;

use core::hint::cold_path;

use crate::serial::CobsTx;
use crate::{DisplayAsyncMutex, UartAsyncMutex};

pub static SCAN: Signal<CriticalSectionRawMutex, Scan> = Signal::new();
pub static KEYS: StaticCell<Keyscan> = StaticCell::new();
pub static LEDS: StaticCell<Output> = StaticCell::new();

const ROW_LEN: usize = 8;
const COL_LEN: usize = 8;

#[derive(Debug)]
enum Error {
    UpdateOverflow,
}

type KeyUpdate = Vec<u8, 8>;

// Column x Row
pub struct Scan {
    pub state: [u8; 8],
    pub scan_time: u64,
}
impl Scan {
    pub fn new() -> Self {
        Self {
            state: [0; 8],
            scan_time: 0,
        }
    }
}

#[embassy_executor::task]
pub async fn key_scan(keys: &'static mut Keyscan<'static>, uart_tx: &'static UartAsyncMutex, display: &'static DisplayAsyncMutex) {
    let mut str: String<16> = String::new();
    let text_style = MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build();
    let mut max = Duration::default();

    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        // If we get more than 8 changes send a full keystate scan
        let start: Instant = Instant::now();
        // let scan = keys.scan_no_debounce();
        // let scan = keys.scan_integrate();
        let scan = keys.scan_shift();
        let elapsed = Instant::now() - start;
        match scan {
            Ok(update) => {
                if !update.is_empty() {
                    let mut uart_tx = uart_tx.lock().await;
                    uart_tx.write_cobs(start.as_micros().to_le_bytes().as_slice()).await.unwrap();
                    uart_tx.write_cobs(update.as_slice()).await.unwrap();
                }
            },
            Err(_) => {
                cold_path();
                let mut uart_tx = uart_tx.lock().await;
                uart_tx.write_cobs(start.as_micros().to_le_bytes().as_slice()).await.unwrap();
                uart_tx.write_cobs(keys.get_full_state().as_slice()).await.unwrap();
            },
        };
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
    select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    input_pins: [Input<'a>; 8],

    state: [[u8; ROW_LEN]; COL_LEN],
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; 8]) -> Self {
        Self {
            select_pins,
            _enable: enable,
            input_pins,
            state: [[0u8; ROW_LEN]; COL_LEN],
        }
    }

    #[inline(always)]
    pub fn set(&mut self, val: u8) {
        for i in 0..self.select_pins.len() {
            self.select_pins[i].set_level((val & (1 << i) != 0).into());
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

    pub fn read_inputs(&mut self) -> u8 {
        let mut bits: u8 = 0;
        for i in 0..self.input_pins.len() {
            bits |= (self.input_pins[i].is_high() as u8) << i;
        }
        bits
    }

    pub fn scan(&mut self) -> [u8; 8] {
        let mut state = [0; 8];
        for i in 0..self.select_pins.len() {
            self.set(i as u8);
            state[i] |= self.read_inputs();
        }
        state
    }

    fn get_full_state(&self) -> [u8; 8] {
        let mut state: [u8; 8] = [0; 8];
        for (i, col) in self.state.iter().enumerate() {
            for (j, val) in col.iter().enumerate() {
                state[i] |= val << j;
            }
        }
        state
    }

    #[allow(dead_code)]
    fn scan_no_debounce(&mut self) -> Result<KeyUpdate, Error> {
        let mut update: KeyUpdate = Vec::new();
        let mut overflow = false;
        for col in 0..8 {
            self.set_raw(col as _);
            for row in 0..self.input_pins.len() {
                let val = self.input_pins[row].is_high() as u8;
                if self.state[col][row] != val {
                    self.state[col][row] = val;
                    overflow = update.push((col as u8) << 5 | (row as u8) << 1 | val).is_err();
                }
            }
        }
        if overflow {
            Err(Error::UpdateOverflow)
        } else {
            Ok(update)
        }
    }

    #[allow(dead_code)]
    fn scan_integrate(&mut self) -> Result<KeyUpdate, Error> {
        let mut update: KeyUpdate = Vec::new();
        let mut overflow = false;
        for col in 0..8 {
            self.set_raw(col as _);

            for row in 0..self.input_pins.len() {
                if self.input_pins[row].is_high() {
                    if self.state[col][row] < 10 {
                        self.state[col][row] += 1;
                        if self.state[col][row] == 5 {
                            self.state[col][row] = 10;
                            overflow = update.push((col as u8) << 5 | (row as u8) << 1 | 1).is_err();
                        }
                    }
                } else {
                    if self.state[col][row] > 0 {
                        self.state[col][row] -= 1;
                        if self.state[col][row] == 5 {
                            self.state[col][row] = 0;
                            overflow = update.push((col as u8) << 5 | (row as u8) << 1 | 0).is_err();
                        }
                    }
                }
            }
        }
        if overflow {
            Err(Error::UpdateOverflow)
        } else {
            Ok(update)
        }
    }

    #[allow(dead_code)]
    fn scan_shift(&mut self) -> Result<KeyUpdate, Error> {
        let mut update: KeyUpdate = Vec::new();
        let mut overflow = false;
        for col in 0..8 {
            self.set_raw(col as _);

            for row in 0..self.input_pins.len() {
                let mut val = self.state[col][row] & 0b1000_0000;
                val |= (self.state[col][row] << 1) & 0b0001_1111;
                val |= self.input_pins[row].is_high() as u8;
                if val == 0b0001_1111 {
                    self.state[col][row] = 0b1001_1111;
                    overflow = update.push((col as u8) << 5 | (row as u8) << 1 | 1).is_err();
                } else if val == 0b1000_0000 {
                    self.state[col][row] = 0;
                    overflow = update.push((col as u8) << 5 | (row as u8) << 1 | 0).is_err();
                } else {
                    self.state[col][row] = val;
                }
            }
        }
        if overflow {
            Err(Error::UpdateOverflow)
        } else {
            Ok(update)
        }
    }

    #[allow(dead_code)]
    async fn send_state(col: u8, row: u8, val: u8, uart_tx: &UartAsyncMutex) {
        let mut msg: Vec<u8, 16> = Vec::new();
        let time = Instant::now();

        msg.push((col as u8) << 5 | (row as u8) << 1 | val).unwrap();
        msg.extend_from_slice(&time.as_micros().to_le_bytes()).unwrap();

        let mut uart_tx = uart_tx.lock().await;
        uart_tx.write_cobs(msg.as_slice()).await.unwrap();
    }
}
