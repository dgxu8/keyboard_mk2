use embassy_time::{Instant, Timer};
use embassy_stm32::gpio::{Input, Output};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;


pub static SCAN: Signal<CriticalSectionRawMutex, Scan> = Signal::new();

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
pub async fn key_scan(keys: &'static mut Keyscan<'static>, led: &'static mut Output<'static>) {
    loop {
        let mut scan: Scan = Scan::new();
        Timer::after_millis(5).await;
        let start: Instant = Instant::now();
        scan.state = keys.scan_full();
        let elapsed = Instant::now() - start;
        if scan.state[0] > 0 {
            led.set_high();
            scan.scan_time = elapsed.as_micros();
            SCAN.signal(scan);
        } else {
            led.set_low();
        }
    }
}

pub struct Keyscan<'a> {
    select_pins: [Output<'a>; 3],
    _enable: Output<'a>,
    input_pins: [Input<'a>; 8],
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; 8]) -> Self {
        Self {
            select_pins,
            _enable: enable,
            input_pins,
        }
    }

    pub fn set(&mut self, val: u8) {
        for i in 0..self.select_pins.len() {
            self.select_pins[i].set_level((val & (1 << i) != 0).into());
        }
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

    pub fn scan_full(&mut self) -> [u8; 8] {
        let mut state = [0; 8];
        for i in 0..self.select_pins.len() {
            for j in 0..self.select_pins.len() {
                self.select_pins[j].set_level((i & (1 << j) != 0).into());
            }
            for j in 0..self.input_pins.len() {
                state[i] |= (self.input_pins[j].is_high() as u8) << j;
            }
        }
        state
    }
}
