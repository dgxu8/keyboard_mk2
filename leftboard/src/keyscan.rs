use embassy_stm32::{gpio::{Input, Output}, pac};
use embassy_time::{Duration, Instant, Ticker};
use util::debounce;

static ROW_LEN: usize = 5;
static COL_LEN: usize = 8;

#[embassy_executor::task]
pub async fn run(mut keys: Keyscan<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        let start = Instant::now();
        let change =  keys.scan();
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
    fn scan(&mut self) -> bool {
        let mut change = false;
        // We enter here w/ decoder set to col zero
        for col in 0..8 {
            let notify = |row, state| {
                defmt::info!("({},{}): {}", col, row, state);
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
