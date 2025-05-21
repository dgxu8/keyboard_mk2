use embassy_stm32::gpio::{Input, Output};

pub struct Keyscan<'a> {
    select_pins: [Output<'a>; 3],
    enable: Output<'a>,
    input_pins: [Input<'a>; 8],
}

impl<'a> Keyscan<'a> {
    pub fn new(select_pins: [Output<'a>; 3], enable: Output<'a>, input_pins: [Input<'a>; 8]) -> Self {
        Self {
            select_pins,
            enable,
            input_pins,
        }
    }

    pub fn set(&mut self, val: u8) {
        self.enable.set_low();
        for i in 0..self.select_pins.len() {
            self.select_pins[i].set_level((val & (1 << i) != 0).into());
        }
        // self.select_pins[0].set_level((val & 0x1 != 0).into());
        // self.select_pins[1].set_level((val & 0x2 != 0).into());
        // self.select_pins[2].set_level((val & 0x4 != 0).into());
        self.enable.set_high();
    }

    pub fn read_inputs(&mut self) -> u8 {
        let mut bits: u8 = 0;
        for i in 0..self.input_pins.len() {
            bits |= (self.input_pins[i].is_high() as u8) << i;
        }
        bits
    }
}
