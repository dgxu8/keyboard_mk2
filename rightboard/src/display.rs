use core::fmt::Write;

use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::pipe::Pipe;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_9X18};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use embedded_graphics::{image::Image, prelude::Point};
use heapless::String;
use ssd1306::mode::BufferedGraphicsModeAsync;
use ssd1306::prelude::I2CInterface;
use ssd1306::size::DisplaySize128x32;
use ssd1306::Ssd1306Async;

use crate::rotary::EncoderState;

macro_rules! raw_to_image {
    ($file:expr, $width:expr, $x:expr, $y:expr) => {
        Image::new(&ImageRaw::new(include_bytes!($file), $width), Point::new($x as i32, $y as i32))
    };
}

pub static OLED_STR: Pipe<CriticalSectionRawMutex, 16> = Pipe::new();

pub static DISPLAY_DRAW: Channel<CriticalSectionRawMutex, Draw, 5> = Channel::new();
pub enum Draw {
    Numlock(bool),
    Capslock(bool),
    Volume(u8),

    /* - Development Only - */
    Timestamp(u64),
    EncoderState(EncoderState),
    String(u8),  // column offset
    FlashIco,
}

type MonoImage = Image<'static, ImageRaw<'static, BinaryColor>>;
type DisplayAsync = Ssd1306Async<I2CInterface<I2c<'static, Async, Master>>, DisplaySize128x32, BufferedGraphicsModeAsync<DisplaySize128x32>>;

const WIDTH: u32 = 128;
const HEIGHT: u32 = 32;

const ICON_WIDTH: u32 = 13;
const ICON_HEIGHT: u32 = 19;

// Put Numlock to the far right side but centered
const NUM_X: u32 = WIDTH - ICON_WIDTH;
const NUM_Y: u32 = (HEIGHT - ICON_HEIGHT) / 2;

// Align Capslock next to numlock
const CAPS_X: u32 = NUM_X - ICON_WIDTH - 2;
const CAPS_Y: u32 = NUM_Y;

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn display_draw(mut display: DisplayAsync) {

    let text_style = MonoTextStyleBuilder::new().font(&FONT_5X8).text_color(BinaryColor::On).build();
    let recv = DISPLAY_DRAW.receiver();

    const NUM_ICON: MonoImage = raw_to_image!("./numlock.raw", ICON_WIDTH, NUM_X, NUM_Y);
    const CAPS_ICON: MonoImage = raw_to_image!("./capslock.raw", ICON_WIDTH, CAPS_X, CAPS_Y);

    let mut str: String<16> = String::new();
    loop {
        let fut = recv.receive().await;
        task_profiler::set!();
        match fut {
            Draw::Numlock(state) => {
                display.draw_image(&NUM_ICON, state);
            },
            Draw::Capslock(state) => {
                display.draw_image(&CAPS_ICON, state);
            },
            Draw::Volume(level) => {
                display.draw_volume(level, Point::new(CAPS_X as i32 - (10 * 4), 8));
            },
            Draw::Timestamp(time) => {
                str.clear();
                core::write!(&mut str, "{}", time).unwrap();

                display.clear_box(Point::zero(), Size::new(12*5, 10));
                Text::with_baseline(&str, Point::zero(), text_style, Baseline::Top).draw(&mut display).unwrap();
            },
            Draw::EncoderState(state) => {
                str.clear();
                core::write!(&mut str, "{}:{}", state.pos, state.interrupts).unwrap();

                display.clear_box(Point::new(0, 10), Size::new(12*5, 10));
                Text::with_baseline(&str, Point::new(0, 10), text_style, Baseline::Top).draw(&mut display).unwrap();
            },
            Draw::String(col) => {
                let mut buf = [0; 12];
                if let Ok(len) = OLED_STR.try_read(&mut buf) {
                    display.clear_box(Point::new(0, col as _), Size::new(12*5, 10));
                    let s = core::str::from_utf8(&buf[..len]).unwrap();
                    Text::with_baseline(s, Point::new(0, col as _), text_style, Baseline::Top).draw(&mut display).unwrap();
                }
            },
            Draw::FlashIco => {
                display.clear_box(Point::new(0, 10), Size::new(12*5, 10));
                Text::with_baseline("Flashing", Point::new(0, 10), text_style, Baseline::Top).draw(&mut display).unwrap();
            },
        }
        display.flush().await.unwrap();
        {
            str.clear();
            core::write!(&mut str, "{} {}", task_profiler::get_sync!(), task_profiler::get_async!()).unwrap();
            display.clear_box(Point::new(0, 20), Size::new(16*5, 10));
            Text::with_baseline(&str, Point::new(0, 20), text_style, Baseline::Top).draw(&mut display).unwrap();
            display.flush().await.unwrap();
        }
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
trait KBHelper {
    fn draw_image(&mut self, image: &MonoImage, draw: bool);
    fn draw_volume(&mut self, level: u8, pos: Point);
    fn clear_box(&mut self, pos: Point, size: Size);
}

const CLEAR_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
    .fill_color(BinaryColor::Off)
    .build();

impl KBHelper for DisplayAsync {
    fn draw_image(&mut self, image: &MonoImage, draw: bool) {
        if draw {
            image.draw(&mut self.color_converted()).unwrap();
        } else {
            image.bounding_box().into_styled(CLEAR_STYLE).draw(&mut self.color_converted()).unwrap();
        }
    }
    fn draw_volume(&mut self, level: u8, pos: Point) {
        const STYLE: MonoTextStyle<'static, BinaryColor> = MonoTextStyleBuilder::new().font(&FONT_9X18).text_color(BinaryColor::On).build();
        let mut str: String<4> = String::new();
        core::write!(&mut str, "{:>3}%", level).unwrap();
        self.clear_box(pos, Size::new(3*9, 18));
        Text::with_baseline(&str, pos, STYLE, Baseline::Top).draw(self).unwrap();
    }
    fn clear_box(&mut self, pos: Point, size: Size) {
        Rectangle::new(pos, size)
            .into_styled(CLEAR_STYLE)
            .draw(self)
            .unwrap();
    }
}
