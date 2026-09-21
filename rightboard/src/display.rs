use core::fmt::Write;
use core::ops::Deref;

use display_interface::DisplayError;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::pipe::Pipe;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Instant;
use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_9X18};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use embedded_graphics::prelude::Point;
use heapless::String;
use ssd1306::mode::BufferedGraphicsModeAsync;
use ssd1306::prelude::I2CInterface;
use ssd1306::size::{DisplaySize128x32, DisplaySizeAsync};
use ssd1306::Ssd1306Async;
use tinybmp::RawBmp;

use crate::rotary::EncoderState;

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

type DisplayAsync = Ssd1306Async<I2CInterface<I2c<'static, Async, Master>>, DisplaySize128x32, BufferedGraphicsModeAsync<DisplaySize128x32>>;

const WIDTH: usize = <DisplaySize128x32 as DisplaySizeAsync>::WIDTH as usize;  // 16
const HEIGHT: usize = <DisplaySize128x32 as DisplaySizeAsync>::HEIGHT as usize;  // 4

// Put capslock to the far right side but centered
const CAPS_X: u32 = 115; // 128 - 13
const CAPS_Y: u32 = 6; // (32 - 19) / 2

// Align numlock next to capslock
const NUM_X: u32 = 100; // 2 to the left of NUM X
const NUM_Y: u32 = 6;

const fn const_unwrap<T: Copy, E: Copy>(res: Result<T, E>) -> T {
    match res {
        Ok(val) => val,
        Err(_) => panic!("Failed to import bitmap"),
    }
}

macro_rules! bmp_to_icon {
    ($file:expr, $x:expr, $y:expr) => {{
        const BMP: RawBmp<'static> = const_unwrap(RawBmp::from_slice(include_bytes!($file)));
        const WIDTH: usize = BMP.header().image_size.width as usize;
        const BUFFER: [u8; {WIDTH * 4}] = bmp_to_ssd1306::<{WIDTH * 4}>(&BMP, $y as usize);
        Icon::new(&BUFFER, $x as u8, WIDTH as u8)
    }};
}
struct Icon {
    buf: &'static [u8],
    x_start: u8,
    x_end: u8,
}

impl Icon {
    const fn new(buf: &'static [u8], x: u8, width: u8) -> Self {
        Icon {
            buf,
            x_start: x,
            x_end: x + width,
        }
    }

    #[inline(always)]
    async fn display(&self, display: &mut DisplayAsync, draw: bool) -> Result<(), DisplayError> {
        if draw {
            self.draw(display).await
        } else {
            self.clear(display).await
        }
    }

    async fn draw(&self, display: &mut DisplayAsync) -> Result<(), DisplayError> {
        let start = Instant::now();
        display.set_draw_area((self.x_start, 0), (self.x_end, 32)).await?;
        let mid = Instant::now();
        display.draw(self.buf).await?;
        let end = Instant::now();
        defmt::info!("draw> set: {}, draw: {}",
            (mid-start).as_micros() as u16,
            (end-mid).as_micros() as u16,
        );
        Ok(())
    }

    async fn clear(&self, display: &mut DisplayAsync) -> Result<(), DisplayError> {
        let start = Instant::now();
        display.set_draw_area((self.x_start, 0), (self.x_end, 32)).await?;
        let mid = Instant::now();
        display.draw(&BLANK[..self.buf.len()]).await?;
        let end = Instant::now();
        defmt::info!("clear> set: {}, draw: {}",
            (mid-start).as_micros() as u16,
            (end-mid).as_micros() as u16,
        );
        Ok(())
    }
}

impl Deref for Icon {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.buf
    }
}

static BLANK: [u8; 512] = [0; 512];

const fn bmp_to_ssd1306<const N: usize>(bmp: &RawBmp, y_offset: usize) -> [u8; N] {
    // [y][x]
    let header = bmp.header();
    let row_padded = header.bytes_per_row().unwrap();
    let height = header.image_size.height as usize;
    let width = header.image_size.width as usize;
    let width_bytes = (width + 7) / 8;
    let data = bmp.image_data();

    let mut tmp_buf = [[0; WIDTH]; HEIGHT];
    let mut row = 0;
    while row < height {
        let mut col = 0;
        while col < width_bytes {
            let mut pixels = data[(row * row_padded) + col];
            let mut bit = 0;
            while pixels != 0 {
                // The bits are big endian (left most pixel is the most significant bit)
                tmp_buf[row+y_offset][(col*8)+bit] = (pixels >> 7) & 0x1;
                pixels <<= 1;
                bit += 1;
            }
            col += 1;
        }
        row += 1;
    }

    let mut buf = [0; N];
    let mut seg = 0;
    let mut idx = 0;
    while seg < 4 {
        let mut x = 0;
        while x < width {
            let mut i = 0;
            let mut val = 0;
            while i < 8 {
                val |= (tmp_buf[(seg*8)+i][x]) << i;
                i += 1;
            }
            buf[idx] = val;
            idx += 1;
            x += 1;
        }
        seg += 1;
    }
    buf
}

#[task_profiler::profile]
#[embassy_executor::task]
pub async fn display_draw(mut display: DisplayAsync) {

    let init_start = Instant::now();
    let text_style = MonoTextStyleBuilder::new().font(&FONT_5X8).text_color(BinaryColor::On).build();
    let recv = DISPLAY_DRAW.receiver();
    static NUM_ICON: Icon = bmp_to_icon!("../bitmaps/numlock.bmp", NUM_X, NUM_Y);
    static CAPS_ICON: Icon = bmp_to_icon!("../bitmaps/capslock.bmp", CAPS_X, CAPS_Y);
    let init_time = Instant::now() - init_start;

    defmt::info!("> init time: {}", init_time.as_micros() as u16);

    let mut str: String<16> = String::new();
    loop {
        let fut = recv.receive().await;
        task_profiler::set!();
        match fut {
            Draw::Numlock(state) => {
                NUM_ICON.display(&mut display, state).await.unwrap();
                continue;
            },
            Draw::Capslock(state) => {
                CAPS_ICON.display(&mut display, state).await.unwrap();
                continue;
            },
            Draw::Volume(level) => {
                display.draw_volume(level, Point::new(NUM_X as i32 - (10 * 4), 8));
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
        task_profiler::print!();
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
trait KBHelper {
    fn draw_volume(&mut self, level: u8, pos: Point);
    fn clear_box(&mut self, pos: Point, size: Size);
}

const CLEAR_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
    .fill_color(BinaryColor::Off)
    .build();

impl KBHelper for DisplayAsync {
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
