#![no_std]
#![no_main]
#![feature(cold_path)]
#![feature(likely_unlikely)]
#![feature(impl_trait_in_assoc_type)]

extern crate panic_halt;

pub mod keyscan;
pub mod rotary;
pub mod serial;
pub mod display;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_io::ReadReady;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllSource, Sysclk};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, i2c, usart, Config};
use embassy_time::Timer;
use ssd1306::mode::DisplayConfigAsync;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x32;
use ssd1306::{I2CDisplayInterface, Ssd1306Async};
use static_cell::StaticCell;

use crate::rotary::{encoder_monitor, ENCODER_STATE};
use crate::keyscan::{key_scan, Keyscan, ALT_EN, KEYS, REPORT_FULL};
use crate::serial::{CobsRx, CobsTx, SerialBuffer};
use crate::display::{display_draw, Draw, DISPLAY_DRAW};

bind_interrupts!(struct UsartIrqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

bind_interrupts!(struct I2CIrqs {
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

type UartAsyncMutex = Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = Config::default();
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.hsi = true;
    config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        div: PllDiv::DIV2,
        mul: PllMul::MUL4,
    });
    let p = embassy_stm32::init(config);
    //
    // Configure LEDs
    let mut led0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PB5, Level::High, Speed::Low);

    // Configure Decoder
    let key_select_pins: [Output; 3] = [
        Output::new(p.PA7, Level::Low, Speed::VeryHigh),
        Output::new(p.PA6, Level::Low, Speed::VeryHigh),
        Output::new(p.PA5, Level::Low, Speed::VeryHigh),
    ];
    // Configure Decoder input
    let key_inputs: [Input; 8] = [
        Input::new(p.PA8, Pull::Down),
        Input::new(p.PA9, Pull::Down),
        Input::new(p.PA10, Pull::Down),
        Input::new(p.PA11, Pull::Down),
        Input::new(p.PA12, Pull::Down),
        Input::new(p.PA13, Pull::Down),
        Input::new(p.PA14, Pull::Down),
        Input::new(p.PA15, Pull::Down),
    ];

    let keys = Keyscan::new(
        key_select_pins,
        Output::new(p.PA4, Level::High, Speed::High),
        key_inputs,
        Input::new(p.PB15, Pull::None),
        Input::new(p.PB0, Pull::Down),
    );

    // Configure usart default:
    // baudrate: 115200,
    // data_bits: DataBits::DataBits8,
    // stop_bits: StopBits::STOP1,
    // parity: Parity::ParityNone,
    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 2_000_000;
    let uart = Uart::new(
        p.USART2,
        p.PA3, p.PA2,  // RX, TX
        UsartIrqs,
        p.DMA1_CH7, p.DMA1_CH6,  // TX, RX
        uart_cfg,
    ).unwrap();
    let (uart_tx, uart_rx) = uart.split();
    led0.set_high();

    // Keys and led1 are moved into keys_scan's context forever. If we need shared access we need
    // to wrap Keyscan/Output in an embassy_sync::mutex and put that into the StaticCell
    static UART_TX: StaticCell<UartAsyncMutex> = StaticCell::new();
    let uart_tx = UART_TX.init(Mutex::new(uart_tx));

    let keys = KEYS.init(keys);

    // Rotary Encoder
    let rot_pin_a = ExtiInput::new(p.PB14, p.EXTI14, Pull::None);
    let rot_pin_b = ExtiInput::new(p.PB13, p.EXTI13, Pull::None);
    spawner.spawn(encoder_monitor(rot_pin_b, rot_pin_a)).unwrap();

    const RX_BUF_SIZE: usize = 32;
    let mut rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];
    let mut uart_rx: RingBufferedUartRx = uart_rx.into_ring_buffered(&mut rx_buf);
    // Need to use singleton!() if rx_buf goes out of scope but uart_rx is still needed, below
    // use cortex_m::singleton;
    // let rx_buf: &mut [u8; RX_BUF_SIZE] = singleton!(: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE]).unwrap();
    // let uart_rx = uart_rx.into_ring_buffered(rx_buf);
    uart_rx.start_uart();

    // The datasheet says that the max frequency is 1MHz but 8MHz seems to work
    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, I2CIrqs, p.DMA1_CH4, p.DMA1_CH5, Hertz::mhz(8), Default::default());
    let i2c_intf = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(i2c_intf, DisplaySize128x32, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await.unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().await.unwrap();

    spawner.spawn(display_draw(display)).unwrap();
    spawner.spawn(key_scan(keys, uart_tx)).unwrap();

    let oled = DISPLAY_DRAW.sender();

    led1.set_low();
    loop {
        Timer::after_millis(5).await;
        if uart_rx.read_ready().unwrap() {
            let mut buffer = SerialBuffer::new();
            uart_rx.read_cobs(&mut buffer).await.unwrap();
            match buffer[0] {
                serial::ALT_ENABLE => ALT_EN.signal(buffer[1] == 1),
                serial::GET_STATE => REPORT_FULL.signal(true),
                serial::CAPSLOCK => oled.send(Draw::Capslock(buffer[1] == 1)).await,
                x => {
                    let mut uart_tx = uart_tx.lock().await;
                    uart_tx.send_nack(x).await.unwrap();
                },
            }
        }
        if let Some(rotary) = ENCODER_STATE.try_take() {
            oled.send(Draw::EncoderState(rotary)).await;
        }
    }
}
