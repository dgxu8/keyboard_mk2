#![no_std]
#![no_main]
#![feature(cold_path)]
#![feature(likely_unlikely)]
#![feature(impl_trait_in_assoc_type)]

extern crate panic_halt;

pub mod keyscan;
pub mod rotary;
pub mod display;

use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::flash::Flash;
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllSource, Sysclk};
use embassy_stm32::usart::{RingBufferedUartRx, Uart};
use embassy_stm32::{bind_interrupts, peripherals, i2c, usart, Config};
use embedded_io_async::Write;
use ssd1306::mode::DisplayConfigAsync;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x32;
use ssd1306::{I2CDisplayInterface, Ssd1306Async};
use static_cell::StaticCell;

use util::cobs_uart::{self, cobs_config, CmdId, CobsBuffer, CobsRx, CobsTx, RspnId, UartTxMutex};
use util::logger::BUFFER;

use crate::rotary::{encoder_monitor, ENCODER_STATE};
use crate::keyscan::{key_scan, Keymap, Keyscan, ALT_EN, KEYMAP, KEYS, REPORT_FULL};
use crate::display::{display_draw, Draw, DISPLAY_DRAW, OLED_STR};

bind_interrupts!(struct UsartIrqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

bind_interrupts!(struct I2CIrqs {
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

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
    let uart = Uart::new(
        p.USART2,
        p.PA3, p.PA2,  // RX, TX
        UsartIrqs,
        p.DMA1_CH7, p.DMA1_CH6,  // TX, RX
        cobs_config(),
    ).unwrap();
    let (uart_tx, uart_rx) = uart.split();
    led0.set_high();

    // Keys and led1 are moved into keys_scan's context forever. If we need shared access we need
    // to wrap Keyscan/Output in an embassy_sync::mutex and put that into the StaticCell
    static UART_TX: StaticCell<UartTxMutex> = StaticCell::new();
    let uart_tx = UART_TX.init(Mutex::new(uart_tx));

    let keys = KEYS.init(keys);

    // Rotary Encoder
    let rot_pin_a = ExtiInput::new(p.PB14, p.EXTI14, Pull::None);
    let rot_pin_b = ExtiInput::new(p.PB13, p.EXTI13, Pull::None);

    let flash = Flash::new_blocking(p.FLASH);
    let keymap = Keymap::new(flash, 0);
    let keymap = KEYMAP.init(Mutex::new(keymap));

    const RX_BUF_SIZE: usize = 32;
    let mut rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];
    let mut uart_rx: RingBufferedUartRx = uart_rx.into_ring_buffered(&mut rx_buf);
    // Need to use singleton!() if rx_buf goes out of scope but uart_rx is still needed, below
    // use cortex_m::singleton;
    // let rx_buf: &mut [u8; RX_BUF_SIZE] = singleton!(: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE]).unwrap();
    // let uart_rx = uart_rx.into_ring_buffered(rx_buf);
    uart_rx.start_uart();

    // The datasheet says that the max frequency is 1MHz but 8MHz seems to work
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = Hertz::mhz(8);
    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, I2CIrqs, p.DMA1_CH4, p.DMA1_CH5, i2c_cfg);
    let i2c_intf = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(i2c_intf, DisplaySize128x32, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await.unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().await.unwrap();

    spawner.spawn(encoder_monitor(rot_pin_b, rot_pin_a, uart_tx, keymap)).unwrap();
    spawner.spawn(display_draw(display)).unwrap();
    spawner.spawn(key_scan(keys, uart_tx, keymap)).unwrap();
    spawner.spawn(run_logger(uart_tx)).unwrap();

    let oled = DISPLAY_DRAW.sender();

    led1.set_low();
    let mut recv_buf = CobsBuffer::new();
    loop {
        match select(uart_rx.recv(&mut recv_buf), ENCODER_STATE.wait()).await {
            Either::First(Ok(id)) => {
                match id {
                    CmdId::AltEnable => ALT_EN.signal(recv_buf[0] == 1),
                    CmdId::GetState => REPORT_FULL.signal(true),
                    CmdId::Capslock  => oled.send(Draw::Capslock(recv_buf[0] == 1)).await,
                    CmdId::Volume  => oled.send(Draw::Volume(recv_buf[0])).await,
                    CmdId::OLEDMsg => {
                        if recv_buf.len() - 1 < OLED_STR.free_capacity()  {
                            OLED_STR.write_all(&recv_buf).await;
                            oled.send(Draw::String(10)).await;
                        } else {
                            defmt::warn!("Not enough space for str: {}", recv_buf.len() - 1);
                        }
                    },
                    CmdId::ReadyFlash => {
                        oled.send(Draw::FlashIco).await;
                        let mut uart_tx = uart_tx.lock().await;
                        uart_tx.send_ack(CmdId::ReadyFlash as u8).await;
                    },
                    CmdId::SaveKeymap => {
                        defmt::info!("Saving rightboard");
                        let mut map = keymap.lock().await;
                        map.save();
                    },
                    CmdId::ClearKeymap => {
                        defmt::info!("Clearing rightboard");
                        let mut map = keymap.lock().await;
                        map.clear();
                    },
                    CmdId::UpdateKeymap => {
                        let mut map = keymap.lock().await;
                        map.update_right(&recv_buf);
                    },
                    CmdId::UpdateAltKeymap => {
                        let mut map = keymap.lock().await;
                        map.update_numpad(&recv_buf);
                    },
                    CmdId::UpdateRotary => {
                        let mut map = keymap.lock().await;
                        if map.update_rotary(&recv_buf).is_err() {
                            defmt::warn!("Failed to update rotary binds");
                        }
                    }
                    _ => (),
                }
            },
            Either::First(Err(cobs_uart::Error::InvalidId(val))) => {
                let mut uart_tx = uart_tx.lock().await;
                uart_tx.send_nack(val).await;
            },
            Either::First(Err(_)) => (),
            Either::Second(rotary) => {
                oled.send(Draw::EncoderState(rotary)).await;
                continue;
            },
        }
        recv_buf.reset();
    }
}

#[embassy_executor::task]
async fn run_logger(uart_tx: &'static UartTxMutex) {
    const BUF_LEN: usize = 32;
    let mut buf = [0; BUF_LEN];
    buf[1] = RspnId::DefmtMsg as u8;  // This will never change

    Timer::after_secs(1).await;
    loop {
        //  5   |-----| len = 4 + 2
        //  0 1 2 3 4 5
        // +-+-+-+-+-+-+
        // |x|x|d|d|d|0|
        // +-+-+-+-+-+-+
        //
        //      |------| len = 29 + 2
        //  0 1 2 ...  30
        // +-+-+-+- -+---+
        // |x|x|d|   | d |
        // +-+-+-+- -+---+
        let len = BUFFER.read(&mut buf[2..BUF_LEN-1]).await + 2;
        if buf[len-1] == 0 {
            buf[0] = len as u8 - 1;
            buf[len-1] = 1;
        } else {
            buf[0] = len as u8;
        }
        buf[len] = 0;

        let mut uart_tx = uart_tx.lock().await;
        unsafe { uart_tx.write_all(&buf[..len+1]).await.unwrap_unchecked() };
    }
}
