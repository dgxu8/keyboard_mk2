#![no_std]
#![no_main]

extern crate panic_halt;

pub mod keyscan;

use core::fmt::Write;

use heapless::String;
use keyscan::{key_scan, SCAN, Keyscan};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllSource, Sysclk};
use embassy_stm32::usart::{RingBufferedUartRx, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config};
use embassy_time::{Instant, Timer};
use embedded_io::ReadReady;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
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

    // Configure Decoder
    let key_select_pins: [Output; 3] = [
        Output::new(p.PA7, Level::Low, Speed::High),
        Output::new(p.PA6, Level::Low, Speed::High),
        Output::new(p.PA5, Level::Low, Speed::High),
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
    );

    // Configure LEDs
    let mut led0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let led1 = Output::new(p.PB5, Level::Low, Speed::Low);

    // Configure usart default:
    // baudrate: 115200,
    // data_bits: DataBits::DataBits8,
    // stop_bits: StopBits::STOP1,
    // parity: Parity::ParityNone,
    let uart = Uart::new(
        p.USART2,
        p.PA3, p.PA2,  // RX, TX
        Irqs,
        p.DMA1_CH4, p.DMA1_CH5,  // TX, RX
        usart::Config::default()
    ).unwrap();
    let (mut uart_tx, uart_rx) = uart.split();

    uart_tx.write(b"Test\r\n").await.unwrap();
    led0.set_high();

    // Keys and led1 are moved into keys_scan's context forever. If we need shared access we need
    // to wrap Keyscan/Output in an embassy_sync::mutex and put that into the StaticCell
    static KEYS: StaticCell<Keyscan> = StaticCell::new();
    static LEDS: StaticCell<Output> = StaticCell::new();
    let keys = KEYS.init(keys);
    let led1 = LEDS.init(led1);
    spawner.spawn(key_scan(keys, led1)).unwrap();

    const RX_BUF_SIZE: usize = 16;
    let mut rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];
    let mut uart_rx: RingBufferedUartRx = uart_rx.into_ring_buffered(&mut rx_buf);
    // Need to use singleton!() if rx_buf goes out of scope but uart_rx is still needed, below
    // use cortex_m::singleton;
    // let rx_buf: &mut [u8; RX_BUF_SIZE] = singleton!(: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE]).unwrap();
    // let uart_rx = uart_rx.into_ring_buffered(rx_buf);
    uart_rx.start_uart();
    loop {
        Timer::after_millis(5).await;
        let mut buffer: [u8; 1] = [0; 1];
        if uart_rx.read_ready().unwrap() {
            uart_rx.read(&mut buffer).await.unwrap();
            uart_tx.write(&buffer).await.unwrap();
        }
        if let Some(scan) = SCAN.try_take() {
            let mut msg: String<30> = String::new();
            core::write!(&mut msg, "{:03}: 0x{:02X}\r\n", scan.scan_time, scan.state[0]).unwrap();
            uart_tx.write(msg.as_bytes()).await.unwrap();
        }
    }
}
