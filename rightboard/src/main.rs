#![no_std]
#![no_main]

extern crate panic_halt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllSource, Sysclk};
use embassy_stm32::usart::{RingBufferedUartRx, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config};
use embassy_time::{block_for, Duration, Timer};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

fn set_state(decoder: &mut [Output; 3], enable: &mut Output, idx: u8) {
    enable.set_low();
    decoder[0].set_level((idx & 0x1 != 0).into());
    decoder[1].set_level((idx & 0x2 != 0).into());
    decoder[2].set_level((idx & 0x4 != 0).into());
    enable.set_high();
}

fn scan_inputs(inputs: &mut [Input; 8]) -> u8 {
    let mut bits: u8 = 0;
    for i in 0..inputs.len() {
        bits |= (inputs[i].is_high() as u8) << i;
    }
    bits
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
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
    let mut decoder: [Output; 3] = [
        Output::new(p.PA7, Level::Low, Speed::High),
        Output::new(p.PA6, Level::Low, Speed::High),
        Output::new(p.PA5, Level::Low, Speed::High),
    ];
    let mut decoder_en = Output::new(p.PA4, Level::Low, Speed::High);

    // Configure Decoder input
    let mut inputs: [Input; 8] = [
        Input::new(p.PA8, Pull::Down),
        Input::new(p.PA9, Pull::Down),
        Input::new(p.PA10, Pull::Down),
        Input::new(p.PA11, Pull::Down),
        Input::new(p.PA12, Pull::Down),
        Input::new(p.PA13, Pull::Down),
        Input::new(p.PA14, Pull::Down),
        Input::new(p.PA15, Pull::Down),
    ];

    // Configure LEDs
    let mut led0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PB5, Level::Low, Speed::Low);

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

    const RX_BUF_SIZE: usize = 16;
    let mut rx_buf: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];
    let mut uart_rx: RingBufferedUartRx = uart_rx.into_ring_buffered(&mut rx_buf);

    // Need to use singleton!() if rx_buf goes out of scope but uart_rx is still needed, below
    // use cortex_m::singleton;
    // let rx_buf: &mut [u8; RX_BUF_SIZE] = singleton!(: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE]).unwrap();
    // let uart_rx = uart_rx.into_ring_buffered(rx_buf);

    uart_rx.start_uart();
    loop {
        let mut buffer: [u8; 1] = [0; 1];
        let mut bits: u8 = 0;
        Timer::after_millis(5).await;
        for i in 0..8 {
            set_state(&mut decoder, &mut decoder_en, i);
            block_for(Duration::from_micros(1));
            bits |= scan_inputs(&mut inputs);
        }
        if bits > 0 {
            led1.set_high();
        } else {
            led1.set_low();
        }
        if uart_rx.read(&mut buffer).await.unwrap() > 0 {
            uart_tx.write(&buffer).await.unwrap();
        }
    }
}
