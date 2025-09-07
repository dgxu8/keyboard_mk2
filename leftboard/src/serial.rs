use embassy_futures::{join::join, select::{select, Either}};
use embassy_stm32::{mode::Async, peripherals::USB, usart::{RingBufferedUartRx, Uart, UartTx}, usb::Driver};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use embedded_io::ReadReady;
use embedded_io_async::Write;
use util::cobs_uart::{bl_config, cobs_config, Clearable, CobsRx, SerialBuffer};

#[derive(PartialEq, Eq)]
pub enum UartState {
    Normal,
    LoaderBridge,
}

pub static UART_STATE: Signal<CriticalSectionRawMutex, UartState> = Signal::new();

#[embassy_executor::task]
pub async fn run(uart: Uart<'static, Async>, mut class: CdcAcmClass<'static, Driver<'static, USB>>) -> ! {
    let mut rx_buf: [u8; 64] = [0; 64];
    let (mut uart_tx, uart_rx) = uart.split();
    let mut uart_rx = uart_rx.into_ring_buffered(&mut rx_buf);

    class.wait_connection().await;
    let (mut usb_tx, mut usb_rx) = class.split();

    uart_rx.start_uart();
    loop {
        Timer::after_millis(5).await;
        if UART_STATE.try_take() == Some(UartState::LoaderBridge){
            defmt::info!("Switching device to bootloader");
            uart_rx.clear().await.unwrap();
            uart_rx.set_config(&bl_config()).unwrap();
            rb_bridge(&mut uart_tx, &mut uart_rx, &mut usb_tx, &mut usb_rx).await;
            uart_rx.set_config(&cobs_config()).unwrap();
            uart_rx.clear().await.unwrap();
        }
        while uart_rx.read_ready().unwrap_or(false) {
            handle_cobs(&mut uart_rx).await;
        }
    }
}

#[inline(always)]
async fn handle_cobs<'a>(uart_rx: &mut RingBufferedUartRx<'a>) {
    let mut buffer = SerialBuffer::new();
    match uart_rx.read_cobs(&mut buffer).await {
        Ok(_) => (),
        Err(_) => return,
    }
    match buffer[0] {
        _ => {
            if buffer.len() == 1 {
                defmt::info!("[{}]", buffer[0]);
            } else {
                defmt::info!("[{}] {}", buffer[0], buffer[1..buffer.len()]);
            }
        }
    }
}

async fn rb_bridge<'a, 'b>(uart_tx: &mut UartTx<'a, Async>, uart_rx: &mut RingBufferedUartRx<'a>,
    usb_tx: &mut Sender<'b, Driver<'b, USB>>, usb_rx: &mut Receiver<'b, Driver<'b, USB>>) {
    usb_rx.wait_connection().await;

    let pc_to_rb_fut = async {
        let mut buf = [0; 64];
        loop {
            let len = usb_rx.read_packet(&mut buf).await.unwrap();
            uart_tx.write_all(&buf[..len]).await.unwrap();
        }
    };
    let rb_to_pc_fut = async {
        let mut buf = [0; 63];  // write one less than max packet size
        loop {
            let len = uart_rx.read(&mut buf).await.unwrap();
            usb_tx.write_packet(&mut buf[..len]).await.unwrap();
        }
    };

    let bridge_fut = join(pc_to_rb_fut, rb_to_pc_fut);
    match select(bridge_fut, UART_STATE.wait()).await {
        Either::First(_) => defmt::warn!("WTF how did we get here?"),
        Either::Second(val) => {
            if val == UartState::LoaderBridge {
                defmt::warn!("Got signal to go into bridge mode when already in bridge mode")
            }
        }
    }
}
