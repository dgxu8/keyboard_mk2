use embassy_futures::{join::join, select::{select, Either}};
use embassy_stm32::{mode::Async, peripherals::USB, usart::{RingBufferedUartRx, Uart, UartTx}, usb::Driver};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender};
use embedded_io_async::Write;
use util::cobs_uart::{bl_config, cobs_config, Clearable, CobsBuffer, CobsRx, SerialBuffer};

#[derive(PartialEq, Eq)]
pub enum UartState {
    Normal,
    LoaderBridge,
    Bridge,
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

    let mut recv_buf = CobsBuffer::new();

    loop {
        match select(uart_rx.read_cobs(&mut recv_buf), UART_STATE.wait()).await {
            Either::First(rslt) => {
                if let Err(_) = rslt {
                    defmt::warn!("Got cobs read error");
                } else {
                    handle_cobs(&recv_buf);
                    recv_buf.reset();
                }
            },
            Either::Second(state) => {
                if state == UartState::LoaderBridge {
                    defmt::info!("Switching device to bootloader");
                    uart_rx.clear().await.unwrap();
                    uart_rx.set_config(&bl_config()).unwrap();
                    rb_bridge(&mut uart_tx, &mut uart_rx, &mut usb_tx, &mut usb_rx).await;
                    uart_rx.set_config(&cobs_config()).unwrap();
                    uart_rx.clear().await.unwrap();
                    recv_buf.reset();
                } else if state == UartState::Bridge {
                    defmt::info!("Setting up bridge to rightboard");
                    uart_rx.clear().await.unwrap();
                    rb_bridge(&mut uart_tx, &mut uart_rx, &mut usb_tx, &mut usb_rx).await;
                    uart_rx.clear().await.unwrap();
                    recv_buf.reset();
                }
            }
        }
    }
}

#[inline(always)]
fn handle_cobs(payload: &SerialBuffer) {
    match payload[0] {
        _ => {
            if payload.len() == 1 {
                defmt::info!("[{}]", payload[0]);
            } else {
                defmt::info!("[{}] {}", payload[0], payload[1..payload.len()]);
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
