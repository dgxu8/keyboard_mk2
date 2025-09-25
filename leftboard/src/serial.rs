use embassy_futures::{join::join, select::{select, Either}};
use embassy_stm32::{mode::Async, usart::UartRx};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_io_async::Write;
use util::cobs_uart::{bl_config, cobs_config, Clearable, CobsBuffer, CobsRx, RspnId, SerialRx, UartTxMutex};

use crate::{logger::bridge_rb, usb::{CdcDev, UsbRx, UsbTx}};

#[derive(PartialEq, Eq, defmt::Format)]
pub enum UartState {
    Normal,
    LoaderBridge,
    Bridge,
}

pub static UART_STATE: Signal<CriticalSectionRawMutex, UartState> = Signal::new();

#[embassy_executor::task]
pub async fn run(uart_tx: &'static UartTxMutex, uart_rx: UartRx<'static, Async>, mut class: CdcDev) -> ! {
    let mut rx_buf: [u8; 64] = [0; 64];

    let mut uart_rx = uart_rx.into_ring_buffered(&mut rx_buf);

    class.wait_connection().await;
    let (mut usb_tx, mut usb_rx) = class.split();
    uart_rx.start_uart();
    let mut recv_buf = CobsBuffer::new();
    loop {
        match select(uart_rx.recv(&mut recv_buf), UART_STATE.wait()).await {
            Either::First(Ok(RspnId::DefmtMsg)) => {
                bridge_rb(&mut usb_tx, &recv_buf).await;
            },
            Either::First(Ok(id)) => {
                defmt::info!("[{:?}] {:?}", id, recv_buf.as_slice());
            },
            Either::First(Err(e)) => {
                defmt::warn!("Cobs error: {:?}", e);
            },
            Either::Second(UartState::Bridge) => {
                defmt::info!("Setting up bridge to rightboard");
                uart_rx.clear().await.unwrap();
                rb_bridge(uart_tx, &mut uart_rx, &mut usb_tx, &mut usb_rx).await;
                uart_rx.clear().await.unwrap();
            }
            Either::Second(UartState::LoaderBridge) => {
                defmt::info!("Switching device to bootloader");
                uart_rx.clear().await.unwrap();
                uart_rx.set_config(&bl_config()).unwrap();
                rb_bridge(uart_tx, &mut uart_rx, &mut usb_tx, &mut usb_rx).await;
                uart_rx.set_config(&cobs_config()).unwrap();
                uart_rx.clear().await.unwrap();
            }
            Either::Second(state) => {
                defmt::warn!("Got weird state {:?}", state);
                continue;
            },
        }
        recv_buf.reset();
    }
}

async fn rb_bridge<'a, 'b>(uart_tx: &'a UartTxMutex, uart_rx: &mut SerialRx<'a>,
    usb_tx: &mut UsbTx<'b>, usb_rx: &mut UsbRx<'b>) {

    let pc_to_rb_fut = async {
        let mut buf = [0; 64];
        loop {
            let len = usb_rx.read_packet(&mut buf).await.unwrap();
            let mut uart_tx = uart_tx.lock().await;
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
