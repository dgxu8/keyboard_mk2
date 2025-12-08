use embassy_futures::{join::join, select::{select, Either}};
use embassy_stm32::{mode::Async, usart::UartRx};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use static_cell::StaticCell;
use util::keymap::KeyType;
use embedded_io_async::Write;
use util::cobs_uart::{bl_config, cobs_config, Clearable, CmdId, CobsBuffer, CobsRx, RspnId, SerialRx, UartTxMutex};

use crate::{KeyStateMutex, KEY_CHANGE};
use crate::{logger::bridge_rb, usb::{CdcDev, UsbRx, UsbTx}};

#[derive(PartialEq, Eq, defmt::Format)]
pub enum UartState {
    Normal,
    LoaderBridge,
    Bridge,
}

pub static UART_STATE: Signal<CriticalSectionRawMutex, UartState> = Signal::new();
pub static FLASH_READY: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static RB_NOTIFY: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static RB_KEYSTATE: StaticCell<KeyStateMutex> = StaticCell::new();

#[embassy_executor::task]
pub async fn run(uart_tx: &'static UartTxMutex, uart_rx: UartRx<'static, Async>, mut class: CdcDev, key_state: &'static KeyStateMutex) -> ! {
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
            Either::First(Ok(RspnId::Ack)) => {
                if recv_buf[0] == CmdId::ReadyFlash as u8 {
                    FLASH_READY.signal(true);
                }
            },
            Either::First(Ok(id)) => {
                handle_rb_serial(id, &recv_buf, key_state).await;
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

async fn handle_rb_serial(id: RspnId, buf: &CobsBuffer, key_state: &KeyStateMutex) {
    match id {
        RspnId::KeyChange => {
            let mut key_state = key_state.lock().await;
            for byte in buf.iter() {
                let (key, pressed) = KeyType::decode_update(*byte);
                if pressed {
                    key_state.set(key);
                } else {
                    key_state.reset(key);
                }
            }
            KEY_CHANGE.signal(true);
            RB_NOTIFY.signal(true);
        },
        RspnId::FullState => {
            let mut key_state = key_state.lock().await;
            let start = Instant::now();
            if key_state.deserialize(&buf) {
                KEY_CHANGE.signal(true);
            }
            let dur = Instant::now() - start;
            defmt::info!("Deserialization: {}us", dur.as_micros())
        },
        RspnId::Timestamp => {
            let timestamp = u64::from_le_bytes(buf[..8].try_into().unwrap());
            defmt::info!("Timestamp: {}us", timestamp);
        },
        id => defmt::info!("[{:?}] {:?}", id, buf.as_slice()),
    }
}

async fn rb_bridge<'a, 'b>(uart_tx: &'a UartTxMutex, uart_rx: &mut SerialRx<'a>,
    usb_tx: &mut UsbTx<'b>, usb_rx: &mut UsbRx<'b>) {

    let pc_to_rb_fut = async {
        let mut buf = [0; 64];
        loop {
            let len = match usb_rx.read_packet(&mut buf).await {
                Ok(len) => len,
                Err(e) => {
                    defmt::warn!("[pc] -> rb: {:?}", e);
                    continue;
                }
            };
            let mut uart_tx = uart_tx.lock().await;
            if let Err(e) = uart_tx.write_all(&buf[..len]).await {
                defmt::warn!("pc -> [rb] {:?}", e);
            }
        }
    };
    let rb_to_pc_fut = async {
        let mut buf = [0; 63];  // write one less than max packet size
        loop {
            let len = match uart_rx.read(&mut buf).await {
                Ok(len) => len,
                Err(e) => {
                    defmt::warn!("pc <- [rb] {:?}", e);
                    continue;
                }
            };
            if let Err(e) = usb_tx.write_packet(&mut buf[..len]).await {
                defmt::warn!("[pc] <- rb {:?}", e);
            }
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
