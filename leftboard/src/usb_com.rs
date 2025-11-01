use embassy_futures::join::join;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer, WithTimeout};
use embassy_usb::driver::EndpointError;
use embedded_hal::digital::{OutputPin, PinState};
use strum::FromRepr;
use util::cobs_uart::{CmdId, CobsTx, UartTxMutex};

use crate::keyscan::KeymapMutex;
use crate::logger::BRIDGE_RB_DEFMT;
use crate::serial::FLASH_READY;
use crate::usb::UsbRx;
use crate::{logger::OUTPUT_DEFMT, serial::{UartState, UART_STATE}, set_boot_option};

// use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
// pub static BUFFER: Pipe<CriticalSectionRawMutex, 256> = Pipe::new();
//
// #[macro_export]
// macro_rules! push_message_sync {
//     ($($arg:tt)*) => {{
//         use core::fmt::Write;
//         use heapless::String;
//
//         let mut tmp_str: String<64> = String::new();
//         core::write!(&mut tmp_str, $($arg)*).unwrap();
//         tmp_str.push_str("\r\n").unwrap();
//         if tmp_str.len() > $crate::usb_com::BUFFER.free_capacity() {
//             $crate::usb_com::BUFFER.clear();
//         }
//         let fut = $crate::usb_com::BUFFER.write_all(tmp_str.as_bytes());
//         embassy_futures::block_on(fut);
//     }};
// }
//
// #[macro_export]
// macro_rules! push_message {
//     ($($arg:tt)*) => {{
//         use core::fmt::Write;
//         use heapless::String;
//
//         let mut tmp_str: String<64> = String::new();
//         core::write!(&mut tmp_str, $($arg)*).unwrap();
//         if tmp_str.len() > $crate::usb_com::BUFFER.free_capacity() {
//             $crate::usb_com::BUFFER.clear();
//         }
//         tmp_str.push_str("\r\n").unwrap();
//         $crate::usb_com::BUFFER.write_all(tmp_str.as_bytes()).await;
//     }};
// }

pub struct CoprocCtrl<'a> {
    pub boot0: Output<'a>,
    pub n_rst: Output<'a>,
}

impl<'a> CoprocCtrl<'a> {
    pub async fn reset(&mut self, boot0_state: PinState) {
        self.n_rst.set_low();
        self.boot0.set_state(boot0_state).unwrap();
        Timer::after_millis(5).await;
        self.n_rst.set_high();
    }
}

#[derive(FromRepr, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
enum Cmd {
    Defmt = 0,
    Bootloader = 1,
    RbBootloader= 2,
    RbBridge = 3,
    RbEndBridge = 4,
    RbDefmt = 5,

    ClearKeymap = 6,
    SaveKeymap = 7,
    UpdateLeftKeymap = 8,
    UpdateRightKeymap = 9,
    UpdateAltKeymap = 10,
    UpdateRotarymap = 11,
}

#[inline(always)]
async fn handle_data<'a>(id: Cmd, uart_tx: &'static UartTxMutex, rb_ctrl: &mut CoprocCtrl<'a>) -> UartState {
    match id {
        Cmd::Defmt => OUTPUT_DEFMT.signal(true),
        Cmd::Bootloader => set_boot_option(false, true, false),
        Cmd::RbBootloader => {
            FLASH_READY.reset();
            {
                let mut uart_tx = uart_tx.lock().await;
                uart_tx.send(CmdId::ReadyFlash, &[]).await.unwrap();
            }
            _ = FLASH_READY.wait().with_timeout(Duration::from_secs(3)).await;
            rb_ctrl.reset(PinState::High).await;
            UART_STATE.signal(UartState::LoaderBridge);
            return UartState::LoaderBridge;
        },
        Cmd::RbBridge => {
            UART_STATE.signal(UartState::Bridge);
            return UartState::Bridge;
        },
        Cmd::RbDefmt => {
            defmt::info!("Starting defmt bridge");
            BRIDGE_RB_DEFMT.signal(true);
        },
        _ => (),
    }
    UartState::Normal
}

#[embassy_executor::task]
pub async fn run(mut data_in: UsbRx<'static>, uart_tx: &'static UartTxMutex, mut rb_ctrl: CoprocCtrl<'static>, keymap: &'static KeymapMutex) {
    let mut recv_buf = [0; 64];
    let mut state = UartState::Normal;

    data_in.wait_connection().await;
    loop {
        let size = match data_in.read_packet(&mut recv_buf).await {
            Ok(size) => size,
            Err(EndpointError::Disabled) => critical_section::with(|_| cortex_m::peripheral::SCB::sys_reset()),
            Err(_) => continue,
        };

        match Cmd::from_repr(recv_buf[0]) {
            Some(Cmd::UpdateLeftKeymap) => {
                let mut keymap = keymap.lock().await;
                keymap.update(&mut recv_buf[1..size]);
            },
            Some(Cmd::UpdateRightKeymap) => {
                let mut uart_tx = uart_tx.lock().await;
                if uart_tx.send(CmdId::UpdateKeymap, &recv_buf[1..size]).await.is_err() {
                    defmt::warn!("Failed to send right keymap");
                }
            },
            Some(Cmd::UpdateAltKeymap) => {
                let mut uart_tx = uart_tx.lock().await;
                if uart_tx.send(CmdId::UpdateAltKeymap, &recv_buf[1..size]).await.is_err() {
                    defmt::warn!("Failed to send numpad keymap");
                }
            },
            Some(Cmd::UpdateRotarymap) => {
                let mut uart_tx = uart_tx.lock().await;
                if uart_tx.send(CmdId::UpdateRotary, &recv_buf[1..size]).await.is_err() {
                    defmt::warn!("Failed to send Rotary mapping");
                }
            },
            Some(Cmd::SaveKeymap) => {
                defmt::info!("Saving keymap");
                let send_save_fut = async {
                    let mut uart_tx = uart_tx.lock().await;
                    if uart_tx.send(CmdId::SaveKeymap, &[]).await.is_err() {
                        defmt::warn!("Failed to send save keymap");
                    }
                };
                let save_fut = async {
                    let mut keymap = keymap.lock().await;
                    keymap.save();
                };
                join(send_save_fut, save_fut).await;
            },
            Some(Cmd::ClearKeymap) => {
                defmt::info!("Clearing keymap");
                let send_clear_fut = async {
                    let mut uart_tx = uart_tx.lock().await;
                    if uart_tx.send(CmdId::ClearKeymap, &[]).await.is_err() {
                        defmt::warn!("Failed to send clear keymap");
                    }
                };
                let clear_fut = async {
                    let mut keymap = keymap.lock().await;
                    keymap.clear();
                };
                join(send_clear_fut, clear_fut).await;
            },
            Some(cmd) if state == UartState::Normal => {
                state = handle_data(cmd, uart_tx, &mut rb_ctrl).await;
            }
            Some(Cmd::RbEndBridge) => {
                defmt::info!("Closing bridge");
                if state == UartState::LoaderBridge {
                    rb_ctrl.reset(PinState::Low).await;
                }
                state = UartState::Normal;
                UART_STATE.signal(UartState::Normal);
            },
            Some(cmd) => defmt::warn!("Error: Cant send {:?} when in bridge mode", cmd),
            None =>(),
        }
    }
}
