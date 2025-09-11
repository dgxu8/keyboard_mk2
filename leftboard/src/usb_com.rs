use embassy_stm32::gpio::Output;
use embassy_stm32::{peripherals::USB, usb::Driver};
use embassy_time::Timer;
use embassy_usb::{class::cdc_acm::Receiver, driver::EndpointError};
use embedded_hal::digital::{OutputPin, PinState};

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

const START_DEFMT: u8 = 0;
const ENTER_BL: u8 = 1;

const START_BL_BRIDGE: u8 = 2;
const END_BL_BRIDGE: u8 = 3;

const START_BRIDGE: u8 = 4;
const END_BRIDGE: u8 = 5;

const UPDATE_KEYMAP: u8 = 6;

async fn handle_data<'a>(id: u8, rb_ctrl: &mut CoprocCtrl<'a>) {
    match id {
        START_DEFMT => OUTPUT_DEFMT.signal(true),
        ENTER_BL => set_boot_option(false, true, false),
        START_BL_BRIDGE => {
            rb_ctrl.reset(PinState::High).await;
            UART_STATE.signal(UartState::LoaderBridge);
        },
        END_BL_BRIDGE => {
            rb_ctrl.reset(PinState::Low).await;
            UART_STATE.signal(UartState::Normal);
        },
        START_BRIDGE => {
            UART_STATE.signal(UartState::Bridge);
        },
        END_BRIDGE => {
            UART_STATE.signal(UartState::Normal);
        },
        UPDATE_KEYMAP => (),
        _ => (),
    }
}

#[embassy_executor::task]
pub async fn run(mut data_in: Receiver<'static, Driver<'static, USB>>, mut rb_ctrl: CoprocCtrl<'static>) {
    loop {
        data_in.wait_connection().await;

        loop {
            let mut recv_buf = [0; 1];
            match data_in.read_packet(&mut recv_buf).await {
                Ok(_id) => handle_data(recv_buf[0], &mut rb_ctrl).await,
                Err(e) => {
                    if e == EndpointError::Disabled {
                        critical_section::with(|_| cortex_m::peripheral::SCB::sys_reset());
                    }
                },
            }
        }
    }
}
