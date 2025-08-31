use embassy_stm32::{peripherals::USB, usb::Driver};
use embassy_time::Timer;
use embassy_usb::{class::cdc_acm::Receiver, driver::EndpointError};

use crate::{logger::OUTPUT_DEFMT, set_boot_option};

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

const START_DEFMT: u8 = 0;
const START_BL: u8 = 1;
const START_RB_BL: u8 = 2;
const END_RB_BL: u8 = 3;
const UPDATE_KEYMAP: u8 = 4;

async fn handle_data<'a>(id: u8) {
    match id {
        START_DEFMT => OUTPUT_DEFMT.signal(true),
        START_BL => set_boot_option(false, true, false),
        START_RB_BL => (),
        END_RB_BL => (),
        UPDATE_KEYMAP => (),
        _ => (),
    }
}

#[embassy_executor::task]
pub async fn run(mut data_in: Receiver<'static, Driver<'static, USB>>) {
    loop {
        data_in.wait_connection().await;

        loop {
            let mut recv_buf = [0; 1];
            match data_in.read_packet(&mut recv_buf).await {
                Ok(_id) => handle_data(recv_buf[0]).await,
                Err(e) => {
                    if e == EndpointError::Disabled {
                        critical_section::with(|_| cortex_m::peripheral::SCB::sys_reset());
                    }
                },
            }
        }
    }
}
