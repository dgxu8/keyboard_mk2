use core::{cmp, sync::atomic::{AtomicBool, Ordering}};

use embassy_stm32::{peripherals::USB, usb::Driver};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb::{class::{cdc_acm::{self, CdcAcmClass, Receiver, Sender}, hid::{self, HidReaderWriter, ReportId, RequestHandler}}, control::OutResponse, driver::{self, EndpointError}, Builder, Handler, UsbDevice};
use static_cell::StaticCell;
use usbd_hid::{descriptor::*};
use serde::ser::{Serialize, Serializer, SerializeTuple};

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0xE0, usage_max = 0xE7) = {
            #[packed_bits 8]
            #[item_settings data,variable,absolute]
            modifier=input;
        };
        (usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings constant,variable,absolute]
            reserved=input;
        };
        (usage_page = LEDS, usage_min = 0x01, usage_max = 0x05) = {
            #[packed_bits 5]
            #[item_settings data,variable,absolute]
            leds=output;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0x65) = {
            #[packed_bits 101]
            #[item_settings data,variable,absolute]
            keycodes=input;
        };
    }
)]

pub struct NKROKeyboardReport {
    pub modifier: u8,
    pub reserved: u8,
    pub leds: u8,
    pub keycodes: [u8; 13],
}

pub static CAPS_LOCK: Signal<CriticalSectionRawMutex, u8> = Signal::new();

static CFG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CTRL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static DEV_HANDLER: StaticCell<MyDeviceHandler> = StaticCell::new();

static CDC_STATE: StaticCell<cdc_acm::State> = StaticCell::new();
static DEFMT_STATE: StaticCell<cdc_acm::State> = StaticCell::new();
static HID_STATE: StaticCell<hid::State> = StaticCell::new();

pub type UsbTx<'a> = Sender<'a, Driver<'a, USB>>;
pub type UsbRx<'a> = Receiver<'a, Driver<'a, USB>>;
pub type CdcDev = CdcAcmClass<'static, Driver<'static, USB>>;
type HidRW = HidReaderWriter<'static, Driver<'static, USB>, 1, 15>;

pub fn init_usb(driver: Driver<'static, USB>) -> (UsbDevice<'static, Driver<'static, USB>>, HidRW, CdcDev, CdcDev) {
    // This has linux quirk to disable echo for devices with this pid/vid
    // let mut usb_cfg = embassy_usb::Config::new(0x045b, 0x024d);

    let mut usb_cfg = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_cfg.manufacturer = Some("Rustboard");
    usb_cfg.product = Some("Keyboard");
    usb_cfg.serial_number = Some("1337");

    let cfg_desc = CFG_DESC.init([0; 256]);
    let ctrl_buf = CTRL_BUF.init([0; 64]);
    let device_handler = DEV_HANDLER.init(MyDeviceHandler::new());

    let state = CDC_STATE.init(cdc_acm::State::new());
    let defmt_state = DEFMT_STATE.init(cdc_acm::State::new());
    let hid_state = HID_STATE.init(hid::State::new());
    // Not allocating anything for BOS since there isn't any reason to support the LPM USB 2.0
    // extension.
    let mut builder = Builder::new(
        driver,
        usb_cfg,
        cfg_desc,
        &mut [],
        &mut [],
        ctrl_buf
    );
    builder.handler(device_handler);

    let hid_cfg = embassy_usb::class::hid::Config {
        report_descriptor: NKROKeyboardReport::desc(),
        request_handler: None,
        poll_ms: 50,
        max_packet_size: 15,
    };
    let h = HidRW::new(&mut builder, hid_state, hid_cfg);

    let ctrl_com = CdcAcmClass::new(&mut builder, state, 64);
    let rb_com = CdcAcmClass::new(&mut builder, defmt_state, 64);

    let  usb = builder.build();

    (usb, h, ctrl_com, rb_com)
}

pub trait UsbSerial {
    async fn write_packets(&mut self, payload: &[u8]) -> Result<(), EndpointError>;
}

impl<'a, D: driver::Driver<'a>> UsbSerial for Sender<'a, D> {
    async fn write_packets(&mut self, payload: &[u8]) -> Result<(), EndpointError> {
        let mut buf = payload;
        let mut len = cmp::min(buf.len(), self.max_packet_size() as usize);

        while len > 0 {
            self.write_packet(&buf[..len]).await?;
            buf = &buf[len..];
            len = cmp::min(buf.len(), self.max_packet_size() as usize);
        }

        // If we are a multiple of max packet size we need to send a zero size packet
        if payload.len() % self.max_packet_size() as usize == 0 {
            self.write_packet(&[]).await?;
        }
        Ok(())
    }
}

pub struct HidRqstHndlr;

impl RequestHandler for HidRqstHndlr {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        defmt::info!("get_report id: {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, _data: &[u8]) -> OutResponse {
        defmt::info!("set_report: {:?}", id);
        match id {
            ReportId::Out(id) => {
                CAPS_LOCK.signal((id & 0x2) >> 1);
            },
            _ => (),
        }
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, _dur: u32) {
        defmt::info!("set_idle_ms: {:?}", id);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        defmt::info!("get_idle_ms: {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        defmt::info!("enabled: {}", enabled);
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        defmt::info!("reset");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        defmt::info!("addr: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        defmt::info!("configured: {}", configured);
    }
}
