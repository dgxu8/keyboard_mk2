use core::{cmp, sync::atomic::{AtomicBool, Ordering}};

use embassy_stm32::{peripherals::USB, usb::Driver};
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex}, signal::Signal};
use embassy_usb::{Builder, Handler, UsbDevice, class::{cdc_acm::{self, CdcAcmClass, Receiver, Sender}, hid::{self, HidBootProtocol, HidReaderWriter, HidSubclass, ReportId, RequestHandler}}, control::OutResponse, driver::{self, EndpointError}};
use static_cell::StaticCell;

pub static CAPS_LOCK: Signal<CriticalSectionRawMutex, u8> = Signal::new();
pub static USB_READY: Signal<ThreadModeRawMutex, bool> = Signal::new();

static CFG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CTRL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static DEV_HANDLER: StaticCell<MyDeviceHandler> = StaticCell::new();

static CDC_STATE: StaticCell<cdc_acm::State> = StaticCell::new();
static DEFMT_STATE: StaticCell<cdc_acm::State> = StaticCell::new();
static HID_STATE: StaticCell<hid::State> = StaticCell::new();

pub type UsbTx<'a> = Sender<'a, Driver<'a, USB>>;
pub type UsbRx<'a> = Receiver<'a, Driver<'a, USB>>;
pub type CdcDev = CdcAcmClass<'static, Driver<'static, USB>>;
type HidRW = HidReaderWriter<'static, Driver<'static, USB>, 1, 15>;

#[repr(C, packed)]
pub struct NKROReport {
    modifier: u8,
    reserved: u8,  // Required padding
    keycodes: [u8; 13],
}

impl NKROReport {
    const DESCRIPTOR: &'static[u8] = &[
        0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
        0x09, 0x06,         // USAGE (Keyboard)
        0xa1, 0x01,         // COLLECTION (Application)
        // Modifiers
        0x05, 0x07,         //   USAGE_PAGE (Keyboard)
        0x19, 0xe0,         //   USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7,         //   USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00,         //   LOGICAL_MINIMUM (0)
        0x25, 0x01,         //   LOGICAL_MAXIMUM (1)
        0x75, 0x01,         //   REPORT_SIZE (1)
        0x95, 0x08,         //   REPORT_COUNT (8)
        0x81, 0x02,         //   INPUT (Data,Var,Abs)
        // Reserved Byte
        0x75, 0x08,         //   REPORT_SIZE (8)
        0x95, 0x01,         //   REPORT_COUNT (1)
        0x81, 0x01,         //   INPUT (Cnst,Ary,Abs)
        // LED Output Report
        0x05, 0x08,         //   USAGE_PAGE (LEDs)
        0x19, 0x01,         //   USAGE_MINIMUM (Num Lock)
        0x29, 0x05,         //   USAGE_MAXIMUM (Kana)
        0x75, 0x01,         //   REPORT_SIZE (1)
        0x95, 0x05,         //   REPORT_COUNT (5)
        0x91, 0x02,         //   OUTPUT (Data,Var,Abs)
        0x75, 0x03,         //   REPORT_SIZE (3)
        0x95, 0x01,         //   REPORT_COUNT (1)
        0x91, 0x01,         //   OUTPUT (Cnst,Ary,Abs)
        // Keys bitmap
        0x05, 0x07,         //   USAGE_PAGE (Keyboard)
        0x19, 0x00,         //   USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65,         //   USAGE_MAXIMUM (Keyboard Application)
        0x15, 0x00,         //   LOGICAL_MINIMUM (0)
        0x25, 0x01,         //   LOGICAL_MAXIMUM (1)
        0x75, 0x01,         //   REPORT_SIZE (1)
        0x95, 0x65,         //   REPORT_COUNT (101) maybe needs to be 102
        0x81, 0x02,         //   INPUT (Data,Var,Abs)
        0x75, 0x03,         //   REPORT_SIZE (3)
        0x95, 0x01,         //   REPORT_COUNT (1)
        0x81, 0x01,         //   INPUT (Cnst,Ary,Abs)
        0xc0                // END_COLLECTION
    ];

    pub fn new(modifier: u8, keycodes: [u8; 13]) -> Self {
        Self {
            modifier,
            keycodes,
            reserved: 0,
        }
    }

    pub fn into_bytes(self) -> [u8; 15] {
        unsafe { core::mem::transmute(self) }
    }
}

#[embassy_executor::task]
pub async fn run_usb(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[inline(always)]
pub async fn wait_for_usb() {
    while USB_READY.wait().await == false {
        defmt::info!("Not configured");
    }
}

pub fn init_usb(driver: Driver<'static, USB>) -> (UsbDevice<'static, Driver<'static, USB>>, HidRW, CdcDev, CdcDev) {
    // This has linux quirk to disable echo for devices with this pid/vid
    // let mut usb_cfg = embassy_usb::Config::new(0x045b, 0x024d);

    let mut usb_cfg = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_cfg.manufacturer = Some("Rustboard");
    usb_cfg.product = Some("Keyboard");
    usb_cfg.serial_number = Some("1337");

    let cfg_desc = CFG_DESC.init([0; 256]);
    let ctrl_buf = CTRL_BUF.init([0; 64]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let device_handler = DEV_HANDLER.init(MyDeviceHandler::new());

    let state = CDC_STATE.init(cdc_acm::State::new());
    let defmt_state = DEFMT_STATE.init(cdc_acm::State::new());
    let hid_state = HID_STATE.init(hid::State::new());
    let mut builder = Builder::new(
        driver,
        usb_cfg,
        cfg_desc,
        bos_desc,
        &mut [],
        ctrl_buf
    );
    builder.handler(device_handler);

    let hid_cfg = embassy_usb::class::hid::Config {
        report_descriptor: NKROReport::DESCRIPTOR,
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 15,
        hid_subclass: HidSubclass::Boot, // TODO: Maybe not true
        hid_boot_protocol: HidBootProtocol::Keyboard,
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
        USB_READY.signal(configured);
        self.configured.store(configured, Ordering::Relaxed);
        defmt::info!("configured: {}", configured);
    }

    fn suspended(&mut self, suspended: bool) {
        defmt::info!("suspended: {}", suspended);
    }
}
