#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

extern crate panic_halt;

use core::sync::atomic::{AtomicBool, Ordering};

use defmt;
use defmt_serial as _;
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::mux::Clk48sel;
use embassy_stm32::rcc::{Hsi48Config, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::usart::Uart;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usart, usb, Config};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
use embassy_usb::class::hid::{self, HidReaderWriter, ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use static_cell::StaticCell;
use usbd_hid::descriptor::*;
use serde::ser::{Serialize, Serializer, SerializeTuple};

bind_interrupts!(struct USBIrqs {
    USB => usb::InterruptHandler<peripherals::USB>;
});

bind_interrupts!(struct UsartIrqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;

});

static UART: StaticCell<Uart<'_, Async>> = StaticCell::new();

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.hsi = true;
    config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL10,
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV2),
    });

    config.rcc.hsi48 = Some(Hsi48Config {sync_from_usb: true});
    config.rcc.mux.clk48sel = Clk48sel::HSI48;
    let p = embassy_stm32::init(config);

    let mut led0 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PB1, Level::Low, Speed::Low);

    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 2_000_000;
    let uart = Uart::new(
        p.USART2,
        p.PA3, p.PA2,  // RX, TX
        UsartIrqs,
        p.DMA1_CH7, p.DMA1_CH6, // TX, RX
        uart_cfg
    ).unwrap();

    defmt_serial::defmt_serial(UART.init(uart));
    defmt::info!("Starting");

    {
        let optr = embassy_stm32::pac::FLASH.optr().read();
        if !optr.n_boot0() || !optr.n_boot1() || !optr.n_swboot0() {
            defmt::debug!("Option bits: {}, {}, {}. Correcting...", optr.n_boot0(), optr.n_boot1(), optr.n_swboot0());
            set_boot_option(true, true, true).await;
        } else {
            defmt::debug!("Option bits correct");
        }
    }

    let driver = Driver::new(p.USB, USBIrqs, p.PA12, p.PA11);
    let mut usb_cfg = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_cfg.manufacturer = Some("Rustboard");
    usb_cfg.product = Some("HID Keyboard");
    usb_cfg.serial_number = Some("1337");

    let mut cfg_desc = [0; 256];
    let mut bos_desc = [0; 256];
    let mut ctrl_buf = [0; 64];
    let mut rqst_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = cdc_acm::State::new();
    let mut hid_state = hid::State::new();
    let mut builder = Builder::new(
        driver,
        usb_cfg,
        &mut cfg_desc,
        &mut bos_desc,
        &mut [],
        &mut ctrl_buf
    );
    builder.handler(&mut device_handler);

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let hid_cfg = embassy_usb::class::hid::Config {
        report_descriptor: NKROKeyboardReport::desc(),
        request_handler: None,
        poll_ms: 50,
        max_packet_size: 15,
    };

    let h = HidReaderWriter::<_, 1, 15>::new(&mut builder, &mut hid_state, hid_cfg);

    let mut usb = builder.build();
    let usb_fut = usb.run();

    let (reader, mut writer) = h.split();

    let echo_fut = async {
        loop {
            class.wait_connection().await;
            echo(&mut class, &mut led0).await;
        }
    };
    let blink_fut = async {
        let mut keycode = [0; 13];
        loop {
            led1.toggle();
            Timer::after_millis(1000).await;

            keycode[10] = 1 << 3;
            // keycode[10] = 1 << KeyboardUsage::KeyboardAa as u8;
            let report = NKROKeyboardReport {
                keycodes: keycode,
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {},
                Err(e) => defmt::info!("Error sending key {:?}", e),
            };
            Timer::after_millis(100).await;
            keycode[10] = 0;
            let report = NKROKeyboardReport {
                keycodes: keycode,
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {},
                Err(e) => defmt::info!("Error no sending key {:?}", e),
            };
        }
    };
    let out_fut = async {
        reader.run(false, &mut rqst_handler).await;
    };
    join4(usb_fut, echo_fut, blink_fut, out_fut).await;
}

async fn set_boot_option(n_boot0: bool, n_boot1: bool, n_swboot0: bool) -> ! {
    const FLASH_KEY1: u32 = 0x4567_0123;
    const FLASH_KEY2: u32 = 0xCDEF_89AB;
    const OPT_KEY1: u32 =  0x0819_2A3B;
    const OPT_KEY2: u32 =  0x4C5D_6E7F;
    while embassy_stm32::pac::FLASH.sr().read().bsy() {
        Timer::after_millis(1).await;
    }
    // Unlock FLASH
    embassy_stm32::pac::FLASH.keyr().write_value(FLASH_KEY1);
    embassy_stm32::pac::FLASH.keyr().write_value(FLASH_KEY2);
    // Unlock OPT Registers
    embassy_stm32::pac::FLASH.optkeyr().write_value(OPT_KEY1);
    embassy_stm32::pac::FLASH.optkeyr().write_value(OPT_KEY2);

    let cr = embassy_stm32::pac::FLASH.cr().read();
    defmt::info!("cr: {}, {}", cr.lock(), cr.optlock());

    // Write Registers
    let mut optr = embassy_stm32::pac::FLASH.optr().read();
    optr.set_n_boot0(n_boot0);  // Set boot0 bit to 1
    optr.set_n_boot1(n_boot1);  // Set boot1 bit to 1
    optr.set_n_swboot0(n_swboot0); // Switch to using boot0 bit for determining whether or not we
                                   // boot into the bootloadr
    embassy_stm32::pac::FLASH.optr().write_value(optr);

    // Commit option registers update
    let mut cr = embassy_stm32::pac::FLASH.cr().read();
    cr.set_optstrt(true);
    embassy_stm32::pac::FLASH.cr().write_value(cr);

    while embassy_stm32::pac::FLASH.sr().read().bsy() {
        Timer::after_millis(1).await;
    }

    // Reload option bytes, required to actually commit the option bytes for some reason
    let mut cr = embassy_stm32::pac::FLASH.cr().read();
    cr.set_obl_launch(true);
    embassy_stm32::pac::FLASH.cr().write_value(cr);

    while embassy_stm32::pac::FLASH.cr().read().obl_launch() {
        Timer::after_millis(1).await;
    }

    // Force reset incase we don't reboot
    cortex_m::peripheral::SCB::sys_reset();
}

async fn echo<'a, 'd, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>, led: &mut Output<'a>) {
    let mut buf = [0; 64];
    loop {
        match class.read_packet(&mut buf).await {
            Ok(n) => {
                let data = &buf[..n];
                class.write_packet(data).await.unwrap();
                led.toggle();
            }
            Err(e) => {
                defmt::info!("echo disconnected: {:?}", e);
                // For now, go to bootloader
                set_boot_option(false, true, false).await;
            }
        }
    }
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        defmt::info!("get_report id:{:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, _data: &[u8]) -> OutResponse {
        defmt::info!("set_report id:{:?}", id);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, _dur: u32) {
        defmt::info!("set_idle_ms id:{:?}", id);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        defmt::info!("get_idle_ms id:{:?}", id);
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
        defmt::info!("enabled: {:?}", enabled);
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        defmt::info!("reset");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        defmt::info!("addressed: {:?}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        defmt::info!("configured: {:?}", configured);
    }
}
