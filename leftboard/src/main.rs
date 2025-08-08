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
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

bind_interrupts!(struct USBIrqs {
    USB => usb::InterruptHandler<peripherals::USB>;
});

bind_interrupts!(struct UsartIrqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;

});

static UART: StaticCell<Uart<'_, Async>> = StaticCell::new();

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
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 50,
        max_packet_size: 8,
    };

    let h = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut hid_state, hid_cfg);

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
        loop {
            led1.toggle();
            Timer::after_millis(500).await;
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {},
                Err(_) => {}
            };
            Timer::after_millis(100).await;
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {},
                Err(_) => {}
            };
        }
    };
    let out_fut = async {
        reader.run(false, &mut rqst_handler).await;
    };
    join4(usb_fut, echo_fut, blink_fut, out_fut).await;
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
            Err(_) => {
                return;
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
