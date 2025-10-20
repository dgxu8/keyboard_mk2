#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

extern crate panic_halt;

mod usb_com;
mod usb;
mod logger;
mod serial;
mod keyscan;

use cortex_m::asm;
use defmt;

use embassy_executor::Spawner;
use embassy_futures::join;
use embassy_stm32::rcc::mux::Clk48sel;
use embassy_stm32::rcc::{Hsi48Config, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::usart::Uart;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use util::cobs_uart::{cobs_config, UartTxMutex};

use crate::keyscan::Keyscan;
use crate::usb::{init_usb, MyRequestHandler, NKROKeyboardReport};
use crate::usb_com::CoprocCtrl;

bind_interrupts!(struct USBIrqs {
    USB => embassy_stm32::usb::InterruptHandler<peripherals::USB>;
});

bind_interrupts!(struct UsartIrqs {
    USART2 => embassy_stm32::usart::InterruptHandler<peripherals::USART2>;
});

// static UART: StaticCell<Uart<'static, Async>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Validate boot options before doing anything else
    check_boot_option();

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

    let _led0 = Output::new(p.PB0, Level::Low, Speed::Low);
    // embassy_stm32::pac::GPIOB.bsrr().write(|w| w.set_bs(0, true));
    let mut led1 = Output::new(p.PB1, Level::Low, Speed::Low);

    let select_pins = [
        Output::new(p.PA4, Level::Low, Speed::VeryHigh),
        Output::new(p.PA5, Level::Low, Speed::VeryHigh),
        Output::new(p.PA6, Level::Low, Speed::VeryHigh),
    ];
    let keys_input = [
        Input::new(p.PB3, Pull::Down),
        Input::new(p.PB4, Pull::Down),
        Input::new(p.PB5, Pull::Down),
        Input::new(p.PB6, Pull::Down),
        Input::new(p.PB7, Pull::Down),
    ];
    let keys = Keyscan::new(
        select_pins,
        Output::new(p.PA7, Level::High, Speed::Low),
        keys_input,
    );

    let mut rb_ctrl = CoprocCtrl {
        boot0: Output::new(p.PA14, Level::Low, Speed::Low),
        n_rst: Output::new(p.PA15, Level::Low, Speed::Low),
    };

    let uart = Uart::new(
        p.USART2,
        p.PA3, p.PA2,  // RX, TX
        UsartIrqs,
        p.DMA1_CH7, p.DMA1_CH6, // TX, RX
        cobs_config()
    ).unwrap();
    let (uart_tx, uart_rx) = uart.split();
    static UART_TX: StaticCell<UartTxMutex> = StaticCell::new();
    let uart_tx = UART_TX.init(Mutex::new(uart_tx));

    let driver = Driver::new(p.USB, USBIrqs, p.PA12, p.PA11);
    let (mut usb, hid, acm0, acm1) = init_usb(driver);
    let usb_usages = usb.buffer_usage();
    defmt::info!("{:?}", usb_usages);
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();
    let (defmt_out, acm0_in) = acm0.split();

    Timer::after_millis(5).await;
    rb_ctrl.n_rst.set_high();

    spawner.spawn(logger::run(defmt_out)).unwrap();
    spawner.spawn(usb_com::run(acm0_in, uart_tx, rb_ctrl)).unwrap();
    spawner.spawn(serial::run(uart_tx, uart_rx, acm1)).unwrap();
    spawner.spawn(keyscan::run(keys)).unwrap();

    let blink_fut = async {
        let mut keycode = [0; 13];
        loop {
            led1.toggle();
            Timer::after_millis(1500).await;

            keycode[10] = 1 << 3;
            defmt::info!("Key: {}", keycode[10]);
            // keycode[10] = 1 << KeyboardUsage::KeyboardAa as u8;
            let report = NKROKeyboardReport {
                keycodes: keycode,
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {},
                Err(e) => defmt::warn!("Error sending key {:?}", e as u8),
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
                Err(e) => defmt::warn!("Error no sending key {:?}", e as u8),
            };
        }
    };
    let mut rqst_handler = MyRequestHandler {};
    let out_fut = async {
        reader.run(true, &mut rqst_handler).await;
    };
    join::join3(usb_fut, blink_fut, out_fut).await;
}

#[inline(always)]
fn check_boot_option() {
    let optr = embassy_stm32::pac::FLASH.optr().read();
    if !optr.n_boot0() || !optr.n_boot1() || !optr.n_swboot0() {
        set_boot_option(true, true, true);
    }
}

fn set_boot_option(n_boot0: bool, n_boot1: bool, n_swboot0: bool) -> ! {
    const FLASH_KEY1: u32 = 0x4567_0123;
    const FLASH_KEY2: u32 = 0xCDEF_89AB;
    const OPT_KEY1: u32 =  0x0819_2A3B;
    const OPT_KEY2: u32 =  0x4C5D_6E7F;
    while embassy_stm32::pac::FLASH.sr().read().bsy() {
        asm::nop();
    }
    // Unlock FLASH
    embassy_stm32::pac::FLASH.keyr().write_value(FLASH_KEY1);
    embassy_stm32::pac::FLASH.keyr().write_value(FLASH_KEY2);
    // Unlock OPT Registers
    embassy_stm32::pac::FLASH.optkeyr().write_value(OPT_KEY1);
    embassy_stm32::pac::FLASH.optkeyr().write_value(OPT_KEY2);

    // let cr = embassy_stm32::pac::FLASH.cr().read();
    // defmt::info!("cr: {}, {}", cr.lock(), cr.optlock());

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
        asm::nop();
    }

    // Reload option bytes, required to actually commit the option bytes for some reason
    let mut cr = embassy_stm32::pac::FLASH.cr().read();
    cr.set_obl_launch(true);
    embassy_stm32::pac::FLASH.cr().write_value(cr);

    while embassy_stm32::pac::FLASH.cr().read().obl_launch() {
        asm::nop();
    }

    // Force reset incase we don't reboot
    cortex_m::peripheral::SCB::sys_reset();
}
