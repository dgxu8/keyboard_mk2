use core::ops::{Deref, DerefMut};

use embassy_stm32::gpio::Pull;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Config, Parity, RingBufferedUartRx, UartTx};
use embedded_io::ReadReady;
use embedded_io_async::Write;
use heapless::Vec;

pub fn cobs_config() -> Config {
    let mut config = Config::default();
    config.baudrate = 4_000_000;
    config.rx_pull = Pull::Up;
    config
}

pub fn bl_config() -> Config {
    let mut config = Config::default();
    config.rx_pull = Pull::Up;
    config.parity = Parity::ParityEven;
    config
}

#[derive(Debug)]
pub enum Error {
    UsartError(usart::Error),
    OutOfMemory,
    PacketError,
    EmptyBuffer,
    InvalidId,
    ReadError(u8),
}

impl From <u8> for Error {
    #[cold]
    fn from(_: u8) -> Self {
        Error::OutOfMemory
    }
}

impl From <usart::Error> for Error {
    #[cold]
    fn from(error: usart::Error) -> Self {
        Error::UsartError(error)
    }
}

pub const ACK: u8 = 0;
pub const NAK: u8 = 1;

pub const KEY_CHANGE: u8 = 2;
pub const GET_STATE: u8 = 3;
pub const FULL_STATE: u8 = 4;
pub const ALT_STATE: u8 = 5;
pub const ALT_ENABLE: u8 = 6;
pub const CAPSLOCK: u8 = 7;
pub const ROTARY_CHANGE: u8 = 8;
pub const VOLUME: u8 = 9;

pub const TIMESTAMP: u8 = 10; // Temp

pub type SerialBuffer = Vec<u8, 32>;

pub struct CobsBuffer {
    buf: SerialBuffer,
    ovfl_idx: usize,
}

impl CobsBuffer {
    pub fn new() -> CobsBuffer {
        CobsBuffer {
            buf: SerialBuffer::new(),
            ovfl_idx: 0,
        }
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.ovfl_idx = 0;
    }
}

impl Deref for CobsBuffer {
    type Target = SerialBuffer;

    fn deref(&self) -> &Self::Target {
        &self.buf
    }
}

impl DerefMut for CobsBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsTx {
    async fn write_cobs(&mut self, id: u8, payload: &[u8]) -> Result<(), Error>;
    async fn send_ack(&mut self, id: u8) -> Result<(), Error>;
    async fn send_nack(&mut self, id: u8) -> Result<(), Error>;
}

impl<'a> CobsTx for UartTx<'a, Async> {
    async fn send_ack(&mut self, id: u8) -> Result<(), Error> {
        assert_ne!(id, ACK);
        let ack: [u8; 4] = [0x1, 0x2, id, 0x0];
        self.write_all(ack.as_slice()).await?;
        Ok(())
    }
    async fn send_nack(&mut self, id: u8) -> Result<(), Error> {
        assert_ne!(id, ACK);
        let ack: [u8; 4] = [0x3, NAK, id as _, 0x0];
        self.write_all(ack.as_slice()).await?;
        Ok(())
    }
    async fn write_cobs(&mut self, id: u8, payload: &[u8]) -> Result<(), Error> {
        assert_ne!(id, ACK);
        let mut packet: SerialBuffer = Vec::from_slice(&[2, id as u8]).unwrap();
        let mut overhead_idx = 0;
        for byte in payload {
            let byte = *byte;
            if byte == 0 {
                overhead_idx = packet.len();
            }
            packet.push(byte)?;
            packet[overhead_idx] += 1;
        }
        packet.push(0)?;
        self.write_all(packet.as_slice()).await?;
        Ok(())
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsRx {
    async fn read_cobs<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<(), Error>;
}

impl<'a> CobsRx for RingBufferedUartRx<'a> {
    async fn read_cobs<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<(), Error> {
        let mut byte: [u8; 1] = [0; 1];

        while rslt.ovfl_idx == 0 {
            self.read(&mut byte).await?;
            rslt.ovfl_idx = byte[0] as _;
        }

        loop {
            self.read(&mut byte).await?;
            rslt.ovfl_idx -= 1;
            if byte[0] == 0 {
                break;
            }

            if rslt.ovfl_idx == 0 {
                rslt.ovfl_idx = byte[0] as _;
                rslt.push(0)?;
            } else {
                rslt.push(byte[0])?;
            }
        }
        if rslt.ovfl_idx != 0 || rslt.len() < 1 {
            rslt.reset();
            return Err(Error::PacketError);
        }
        Ok(())
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait Clearable {
    async fn clear(&mut self) -> Result<(), Error>;
}

impl<'a> Clearable for RingBufferedUartRx<'a> {
    async fn clear(&mut self) -> Result<(), Error> {
        let mut byte: [u8; 1] = [0; 1];
        while self.read_ready().unwrap_or(false) {
            self.read(&mut byte).await?;
        }
        Ok(())
    }
}
