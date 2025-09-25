use core::ops::{Deref, DerefMut};

use embassy_stm32::gpio::Pull;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Config, Parity, RingBufferedUartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io::ReadReady;
use embedded_io_async::Write;
use heapless::Vec;
use strum::FromRepr;

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

#[derive(Debug, PartialEq, Eq, defmt::Format)]
pub enum Error {
    UsartError(usart::Error),
    OutOfMemory,
    PacketError,
    InvalidId(u8),
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

#[derive(FromRepr, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum CmdId {
    Ack = 0,
    Nak = 1,

    GetState = 2, // Expects RspnId::FullState or RspnId::AltState

    // Basically a second layer where the only difference is that we go from arrow keys -> num keys
    AltEnable = 3,
    Capslock = 4,
    Volume = 5,
    OLEDMsg = 6,
}

#[derive(FromRepr, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RspnId {
    Ack = 0,
    Nak = 1,

    // Triggered via CmdId::GetState
    FullState = 2,
    AltState = 3,

    KeyChange = 4,
    RotaryChange = 5,

    Timestamp = 64,
    DefmtMsg = 128,
}


pub type SerialBuffer = Vec<u8, 32>;
pub type SerialRx<'a> = RingBufferedUartRx<'a>;
pub type SerialTx<'a> = UartTx<'a, Async>;
pub type UartTxMutex = Mutex<CriticalSectionRawMutex, SerialTx<'static>>;

pub struct CobsBuffer {
    buf: SerialBuffer,
    ovfl_idx: usize,
    opcode: Option<u8>,
}

impl CobsBuffer {
    pub fn new() -> CobsBuffer {
        CobsBuffer {
            buf: SerialBuffer::new(),
            ovfl_idx: 0,
            opcode: None,
        }
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.ovfl_idx = 0;
        self.opcode = None;
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

#[cfg(not(feature = "coproc"))]
type WriteType = CmdId;
#[cfg(feature = "coproc")]
type WriteType = RspnId;

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsTx {
    async fn write_cobs(&mut self, id: u8, payload: &[u8]) -> Result<(), Error>;
    async fn send_ack(&mut self, id: u8) -> Result<(), Error>;
    async fn send_nack(&mut self, id: u8) -> Result<(), Error>;
    async fn send(&mut self, id: WriteType, payload: &[u8]) -> Result<(), Error>;
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
        let mut packet: SerialBuffer = Vec::from_slice(&[2, id]).unwrap();
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
    #[inline(always)]
    async fn send(&mut self, id: WriteType, payload: &[u8]) -> Result<(), Error> {
        self.write_cobs(id as u8, payload).await
    }
}

#[cfg(not(feature = "coproc"))]
type ReadType = RspnId;
#[cfg(feature = "coproc")]
type ReadType = CmdId;

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsRx {
    async fn read_cobs<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<u8, Error>;
    async fn recv<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<ReadType, Error>;
}

impl<'a> CobsRx for RingBufferedUartRx<'a> {
    #[inline(always)]
    async fn read_cobs<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<u8, Error> {
        let mut byte: [u8; 1] = [0; 1];

        while rslt.ovfl_idx == 0 {
            self.read(&mut byte).await?;
            rslt.ovfl_idx = byte[0] as _;
        }

        if rslt.opcode == None {
            self.read(&mut byte).await?;
            rslt.ovfl_idx -= 1;
            if rslt.ovfl_idx == 0 {
                rslt.ovfl_idx = byte[0] as _;
                rslt.opcode = Some(0);
            } else {
                rslt.opcode = Some(byte[0]);
            }
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
        if rslt.ovfl_idx != 0 {
            return Err(Error::PacketError);
        }
        Ok(rslt.opcode.unwrap())
    }
    async fn recv<'b>(&mut self, rslt: &'b mut CobsBuffer) -> Result<ReadType, Error> {
        let id = self.read_cobs(rslt).await?;
        ReadType::from_repr(id).ok_or(Error::InvalidId(id))
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
