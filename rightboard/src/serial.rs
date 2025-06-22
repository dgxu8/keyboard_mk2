use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, RingBufferedUartRx, UartTx};
use embassy_time::{with_timeout, Duration};
use heapless::Vec;

#[derive(Debug)]
pub enum Error {
    UsartError(usart::Error),
    OutOfMemory,
    PacketError,
    EmptyBuffer,
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

#[derive(Debug)]
#[derive(PartialEq, Eq)]
pub enum PacketId {
    Ack = 0,
    Nak = 1,
    KeyChange = 2,
    FullState = 3,
    AltState = 4,
    Timestamp = 5, // Temp
}

pub type SerialBuffer = Vec<u8, 32>;

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsTx {
    async fn write_cobs(&mut self, id: PacketId, payload: &[u8]) -> Result<(), Error>;
    async fn send_ack(&mut self, id: PacketId) -> Result<(), Error>;
    async fn send_nack(&mut self, id: PacketId) -> Result<(), Error>;
}

impl<'a> CobsTx for UartTx<'a, Async> {
    async fn send_ack(&mut self, id: PacketId) -> Result<(), Error> {
        assert_ne!(id, PacketId::Ack);
        let ack: [u8; 4] = [0x1, 0x2, id as _, 0x0];
        self.write(ack.as_slice()).await?;
        Ok(())
    }
    async fn send_nack(&mut self, id: PacketId) -> Result<(), Error> {
        assert_ne!(id, PacketId::Ack);
        let ack: [u8; 4] = [0x3, PacketId::Nak as _, id as _, 0x0];
        self.write(ack.as_slice()).await?;
        Ok(())
    }
    async fn write_cobs(&mut self, id: PacketId, payload: &[u8]) -> Result<(), Error> {
        assert_ne!(id, PacketId::Ack);
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
        self.write(packet.as_slice()).await?;
        Ok(())
    }
}

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsRx {
    async fn read_cobs(&mut self, payload: &mut SerialBuffer) -> Result<(), Error>;
}

impl<'a> CobsRx for RingBufferedUartRx<'a> {
    async fn read_cobs(&mut self, payload: &mut SerialBuffer) -> Result<(), Error> {
        let mut byte: [u8; 1] = [0; 1];
        if let Err(_) = with_timeout(Duration::from_millis(100), self.read(&mut byte)).await {
            return Err(Error::EmptyBuffer);
        }

        let mut overflow_idx = byte[0];
        loop {
            self.read(&mut byte).await?;
            if byte[0] == 0 {
                break;
            }

            overflow_idx -= 1;
            if overflow_idx == 0 {
                overflow_idx = byte[0];
                payload.push(0)?;
            } else {
                payload.push(byte[0])?;
            }
        }

        if overflow_idx != 1 || payload.len() < 1 {
            // TODO: Maybe send NACK?
            Err(Error::PacketError)
        } else {
            // TODO: Maybe send ACK?
            Ok(())
        }
    }
}
