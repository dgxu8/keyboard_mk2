use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{with_timeout, Duration};
use heapless::Vec;

#[derive(Debug)]
pub enum Error {
    OutOfMemory,
    PacketError,
    EmptyBuffer,
}

pub type SerialBuffer = Vec<u8, 32>;

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait CobsTx {
    async fn write_cobs(&mut self, payload: &[u8]) -> Result<(), Error>;
}

impl<'a> CobsTx for UartTx<'a, Async> {
    async fn write_cobs(&mut self, payload: &[u8]) -> Result<(), Error> {
        let mut packet: SerialBuffer = Vec::new();
        packet.push(1).unwrap();

        let mut overhead_idx = 0;
        for byte in payload {
            let byte = *byte;
            if byte == 0 {
                overhead_idx = packet.len();
            }
            match packet.push(byte) {
                Ok(x) => x,
                Err(_) => return Err(Error::OutOfMemory),
            };
            packet[overhead_idx] += 1;
        }
        match packet.push(0) {
            Ok(x) => x,
            Err(_) => return Err(Error::OutOfMemory),
        };
        self.write(packet.as_slice()).await.unwrap();
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
            self.read(&mut byte).await.unwrap();
            if byte[0] == 0 {
                break;
            }

            overflow_idx -= 1;
            if overflow_idx == 0 {
                overflow_idx = byte[0];
                payload.push(0).unwrap();
            } else {
                payload.push(byte[0]).unwrap();
            }
        }

        if overflow_idx != 1 || payload.len() < 1 {
            Err(Error::PacketError)
        } else {
            Ok(())
        }
    }
}
