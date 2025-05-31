use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;
use heapless::Vec;

#[derive(Debug)]
pub enum Error {
    OutOfMemory,
}

#[trait_variant::make(Send)]  // Needed for public async trait
pub trait Cobs {
    async fn write_cobs(&mut self, payload: &[u8]) -> Result<(), Error>;
}

impl<'a> Cobs for UartTx<'a, Async> {
    async fn write_cobs(&mut self, payload: &[u8]) -> Result<(), Error> {
        let mut packet: Vec<u8, 32> = Vec::new();
        if let Err(x) = cobs_encode(payload, &mut packet) {
            return Err(x)
        }
        self.write(packet.as_slice()).await.unwrap();
        Ok(())
    }
}

fn cobs_encode(payload: &[u8], packet: &mut Vec<u8, 32>) -> Result<(), Error> {
    packet.clear();
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
    Ok(())
}
