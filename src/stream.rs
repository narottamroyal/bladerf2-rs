use num_complex::Complex32;
use nusb::{
    Interface,
    io::{EndpointRead, EndpointWrite},
    transfer::{Bulk, In, Out},
};
use std::{
    io::{Read, Write},
    time::Duration,
};

use crate::{BladeRf2Error, Channel, Result, drivers::rfic, nios::Nios};
use crate::{
    Direction,
    usb::{self, Endpoint},
};

pub(crate) const BUFFER_SIZE: usize = 8192;

pub struct RxStream {
    reader: EndpointRead<Bulk>,
    interface: Interface,
    nios: Nios,
    channels: Vec<Channel>,
}

impl RxStream {
    pub fn new(interface: &Interface, nios: &Nios, channels: &[usize]) -> Result<Self> {
        let channels = match channels {
            [0] => vec![Channel::Rx0],
            [1] => vec![Channel::Rx1],
            [0, 1] => vec![Channel::Rx0, Channel::Rx1],
            _ => return Err(BladeRf2Error::ChannelConfig),
        };

        let reader = interface
            .endpoint::<Bulk, In>(Endpoint::SampleIn as u8)?
            .reader(BUFFER_SIZE)
            .with_num_transfers(32)
            .with_read_timeout(Duration::from_secs(5));

        Ok(Self {
            reader,
            interface: interface.clone(),
            nios: nios.clone(),
            channels,
        })
    }

    pub fn activate(&mut self) -> Result<()> {
        for &channel in self.channels.as_slice() {
            rfic::enable_module(&mut self.nios, channel, true)?;
        }
        usb::enable_module(&self.interface, Direction::Rx, true)?;
        Ok(())
    }

    pub fn deactivate(&mut self) -> Result<()> {
        usb::enable_module(&self.interface, Direction::Rx, false)?;
        for &channel in self.channels.as_slice() {
            rfic::enable_module(&mut self.nios, channel, false)?;
        }
        Ok(())
    }

    pub(crate) fn read_i16(&mut self, buf: &mut [i16]) -> Result<()> {
        self.reader.read_exact(bytemuck::cast_slice_mut(buf))?;
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [&mut [Complex32]]) -> Result<()> {
        assert_eq!(self.channels.len(), buf.len());
        let data = &mut [0; 4096];

        match buf.len() {
            1 => {
                for buffer in buf[0].chunks_mut(2048) {
                    let length = buffer.len() * 2;
                    self.read_i16(&mut data[..length])?;
                    q11_to_c32(&data[..length], buffer);
                }
            }
            2 => {
                let [buf0, buf1] = buf
                    .get_disjoint_mut([0, 1])
                    .expect("Two buffers are required");
                for (buffer0, buffer1) in buf0.chunks_mut(1024).zip(buf1.chunks_mut(1024)) {
                    let length = buffer0.len() * 4;
                    self.read_i16(&mut data[..length])?;

                    let data0 = data[..length].chunks_exact(2).step_by(2).flatten();
                    q11_to_c32(data0, buffer0);
                    let data1 = data[..length].chunks_exact(2).skip(1).step_by(2).flatten();
                    q11_to_c32(data1, buffer1);
                }
            }
            _ => return Err(BladeRf2Error::ChannelConfig),
        }
        Ok(())
    }
}

impl Drop for RxStream {
    fn drop(&mut self) {
        self.deactivate().expect("Failed to deactivate RX stream")
    }
}

pub struct TxStream {
    writer: EndpointWrite<Bulk>,
    interface: Interface,
    nios: Nios,
    channels: Vec<Channel>,
}

impl TxStream {
    pub fn new(interface: &Interface, nios: &Nios, channels: &[usize]) -> Result<Self> {
        let channels = match channels {
            [0] => vec![Channel::Tx0],
            [1] => vec![Channel::Tx1],
            [0, 1] => vec![Channel::Tx0, Channel::Tx1],
            _ => return Err(BladeRf2Error::ChannelConfig),
        };

        let writer = interface
            .endpoint::<Bulk, Out>(Endpoint::SampleOut as u8)?
            .writer(BUFFER_SIZE)
            .with_num_transfers(32)
            .with_write_timeout(Duration::from_secs(5));

        Ok(Self {
            writer,
            interface: interface.clone(),
            nios: nios.clone(),
            channels,
        })
    }

    pub fn activate(&mut self) -> Result<()> {
        for &channel in self.channels.as_slice() {
            rfic::enable_module(&mut self.nios, channel, true)?;
        }
        usb::enable_module(&self.interface, Direction::Tx, true)?;
        Ok(())
    }

    pub fn deactivate(&mut self) -> Result<()> {
        usb::enable_module(&self.interface, Direction::Tx, false)?;
        for &channel in self.channels.as_slice() {
            rfic::enable_module(&mut self.nios, channel, false)?;
        }
        Ok(())
    }

    pub(crate) fn write_i16(&mut self, buf: &[i16]) -> Result<()> {
        self.writer.write_all(bytemuck::cast_slice(buf))?;
        Ok(())
    }

    pub fn write(&mut self, buf: &[&[Complex32]]) -> Result<()> {
        assert_eq!(self.channels.len(), buf.len());
        let data = &mut [0; 4096];

        match buf.len() {
            1 => {
                for buffer in buf[0].chunks(2048) {
                    c32_to_q11(buffer, data.iter_mut());
                    self.write_i16(data)?;
                }
            }
            2 => {
                for (buffer0, buffer1) in buf[0].chunks(1024).zip(buf[1].chunks(1024)) {
                    let data0 = data.chunks_exact_mut(2).step_by(2).flatten();
                    c32_to_q11(buffer0, data0);
                    let data1 = data.chunks_exact_mut(2).skip(1).step_by(2).flatten();
                    c32_to_q11(buffer1, data1);

                    self.write_i16(data)?;
                }
            }
            _ => return Err(BladeRf2Error::ChannelConfig),
        }
        Ok(())
    }
}

impl Drop for TxStream {
    fn drop(&mut self) {
        self.deactivate().expect("Failed to deactivate TX stream")
    }
}

fn q11_to_c32<'a>(input: impl IntoIterator<Item = &'a i16>, output: &mut [Complex32]) {
    const SCALE: f32 = 1.0 / 2048.0;
    input
        .into_iter()
        .zip(bytemuck::cast_slice_mut(output))
        .for_each(|(&x, y)| *y = x as f32 * SCALE);
}

fn c32_to_q11<'a>(input: &[Complex32], output: impl IntoIterator<Item = &'a mut i16>) {
    const SCALE: f32 = 2048.0;
    bytemuck::cast_slice::<_, f32>(input)
        .iter()
        .zip(output)
        .for_each(|(&x, y)| *y = (x * SCALE).round().clamp(-SCALE, SCALE - 1.0) as i16);
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_q11_and_c32_conversion() {
        let input = &std::array::from_fn::<_, 4096, _>(|i| i as i16 - 2048);
        let buffer = &mut [Complex32::ZERO; 2048];
        let output = &mut [0; 4096];
        q11_to_c32(input, buffer);
        c32_to_q11(buffer, output.iter_mut());
        assert_eq!(input, output);
    }
}
