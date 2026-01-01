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

use crate::{Channel, Result, drivers::rfic, nios::Nios};
use crate::{
    Direction,
    usb::{self, Endpoint},
};

pub(crate) const BUFFER_SIZE: usize = 8192;

pub struct RxStream {
    reader: EndpointRead<Bulk>,
    interface: Interface,
    nios: Nios,
}

impl RxStream {
    pub fn new(interface: &Interface, nios: &Nios) -> Result<Self> {
        let reader = interface
            .endpoint::<Bulk, In>(Endpoint::SampleIn as u8)?
            .reader(BUFFER_SIZE)
            .with_num_transfers(8)
            .with_read_timeout(Duration::from_secs(5));

        Ok(Self {
            reader,
            interface: interface.clone(),
            nios: nios.clone(),
        })
    }

    pub fn activate(&mut self) -> Result<()> {
        rfic::enable_module(&mut self.nios, Channel::Rx0, true)?;
        usb::enable_module(&self.interface, Direction::Rx, true)
    }

    pub fn deactivate(&mut self) -> Result<()> {
        usb::enable_module(&self.interface, Direction::Rx, false)?;
        rfic::enable_module(&mut self.nios, Channel::Rx0, false)
    }

    pub fn read_i16(&mut self, buf: &mut [i16]) -> Result<()> {
        self.reader.read_exact(bytemuck::cast_slice_mut(buf))?;
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [Complex32]) -> Result<()> {
        let scale = |x: i16| x as f32 / 2048.0;
        let raw = &mut [0; 4096];
        for data in buf.chunks_mut(2048) {
            let length = data.len() * 2;
            self.read_i16(&mut raw[..length])?;
            raw[..length]
                .chunks_exact(2)
                .map(|x| Complex32::new(scale(x[0]), scale(x[1])))
                .zip(data.iter_mut())
                .for_each(|(x, y)| *y = x);
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
}

impl TxStream {
    pub fn new(interface: &Interface, nios: &Nios) -> Result<Self> {
        let writer = interface
            .endpoint::<Bulk, Out>(Endpoint::SampleOut as u8)?
            .writer(BUFFER_SIZE)
            .with_num_transfers(8)
            .with_write_timeout(Duration::from_secs(5));

        Ok(Self {
            writer,
            interface: interface.clone(),
            nios: nios.clone(),
        })
    }

    pub fn activate(&mut self) -> Result<()> {
        rfic::enable_module(&mut self.nios, Channel::Tx0, true)?;
        usb::enable_module(&self.interface, Direction::Tx, true)
    }

    pub fn deactivate(&mut self) -> Result<()> {
        usb::enable_module(&self.interface, Direction::Tx, false)?;
        rfic::enable_module(&mut self.nios, Channel::Tx0, false)
    }

    pub fn write_i16(&mut self, buf: &[i16]) -> Result<()> {
        self.writer.write_all(bytemuck::cast_slice(buf))?;
        Ok(())
    }

    pub fn write(&mut self, buf: &[Complex32]) -> Result<()> {
        let scale = |x: f32| (x * 2048.0).clamp(-2048.0, 2047.0) as i16;
        for data in buf.chunks(2048) {
            let mut buf = [0; 4096];
            data.iter()
                .flat_map(|x| [scale(x.re), scale(x.im)])
                .zip(buf.iter_mut())
                .for_each(|(x, y)| *y = x);
            self.write_i16(&buf)?;
        }
        Ok(())
    }
}

impl Drop for TxStream {
    fn drop(&mut self) {
        self.deactivate().expect("Failed to deactivate TX stream")
    }
}
