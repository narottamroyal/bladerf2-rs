use nusb::{
    Interface,
    io::{EndpointRead, EndpointWrite},
    transfer::{Bulk, In, Out},
};
use std::{
    io::{Read, Write},
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{BladeRf2Error, Result, usb::Endpoint};
use core::mem::size_of;

pub(crate) trait NiosData: Copy {
    fn read(buf: &[u8]) -> Option<Self>;
    fn write(self, buf: &mut [u8]);
}

macro_rules! impl_nios_data {
    ($t:ty) => {
        impl NiosData for $t {
            fn read(buf: &[u8]) -> Option<Self> {
                let buf = buf.get(..size_of::<Self>())?.try_into().ok()?;
                Some(<$t>::from_le_bytes(buf))
            }

            fn write(self, buf: &mut [u8]) {
                buf[..size_of::<Self>()].copy_from_slice(&self.to_le_bytes());
            }
        }
    };
}

impl_nios_data!(u8);
impl_nios_data!(u16);
impl_nios_data!(u32);
impl_nios_data!(u64);

pub(crate) trait NiosAddress {
    fn write(&self, buf: &mut [u8]);
    fn offset() -> usize;
}

macro_rules! impl_nios_address {
    ($t:ty) => {
        impl NiosAddress for $t {
            fn write(&self, buf: &mut [u8]) {
                buf[..size_of::<Self>()].copy_from_slice(&self.to_le_bytes());
            }

            fn offset() -> usize {
                size_of::<Self>() + 4
            }
        }
    };
}

impl_nios_address!(u8);
impl_nios_address!(u16);

pub(crate) const BUFFER_SIZE: usize = 16;

pub(crate) trait NiosPacket: Copy + Into<u8> {
    type Address: NiosAddress;
    type Data: NiosData + Default;
    const MAGIC: u8;

    fn encode(self, addr: Self::Address, data: Self::Data, write: bool) -> [u8; BUFFER_SIZE] {
        let mut buf = [0u8; 16];
        buf[0] = Self::MAGIC;
        buf[1] = self.into();
        buf[2] = if write { 1 } else { 0 };
        addr.write(&mut buf[4..]);
        data.write(&mut buf[Self::Address::offset()..]);

        buf
    }

    fn decode(self, buf: &[u8; BUFFER_SIZE]) -> Result<Self::Data> {
        // Check success flag
        if (buf[2] & 0x02) == 0 {
            return Err(BladeRf2Error::Nios);
        }

        Self::Data::read(&buf[Self::Address::offset()..]).ok_or(BladeRf2Error::ValueConversion)
    }
}

macro_rules! impl_nios_packet {
    ($enum:ty, $magic:literal, $addr:ty, $data:ty) => {
        impl From<$enum> for u8 {
            fn from(e: $enum) -> u8 {
                e as u8
            }
        }

        impl NiosPacket for $enum {
            const MAGIC: u8 = $magic;
            type Address = $addr;
            type Data = $data;
        }
    };
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub(crate) enum Nios8x16 {
    Ad56x1Dac = 3,
    Ina219 = 4,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub(crate) enum Nios8x32 {
    Version = 0,
    Control = 1,
    RfFrontend = 3,
    Adf400x = 4,
}

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub(crate) enum Nios8x64 {
    Timestamp = 0,
}

#[repr(u16)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub(crate) enum Nios16x64 {
    RfIc = 1,
}

impl_nios_packet!(Nios8x16, b'B', u8, u16);
impl_nios_packet!(Nios8x32, b'C', u8, u32);
impl_nios_packet!(Nios8x64, b'D', u8, u64);
impl_nios_packet!(Nios16x64, b'E', u16, u64);

struct Io {
    reader: EndpointRead<Bulk>,
    writer: EndpointWrite<Bulk>,
}

#[derive(Clone)]
pub struct Nios {
    io: Arc<Mutex<Io>>,
}

impl Nios {
    pub fn new(interface: &Interface) -> Result<Self> {
        let reader = interface
            .endpoint::<Bulk, In>(Endpoint::PeripheralIn as u8)?
            .reader(BUFFER_SIZE)
            .with_read_timeout(Duration::from_secs(5));
        let writer = interface
            .endpoint::<Bulk, Out>(Endpoint::PeripheralOut as u8)?
            .writer(BUFFER_SIZE)
            .with_write_timeout(Duration::from_secs(5));

        Ok(Self {
            io: Arc::new(Mutex::new(Io { reader, writer })),
        })
    }

    pub(crate) fn read<Packet: NiosPacket>(
        &self,
        target: Packet,
        address: Packet::Address,
    ) -> Result<Packet::Data> {
        let mut io = self.io.lock().unwrap_or_else(|e| e.into_inner());
        let mut buf = target.encode(address, Packet::Data::default(), false);
        io.writer.write_all(&buf)?;
        io.writer.flush()?;
        io.reader.read_exact(&mut buf)?;
        target.decode(&buf)
    }

    pub(crate) fn write<Packet: NiosPacket>(
        &self,
        target: Packet,
        address: Packet::Address,
        data: Packet::Data,
    ) -> Result<()> {
        let mut io = self.io.lock().unwrap_or_else(|e| e.into_inner());
        let mut buf = target.encode(address, data, true);
        io.writer.write_all(&buf)?;
        io.writer.flush()?;
        io.reader.read_exact(&mut buf)?;
        Ok(())
    }
}
