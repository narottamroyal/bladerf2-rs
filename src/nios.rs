//! NIOS II control protocol.
//!
//! The bladeRF 2.0's FPGA contains a NIOS II soft-core that provides
//! register access to the AD9361 RFIC, the PLL, and other peripherals via a
//! request-response protocol over USB bulk endpoints:
//!
//! 1. Write a 16-byte command packet to the PeripheralOut endpoint.
//! 2. Read a 16-byte response packet from the PeripheralIn endpoint.

use std::mem::size_of;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use nusb::{
    Interface,
    transfer::{Bulk, In, Out},
};

use crate::error::{Error, Result};
use crate::maybe_future::with_timeout;
use crate::usb::{self, Endpoint};

/// Timeout for a NIOS request/response exchange.
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(5);

/// Size of a NIOS packet in bytes.
pub(crate) const PACKET_SIZE: usize = 16;

// ==================== Packet type system ====================

/// Data field of a NIOS packet.
pub(crate) trait NiosData: Copy {
    /// Decode from a byte slice.
    fn read(buf: &[u8]) -> Option<Self>;
    /// Encode into a byte slice.
    fn write(self, buf: &mut [u8]);
}

macro_rules! impl_nios_data {
    ($($t:ty),* $(,)?) => {
        $(
            impl NiosData for $t {
                fn read(buf: &[u8]) -> Option<Self> {
                    let buf: &[u8; size_of::<$t>()] = buf.get(..size_of::<$t>())?.try_into().ok()?;
                    Some(<$t>::from_le_bytes(*buf))
                }
                fn write(self, buf: &mut [u8]) {
                    buf[..size_of::<$t>()].copy_from_slice(&self.to_le_bytes());
                }
            }
        )*
    };
}
impl_nios_data!(u8, u16, u32, u64);

/// Address field of a NIOS packet.
pub(crate) trait NiosAddress {
    /// Encode into a byte slice.
    fn write(&self, buf: &mut [u8]);
    /// Offset of the data field within the packet.
    fn offset() -> usize;
}

macro_rules! impl_nios_address {
    ($($t:ty),* $(,)?) => {
        $(
            impl NiosAddress for $t {
                fn write(&self, buf: &mut [u8]) {
                    buf[..size_of::<$t>()].copy_from_slice(&self.to_le_bytes());
                }
                fn offset() -> usize { size_of::<$t>() + 4 }
            }
        )*
    };
}
impl_nios_address!(u8, u16);

/// A NIOS packet type: magic byte + address type + data type.
pub(crate) trait NiosPacket: Copy + Into<u8> {
    /// Address type.
    type Address: NiosAddress;
    /// Data type.
    type Data: NiosData + Default;
    /// Magic byte identifying the packet family.
    const MAGIC: u8;

    fn encode(self, addr: Self::Address, data: Self::Data, write: bool) -> [u8; PACKET_SIZE] {
        let mut buf = [0u8; PACKET_SIZE];
        buf[0] = Self::MAGIC;
        buf[1] = self.into();
        buf[2] = if write { 1 } else { 0 };
        addr.write(&mut buf[4..]);
        data.write(&mut buf[Self::Address::offset()..]);
        buf
    }

    fn decode(self, buf: &[u8; PACKET_SIZE]) -> Result<Self::Data> {
        // Check the success flag (bit 1)
        if (buf[2] & 0x02) == 0 {
            return Err(Error::Nios);
        }
        Self::Data::read(&buf[Self::Address::offset()..]).ok_or(Error::ValueConversion)
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

/// 8-bit address, 16-bit data packets.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Nios8x16 {
    /// AD5621 DAC control.
    Ad56x1Dac = 3,
    /// INA219 power monitor.
    Ina219 = 4,
}

/// 8-bit address, 32-bit data packets.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Nios8x32 {
    /// Version register.
    Version = 0,
    /// Control register.
    Control = 1,
    /// RF front-end control.
    RfFrontend = 3,
    /// ADF4002 PLL control.
    Adf400x = 4,
}

/// 8-bit address, 64-bit data packets.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Nios8x64 {
    /// Timestamp register.
    Timestamp = 0,
}

/// 16-bit address, 64-bit data packets.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum Nios16x64 {
    /// AD9361 direct SPI register access.
    Ad9361 = 0,
    /// RFIC high-level commands.
    RfIc = 1,
}

impl_nios_packet!(Nios8x16, b'B', u8, u16);
impl_nios_packet!(Nios8x32, b'C', u8, u32);
impl_nios_packet!(Nios8x64, b'D', u8, u64);
impl_nios_packet!(Nios16x64, b'E', u16, u64);

// ==================== Request/response pipe ====================

/// Raw request/response pipe to the NIOS core, built directly on the bulk
/// endpoint queue API (no nusb io adapters or runtime features needed).
struct Io {
    out: nusb::Endpoint<Bulk, Out>,
    input: nusb::Endpoint<Bulk, In>,
    /// IN buffer size aligned to the endpoint packet size (nusb requires IN
    /// buffer sizes to be multiples of the packet size).
    in_size: usize,
}

impl Io {
    fn in_size(input: &nusb::Endpoint<Bulk, In>) -> usize {
        usb::aligned_size(PACKET_SIZE, input.max_packet_size())
    }

    async fn transfer<Packet: NiosPacket>(
        &mut self,
        target: Packet,
        address: Packet::Address,
        data: Packet::Data,
        write: bool,
    ) -> Result<Packet::Data> {
        let packet = target.encode(address, data, write);

        if self.input.pending() == 0 {
            self.input.submit(nusb::transfer::Buffer::new(self.in_size));
        }

        self.out.submit(packet.to_vec().into());

        let response = with_timeout(
            async {
                usb::next_out(&mut self.out).await?;
                usb::next_in(&mut self.input).await
            },
            Some(TRANSFER_TIMEOUT),
        )
        .await
        .inspect_err(|e| tracing::error!(target: "bladerf2::nios", "nios: request failed: {e}"))?;

        let response_bytes: &[u8; PACKET_SIZE] = response
            .get(..PACKET_SIZE)
            .ok_or(Error::ValueConversion)?
            .try_into()
            .map_err(|_| Error::ValueConversion)?;

        target.decode(response_bytes)
    }
}

/// Exclusive access to the pipe: it is removed from the slot for the
/// duration of a transfer so no lock is held across I/O, and overlapping
/// control operations fail with [`Error::Busy`] instead of interleaving
/// packets. Restored on drop.
struct IoLease<'a> {
    slot: &'a Mutex<Option<Io>>,
    io: Option<Io>,
}

impl<'a> IoLease<'a> {
    fn new(slot: &'a Mutex<Option<Io>>) -> Result<Self> {
        let io = slot
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take()
            .ok_or(Error::Busy)?;
        Ok(Self { slot, io: Some(io) })
    }

    fn io_mut(&mut self) -> &mut Io {
        self.io.as_mut().expect("lease owns the pipe")
    }
}

impl Drop for IoLease<'_> {
    fn drop(&mut self) {
        *self.slot.lock().unwrap_or_else(PoisonError::into_inner) = self.io.take();
    }
}

/// NIOS II control channel (see the [module documentation](self)).
///
/// Internally serializes packet exchanges; multi-command sequences should be
/// driven from a single session to keep them from interleaving.
#[derive(Clone)]
pub struct NiosCore {
    io: Arc<Mutex<Option<Io>>>,
}

impl std::fmt::Debug for NiosCore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NiosCore").finish_non_exhaustive()
    }
}

impl NiosCore {
    /// Create a NIOS control channel from a claimed USB interface.
    // On wasm the nusb endpoints are not `Send`, so the `Arc` is only ever
    // shared on one thread there; one type serves both targets.
    #[cfg_attr(target_arch = "wasm32", allow(clippy::arc_with_non_send_sync))]
    pub(crate) fn new(interface: &Interface) -> Result<Self> {
        let out = interface.endpoint::<Bulk, Out>(Endpoint::PeripheralOut as u8)?;
        let input = interface.endpoint::<Bulk, In>(Endpoint::PeripheralIn as u8)?;
        let in_size = Io::in_size(&input);
        Ok(Self {
            io: Arc::new(Mutex::new(Some(Io {
                out,
                input,
                in_size,
            }))),
        })
    }

    /// Read a register via the NIOS protocol.
    pub(crate) async fn read<Packet: NiosPacket>(
        &self,
        target: Packet,
        address: Packet::Address,
    ) -> Result<Packet::Data> {
        let mut lease = IoLease::new(&self.io)?;
        lease
            .io_mut()
            .transfer(target, address, Packet::Data::default(), false)
            .await
    }

    /// Write a register via the NIOS protocol.
    pub(crate) async fn write<Packet: NiosPacket>(
        &self,
        target: Packet,
        address: Packet::Address,
        data: Packet::Data,
    ) -> Result<()> {
        let mut lease = IoLease::new(&self.io)?;
        lease.io_mut().transfer(target, address, data, true).await?;
        Ok(())
    }
}
