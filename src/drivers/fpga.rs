//! FPGA control register: RX sample source and stream timestamps.

use crate::error::{Error, Result};
use crate::nios::{Nios8x32, NiosCore};

/// Where RX samples come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxSource {
    /// The RFIC: normal operation.
    Rfic,
    /// A 12-bit test counter, to check the RX path without a signal. I counts
    /// up from 256 each time RX is enabled, wrapping from 2047 to -2047, and Q
    /// is -I. Every channel gets the same value.
    Counter,
    /// TX samples, looped back inside the FPGA. Only applies while RX is
    /// enabled; otherwise TX samples go to the RFIC and are transmitted.
    Loopback,
}

/// The source field of the control register (bits 8-10), and its values.
const RX_SOURCE_SHIFT: u32 = 8;
const RX_SOURCE_MASK: u32 = 0x7 << RX_SOURCE_SHIFT;
const RFIC: u32 = 0;
const COUNTER: u32 = 1;
const LOOPBACK: u32 = 4;

/// Where RX samples come from.
pub(crate) async fn rx_source(nios: &NiosCore) -> Result<RxSource> {
    let control = nios.read(Nios8x32::Control, 0).await?;
    match (control & RX_SOURCE_MASK) >> RX_SOURCE_SHIFT {
        RFIC => Ok(RxSource::Rfic),
        COUNTER => Ok(RxSource::Counter),
        LOOPBACK => Ok(RxSource::Loopback),
        _ => Err(Error::ValueConversion),
    }
}

/// Take RX samples from `source`.
pub(crate) async fn set_rx_source(nios: &NiosCore, source: RxSource) -> Result<()> {
    let value = match source {
        RxSource::Rfic => RFIC,
        RxSource::Counter => COUNTER,
        RxSource::Loopback => LOOPBACK,
    };
    let control = nios.read(Nios8x32::Control, 0).await?;
    let control = (control & !RX_SOURCE_MASK) | (value << RX_SOURCE_SHIFT);
    nios.write(Nios8x32::Control, 0, control).await
}

/// Control register bit that turns on stream metadata and runs the timestamp
/// counters (the FPGA's `meta_sync`).
const TIMESTAMPS: u32 = 1 << 16;
/// Control register bit for USB 2 DMA buffers (the FPGA's `usb_speed`).
const SMALL_DMA: u32 = 1 << 7;

/// Size of one metadata message (one FX3 DMA buffer): 2048 32-bit words, or
/// 1024 with USB 2 DMA buffers.
const MESSAGE_SIZE: usize = 8192;
const SMALL_MESSAGE_SIZE: usize = 4096;

/// Turn stream metadata and the RX and TX timestamp counters on or off. The
/// counters are held at 0 while off, and count from 0 once turned on.
pub(crate) async fn set_timestamps(nios: &NiosCore, on: bool) -> Result<()> {
    let control = nios.read(Nios8x32::Control, 0).await?;
    let control = if on {
        control | TIMESTAMPS
    } else {
        control & !TIMESTAMPS
    };
    nios.write(Nios8x32::Control, 0, control).await
}

/// Size of each stream message while timestamps are on, or `None` while they
/// are off.
pub(crate) async fn metadata_message_size(nios: &NiosCore) -> Result<Option<usize>> {
    let control = nios.read(Nios8x32::Control, 0).await?;
    Ok(message_size(control))
}

fn message_size(control: u32) -> Option<usize> {
    if control & TIMESTAMPS == 0 {
        None
    } else if control & SMALL_DMA == 0 {
        Some(MESSAGE_SIZE)
    } else {
        Some(SMALL_MESSAGE_SIZE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_size_follows_control_bits() {
        assert_eq!(message_size(0), None);
        assert_eq!(message_size(SMALL_DMA), None);
        assert_eq!(message_size(1 << 16), Some(8192));
        assert_eq!(message_size((1 << 16) | (1 << 7)), Some(4096));
    }
}
