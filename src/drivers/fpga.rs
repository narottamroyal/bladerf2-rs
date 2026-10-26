//! FPGA control register: RX sample source.

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
