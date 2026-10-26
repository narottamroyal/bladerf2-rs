//! FPGA control register: RX sample source (loopback).

use crate::error::Result;
use crate::nios::{Nios8x32, NiosCore};

/// RX sample source bits, and their value for TX-to-RX loopback.
const RX_MUX_MASK: u32 = 0x7 << 8;
const RX_MUX_LOOPBACK: u32 = 4 << 8;

/// Whether the FPGA loops TX samples back to RX.
pub(crate) async fn loopback(nios: &NiosCore) -> Result<bool> {
    let control = nios.read(Nios8x32::Control, 0).await?;
    Ok(control & RX_MUX_MASK == RX_MUX_LOOPBACK)
}

/// Turn FPGA loopback on or off. Returns whether it was on.
pub(crate) async fn set_loopback(nios: &NiosCore, enable: bool) -> Result<bool> {
    let control = nios.read(Nios8x32::Control, 0).await?;
    let mut new = control & !RX_MUX_MASK;
    if enable {
        new |= RX_MUX_LOOPBACK;
    }
    nios.write(Nios8x32::Control, 0, new).await?;
    Ok(control & RX_MUX_MASK == RX_MUX_LOOPBACK)
}
