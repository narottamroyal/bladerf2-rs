//! ADF4002 PLL reference clock control.
//!
//! Locks the board's VCTCXO to an external reference on REFIN.
//!
//! The reference frequency is `VCTCXO × R / N`, so it is read back from the R
//! and N divider registers rather than stored. The dividers are integers, so
//! the value read back can differ slightly from the one requested.

use std::ops::RangeInclusive;

use crate::drivers::ad5621::Ad5621;
use crate::error::{Error, Result};
use crate::nios::{Nios8x32, NiosCore};

/// Reference frequency programmed at power-up, in Hz.
const DEFAULT_REFCLK_HZ: u64 = 10_000_000;
/// Reference frequencies the PLL accepts, in Hz.
pub(crate) const REFCLK_RANGE: RangeInclusive<u64> = 5_000_000..=300_000_000;

const VCTCXO_HZ: u64 = 38_400_000;
const PLL_ENABLE_BIT: u32 = 1 << 11;

/// Program the power-up reference frequency.
pub(crate) async fn init(nios: &NiosCore, dac: &Ad5621) -> Result<()> {
    set_refclk(nios, dac, DEFAULT_REFCLK_HZ).await
}

/// Whether the PLL is driving the reference clock.
pub(crate) async fn enabled(nios: &NiosCore) -> Result<bool> {
    Ok(nios.read(Nios8x32::Control, 0).await? & PLL_ENABLE_BIT != 0)
}

/// Enable or disable the PLL.
///
/// The VCTCXO DAC and the PLL both drive the reference, so the DAC is
/// released before the PLL is enabled and restored after it is disabled.
pub(crate) async fn set_enabled(nios: &NiosCore, dac: &Ad5621, enable: bool) -> Result<()> {
    if enable {
        dac.set_enabled(nios, false).await?;
        write_enable_bit(nios, true).await
    } else {
        write_enable_bit(nios, false).await?;
        dac.set_enabled(nios, true).await
    }
}

async fn write_enable_bit(nios: &NiosCore, enable: bool) -> Result<()> {
    let data = nios.read(Nios8x32::Control, 0).await?;
    let data = if enable {
        data | PLL_ENABLE_BIT
    } else {
        data & !PLL_ENABLE_BIT
    };
    nios.write(Nios8x32::Control, 0, data).await
}

/// Program the reference frequency the PLL locks the VCTCXO to.
///
/// The PLL is enabled while the dividers are written and restored afterwards.
pub(crate) async fn set_refclk(nios: &NiosCore, dac: &Ad5621, ref_hz: u64) -> Result<()> {
    let (r, n) = calculate_pll_ratio(ref_hz)?;

    let was_enabled = enabled(nios).await?;
    if !was_enabled {
        set_enabled(nios, dac, true).await?;
    }

    // The latch index is the NIOS address: 0 = R, 1 = N, 2 = function.
    for (address, data) in build_registers(r, n).into_iter().enumerate() {
        nios.write(Nios8x32::Adf400x, address as u8, data).await?;
    }

    if !was_enabled {
        set_enabled(nios, dac, false).await?;
    }
    Ok(())
}

/// Reference frequency the PLL is programmed for, in Hz, from the R and N
/// divider registers.
pub(crate) async fn refclk(nios: &NiosCore) -> Result<u64> {
    let r = u64::from((nios.read(Nios8x32::Adf400x, 0).await? >> 2) & 0x3FFF);
    let n = u64::from((nios.read(Nios8x32::Adf400x, 1).await? >> 8) & 0x1FFF);
    if n == 0 {
        return Err(Error::PllDividerRatio);
    }
    Ok(r * VCTCXO_HZ / n)
}

/// Calculate the PLL R and N dividers for the VCTCXO by brute-force search,
/// biased towards smaller dividers (first fit) for better PLL stability.
fn calculate_pll_ratio(ref_hz: u64) -> Result<(u16, u16)> {
    const R_MAX: u16 = 16383;
    const N_MAX: u16 = 8191;
    const TOLERANCE: f64 = 0.00001;

    if !REFCLK_RANGE.contains(&ref_hz) {
        return Err(Error::OutOfRange);
    }

    let target = VCTCXO_HZ as f64 / ref_hz as f64;

    for r in 1..=R_MAX {
        let n = (target * r as f64).round();
        if n > N_MAX as f64 {
            continue;
        }
        let n = n as u16;
        let ratio = n as f64 / r as f64;
        if (ratio - target).abs() < TOLERANCE {
            return Ok((r, n));
        }
    }

    Err(Error::PllDividerRatio)
}

fn build_registers(r: u16, n: u16) -> [u32; 3] {
    let mut regs = [0u32; 3];

    // Register 0: Reference Counter Latch (control bits 00, R counter 14 bits)
    regs[0] = ((r as u32) & 0x3FFF) << 2;

    // Register 1: N Counter Latch (control bits 01, N counter 13 bits)
    regs[1] = 1 | (((n as u32) & 0x1FFF) << 8);

    // Register 2: Function Latch (control bits 10)
    regs[2] = 2;
    regs[2] |= (0x1 & 0x7) << 4; // muxout: digital lock detect
    regs[2] |= 1 << 7; // PD polarity: positive
    regs[2] |= 0x7 << 15; // current setting 1: 5 mA
    regs[2] |= 0x7 << 18; // current setting 2: 5 mA

    regs
}
