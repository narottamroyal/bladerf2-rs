//! ADF4002 PLL reference clock control.
//!
//! Generates the AD9361 reference clock from the board's VCTCXO. Register
//! encoding ported from libbladeRF's `adf400x.c`.

use crate::drivers::ad5621::Ad5621;
use crate::error::{Error, Result};
use crate::nios::{Nios8x32, NiosCore};

const DEFAULT_REFERENCE_HZ: u64 = 10_000_000;
const VCTCXO_HZ: u64 = 38_400_000;
const PLL_ENABLE_BIT: u32 = 1 << 11;

/// Initialize the PLL with the default 10 MHz reference.
pub async fn init(nios: &NiosCore, dac: &Ad5621) -> Result<()> {
    configure_pll_reference(nios, dac, DEFAULT_REFERENCE_HZ).await
}

/// Power up the PLL (DAC output is tristated while the PLL drives the ref).
pub async fn enable(nios: &NiosCore, dac: &Ad5621) -> Result<()> {
    dac.disable(nios).await?;
    let mut data = nios.read(Nios8x32::Control, 0).await?;
    if data & PLL_ENABLE_BIT == 0 {
        data |= PLL_ENABLE_BIT;
        nios.write(Nios8x32::Control, 0, data).await?;
    }
    Ok(())
}

/// Power down the PLL and re-enable the DAC.
pub async fn disable(nios: &NiosCore, dac: &Ad5621) -> Result<()> {
    let mut data = nios.read(Nios8x32::Control, 0).await?;
    if data & PLL_ENABLE_BIT != 0 {
        data &= !PLL_ENABLE_BIT;
        nios.write(Nios8x32::Control, 0, data).await?;
    }
    dac.enable(nios).await
}

/// Program the PLL reference divider for an external reference frequency.
pub async fn configure_pll_reference(nios: &NiosCore, dac: &Ad5621, ref_freq: u64) -> Result<()> {
    let (r, n) = calculate_pll_ratio(ref_freq, VCTCXO_HZ)?;

    let enabled = nios.read(Nios8x32::Control, 0).await? & PLL_ENABLE_BIT != 0;
    if !enabled {
        enable(nios, dac).await?;
    }

    for data in build_registers(r, n) {
        nios.write(Nios8x32::Adf400x, 0, data).await?;
    }

    if !enabled {
        disable(nios, dac).await?;
    }
    Ok(())
}

/// Calculate the PLL R and N dividers by brute-force search, biased towards
/// smaller dividers (first fit) for better PLL stability.
fn calculate_pll_ratio(ref_freq: u64, clock_freq: u64) -> Result<(u16, u16)> {
    const R_MAX: u16 = 16383;
    const N_MAX: u16 = 8191;
    const TOLERANCE: f64 = 0.00001;

    if !(5_000_000..=300_000_000).contains(&ref_freq) {
        return Err(Error::OutOfRange);
    }
    if !(5_000_000..=400_000_000).contains(&clock_freq) {
        return Err(Error::OutOfRange);
    }

    let target = clock_freq as f64 / ref_freq as f64;

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
