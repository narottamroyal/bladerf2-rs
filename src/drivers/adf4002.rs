use crate::{
    BladeRf2Error, Result,
    drivers::ad5621::Ad5621,
    nios::{Nios, Nios8x32},
};

const DEFAULT_REFERENCE_HZ: u64 = 10_000_000;
const VCTCXO_HZ: u64 = 38_400_000;
const PLL_ENABLE_BIT: u32 = 1 << 11;

pub fn init(nios: &Nios, dac: &Ad5621) -> Result<()> {
    configure_pll_reference(nios, dac, DEFAULT_REFERENCE_HZ)
}

pub fn enable(nios: &Nios, dac: &Ad5621) -> Result<()> {
    dac.disable(nios)?;
    let mut data = nios.read(Nios8x32::Control, 0)?;
    if data & PLL_ENABLE_BIT == 0 {
        data |= PLL_ENABLE_BIT;
        nios.write(Nios8x32::Control, 0, data)?;
    }
    Ok(())
}

pub fn disable(nios: &Nios, dac: &Ad5621) -> Result<()> {
    let mut data = nios.read(Nios8x32::Control, 0)?;
    if data & PLL_ENABLE_BIT != 0 {
        data &= !PLL_ENABLE_BIT;
        nios.write(Nios8x32::Control, 0, data)?;
    }
    dac.enable(nios)
}

pub fn configure_pll_reference(nios: &Nios, dac: &Ad5621, ref_freq: u64) -> Result<()> {
    let (r, n) = calculate_pll_ratio(ref_freq, VCTCXO_HZ)?;

    let enabled = nios.read(Nios8x32::Control, 0)? & PLL_ENABLE_BIT != 0;
    if !enabled {
        enable(nios, dac)?;
    }

    for data in build_registers(r, n) {
        nios.write(Nios8x32::Adf400x, 0, data)?;
    }

    if !enabled {
        disable(nios, dac)?;
    }
    Ok(())
}

/// Calculates the PLL R and N dividers using a brute-force search.
/// Biased towards smaller dividers (first-fit) to improve PLL stability.
fn calculate_pll_ratio(ref_freq: u64, clock_freq: u64) -> Result<(u16, u16)> {
    const R_MAX: u16 = 16383;
    const N_MAX: u16 = 8191;
    const TOLERANCE: f64 = 0.00001;

    if !(5_000_000..=300_000_000).contains(&ref_freq) {
        return Err(BladeRf2Error::OutOfRange);
    }
    if !(5_000_000..=400_000_000).contains(&clock_freq) {
        return Err(BladeRf2Error::OutOfRange);
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

    Err(BladeRf2Error::PllDividerRatio)
}

fn build_registers(r: u16, n: u16) -> [u32; 3] {
    let mut regs = [0u32; 3];

    // --- Register 0: Reference Counter Latch ---
    // bits 0-1: Control Bits (00)
    // bits 2-15: R Counter (14 bits)
    regs[0] = 0;
    regs[0] |= ((r as u32) & 0x3FFF) << 2;

    // --- Register 1: N Counter Latch ---
    // bits 0-1: Control Bits (01)
    // bits 8-20: N Counter (13 bits)
    regs[1] = 1;
    regs[1] |= ((n as u32) & 0x1FFF) << 8;

    // --- Register 2: Function Latch ---
    // bits 0-1: Control Bits (10)
    regs[2] = 2;
    // Muxout control: 0x1 (digital lock detect) at bits 4-6
    regs[2] |= (0x1 & 0x7) << 4;
    // PD Polarity: 1 (positive) at bit 7
    regs[2] |= 1 << 7;
    // Current Setting 1: 0x7 (5 mA) at bits 15-17
    regs[2] |= 0x7 << 15;
    // Current Setting 2: 0x7 (5 mA) at bits 18-20
    regs[2] |= 0x7 << 18;

    regs
}
