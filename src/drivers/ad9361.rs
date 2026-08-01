use crate::Result;
use crate::nios::{Nios, Nios16x64};

const AD_WRITE_BIT: u16 = 1 << 15;
const AD_ADDR_MASK: u16 = 0x03FF;

#[repr(u16)]
enum Register {
    CalibrationConfig1 = 0x169,
    DcOffsetConfig2 = 0x18B,
}

// Calibration Config 1 (0x169) bits
const ENABLE_PHASE_CORR: u8 = 1 << 7; // D7
const ENABLE_GAIN_CORR: u8 = 1 << 6; // D6
const FREE_RUN_MODE: u8 = 1 << 3; // D3
const ENABLE_CORR_WORD_DECIMATION: u8 = 1 << 2; // D2
#[allow(unused)]
const ENABLE_TRACKING_MODE_CH1: u8 = 1 << 0; // D0

// DC Offset Config 2 (0x18B) bits
const DC_OFFSET_CONFIG2_MUST_BE_1: u8 = 1 << 7; // D7: must be 1
#[allow(unused)]
const ENABLE_BB_DC_OFFSET_TRACKING: u8 = 1 << 5; // D5
#[allow(unused)]
const ENABLE_RF_OFFSET_TRACKING: u8 = 1 << 3; // D3

fn spi_write(nios: &Nios, register: Register, value: u8) -> Result<()> {
    let address = AD_WRITE_BIT | (register as u16 & AD_ADDR_MASK);
    let data = (value as u64) << 56;
    nios.write(Nios16x64::Ad9361, address, data)
}

pub fn disable_tracking_control(nios: &Nios) -> Result<()> {
    spi_write(nios, Register::DcOffsetConfig2, DC_OFFSET_CONFIG2_MUST_BE_1)?;
    spi_write(
        nios,
        Register::CalibrationConfig1,
        ENABLE_PHASE_CORR | ENABLE_GAIN_CORR | FREE_RUN_MODE | ENABLE_CORR_WORD_DECIMATION,
    )?;
    Ok(())
}
