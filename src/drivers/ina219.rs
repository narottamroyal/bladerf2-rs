use crate::Result;
use crate::nios::{Nios, Nios8x16};

#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Address {
    Config = 0,
    ShuntVoltage = 1,
    BusVoltage = 2,
    Power = 3,
    Current = 4,
    Calibration = 5,
}

impl From<Address> for u8 {
    fn from(addr: Address) -> Self {
        addr as u8
    }
}

// Value of the current sense resistor (R132)
const R_SHUNT: f32 = 1e-3;

pub fn init(nios: &Nios) -> Result<()> {
    // Soft-reset INA219
    const RESET: u16 = 0x8000;
    nios.write(Nios8x16::Ina219, Address::Config.into(), RESET)?;
    while nios.read(Nios8x16::Ina219, Address::Config.into())? & RESET != 0 {}

    // Initialize config register
    nios.write(Nios8x16::Ina219, Address::Config.into(), 0x019F)?;

    // Initialize calibration register
    let calibration = ((0.04096 / (0.001 * R_SHUNT)) + 0.5) as u16;
    nios.write(Nios8x16::Ina219, Address::Calibration.into(), calibration)?;

    Ok(())
}
