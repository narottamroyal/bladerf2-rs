//! INA219 power monitor driver (main power rail diagnostics).

use crate::error::{Error, Result};
use crate::maybe_future;
use crate::nios::{Nios8x16, NiosCore};

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

/// Value of the current sense resistor (R132).
const R_SHUNT: f32 = 1e-3;

/// Soft-reset configuration value.
const RESET: u16 = 0x8000;

/// Initialize the INA219: soft-reset, configure, and program the calibration
/// register (required for current/power conversion).
pub async fn init(nios: &NiosCore) -> Result<()> {
    // Soft-reset the INA219
    nios.write(Nios8x16::Ina219, Address::Config.into(), RESET)
        .await?;
    for _ in 0..1000 {
        if nios.read(Nios8x16::Ina219, Address::Config.into()).await? & RESET == 0 {
            break;
        }
        maybe_future::sleep(std::time::Duration::from_millis(1)).await;
    }

    // Verify the reset completed; continuing would leave the calibration
    // write below landing on an unresponsive chip.
    if nios.read(Nios8x16::Ina219, Address::Config.into()).await? & RESET != 0 {
        tracing::warn!("INA219 did not complete soft reset");
        return Err(Error::Timeout);
    }

    // Initialize the config register
    nios.write(Nios8x16::Ina219, Address::Config.into(), 0x019F)
        .await?;

    // Initialize the calibration register (required for current/power)
    let calibration = ((0.04096 / (0.001 * R_SHUNT)) + 0.5) as u16;
    nios.write(Nios8x16::Ina219, Address::Calibration.into(), calibration)
        .await?;

    Ok(())
}
