//! AD5621 DAC control for VCTCXO frequency trimming.
//!
//! The AD5621 is a 12-bit DAC controlling the trim voltage of the board's
//! 38.4 MHz VCTCXO. The trim value comes from flash calibration data read at
//! initialization.

use crate::error::Result;
use crate::nios::{Nios8x16, NiosCore};

/// DAC value that puts the output into high-impedance mode.
const HIGH_Z: u16 = 0xC000;

/// AD5621 DAC handle.
#[derive(Debug, Clone)]
pub struct Ad5621 {
    dac_trim: u16,
}

impl Ad5621 {
    /// Create a handle holding the calibration trim value.
    pub(crate) fn new(dac_trim: u16) -> Self {
        Self { dac_trim }
    }

    /// The calibration trim value applied to the VCTCXO.
    pub fn trim_value(&self) -> u16 {
        self.dac_trim
    }

    /// Apply the stored calibration trim value (re-enables the DAC output).
    pub(crate) async fn enable(&self, nios: &NiosCore) -> Result<()> {
        let value = nios.read(Nios8x16::Ad56x1Dac, 0).await?;
        if value != self.dac_trim {
            nios.write(Nios8x16::Ad56x1Dac, 0, self.dac_trim).await?;
        }
        Ok(())
    }

    /// Put the DAC output into high-impedance mode.
    pub(crate) async fn disable(&self, nios: &NiosCore) -> Result<()> {
        let value = nios.read(Nios8x16::Ad56x1Dac, 0).await?;
        if value != HIGH_Z {
            nios.write(Nios8x16::Ad56x1Dac, 0, HIGH_Z).await?;
        }
        Ok(())
    }
}
