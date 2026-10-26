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
pub(crate) struct Ad5621 {
    dac_trim: u16,
}

impl Ad5621 {
    /// Create a handle holding the calibration trim value.
    pub(crate) fn new(dac_trim: u16) -> Self {
        Self { dac_trim }
    }

    /// The calibration trim value applied to the VCTCXO.
    pub(crate) fn trim_value(&self) -> u16 {
        self.dac_trim
    }

    /// Enable the DAC (apply the calibration trim value), or put its output
    /// into high-impedance mode.
    pub(crate) async fn set_enabled(&self, nios: &NiosCore, enable: bool) -> Result<()> {
        let target = if enable { self.dac_trim } else { HIGH_Z };
        let value = nios.read(Nios8x16::Ad56x1Dac, 0).await?;
        if value != target {
            nios.write(Nios8x16::Ad56x1Dac, 0, target).await?;
        }
        Ok(())
    }
}
