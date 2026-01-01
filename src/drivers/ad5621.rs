use crate::Result;
use crate::nios::{Nios, Nios8x16};

pub struct Ad5621 {
    dac_trim: u16,
}

impl Ad5621 {
    pub fn new(dac_trim: u16) -> Self {
        Self { dac_trim }
    }

    pub fn enable(&self, nios: &mut Nios) -> Result<()> {
        let value = nios.read(Nios8x16::Ad56x1Dac, 0)?;
        if value != self.dac_trim {
            nios.write(Nios8x16::Ad56x1Dac, 0, self.dac_trim)?;
        }
        Ok(())
    }

    pub fn disable(&self, nios: &mut Nios) -> Result<()> {
        const HIGH_Z: u16 = 0xC000;
        let value = nios.read(Nios8x16::Ad56x1Dac, 0)?;
        if value != HIGH_Z {
            nios.write(Nios8x16::Ad56x1Dac, 0, HIGH_Z)?;
        }
        Ok(())
    }
}
