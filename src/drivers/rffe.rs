//! RF front-end control: bias tee and PLL lock detect.

use crate::Direction;
use crate::error::Result;
use crate::nios::{Nios8x32, NiosCore};

/// Bit positions in the RfFrontend register.
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Control {
    ResetN = 0,
    Enable = 1,
    TxnRx = 2,
    EnAgc = 3,
    SyncIn = 4,
    RxBiasEn = 5,
    RxSpdt1 = 6, // occupies bits 6 & 7
    RxSpdt2 = 8, // occupies bits 8 & 9
    TxBiasEn = 10,
    TxSpdt1 = 11, // occupies bits 11 & 12
    TxSpdt2 = 13, // occupies bits 13 & 14
    MimoRxEn0 = 15,
    MimoTxEn0 = 16,
    MimoRxEn1 = 17,
    MimoTxEn1 = 18,
    AdfMuxout = 19, // input only
    CtrlOut = 24,   // input only, bits 24-31
}

/// Bit of the bias-tee enable for a direction.
fn bias_tee_bit(direction: Direction) -> u32 {
    match direction {
        Direction::Rx => Control::RxBiasEn as u32,
        Direction::Tx => Control::TxBiasEn as u32,
    }
}

/// Whether the bias tee is supplying DC on a direction's antenna port.
pub async fn bias_tee(nios: &NiosCore, direction: Direction) -> Result<bool> {
    let reg = nios.read(Nios8x32::RfFrontend, 0).await?;
    Ok(reg & (1 << bias_tee_bit(direction)) != 0)
}

/// Turn the bias tee on or off for a direction.
pub async fn set_bias_tee(nios: &NiosCore, direction: Direction, enable: bool) -> Result<()> {
    let mut reg = nios.read(Nios8x32::RfFrontend, 0).await?;
    if enable {
        reg |= 1 << bias_tee_bit(direction);
    } else {
        reg &= !(1 << bias_tee_bit(direction));
    }
    nios.write(Nios8x32::RfFrontend, 0, reg).await
}

/// PLL lock detect, from the ADF4002 MUX-out pin.
pub async fn pll_lock_state(nios: &NiosCore) -> Result<bool> {
    let reg = nios.read(Nios8x32::RfFrontend, 0).await?;
    Ok(reg & (1 << Control::AdfMuxout as u32) != 0)
}
