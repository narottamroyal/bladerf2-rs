//! RF front-end control (bias tee).

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

/// Enable the bias tee for a direction (supplies DC on the antenna port).
pub async fn enable_bias_tee(nios: &NiosCore, direction: Direction) -> Result<()> {
    let mut reg = nios.read(Nios8x32::RfFrontend, 0).await?;
    match direction {
        Direction::Rx => reg |= 1 << Control::RxBiasEn as u32,
        Direction::Tx => reg |= 1 << Control::TxBiasEn as u32,
    };
    nios.write(Nios8x32::RfFrontend, 0, reg).await
}

/// Disable the bias tee for a direction.
pub async fn disable_bias_tee(nios: &NiosCore, direction: Direction) -> Result<()> {
    let mut reg = nios.read(Nios8x32::RfFrontend, 0).await?;
    match direction {
        Direction::Rx => reg &= !(1 << Control::RxBiasEn as u32),
        Direction::Tx => reg &= !(1 << Control::TxBiasEn as u32),
    };
    nios.write(Nios8x32::RfFrontend, 0, reg).await
}
