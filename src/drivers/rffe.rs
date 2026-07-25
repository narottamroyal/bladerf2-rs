use crate::{
    Direction, Result,
    nios::{Nios, Nios8x32},
};

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
    RxSpdt1 = 6, // Occupies bits 6 & 7
    RxSpdt2 = 8, // Occupies bits 8 & 9
    TxBiasEn = 10,
    TxSpdt1 = 11, // Occupies bits 11 & 12
    TxSpdt2 = 13, // Occupies bits 13 & 14
    MimoRxEn0 = 15,
    MimoTxEn0 = 16,
    MimoRxEn1 = 17,
    MimoTxEn1 = 18,
    AdfMuxout = 19, // Input only
    CtrlOut = 24,   // Input only, 24-31
}

pub fn enable_bias_tee(nios: &Nios, direction: Direction) -> Result<()> {
    let mut reg = nios.read(Nios8x32::RfFrontend, 0)?;
    match direction {
        Direction::Rx => reg |= 1 << Control::RxBiasEn as u32,
        Direction::Tx => reg |= 1 << Control::TxBiasEn as u32,
    };
    nios.write(Nios8x32::RfFrontend, 0, reg)
}

pub fn disable_bias_tee(nios: &Nios, direction: Direction) -> Result<()> {
    let mut reg = nios.read(Nios8x32::RfFrontend, 0)?;
    match direction {
        Direction::Rx => reg &= !(1 << Control::RxBiasEn as u32),
        Direction::Tx => reg &= !(1 << Control::TxBiasEn as u32),
    };
    nios.write(Nios8x32::RfFrontend, 0, reg)
}
