use std::time::Duration;

use crate::{BladeRf2Error, Direction, Result};
use nusb::{
    Interface, MaybeFuture,
    transfer::{ControlIn, ControlType, Recipient},
};

#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum AltSetting {
    Null = 0,
    RfLink = 1,
    SpiFlash = 2,
    Config = 3,
}

impl From<AltSetting> for u8 {
    fn from(value: AltSetting) -> Self {
        value as u8
    }
}

#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum Endpoint {
    SampleIn = 0x81,
    SampleOut = 0x01,
    PeripheralIn = 0x82,
    PeripheralOut = 0x02,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbCommand {
    QueryVersion = 0,
    QueryFpgaStatus = 1,
    BeginProg = 2,
    EndProg = 3,
    RfRx = 4,
    RfTx = 5,
    QueryDeviceReady = 6,
    QueryFlashId = 7,
    QueryFpgaSource = 8,
    FlashRead = 100,
    FlashWrite = 101,
    FlashErase = 102,
    ReadOtp = 103,
    WriteOtp = 104,
    Reset = 105,
    JumpToBootloader = 106,
    ReadPageBuffer = 107,
    WritePageBuffer = 108,
    LockOtp = 109,
    ReadCalCache = 110,
    InvalidateCalCache = 111,
    RefreshCalCache = 112,
    SetLoopback = 113,
    GetLoopback = 114,
    ReadLogEntry = 115,
}

impl From<UsbCommand> for u8 {
    fn from(value: UsbCommand) -> Self {
        value as u8
    }
}

pub fn vendor_command(interface: &Interface, command: UsbCommand, length: u16) -> Result<Vec<u8>> {
    let data = interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: command.into(),
                value: 0,
                index: 0,
                length,
            },
            Duration::from_secs(1),
        )
        .wait()?;
    Ok(data)
}

fn vendor_command_enable(interface: &Interface, command: UsbCommand, enable: bool) -> Result<()> {
    let data = interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: command.into(),
                value: enable.into(),
                index: 0,
                length: core::mem::size_of::<u32>() as u16,
            },
            Duration::from_secs(1),
        )
        .wait()?
        .try_into()
        .map_err(|_| BladeRf2Error::ValueConversion)?;

    if u32::from_le_bytes(data) == 0 {
        Ok(())
    } else {
        Err(BladeRf2Error::UsbVendor)
    }
}

pub fn change_setting(interface: &Interface, setting: AltSetting) -> Result<()> {
    interface.set_alt_setting(setting.into()).wait()?;
    Ok(())
}

pub fn enable_module(interface: &Interface, direction: Direction, enable: bool) -> Result<()> {
    let command = match direction {
        Direction::Rx => UsbCommand::RfRx,
        Direction::Tx => UsbCommand::RfTx,
    };
    vendor_command_enable(interface, command, enable)
}

// pub fn get_loopback(interface: &Interface) -> Result<bool> {
//     let data = vendor_command(interface, UsbCommand::GetLoopback, 1)?;
//     Ok(data[0] != 0)
// }

// pub fn set_loopback(interface: &Interface, enable: bool) -> Result<()> {
//     let _ = vendor_command_enable(interface, UsbCommand::SetLoopback, enable);
//     change_setting(interface, AltSetting::Null)?;
//     change_setting(interface, AltSetting::RfLink)
// }
