use std::{thread::sleep, time::Duration};

use nusb::Interface;

use crate::{
    BladeRf2Error, Nios, Result,
    drivers::ad5621::Ad5621,
    usb::{AltSetting, UsbCommand, change_setting, vendor_command},
};

pub mod ad5621;
pub mod adf4002;
pub mod flash;
pub mod ina219;
pub mod rffe;
pub mod rfic;

pub fn init(interface: &Interface) -> Result<(Nios, Ad5621)> {
    wait_for_device_ready(interface)?;
    let dac_trim = flash::read_vctcxo_trim(interface)?;
    let dac = Ad5621::new(dac_trim);
    let nios = Nios::new(interface)?;
    ina219::init(&nios)?;
    rfic::init(&nios)?;
    adf4002::init(&nios, &dac)?;
    Ok((nios, dac))
}

fn wait_for_device_ready(interface: &Interface) -> Result<()> {
    for i in 0..30 {
        let data = vendor_command(interface, UsbCommand::QueryDeviceReady, 1)?;
        if data[0] == 1 {
            change_setting(interface, AltSetting::RfLink)?;
            return Ok(());
        }
        println!("Waiting for device to become ready [attempt {}]", i + 1);
        sleep(Duration::from_secs(1));
    }
    Err(BladeRf2Error::Timeout)
}
