use nusb::Interface;
use thiserror::Error;

use crate::usb::{AltSetting, UsbCommand};
use crate::{Result, usb};

pub fn read_vctcxo_trim(interface: &Interface) -> Result<u16> {
    let data = read_cal_data(interface)?;
    let decoder = BinKv::new(data.as_slice());
    for entry in decoder {
        match entry {
            Ok(data) => {
                if data.starts_with("DAC") && data.len() == 7 {
                    return Ok(data[3..7].parse::<u16>().unwrap());
                }
            }
            Err(err) => println!("{err}"),
        }
    }
    println!("Failed to read VCTCXO trim value. Using default value: 0x1FFC");
    Ok(0x1FFC)
}

fn read_cal_data(interface: &Interface) -> Result<Vec<u8>> {
    usb::change_setting(interface, AltSetting::SpiFlash)?;
    let data = usb::vendor_command(interface, UsbCommand::ReadCalCache, 256)?;
    usb::change_setting(interface, AltSetting::RfLink)?;
    Ok(data)
}

struct BinKv<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> BinKv<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
enum KvError {
    #[error("Invalid checksum")]
    Checksum,
    #[error("Data is truncated")]
    Truncated,
    #[error("String conversion failed")]
    InvalidUtf8,
}

impl<'a> Iterator for BinKv<'a> {
    type Item = std::result::Result<&'a str, KvError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.data.len() {
            return None;
        }

        let length = *self.data.get(self.offset)? as usize;
        // Flash/OTP are 0xFF by default
        if length == 0xFF {
            return None;
        }

        let start = self.offset + 1;
        let end = start + length;
        let crc_end = end + 2;

        if crc_end > self.data.len() {
            return Some(Err(KvError::Truncated));
        }

        // Check CRC
        let calculated_crc = checksum(&self.data[self.offset..end]);
        let stored_crc = u16::from_le_bytes([self.data[end], self.data[end + 1]]);
        if calculated_crc != stored_crc {
            return Some(Err(KvError::Checksum));
        }

        self.offset = crc_end;

        match std::str::from_utf8(&self.data[start..end]) {
            Ok(s) => Some(Ok(s)),
            Err(_) => Some(Err(KvError::InvalidUtf8)),
        }
    }
}

fn checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &byte| {
        let mut ret = acc ^ ((byte as u16) << 8);
        for _ in 0..8 {
            ret = if (ret & 0x8000) != 0 {
                (ret << 1) ^ 0x1021
            } else {
                ret <<= 1;
                ret
            };
        }
        ret
    })
}
