//! Access to calibration data stored on the device.
//!
//! The calibration region is read through the FX3 calibration cache via a
//! vendor command, switching to the SPI-flash alternate setting and back.

use crate::error::Result;
use crate::usb::{AltSetting, UsbCommand, UsbTransport};

/// Read the VCTCXO trim value from the calibration cache.
///
/// Parses the length-prefixed, CRC-protected key-value strings and returns
/// the `DACxxxx` entry. Falls back to `0x1FFC` if parsing fails.
pub(crate) async fn read_vctcxo_trim(transport: &UsbTransport) -> Result<u16> {
    let data = read_cal_data(transport).await?;
    let decoder = BinKv::new(data.as_slice());
    for entry in decoder {
        match entry {
            Ok(data) => {
                if data.starts_with("DAC")
                    && data.len() == 7
                    && let Ok(trim) = data[3..7].parse::<u16>()
                {
                    return Ok(trim);
                }
            }
            Err(err) => tracing::warn!("invalid calibration entry: {err}"),
        }
    }
    tracing::warn!("failed to read VCTCXO trim value; using default 0x1FFC");
    Ok(0x1FFC)
}

async fn read_cal_data(transport: &UsbTransport) -> Result<Vec<u8>> {
    transport.change_setting(AltSetting::SpiFlash).await?;
    let data = transport
        .vendor_command(UsbCommand::ReadCalCache, 0, 256)
        .await?;
    transport.change_setting(AltSetting::RfLink).await?;
    Ok(data)
}

/// Iterator over the calibration region's length-prefixed, CRC-16-CCITT
/// protected UTF-8 strings.
struct BinKv<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> BinKv<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }
}

impl<'a> Iterator for BinKv<'a> {
    type Item = std::result::Result<&'a str, &'static str>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.data.len() {
            return None;
        }

        let length = *self.data.get(self.offset)? as usize;
        // Flash/OTP default to 0xFF (empty)
        if length == 0xFF {
            return None;
        }

        let start = self.offset + 1;
        let end = start + length;
        let crc_end = end + 2;

        // The entries after a bad one cannot be located, so stop there.
        if crc_end > self.data.len() {
            self.offset = self.data.len();
            return Some(Err("calibration data is truncated"));
        }
        let calculated_crc = checksum(&self.data[self.offset..end]);
        let stored_crc = u16::from_le_bytes([self.data[end], self.data[end + 1]]);
        if calculated_crc != stored_crc {
            self.offset = self.data.len();
            return Some(Err("calibration entry checksum mismatch"));
        }

        self.offset = crc_end;

        match std::str::from_utf8(&self.data[start..end]) {
            Ok(s) => Some(Ok(s)),
            Err(_) => Some(Err("calibration entry is not valid UTF-8")),
        }
    }
}

/// CRC-16-CCITT (poly 0x1021) as used by the bladeRF calibration format.
fn checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &byte| {
        let mut ret = acc ^ ((byte as u16) << 8);
        for _ in 0..8 {
            ret = if (ret & 0x8000) != 0 {
                (ret << 1) ^ 0x1021
            } else {
                ret << 1
            };
        }
        ret
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A calibration entry: length, contents, then CRC.
    fn entry(contents: &str) -> Vec<u8> {
        let mut entry = vec![contents.len() as u8];
        entry.extend_from_slice(contents.as_bytes());
        let crc = checksum(&entry);
        entry.extend_from_slice(&crc.to_le_bytes());
        entry
    }

    #[test]
    fn reads_entries_until_empty_flash() {
        let mut data = entry("B2A4");
        data.extend(entry("DAC8152"));
        data.push(0xFF);
        let entries: Vec<_> = BinKv::new(&data).collect();
        assert_eq!(entries, [Ok("B2A4"), Ok("DAC8152")]);
    }

    #[test]
    fn stops_after_a_checksum_mismatch() {
        let mut data = entry("DAC8152");
        *data.last_mut().unwrap() ^= 0xFF;
        data.extend(entry("B2A4"));
        let entries: Vec<_> = BinKv::new(&data).collect();
        assert_eq!(entries, [Err("calibration entry checksum mismatch")]);
    }

    #[test]
    fn stops_at_truncated_data() {
        let data = entry("DAC8152");
        let entries: Vec<_> = BinKv::new(&data[..data.len() - 1]).collect();
        assert_eq!(entries, [Err("calibration data is truncated")]);
    }
}
