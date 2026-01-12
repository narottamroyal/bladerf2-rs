use nusb::{DeviceInfo, Interface, MaybeFuture, Speed, list_devices, transfer::TransferError};
use std::fmt::Display;
use thiserror::Error;

use crate::{
    drivers::{
        ad5621::Ad5621,
        adf4002, rffe,
        rfic::{self, GainMode},
    },
    nios::{Nios, Nios8x32, Nios8x64},
    stream::{RxStream, TxStream},
};

mod drivers;
mod nios;
mod stream;
mod usb;

#[derive(Debug, Error)]
pub enum BladeRf2Error {
    #[error("No device found")]
    NoDeviceFound,
    #[error("Invalid USB speed")]
    UsbSpeed,
    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Transfer error: {0}")]
    Transfer(#[from] TransferError),
    #[error("Firmware version not supported")]
    IncompatibleFirmware,
    #[error("Nios transfer unsuccessful")]
    Nios,
    #[error("Unable to convert value")]
    ValueConversion,
    #[error("Timeout")]
    Timeout,
    #[error("Out of range")]
    OutOfRange,
    #[error("PLL divider solution not found")]
    PllDividerRatio,
    #[error("USB vendor command enable failed")]
    UsbVendor,
    #[error("Invalid channel configuration")]
    ChannelConfig,
}

pub type Result<T> = core::result::Result<T, BladeRf2Error>;

pub struct BladeRf2 {
    _info: DeviceInfo,
    interface: Interface,
    nios: Nios,
    dac: Ad5621,
}

impl BladeRf2 {
    const VID: u16 = 0x2CF0;
    const PID: u16 = 0x5250;

    pub fn enumerate() -> Result<Vec<DeviceInfo>> {
        let devices = list_devices()
            .wait()?
            .filter(|dev| {
                dev.vendor_id() == Self::VID
                    && dev.product_id() == Self::PID
                    && dev.product_string() == Some("bladeRF 2.0")
            })
            .collect();
        Ok(devices)
    }

    pub fn open_any() -> Result<Self> {
        let devices = Self::enumerate()?;
        let info = devices.first().ok_or(BladeRf2Error::NoDeviceFound)?;
        Self::open(info.clone())
    }

    pub fn open_serial(serial: &str) -> Result<Self> {
        let devices = Self::enumerate()?;
        let info = devices
            .iter()
            .find(|dev| {
                if let Some(s) = dev.serial_number() {
                    s.starts_with(serial)
                } else {
                    false
                }
            })
            .ok_or(BladeRf2Error::NoDeviceFound)?;
        Self::open(info.clone())
    }

    pub fn open(info: DeviceInfo) -> Result<Self> {
        let device = info.open().wait()?;

        // Check device speed
        if let Some(speed) = device.speed()
            && speed != Speed::Super
        {
            return Err(BladeRf2Error::UsbSpeed);
        }

        let interface = device.claim_interface(0).wait()?;
        if interface.descriptors().count() != 4 {
            return Err(BladeRf2Error::IncompatibleFirmware);
        }

        let (nios, dac) = drivers::init(&interface)?;

        Ok(Self {
            _info: info,
            interface,
            nios,
            dac,
        })
    }

    pub fn bandwidth(&mut self, direction: Direction) -> Result<u64> {
        rfic::get_bandwidth(&mut self.nios, direction)
    }

    pub fn set_bandwidth(&mut self, direction: Direction, bandwidth: u64) -> Result<()> {
        rfic::set_bandwidth(&mut self.nios, direction, bandwidth)
    }

    pub fn frequency(&mut self, direction: Direction) -> Result<u64> {
        rfic::get_frequency(&mut self.nios, direction)
    }

    pub fn set_frequency(&mut self, direction: Direction, frequency: u64) -> Result<()> {
        rfic::set_frequency(&mut self.nios, direction, frequency)
    }

    pub fn gain(&mut self, channel: Channel) -> Result<f64> {
        rfic::get_gain(&mut self.nios, channel)
    }
    pub fn set_gain(&mut self, channel: Channel, gain: f64) -> Result<()> {
        rfic::set_gain(&mut self.nios, channel, gain)
    }

    pub fn gain_mode(&mut self, channel: Channel) -> Result<GainMode> {
        rfic::get_gain_mode(&mut self.nios, channel)
    }

    pub fn set_gain_mode(&mut self, channel: Channel, mode: GainMode) -> Result<()> {
        rfic::set_gain_mode(&mut self.nios, channel, mode)
    }

    pub fn sample_rate(&mut self) -> Result<u64> {
        rfic::get_sample_rate(&mut self.nios)
    }

    pub fn set_sample_rate(&mut self, sample_rate: u64) -> Result<()> {
        rfic::set_sample_rate(&mut self.nios, sample_rate)
    }

    pub fn set_pll_reference(&mut self, ref_freq: u64) -> Result<()> {
        adf4002::configure_pll_reference(&mut self.nios, &self.dac, ref_freq)
    }

    pub fn set_pll(&mut self, enable: bool) -> Result<()> {
        if enable {
            adf4002::enable(&mut self.nios, &self.dac)
        } else {
            adf4002::disable(&mut self.nios, &self.dac)
        }
    }

    pub fn set_bias_tee(&mut self, direction: Direction, enable: bool) -> Result<()> {
        if enable {
            rffe::enable_bias_tee(&mut self.nios, direction)
        } else {
            rffe::disable_bias_tee(&mut self.nios, direction)
        }
    }

    pub fn fpga_version(&mut self) -> Result<Version> {
        let version = self.nios.read(Nios8x32::Version, 0)?;
        Ok(version.into())
    }

    pub fn timestamp(&mut self) -> Result<u64> {
        self.nios.read(Nios8x64::Timestamp, 0)
    }

    pub fn loopback(&mut self) -> Result<bool> {
        const MASK: u32 = 0x7 << 8;
        const LOOPBACK: u32 = 4 << 8;

        let mut gpio = self.nios.read(Nios8x32::Control, 0)?;
        gpio &= MASK;
        Ok(gpio == LOOPBACK)
    }

    pub fn set_loopback(&mut self, enable: bool) -> Result<()> {
        const MASK: u32 = 0x7 << 8;
        const LOOPBACK: u32 = 4 << 8;

        let mut gpio = self.nios.read(Nios8x32::Control, 0)?;
        gpio &= !MASK;
        if enable {
            gpio |= LOOPBACK;
        }
        self.nios.write(Nios8x32::Control, 0, gpio)
    }

    pub fn rx_stream(&mut self, channels: &[usize]) -> Result<RxStream> {
        RxStream::new(&self.interface, &self.nios, channels)
    }

    pub fn tx_stream(&mut self, channels: &[usize]) -> Result<TxStream> {
        TxStream::new(&self.interface, &self.nios, channels)
    }
}

#[derive(Debug, PartialEq)]
pub struct Version {
    major: u8,
    minor: u8,
    patch: u16,
}

impl Display for Version {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

impl From<u32> for Version {
    fn from(value: u32) -> Self {
        let bytes = value.to_le_bytes();
        Self {
            major: bytes[0],
            minor: bytes[1],
            patch: u16::from_le_bytes(bytes[2..=3].try_into().unwrap()),
        }
    }
}

#[repr(u16)]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Rx0 = 0,
    Tx0 = 1,
    Rx1 = 2,
    Tx1 = 3,
}

impl From<Channel> for u16 {
    fn from(value: Channel) -> Self {
        value as u16
    }
}

impl From<Channel> for Direction {
    fn from(value: Channel) -> Self {
        match value {
            Channel::Rx0 | Channel::Rx1 => Direction::Rx,
            Channel::Tx0 | Channel::Tx1 => Direction::Tx,
        }
    }
}

#[repr(u16)]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Rx = 0,
    Tx = 1,
}

impl From<Direction> for u16 {
    fn from(value: Direction) -> Self {
        match value {
            Direction::Rx => Channel::Rx0.into(),
            Direction::Tx => Channel::Tx0.into(),
        }
    }
}

#[cfg(test)]
#[serial_test::serial]
mod test {
    use super::*;
    use num_complex::Complex32;

    #[test]
    fn test_enumerate() {
        let devices = BladeRf2::enumerate().unwrap();
        assert!(devices.len() > 0)
    }

    #[test]
    fn test_version() {
        let mut bladerf = BladeRf2::open_any().unwrap();
        let version = bladerf.fpga_version().unwrap();
        assert_eq!(
            version,
            Version {
                major: 0,
                minor: 15,
                patch: 3
            }
        );
    }

    #[test]
    fn test_bandwidth() {
        const BW1: u64 = 1_000_000;
        const BW2: u64 = 40_000_000;
        let mut bladerf = BladeRf2::open_any().unwrap();

        bladerf.set_bandwidth(Direction::Rx, BW1).unwrap();
        bladerf.set_bandwidth(Direction::Tx, BW2).unwrap();
        let rx = bladerf.bandwidth(Direction::Rx).unwrap();
        let tx = bladerf.bandwidth(Direction::Tx).unwrap();
        assert_eq!(rx, BW1);
        assert_eq!(tx, BW2);

        bladerf.set_bandwidth(Direction::Rx, BW2).unwrap();
        bladerf.set_bandwidth(Direction::Tx, BW1).unwrap();
        let rx = bladerf.bandwidth(Direction::Rx).unwrap();
        let tx = bladerf.bandwidth(Direction::Tx).unwrap();
        assert_eq!(rx, BW2);
        assert_eq!(tx, BW1);
    }

    #[test]
    fn test_frequency() {
        const FREQ1: u64 = 920_000_000;
        const FREQ2: u64 = 433_000_000;
        let mut bladerf = BladeRf2::open_any().unwrap();

        bladerf.set_frequency(Direction::Rx, FREQ1).unwrap();
        bladerf.set_frequency(Direction::Tx, FREQ2).unwrap();
        let freq = bladerf.frequency(Direction::Rx).unwrap();
        let delta = (freq as f64 - FREQ1 as f64).abs();
        assert!(delta < 5.0);
        let freq = bladerf.frequency(Direction::Tx).unwrap();
        let delta = (freq as f64 - FREQ2 as f64).abs();
        assert!(delta < 5.0);

        bladerf.set_frequency(Direction::Rx, FREQ2).unwrap();
        bladerf.set_frequency(Direction::Tx, FREQ1).unwrap();
        let freq = bladerf.frequency(Direction::Rx).unwrap();
        let delta = (freq as f64 - FREQ2 as f64).abs();
        assert!(delta < 5.0);
        let freq = bladerf.frequency(Direction::Tx).unwrap();
        let delta = (freq as f64 - FREQ1 as f64).abs();
        assert!(delta < 5.0);
    }

    #[test]
    fn test_gain() {
        let mut bladerf = BladeRf2::open_any().unwrap();
        let mut stream = bladerf.rx_stream(&[0]).unwrap();

        let mut test_gains = |rx0: f64, rx1: f64, tx0: f64, tx1: f64| {
            bladerf.set_gain(Channel::Rx0, rx0).unwrap();
            bladerf.set_gain(Channel::Rx1, rx1).unwrap();
            bladerf.set_gain(Channel::Tx0, tx0).unwrap();
            bladerf.set_gain(Channel::Tx1, tx1).unwrap();

            // Required to update reported RX gains
            stream.activate().unwrap();

            assert_eq!(bladerf.gain(Channel::Rx0).unwrap(), rx0);
            assert_eq!(bladerf.gain(Channel::Rx1).unwrap(), rx1);
            assert_eq!(bladerf.gain(Channel::Tx0).unwrap(), tx0);
            assert_eq!(bladerf.gain(Channel::Tx1).unwrap(), tx1);
        };

        test_gains(50.0, 40.0, 30.0, 20.0);
        test_gains(20.0, 30.0, 40.0, 50.0);

        // Set to default gains
        test_gains(60.0, 60.0, 56.0, 56.0);
    }

    #[test]
    fn test_gain_mode() {
        let mut bladerf = BladeRf2::open_any().unwrap();

        bladerf
            .set_gain_mode(Channel::Rx0, GainMode::SlowAttack)
            .unwrap();
        bladerf
            .set_gain_mode(Channel::Rx1, GainMode::Manual)
            .unwrap();
        let mode1 = bladerf.gain_mode(Channel::Rx0).unwrap();
        let mode2 = bladerf.gain_mode(Channel::Rx1).unwrap();
        assert_eq!(mode1, GainMode::SlowAttack);
        assert_eq!(mode2, GainMode::Manual);

        bladerf
            .set_gain_mode(Channel::Rx0, GainMode::FastAttack)
            .unwrap();
        bladerf
            .set_gain_mode(Channel::Rx1, GainMode::Hybrid)
            .unwrap();
        let mode1 = bladerf.gain_mode(Channel::Rx0).unwrap();
        let mode2 = bladerf.gain_mode(Channel::Rx1).unwrap();
        assert_eq!(mode1, GainMode::FastAttack);
        assert_eq!(mode2, GainMode::Hybrid);

        bladerf
            .set_gain_mode(Channel::Rx0, GainMode::default())
            .unwrap();
        bladerf
            .set_gain_mode(Channel::Rx1, GainMode::default())
            .unwrap();
        let mode1 = bladerf.gain_mode(Channel::Rx0).unwrap();
        let mode2 = bladerf.gain_mode(Channel::Rx1).unwrap();
        assert_eq!(mode1, GainMode::SlowAttack);
        assert_eq!(mode2, GainMode::SlowAttack);
    }

    #[test]
    fn test_sample_rate() {
        let mut bladerf = BladeRf2::open_any().unwrap();

        bladerf.set_sample_rate(25_000_000).unwrap();
        let samp_rate = bladerf.sample_rate().unwrap();
        assert_eq!(samp_rate, 25_000_000);

        // Check for bug that occurs when switching between high and low sample rates
        bladerf.set_sample_rate(50_000_000).unwrap();
        let samp_rate = bladerf.sample_rate().unwrap();
        assert_eq!(samp_rate, 50_000_000);

        bladerf.set_sample_rate(2_083_334).unwrap();
        let samp_rate = bladerf.sample_rate().unwrap();
        assert_eq!(samp_rate, 2_083_334);

        bladerf.set_sample_rate(40_000_001).unwrap();
        let samp_rate = bladerf.sample_rate().unwrap();
        assert_eq!(samp_rate, 40_000_001);

        bladerf.set_sample_rate(2_083_332).unwrap();
        let samp_rate = bladerf.sample_rate().unwrap();
        assert_eq!(samp_rate, 2_083_332);
    }

    #[test]
    fn test_i16_loopback() {
        let mut bladerf = BladeRf2::open_any().unwrap();
        bladerf.set_gain(Channel::Rx0, 0.0).unwrap();

        bladerf.set_loopback(true).unwrap();
        assert!(bladerf.loopback().unwrap());

        let mut rx = bladerf.rx_stream(&[0]).unwrap();
        let mut tx = bladerf.tx_stream(&[0]).unwrap();
        tx.activate().unwrap();
        rx.activate().unwrap();

        // Check i16 loopback
        let tx_data = &mut std::array::from_fn::<_, 4096, _>(|i| i as i16 - 2048);
        let rx_data = &mut [0; 4096];
        tx.write_i16(tx_data).unwrap();
        rx.read_i16(rx_data).unwrap();
        assert!(tx_data.iter().zip(rx_data.iter()).all(|(x, y)| x == y));

        bladerf.set_loopback(false).unwrap();
        assert!(!bladerf.loopback().unwrap());
    }

    #[test]
    fn test_siso_loopback() {
        let mut bladerf = BladeRf2::open_any().unwrap();
        bladerf.set_gain(Channel::Rx0, 0.0).unwrap();

        bladerf.set_loopback(true).unwrap();
        assert!(bladerf.loopback().unwrap());

        let mut rx = bladerf.rx_stream(&[0]).unwrap();
        let mut tx = bladerf.tx_stream(&[0]).unwrap();
        tx.activate().unwrap();
        rx.activate().unwrap();

        let mut tx_data = [Complex32::ZERO; 4096];
        let mut rx_data = [Complex32::ZERO; 4096];

        for pow in 1..16 {
            // Generate complex sinusoidal data
            for i in 0..tx_data.len() {
                let phase = 2.0 * std::f32::consts::PI * (i as f32) / 2.0_f32.powi(pow);
                tx_data[i] = (Complex32::I * phase).exp() * 0.5;
            }

            tx.write(&[&tx_data]).unwrap();
            rx.read(&mut [&mut rx_data]).unwrap();

            // Check loopback data
            for (a, b) in tx_data.iter().zip(rx_data.iter()) {
                const MAX_ERROR: f32 = 1.0 / 4096.0; // 2^-12
                let delta = a - b;
                assert!(delta.re.abs() <= MAX_ERROR);
                assert!(delta.im.abs() <= MAX_ERROR);
            }
        }

        bladerf.set_loopback(false).unwrap();
        assert!(!bladerf.loopback().unwrap());
    }
}
