//! Pure Rust driver for the [bladeRF 2.0 micro] software-defined radio.
//!
//! The driver talks to the radio directly over USB using [`nusb`], so it needs
//! no vendor library or other native dependency, and it runs unchanged in the
//! browser through WebUSB.
//!
//! # Blocking and async
//!
//! Every operation that touches the device returns a [`MaybeFuture`]:
//!
//! * call `.wait()` to block the current thread (native targets only, no async
//!   runtime required), or
//! * `.await` it from async code.
//!
//! On native targets, `.await` needs one of the `smol` or `tokio` cargo
//! features so that nusb can run blocking USB calls on a runtime. On
//! `wasm32-unknown-unknown` nothing extra is needed: all I/O goes through
//! WebUSB and only the async API is available.
//!
//! # Quick start (blocking)
//!
//! ```no_run
//! use bladerf2::{BladeRf2, Direction, MaybeFuture};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut dev = BladeRf2::open_any()?;
//! println!("serial {}, FPGA {}", dev.serial()?, dev.fpga_version().wait()?);
//!
//! let rf = dev.rf_link_session();
//! rf.set_frequency(Direction::Rx, 915_000_000).wait()?;
//! rf.set_sample_rate(10_000_000).wait()?;
//!
//! let mut rx = rf.rx_stream(&[0])?;
//! rx.activate().wait()?;
//! let mut samples = vec![num_complex::Complex32::ZERO; rx.mtu()];
//! rx.read(&mut [&mut samples], None).wait()?;
//! rx.deactivate().wait()?;
//! # Ok(())
//! # }
//! ```
//!
//! # Quick start (async, any target)
//!
//! Finding a device is the one place where native and browser differ. On
//! native targets every attached radio is listed; in the browser the page must
//! first ask the user for permission with `navigator.usb.requestDevice`, after
//! which [`BladeRf2::list_devices`] returns the granted radios.
//!
//! ```no_run
//! use bladerf2::{BladeRf2, Direction};
//!
//! # async fn run() -> Result<(), Box<dyn std::error::Error>> {
//! let info = BladeRf2::list_devices().await?.into_iter().next().ok_or("no device")?;
//! let mut dev = BladeRf2::open_device(info).await?;
//!
//! let rf = dev.rf_link_session();
//! rf.set_frequency(Direction::Rx, 915_000_000).await?;
//! # Ok(())
//! # }
//! ```
//!
//! # Sessions and streams
//!
//! Configuration (frequency, gain, sample rate, ...) goes through an
//! [`RfLinkSession`] obtained from [`BladeRf2::rf_link_session`]. The session
//! borrows the device mutably, so the compiler guarantees that at most one
//! session exists at a time.
//!
//! Sample streams ([`RxStream`], [`TxStream`]) are created from a session but
//! own everything they need, so they stay valid after the session is dropped
//! and can be moved to another thread or task. Always call `deactivate` when
//! finished: dropping an active stream only deactivates it on a best-effort
//! basis (on wasm, asynchronously).
//!
//! # Examples
//!
//! The `examples` directory has near-identical `rx_sync`, `rx_async` and
//! `rx_web` programs that print device information, change and read back every
//! setting, and receive samples.
//!
//! [bladeRF 2.0 micro]: https://www.nuand.com/bladerf-2-0-micro/

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]

pub mod channel;
pub mod error;
pub mod stream;

mod drivers;
mod maybe_future;
mod nios;
mod session;
mod usb;

pub use channel::{Channel, Direction};
pub use drivers::rfic::{GainMode, GainRange};
pub use error::{Error, Result};
pub use nusb::{DeviceInfo, MaybeFuture};
pub use session::RfLinkSession;
pub use stream::{RxStream, TxStream};

use drivers::ad5621::Ad5621;
use nios::{Nios8x32, NiosCore};
use usb::UsbTransport;

/// Handle to a bladeRF 2.0 micro.
///
/// Obtain one with [`BladeRf2::open_any`] / [`BladeRf2::open_serial`]
/// (blocking, native only) or [`BladeRf2::list_devices`] followed by
/// [`BladeRf2::open_device`] (blocking or async, all targets).
///
/// The handle exposes read-only device information; everything that changes
/// radio state goes through [`BladeRf2::rf_link_session`].
#[derive(Debug)]
pub struct BladeRf2 {
    transport: UsbTransport,
    nios: NiosCore,
    dac: Ad5621,
}

impl BladeRf2 {
    /// USB vendor ID of Nuand devices.
    pub const VID: u16 = 0x2CF0;
    /// USB product ID of the bladeRF 2.0 micro.
    pub const PID: u16 = 0x5250;

    fn is_bladerf2(info: &DeviceInfo) -> bool {
        info.vendor_id() == Self::VID
            && info.product_id() == Self::PID
            && info.product_string() == Some("bladeRF 2.0")
    }

    /// List the bladeRF 2.0 devices this process can open.
    ///
    /// On native targets this is every attached device. In the browser it is
    /// only the devices the user has granted to the page through
    /// `navigator.usb.requestDevice`.
    pub fn list_devices() -> impl MaybeFuture<Output = Result<Vec<DeviceInfo>>> {
        maybe_future::operation(async {
            let devices = maybe_future::usb(nusb::list_devices()).await?;
            Ok(devices.filter(Self::is_bladerf2).collect())
        })
    }

    /// Open the device described by `info` and initialize the radio.
    ///
    /// Requires USB 3.0 SuperSpeed where the platform reports the link speed
    /// (WebUSB does not). After power-up the firmware can take a few seconds
    /// to load the FPGA; this call waits for it.
    pub fn open_device(info: DeviceInfo) -> impl MaybeFuture<Output = Result<Self>> {
        maybe_future::operation(Self::open_inner(info))
    }

    async fn open_inner(info: DeviceInfo) -> Result<Self> {
        let device = maybe_future::usb(info.open()).await?;

        if let Some(speed) = device.speed()
            && speed != nusb::Speed::Super
        {
            return Err(Error::UsbSpeed);
        }

        let interface = maybe_future::usb(device.claim_interface(0)).await?;

        // The RF-link firmware exposes exactly four alternate settings.
        if interface.descriptors().count() != 4 {
            return Err(Error::IncompatibleFirmware);
        }

        let transport = UsbTransport::new(interface, info);
        let (nios, dac) = drivers::init(&transport).await?;
        Ok(Self {
            transport,
            nios,
            dac,
        })
    }

    /// List all attached bladeRF 2.0 devices (blocking, native only).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn enumerate() -> Result<Vec<DeviceInfo>> {
        Self::list_devices().wait()
    }

    /// Open the first attached bladeRF 2.0 (blocking, native only).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open_any() -> Result<Self> {
        let info = Self::enumerate()?
            .into_iter()
            .next()
            .ok_or(Error::NoDeviceFound)?;
        Self::open(info)
    }

    /// Open the bladeRF 2.0 whose serial number starts with `serial`
    /// (blocking, native only).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open_serial(serial: &str) -> Result<Self> {
        let info = Self::enumerate()?
            .into_iter()
            .find(|dev| dev.serial_number().is_some_and(|s| s.starts_with(serial)))
            .ok_or(Error::NoDeviceFound)?;
        Self::open(info)
    }

    /// Open the device described by `info` (blocking, native only).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open(info: DeviceInfo) -> Result<Self> {
        Self::open_device(info).wait()
    }

    /// USB serial number of the device.
    pub fn serial(&self) -> Result<&str> {
        self.transport
            .info()
            .serial_number()
            .ok_or(Error::ValueConversion)
    }

    /// Version of the FPGA bitstream.
    pub fn fpga_version(&self) -> impl MaybeFuture<Output = Result<Version>> + '_ {
        maybe_future::operation(async {
            let raw: u32 = self.nios.read(Nios8x32::Version, 0).await?;
            Ok(Version::from(raw))
        })
    }

    /// VCTCXO DAC trim value read from the device's flash at open time.
    ///
    /// This is the factory calibration of the 38.4 MHz reference oscillator.
    pub fn vctcxo_trim(&self) -> u16 {
        self.dac.trim_value()
    }

    /// Start an [`RfLinkSession`] for configuring the radio and creating
    /// sample streams.
    ///
    /// The session borrows the device mutably, so only one can exist at a
    /// time; the device is usable again once the session is dropped.
    pub fn rf_link_session(&mut self) -> RfLinkSession<'_> {
        RfLinkSession::new(&self.transport, &self.nios, &self.dac)
    }
}

/// Semantic version of the FPGA bitstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    /// Major version.
    pub major: u8,
    /// Minor version.
    pub minor: u8,
    /// Patch version.
    pub patch: u16,
}

impl std::fmt::Display for Version {
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
            patch: u16::from_le_bytes([bytes[2], bytes[3]]),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version_display() {
        let version = Version {
            major: 0,
            minor: 16,
            patch: 0,
        };
        assert_eq!(version.to_string(), "0.16.0");
    }

    #[test]
    fn test_version_from_u32() {
        let version = Version::from(0x0000_1000);
        assert_eq!((version.major, version.minor, version.patch), (0, 16, 0));
    }
}
