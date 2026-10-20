//! Error types for the bladeRF 2.0 driver.

use thiserror::Error;

/// Result type alias for driver operations.
pub type Result<T> = std::result::Result<T, Error>;

/// Errors that can occur when working with a bladeRF 2.0 device.
#[derive(Debug, Error)]
pub enum Error {
    /// No bladeRF 2.0 device was found.
    #[error("no bladeRF 2.0 device found")]
    NoDeviceFound,

    /// The device is not connected via USB 3.0 SuperSpeed.
    #[error("device is not connected via USB 3.0 SuperSpeed")]
    UsbSpeed,

    /// The firmware does not expose the expected 4 alternate settings.
    #[error("incompatible firmware: expected 4 USB alternate settings")]
    IncompatibleFirmware,

    /// A USB device operation failed.
    #[error("USB error: {0}")]
    Usb(#[from] nusb::Error),

    /// An I/O operation failed.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// A USB transfer completed with an error status.
    #[error("transfer error: {0}")]
    Transfer(#[from] nusb::transfer::TransferError),

    /// A NIOS packet exchange failed.
    #[error("NIOS transfer unsuccessful")]
    Nios,

    /// A value could not be converted between representations.
    #[error("unable to convert value")]
    ValueConversion,

    /// An operation timed out.
    #[error("operation timed out")]
    Timeout,

    /// A requested value is outside the valid range.
    #[error("value is outside the valid range")]
    OutOfRange,

    /// No valid PLL divider ratio exists for the requested frequencies.
    #[error("no PLL divider solution found")]
    PllDividerRatio,

    /// A USB vendor command returned a failure status.
    #[error("USB vendor command failed")]
    UsbVendor,

    /// The channel configuration is invalid.
    #[error("invalid channel configuration")]
    ChannelConfig,

    /// A control operation overlapped another in-flight operation on the
    /// NIOS pipe.
    #[error("device is busy")]
    Busy,
}
