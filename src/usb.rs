//! USB transport layer: vendor commands, alternate settings, and raw bulk
//! transfer helpers shared by the NIOS pipe and the sample streams.

use std::time::Duration;

use nusb::{
    Interface,
    transfer::{Bulk, Completion, ControlIn, ControlType, EndpointDirection, Recipient},
};

use crate::error::{Error, Result};
use crate::maybe_future;

/// Timeout for USB vendor commands.
const VENDOR_TIMEOUT: Duration = Duration::from_secs(3);

/// USB alternate settings exposed by the firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub(crate) enum AltSetting {
    /// Idle; no active endpoints.
    Null = 0,
    /// RF link mode: NIOS control + streaming endpoints active.
    RfLink = 1,
    /// SPI flash access.
    SpiFlash = 2,
    /// FPGA configuration.
    Config = 3,
}

/// USB endpoint addresses for the RfLink alternate setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum Endpoint {
    /// Bulk IN endpoint for RX samples.
    SampleIn = 0x81,
    /// Bulk OUT endpoint for TX samples.
    SampleOut = 0x01,
    /// Bulk IN endpoint for NIOS responses.
    PeripheralIn = 0x82,
    /// Bulk OUT endpoint for NIOS commands.
    PeripheralOut = 0x02,
}

/// Vendor-specific USB control request identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)] // full vendor command set; flash/OTP commands for future use
pub(crate) enum UsbCommand {
    /// Query the firmware version.
    QueryVersion = 0,
    /// Query the FPGA configuration status.
    QueryFpgaStatus = 1,
    /// Begin FPGA programming.
    BeginProg = 2,
    /// End FPGA programming.
    EndProg = 3,
    /// Enable/disable the RX streaming module.
    RfRx = 4,
    /// Enable/disable the TX streaming module.
    RfTx = 5,
    /// Query whether the firmware is ready.
    QueryDeviceReady = 6,
    /// Read the SPI flash device ID.
    QueryFlashId = 7,
    /// Query the FPGA bitstream source.
    QueryFpgaSource = 8,
    /// Read from the SPI flash.
    FlashRead = 100,
    /// Write to the SPI flash.
    FlashWrite = 101,
    /// Erase a region of the SPI flash.
    FlashErase = 102,
    /// Read the FX3 OTP region.
    ReadOtp = 103,
    /// Write the FX3 OTP region.
    WriteOtp = 104,
    /// Reset the FX3 USB controller.
    Reset = 105,
    /// Jump to the FX3 bootloader.
    JumpToBootloader = 106,
    /// Read the FX3 page buffer.
    ReadPageBuffer = 107,
    /// Write the FX3 page buffer.
    WritePageBuffer = 108,
    /// Lock the FX3 OTP region.
    LockOtp = 109,
    /// Read a calibration cache entry.
    ReadCalCache = 110,
    /// Invalidate the calibration cache.
    InvalidateCalCache = 111,
    /// Refresh the calibration cache.
    RefreshCalCache = 112,
    /// Set the firmware loopback mode.
    SetLoopback = 113,
    /// Get the firmware loopback mode.
    GetLoopback = 114,
    /// Read a firmware log entry.
    ReadLogEntry = 115,
}

/// USB transport for the bladeRF 2.0.
#[derive(Clone)]
pub struct UsbTransport {
    interface: Interface,
    info: nusb::DeviceInfo,
}

impl std::fmt::Debug for UsbTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("UsbTransport").finish_non_exhaustive()
    }
}

impl UsbTransport {
    /// Create a transport from a claimed USB interface.
    pub(crate) fn new(interface: Interface, info: nusb::DeviceInfo) -> Self {
        Self { interface, info }
    }

    /// The underlying USB interface.
    pub(crate) fn interface(&self) -> &Interface {
        &self.interface
    }

    /// The USB device info.
    pub(crate) fn info(&self) -> &nusb::DeviceInfo {
        &self.info
    }

    /// Change the USB alternate setting.
    pub(crate) async fn change_setting(&self, setting: AltSetting) -> Result<()> {
        maybe_future::usb(self.interface.set_alt_setting(setting as u8)).await?;
        Ok(())
    }

    /// Send a vendor command and return the response data.
    pub(crate) async fn vendor_command(&self, command: UsbCommand, length: u16) -> Result<Vec<u8>> {
        let data = maybe_future::usb(self.interface.control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: command as u8,
                value: 0,
                index: 0,
                length,
            },
            VENDOR_TIMEOUT,
        ))
        .await?;
        Ok(data)
    }

    /// Send a vendor command with an enable/disable parameter and verify the
    /// response status word.
    pub(crate) async fn vendor_command_enable(
        &self,
        command: UsbCommand,
        enable: bool,
    ) -> Result<()> {
        let data = maybe_future::usb(self.interface.control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: command as u8,
                value: enable.into(),
                index: 0,
                length: size_of::<u32>() as u16,
            },
            VENDOR_TIMEOUT,
        ))
        .await?;

        let data: [u8; 4] = data.try_into().map_err(|_| Error::ValueConversion)?;
        if u32::from_le_bytes(data) == 0 {
            Ok(())
        } else {
            Err(Error::UsbVendor)
        }
    }

    /// Enable or disable a streaming module.
    pub(crate) async fn enable_module(
        &self,
        direction: crate::Direction,
        enable: bool,
    ) -> Result<()> {
        let command = match direction {
            crate::Direction::Rx => UsbCommand::RfRx,
            crate::Direction::Tx => UsbCommand::RfTx,
        };
        self.vendor_command_enable(command, enable).await
    }

    /// Query whether the device firmware reports ready.
    pub(crate) async fn is_firmware_ready(&self) -> Result<bool> {
        let data = self.vendor_command(UsbCommand::QueryDeviceReady, 1).await?;
        Ok(data.first() == Some(&1))
    }
}

// ==================== Raw transfer helpers ====================

/// Round `size` up to a multiple of the endpoint packet size (required by
/// nusb for bulk IN transfers).
pub(crate) fn aligned_size(size: usize, packet_size: usize) -> usize {
    size.div_ceil(packet_size).max(1) * packet_size
}

/// Wait for the oldest queued transfer on a bulk endpoint to complete.
/// Runtime-independent: nusb wakes the task from its own completion threads
/// on native targets and from WebUSB events on wasm.
pub(crate) async fn next_complete<D: EndpointDirection>(
    endpoint: &mut nusb::Endpoint<Bulk, D>,
) -> Completion {
    std::future::poll_fn(|cx| endpoint.poll_next_complete(cx)).await
}
