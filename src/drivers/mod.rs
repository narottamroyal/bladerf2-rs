//! Chip-level drivers for the bladeRF 2.0's peripherals and the board
//! initialization sequence.

pub(crate) mod ad5621;
pub(crate) mod ad9361;
pub(crate) mod adf4002;
pub(crate) mod flash;
pub(crate) mod ina219;
pub(crate) mod rffe;
pub(crate) mod rfic;

use crate::error::{Error, Result};
use crate::maybe_future;
use crate::nios::NiosCore;
use crate::usb::UsbTransport;

use ad5621::Ad5621;

/// Initialize all hardware in the correct sequence:
///
/// 1. Wait for the FX3 firmware to report ready (FPGA loaded from flash)
/// 2. Read the VCTCXO trim from the calibration cache (switches to the
///    SPI-flash alternate setting and back to RF-link)
/// 3. Create the NIOS control channel
/// 4. Initialize the power monitor (INA219), RFIC (AD9361), and PLL (ADF4002)
pub(crate) async fn init(transport: &UsbTransport) -> Result<(NiosCore, Ad5621)> {
    wait_for_device_ready(transport).await?;

    let dac_trim = flash::read_vctcxo_trim(transport).await?;
    let dac = Ad5621::new(dac_trim);

    let nios = NiosCore::new(transport.interface())?;

    ina219::init(&nios).await?;
    rfic::init(&nios).await?;
    adf4002::init(&nios, &dac).await?;

    tracing::info!("bladeRF 2.0 initialization complete");

    Ok((nios, dac))
}

/// Wait for the FX3 firmware to report ready. After power-on the firmware
/// takes a few seconds to load the FPGA bitstream from flash; until then the
/// NIOS and streaming endpoints are unavailable.
async fn wait_for_device_ready(transport: &UsbTransport) -> Result<()> {
    const MAX_RETRIES: u32 = 30;
    const RETRY_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);

    for attempt in 0..MAX_RETRIES {
        if transport.is_firmware_ready().await? {
            return Ok(());
        }
        if attempt == 0 {
            tracing::info!("waiting for device to become ready...");
        } else {
            tracing::debug!("retry {}/{MAX_RETRIES}", attempt + 1);
        }
        maybe_future::sleep(RETRY_INTERVAL).await;
    }
    Err(Error::Timeout)
}
