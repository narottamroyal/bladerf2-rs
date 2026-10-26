//! RF-link session: exclusive access to the radio's RF configuration and
//! sample streams.
//!
//! A session is created with [`BladeRf2::rf_link_session`](crate::BladeRf2::rf_link_session),
//! which borrows the device mutably: the compiler guarantees at most one
//! session exists at a time, and the device is usable again once the session
//! is dropped.
//!
//! ```no_run
//! # use bladerf2::{BladeRf2, Direction, MaybeFuture};
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut dev = BladeRf2::open_any().wait()?;
//! let rf = dev.rf_link_session();
//! rf.set_frequency(Direction::Rx, 915_000_000).wait()?;
//! # Ok(())
//! # }
//! ```

use std::ops::RangeInclusive;

use nusb::MaybeFuture;

use crate::channel::{Channel, Direction};
use crate::drivers;
use crate::drivers::rfic::{GainMode, GainRange};
use crate::error::Result;
use crate::maybe_future;
use crate::nios::{Nios8x32, Nios8x64, NiosCore};
use crate::stream::{RxStream, TxStream};
use crate::usb::UsbTransport;

/// Exclusive handle for configuring the radio and creating sample streams.
///
/// Provides frequency, gain, sample rate and bandwidth control, bias tee,
/// loopback, PLL reference selection, a few diagnostics, and RX/TX streams.
/// Obtain one from [`BladeRf2::rf_link_session`](crate::BladeRf2::rf_link_session).
///
/// Methods returning a [`MaybeFuture`] can be run with `.wait()` (blocking,
/// native only) or `.await`.
#[derive(Debug)]
pub struct RfLinkSession<'a> {
    transport: &'a UsbTransport,
    nios: &'a NiosCore,
    dac: &'a drivers::ad5621::Ad5621,
}

impl<'a> RfLinkSession<'a> {
    pub(crate) fn new(
        transport: &'a UsbTransport,
        nios: &'a NiosCore,
        dac: &'a drivers::ad5621::Ad5621,
    ) -> Self {
        Self {
            transport,
            nios,
            dac,
        }
    }
}

impl RfLinkSession<'_> {
    // ==================== Frequency ====================

    /// Supported frequency range for a direction, in Hz.
    pub fn frequency_range(&self, direction: Direction) -> RangeInclusive<u64> {
        drivers::rfic::get_frequency_range(direction)
    }

    /// Current frequency for a direction, in Hz.
    pub fn frequency(&self, direction: Direction) -> impl MaybeFuture<Output = Result<u64>> {
        maybe_future::operation(
            async move { drivers::rfic::get_frequency(self.nios, direction).await },
        )
    }

    /// Set the frequency for a direction, in Hz.
    pub fn set_frequency(
        &self,
        direction: Direction,
        frequency: u64,
    ) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::rfic::set_frequency(self.nios, direction, frequency).await
        })
    }

    // ==================== Sample rate ====================

    /// Supported sample rate range, in Hz.
    pub fn sample_rate_range(&self) -> RangeInclusive<u64> {
        drivers::rfic::get_sample_rate_range()
    }

    /// Current sample rate, in Hz.
    pub fn sample_rate(&self) -> impl MaybeFuture<Output = Result<u64>> {
        maybe_future::operation(async move { drivers::rfic::get_sample_rate(self.nios).await })
    }

    /// Set the sample rate, in Hz.
    pub fn set_sample_rate(&self, rate: u64) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(
            async move { drivers::rfic::set_sample_rate(self.nios, rate).await },
        )
    }

    // ==================== Bandwidth ====================

    /// Supported bandwidth range, in Hz.
    pub fn bandwidth_range(&self) -> RangeInclusive<u64> {
        drivers::rfic::get_bandwidth_range()
    }

    /// Current bandwidth for a direction, in Hz.
    pub fn bandwidth(&self, direction: Direction) -> impl MaybeFuture<Output = Result<u64>> {
        maybe_future::operation(
            async move { drivers::rfic::get_bandwidth(self.nios, direction).await },
        )
    }

    /// Set the bandwidth for a direction, in Hz.
    pub fn set_bandwidth(
        &self,
        direction: Direction,
        bandwidth: u64,
    ) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::rfic::set_bandwidth(self.nios, direction, bandwidth).await
        })
    }

    // ==================== Gain ====================

    /// Gain range for a direction, based on the current frequency.
    pub fn gain_range(
        &self,
        direction: Direction,
    ) -> impl MaybeFuture<Output = Result<&'static GainRange>> {
        maybe_future::operation(
            async move { drivers::rfic::gain_range(self.nios, direction).await },
        )
    }

    /// Current gain for a channel, in dB.
    pub fn gain(&self, channel: Channel) -> impl MaybeFuture<Output = Result<f64>> {
        maybe_future::operation(async move { drivers::rfic::get_gain(self.nios, channel).await })
    }

    /// Set the gain for a channel, in dB.
    ///
    /// Setting an RX gain switches the channel to [`GainMode::Manual`]. The
    /// new gain is applied immediately, so [`gain`](Self::gain) reads it back
    /// right away. If no RX channel is running, this briefly enables and then
    /// disables an RX channel to make the radio apply it; a running
    /// RX stream is never toggled, so setting the gain of the *other* RX
    /// channel while one is streaming only takes effect once it starts.
    pub fn set_gain(&self, channel: Channel, gain: f64) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(
            async move { drivers::rfic::set_gain(self.nios, channel, gain).await },
        )
    }

    /// Current AGC mode for an RX channel.
    pub fn gain_mode(&self, channel: Channel) -> impl MaybeFuture<Output = Result<GainMode>> {
        maybe_future::operation(
            async move { drivers::rfic::get_gain_mode(self.nios, channel).await },
        )
    }

    /// Set the AGC mode for an RX channel.
    pub fn set_gain_mode(
        &self,
        channel: Channel,
        mode: GainMode,
    ) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::rfic::set_gain_mode(self.nios, channel, mode).await
        })
    }

    // ==================== Bias tee ====================

    /// Whether the bias tee is supplying DC power on a direction's antenna port.
    pub fn bias_tee(&self, direction: Direction) -> impl MaybeFuture<Output = Result<bool>> {
        maybe_future::operation(async move { drivers::rffe::bias_tee(self.nios, direction).await })
    }

    /// Turn the bias tee on or off for a direction. While on, it supplies DC
    /// power on that direction's antenna port.
    pub fn set_bias_tee(
        &self,
        direction: Direction,
        enable: bool,
    ) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::rffe::set_bias_tee(self.nios, direction, enable).await
        })
    }

    // ==================== Loopback ====================

    /// Whether FPGA RF loopback is enabled.
    pub fn loopback(&self) -> impl MaybeFuture<Output = Result<bool>> {
        const MASK: u32 = 0x7 << 8;
        const LOOPBACK: u32 = 4 << 8;
        maybe_future::operation(async move {
            let gpio = self.nios.read(Nios8x32::Control, 0).await?;
            Ok((gpio & MASK) == LOOPBACK)
        })
    }

    /// Enable or disable FPGA RF loopback.
    pub fn set_loopback(&self, enable: bool) -> impl MaybeFuture<Output = Result<()>> {
        const MASK: u32 = 0x7 << 8;
        const LOOPBACK: u32 = 4 << 8;
        maybe_future::operation(async move {
            let mut gpio = self.nios.read(Nios8x32::Control, 0).await?;
            gpio &= !MASK;
            if enable {
                gpio |= LOOPBACK;
            }
            self.nios.write(Nios8x32::Control, 0, gpio).await
        })
    }

    // ==================== PLL ====================

    /// Supported PLL reference frequency range, in Hz.
    pub fn pll_refclk_range(&self) -> RangeInclusive<u64> {
        drivers::adf4002::REFCLK_RANGE
    }

    /// PLL reference frequency, in Hz.
    ///
    /// Computed from the PLL divider registers. The dividers are integers, so
    /// this can differ slightly from the value given to
    /// [`set_pll_refclk`](Self::set_pll_refclk).
    pub fn pll_refclk(&self) -> impl MaybeFuture<Output = Result<u64>> {
        maybe_future::operation(async move { drivers::adf4002::refclk(self.nios).await })
    }

    /// Set the PLL reference frequency, in Hz.
    ///
    /// Fails with [`Error::OutOfRange`](crate::Error::OutOfRange) outside
    /// [`pll_refclk_range`](Self::pll_refclk_range).
    pub fn set_pll_refclk(&self, ref_hz: u64) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::adf4002::set_refclk(self.nios, self.dac, ref_hz).await
        })
    }

    /// Whether the PLL is locking the on-board VCTCXO to the reference on
    /// REFIN (J95).
    pub fn pll_enable(&self) -> impl MaybeFuture<Output = Result<bool>> {
        maybe_future::operation(async move { drivers::adf4002::enabled(self.nios).await })
    }

    /// Turn the PLL on or off. While on, it locks the on-board VCTCXO to the
    /// reference on REFIN (J95) and the VCTCXO DAC is released; turning it off
    /// restores the DAC.
    pub fn set_pll_enable(&self, enable: bool) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::adf4002::set_enabled(self.nios, self.dac, enable).await
        })
    }

    /// Whether the PLL reports lock to the reference on REFIN.
    pub fn pll_lock_state(&self) -> impl MaybeFuture<Output = Result<bool>> {
        maybe_future::operation(async move { drivers::rffe::pll_lock_state(self.nios).await })
    }

    // ==================== Diagnostics ====================

    /// Current 64-bit FPGA timestamp counter.
    ///
    /// In testing this has read 0 even while streaming, so it is not yet
    /// known to be a usable clock with this driver.
    pub fn timestamp(&self) -> impl MaybeFuture<Output = Result<u64>> {
        maybe_future::operation(async move { self.nios.read(Nios8x64::Timestamp, 0).await })
    }

    /// Disable RX IQ correction and DC tracking loops.
    pub fn disable_tracking_control(&self) -> impl MaybeFuture<Output = Result<()>> {
        maybe_future::operation(async move {
            drivers::ad9361::disable_tracking_control(self.nios).await
        })
    }

    // ==================== Streaming ====================

    /// Create an RX sample stream for channels `[0]`, `[1]`, or `[0, 1]`.
    pub fn rx_stream(&self, channels: &[usize]) -> Result<RxStream> {
        RxStream::new(self.transport, self.nios, channels)
    }

    /// Create a TX sample stream for channels `[0]`, `[1]`, or `[0, 1]`.
    pub fn tx_stream(&self, channels: &[usize]) -> Result<TxStream> {
        TxStream::new(self.transport, self.nios, channels)
    }
}
