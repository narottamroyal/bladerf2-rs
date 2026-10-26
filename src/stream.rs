//! Sample streaming (SC16 Q11 over USB bulk endpoints).
//!
//! Samples are signed 16-bit I/Q pairs holding 12-bit values, scaled to
//! `[-1.0, 1.0)` by dividing by 2048. Dual-channel streams interleave both
//! channels on the wire (`I0, Q0, I1, Q1, ...`).
//!
//! Every operation returns a [`MaybeFuture`]: call `.wait()` to block, or
//! `.await` it from async code.

use crate::channel::{Channel, Direction};
use crate::drivers::rfic;
use crate::error::{Error, Result};
use crate::maybe_future::{self, with_timeout};
use crate::nios::NiosCore;
use crate::usb::{self, Endpoint as EndpointAddress, UsbTransport};
use num_complex::Complex32;
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, EndpointDirection, In, Out, TransferError};
use std::time::Duration;

/// Requested size of each USB bulk transfer, in bytes, and the most transfers
/// queued with the OS to keep the pipe full. Each WebUSB transfer is a round
/// trip through the browser, so the browser needs larger ones to keep up at
/// high sample rates.
#[cfg(not(target_arch = "wasm32"))]
const TRANSFER_SIZE: usize = 16 * 1024;
#[cfg(not(target_arch = "wasm32"))]
const MAX_PENDING: usize = 16;
#[cfg(target_arch = "wasm32")]
const TRANSFER_SIZE: usize = 128 * 1024;
#[cfg(target_arch = "wasm32")]
const MAX_PENDING: usize = 16;

/// Longest `deactivate` waits for queued transfers before stopping the radio.
const COLLECT_TIMEOUT: Duration = Duration::from_secs(1);

/// Q11 full-scale: samples span `[-2048, 2047]`.
const Q11_SCALE: f32 = 2048.0;

// ==================== Sample format conversion ====================

fn clamp_q11(value: f32) -> i16 {
    (value * Q11_SCALE)
        .round()
        .clamp(-Q11_SCALE, Q11_SCALE - 1.0) as i16
}

/// Convert SC16 Q11 samples (interleaved i16 I/Q pairs) to [`Complex32`].
fn q11_to_c32(input: &[i16], output: &mut [Complex32]) {
    let scale = 1.0 / Q11_SCALE;
    for (pair, out) in input.as_chunks::<2>().0.iter().zip(output) {
        out.re = pair[0] as f32 * scale;
        out.im = pair[1] as f32 * scale;
    }
}

/// Convert [`Complex32`] samples to interleaved SC16 Q11.
fn c32_to_q11(input: &[Complex32], output: &mut [i16]) {
    for (sample, pair) in input.iter().zip(output.as_chunks_mut::<2>().0.iter_mut()) {
        pair[0] = clamp_q11(sample.re);
        pair[1] = clamp_q11(sample.im);
    }
}

/// Append little-endian i16 samples decoded from `bytes` to `out`.
fn extend_i16_le(out: &mut Vec<i16>, bytes: &[u8]) {
    out.extend(
        bytes
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| i16::from_le_bytes(*c)),
    );
}

// ==================== Radio path ====================

/// Channels of `direction` selected by `indices` (one of `[0]`, `[1]`, `[0, 1]`).
fn parse_channels(direction: Direction, indices: &[usize]) -> Result<Vec<Channel>> {
    let all = all_channels(direction);
    match indices {
        [0] | [1] | [0, 1] => Ok(indices.iter().map(|&i| all[i]).collect()),
        _ => Err(Error::ChannelConfig),
    }
}

fn all_channels(direction: Direction) -> [Channel; 2] {
    match direction {
        Direction::Rx => [Channel::Rx0, Channel::Rx1],
        Direction::Tx => [Channel::Tx0, Channel::Tx1],
    }
}

/// Disables RFIC channels, then the direction's USB path.
///
/// This order matters: disabling the USB path first, while the radio still
/// pulls samples, can leave the FX3 TX DMA stuck so the next TX stream stalls.
async fn disable(
    nios: &NiosCore,
    transport: &UsbTransport,
    direction: Direction,
    channels: &[Channel],
) -> Result<()> {
    for &channel in channels {
        rfic::enable_module(nios, channel, false).await?;
    }
    transport.enable_module(direction, false).await
}

/// Disables every channel and both USB paths.
///
/// Channel state outlives the USB connection, so a session that ended without
/// stopping its streams would otherwise corrupt the next one's samples.
pub(crate) async fn disable_all(nios: &NiosCore, transport: &UsbTransport) -> Result<()> {
    for direction in [Direction::Rx, Direction::Tx] {
        disable(nios, transport, direction, &all_channels(direction)).await?;
    }
    Ok(())
}

/// Waits for queued transfers on a bulk endpoint to finish. Cancelled
/// transfers are expected. Fails if one takes longer than `COLLECT_TIMEOUT`.
async fn drain<D: EndpointDirection>(endpoint: &mut nusb::Endpoint<Bulk, D>) -> Result<()> {
    while endpoint.pending() > 0 {
        let completion = with_timeout(
            async {
                Ok::<_, Error>(std::future::poll_fn(|cx| endpoint.poll_next_complete(cx)).await)
            },
            Some(COLLECT_TIMEOUT),
        )
        .await?;
        match completion.status {
            Ok(()) | Err(TransferError::Cancelled) => {}
            Err(error) => return Err(error.into()),
        }
    }
    Ok(())
}

/// A stream's endpoint and radio path, shared by [`RxStream`] and [`TxStream`].
struct Path<D: EndpointDirection> {
    endpoint: nusb::Endpoint<Bulk, D>,
    transport: UsbTransport,
    nios: NiosCore,
    direction: Direction,
    channels: Vec<Channel>,
    /// Whether the path may be enabled, so `Drop` has to stop it.
    active: bool,
}

impl<D: EndpointDirection> Path<D> {
    fn new(
        transport: &UsbTransport,
        nios: &NiosCore,
        direction: Direction,
        address: EndpointAddress,
        channels: &[usize],
    ) -> Result<Self> {
        Ok(Self {
            endpoint: transport.interface().endpoint::<Bulk, D>(address as u8)?,
            transport: transport.clone(),
            nios: nios.clone(),
            direction,
            channels: parse_channels(direction, channels)?,
            active: false,
        })
    }

    /// Enables this stream's channels, then the USB path. The direction's
    /// other channel is disabled, in case a stream dropped while active left
    /// it on (see [`RxStream`]).
    async fn start(&mut self) -> Result<()> {
        self.active = true;
        for channel in all_channels(self.direction) {
            let enable = self.channels.contains(&channel);
            rfic::enable_module(&self.nios, channel, enable).await?;
        }
        self.transport.enable_module(self.direction, true).await
    }

    /// Stops the path and leaves the endpoint idle.
    ///
    /// Native cancels queued transfers first: stopping with transfers queued
    /// times out at high sample rates. WebUSB cannot cancel, so its queue
    /// drains while the radio still runs. Transfers still queued when the path
    /// is disabled leave the endpoint unusable.
    async fn stop(&mut self) -> Result<()> {
        #[cfg(not(target_arch = "wasm32"))]
        self.endpoint.cancel_all();
        #[cfg(target_arch = "wasm32")]
        if self.direction == Direction::Rx {
            // In loopback, RX data only arrives while TX sends, so read from
            // the RFIC until the queue drains. Loopback stops with RX anyway.
            use crate::drivers::fpga::{self, RxSource};
            let looped = fpga::rx_source(&self.nios).await? == RxSource::Loopback;
            if looped {
                fpga::set_rx_source(&self.nios, RxSource::Rfic).await?;
            }
            let drained = drain(&mut self.endpoint).await;
            if looped {
                fpga::set_rx_source(&self.nios, RxSource::Loopback).await?;
            }
            drained?;
        } else {
            drain(&mut self.endpoint).await?;
        }

        disable(&self.nios, &self.transport, self.direction, &self.channels).await?;

        #[cfg(not(target_arch = "wasm32"))]
        drain(&mut self.endpoint).await?;
        maybe_future::usb(self.endpoint.clear_halt()).await?;
        self.active = false;
        Ok(())
    }
}

impl<D: EndpointDirection> Drop for Path<D> {
    /// Stops an active stream. Not possible on wasm, where `Drop` cannot wait
    /// for USB.
    fn drop(&mut self) {
        #[cfg(not(target_arch = "wasm32"))]
        if self.active
            && let Err(error) = maybe_future::block_on(self.stop())
        {
            tracing::warn!(?error, direction = ?self.direction, "stream did not stop cleanly");
        }
    }
}

// ==================== RxStream ====================

/// Receive sample stream.
///
/// Call [`activate`](RxStream::activate) before reading and
/// [`deactivate`](RxStream::deactivate) when done. Received-but-unconsumed
/// samples are buffered across `read` calls, so short transfers never lose
/// data.
///
/// Dropping an active stream stops it on native targets. In the browser it
/// keeps running until the next stream in the same direction is activated, or
/// the device is opened again.
pub struct RxStream {
    path: Path<In>,
    in_size: usize,
    /// Samples received from USB but not yet handed to the caller.
    pending: Vec<i16>,
}

impl std::fmt::Debug for RxStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RxStream")
            .field("channels", &self.path.channels)
            .field("pending_samples", &self.pending.len())
            .finish_non_exhaustive()
    }
}

impl RxStream {
    /// Create an RX stream (one of `[0]`, `[1]`, `[0, 1]`).
    pub(crate) fn new(
        transport: &UsbTransport,
        nios: &NiosCore,
        channels: &[usize],
    ) -> Result<Self> {
        let path = Path::new(
            transport,
            nios,
            Direction::Rx,
            EndpointAddress::SampleIn,
            channels,
        )?;
        let in_size = usb::aligned_size(TRANSFER_SIZE, path.endpoint.max_packet_size());
        Ok(Self {
            path,
            in_size,
            pending: Vec::with_capacity(TRANSFER_SIZE / 2),
        })
    }

    /// Number of channels carried by this stream.
    pub fn num_channels(&self) -> usize {
        self.path.channels.len()
    }

    /// Recommended maximum samples per channel per `read` call.
    pub fn mtu(&self) -> usize {
        TRANSFER_SIZE / 4 / self.path.channels.len()
    }

    /// Activate the stream (enables the RFIC modules and USB RX path).
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(self.path.start())
    }

    /// Deactivate the stream and discard samples that were received but not
    /// yet read.
    pub fn deactivate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            self.path.stop().await?;
            self.pending.clear();
            Ok(())
        })
    }

    /// Reads one buffer per channel. `timeout` bounds the wait for data
    /// (`None` = forever).
    ///
    /// Fills every buffer completely. Excess data received from a USB transfer
    /// is buffered and returned by later calls.
    pub fn read<'a>(
        &'a mut self,
        buffers: &'a mut [&'a mut [Complex32]],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        maybe_future::operation(async move {
            if buffers.len() != self.path.channels.len() {
                return Err(Error::ChannelConfig);
            }
            match buffers.len() {
                1 => with_timeout(self.read_single(buffers[0]), timeout).await,
                2 => with_timeout(self.read_dual(buffers), timeout).await,
                _ => Err(Error::ChannelConfig),
            }
        })
    }

    async fn read_single(&mut self, buffer: &mut [Complex32]) -> Result<usize> {
        let chunk = buffer.len();
        let samples_needed = chunk * 2;
        while self.pending.len() < samples_needed {
            self.fill_pending().await?;
        }
        q11_to_c32(&self.pending[..samples_needed], buffer);
        self.pending.drain(..samples_needed);
        Ok(chunk)
    }

    async fn read_dual(&mut self, buffers: &mut [&mut [Complex32]]) -> Result<usize> {
        let [buf0, buf1] = buffers else {
            return Err(Error::ChannelConfig);
        };
        if buf0.len() != buf1.len() {
            return Err(Error::ChannelConfig);
        }
        let chunk = buf0.len();
        // Wire format interleaves channels: I0, Q0, I1, Q1 per sample pair.
        let samples_needed = chunk * 4;
        while self.pending.len() < samples_needed {
            self.fill_pending().await?;
        }
        let raw = &self.pending[..samples_needed];
        for (i, (s0, s1)) in buf0.iter_mut().zip(buf1.iter_mut()).enumerate() {
            let base = i * 4;
            *s0 = Complex32::new(
                raw[base] as f32 / Q11_SCALE,
                raw[base + 1] as f32 / Q11_SCALE,
            );
            *s1 = Complex32::new(
                raw[base + 2] as f32 / Q11_SCALE,
                raw[base + 3] as f32 / Q11_SCALE,
            );
        }
        self.pending.drain(..samples_needed);
        Ok(chunk)
    }

    /// Top up in-flight transfers and consume one completion into `pending`.
    async fn fill_pending(&mut self) -> Result<()> {
        while self.path.endpoint.pending() < MAX_PENDING {
            self.path.endpoint.submit(Buffer::new(self.in_size));
        }
        let buffer = usb::next_in(&mut self.path.endpoint).await?;
        extend_i16_le(&mut self.pending, &buffer);
        Ok(())
    }
}

// ==================== TxStream ====================

/// Transmit sample stream.
///
/// Call [`activate`](TxStream::activate) before writing and
/// [`deactivate`](TxStream::deactivate) when done. Dropping an active stream
/// behaves as for [`RxStream`].
pub struct TxStream {
    path: Path<Out>,
    transfer_size: usize,
}

impl std::fmt::Debug for TxStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TxStream")
            .field("channels", &self.path.channels)
            .finish_non_exhaustive()
    }
}

impl TxStream {
    /// Create a TX stream (one of `[0]`, `[1]`, `[0, 1]`).
    pub(crate) fn new(
        transport: &UsbTransport,
        nios: &NiosCore,
        channels: &[usize],
    ) -> Result<Self> {
        let path = Path::new(
            transport,
            nios,
            Direction::Tx,
            EndpointAddress::SampleOut,
            channels,
        )?;
        let transfer_size = usb::aligned_size(TRANSFER_SIZE, path.endpoint.max_packet_size());
        Ok(Self {
            path,
            transfer_size,
        })
    }

    /// Number of channels carried by this stream.
    pub fn num_channels(&self) -> usize {
        self.path.channels.len()
    }

    /// Recommended maximum samples per channel per `write` call.
    pub fn mtu(&self) -> usize {
        TRANSFER_SIZE / 4 / self.path.channels.len()
    }

    /// Activate the stream (enables the RFIC modules and USB TX path).
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(self.path.start())
    }

    /// Deactivate the stream. On native targets, samples that were written but
    /// not yet transmitted are discarded, so the tail of a burst can be cut
    /// off. In the browser they are transmitted first.
    pub fn deactivate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(self.path.stop())
    }

    /// Writes one buffer per channel. `timeout` bounds the wait for buffer
    /// space (`None` = forever).
    pub fn write<'a>(
        &'a mut self,
        buffers: &'a [&'a [Complex32]],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        maybe_future::operation(async move {
            if buffers.len() != self.path.channels.len() {
                return Err(Error::ChannelConfig);
            }
            match buffers.len() {
                1 => with_timeout(self.write_single(buffers[0]), timeout).await,
                2 => with_timeout(self.write_dual(buffers), timeout).await,
                _ => Err(Error::ChannelConfig),
            }
        })
    }

    async fn write_single(&mut self, buffer: &[Complex32]) -> Result<usize> {
        let mut raw = vec![0i16; buffer.len() * 2];
        c32_to_q11(buffer, &mut raw);
        self.write_raw(bytemuck::cast_slice(&raw)).await?;
        Ok(buffer.len())
    }

    async fn write_dual(&mut self, buffers: &[&[Complex32]]) -> Result<usize> {
        let [buf0, buf1] = buffers else {
            return Err(Error::ChannelConfig);
        };
        if buf0.len() != buf1.len() {
            return Err(Error::ChannelConfig);
        }
        let chunk = buf0.len();
        let mut raw = vec![0i16; chunk * 4];
        for (i, (s0, s1)) in buf0.iter().zip(buf1.iter()).enumerate() {
            let base = i * 4;
            raw[base] = clamp_q11(s0.re);
            raw[base + 1] = clamp_q11(s0.im);
            raw[base + 2] = clamp_q11(s1.re);
            raw[base + 3] = clamp_q11(s1.im);
        }
        self.write_raw(bytemuck::cast_slice(&raw)).await?;
        Ok(chunk)
    }

    /// Send bytes over the sample pipe, applying backpressure once more than
    /// `MAX_PENDING` transfers are queued with the OS.
    async fn write_raw(&mut self, bytes: &[u8]) -> Result<()> {
        let mut sent = 0;
        while sent < bytes.len() {
            let end = (sent + self.transfer_size).min(bytes.len());
            self.path.endpoint.submit(bytes[sent..end].to_vec().into());
            sent = end;
            while self.path.endpoint.pending() >= MAX_PENDING {
                usb::next_out(&mut self.path.endpoint).await?;
            }
        }
        Ok(())
    }
}

// ==================== Tests ====================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q11_to_c32_scales_correctly() {
        let input = [0i16, 2048, -2048, 1024];
        let mut output = [Complex32::ZERO; 2];
        q11_to_c32(&input, &mut output);
        assert_eq!(output[0], Complex32::new(0.0, 1.0));
        assert_eq!(output[1], Complex32::new(-1.0, 0.5));
    }

    #[test]
    fn c32_to_q11_scales_and_clamps() {
        let input = [Complex32::new(0.0, 0.5), Complex32::new(2.0, -2.0)];
        let mut output = [0i16; 4];
        c32_to_q11(&input, &mut output);
        assert_eq!(output, [0, 1024, 2047, -2048]);
    }

    #[test]
    fn conversion_roundtrip_for_representable_samples() {
        let input = [Complex32::new(0.5, -0.25), Complex32::new(-0.5, 0.25)];
        let mut q11 = [0i16; 4];
        c32_to_q11(&input, &mut q11);
        let mut roundtrip = [Complex32::ZERO; 2];
        q11_to_c32(&q11, &mut roundtrip);
        assert_eq!(input, roundtrip);
    }

    #[test]
    fn extend_i16_le_decodes_little_endian() {
        let mut out = Vec::new();
        extend_i16_le(&mut out, &[0x01, 0x00, 0x00, 0x80]);
        assert_eq!(out, [1, -32768]);
    }
}
