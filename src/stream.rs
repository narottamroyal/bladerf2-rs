//! Sample streaming (SC16 Q11 over USB bulk endpoints).
//!
//! Samples are signed 16-bit I/Q pairs holding 12-bit values, scaled to
//! `[-1.0, 1.0)` by dividing by 2048. Dual-channel streams interleave both
//! channels on the wire (`I0, Q0, I1, Q1, ...`).
//!
//! `read`/`write` are thin synchronous wrappers around `read_async`/
//! `write_async` (via the crate's blocking adapter); `activate`/`deactivate`
//! are plain `async fn`s usable from either context.

use crate::channel::{Channel, Direction};
use crate::error::{Error, Result};
use crate::maybe_future::{self, with_timeout};
use crate::nios::NiosCore;
use crate::usb::{self, Endpoint as EndpointAddress, UsbTransport};
use num_complex::Complex32;
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, In, Out};
use std::time::Duration;

/// Requested size of each USB bulk transfer, in bytes.
const TRANSFER_SIZE: usize = 8192;
/// Maximum transfers queued with the OS to keep the pipe full.
const MAX_PENDING: usize = 4;

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

// ==================== Channel enable / disable ====================

fn parse_channels(direction: Direction, channels: &[usize]) -> Result<Vec<Channel>> {
    let (ch0, ch1) = match direction {
        Direction::Rx => (Channel::Rx0, Channel::Rx1),
        Direction::Tx => (Channel::Tx0, Channel::Tx1),
    };
    match channels {
        [0] => Ok(vec![ch0]),
        [1] => Ok(vec![ch1]),
        [0, 1] => Ok(vec![ch0, ch1]),
        _ => Err(Error::ChannelConfig),
    }
}

/// Enable/disable RFIC channels and the USB data path, in the order that
/// avoids glitches (RFIC first on bring-up, USB path first on teardown).
async fn set_path_enabled(
    nios: &NiosCore,
    transport: &UsbTransport,
    channels: &[Channel],
    direction: Direction,
    enable: bool,
) -> Result<()> {
    if enable {
        for &channel in channels {
            crate::drivers::rfic::enable_module(nios, channel, true).await?;
        }
        transport.enable_module(direction, true).await
    } else {
        transport.enable_module(direction, false).await?;
        for &channel in channels {
            crate::drivers::rfic::enable_module(nios, channel, false).await?;
        }
        Ok(())
    }
}

/// Best-effort teardown for `Drop`, where errors cannot be reported.
fn teardown(transport: UsbTransport, nios: NiosCore, channels: Vec<Channel>, direction: Direction) {
    let cleanup = async move {
        let _ = set_path_enabled(&nios, &transport, &channels, direction, false).await;
    };
    #[cfg(not(target_arch = "wasm32"))]
    maybe_future::block_on(cleanup);
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(cleanup);
}

// ==================== RxStream ====================

/// Receive sample stream.
///
/// Call [`activate`](RxStream::activate) before reading and
/// [`deactivate`](RxStream::deactivate) when done. Dropping the stream
/// deactivates it best-effort. Received-but-unconsumed samples are buffered
/// across `read` calls, so short transfers never lose data.
pub struct RxStream {
    endpoint: nusb::Endpoint<Bulk, In>,
    in_size: usize,
    transport: UsbTransport,
    nios: NiosCore,
    channels: Vec<Channel>,
    /// Samples received from USB but not yet handed to the caller.
    pending: Vec<i16>,
}

impl std::fmt::Debug for RxStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RxStream")
            .field("channels", &self.channels)
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
        let endpoint = transport
            .interface()
            .endpoint::<Bulk, In>(EndpointAddress::SampleIn as u8)?;
        let in_size = usb::aligned_size(TRANSFER_SIZE, endpoint.max_packet_size());
        Ok(Self {
            endpoint,
            in_size,
            transport: transport.clone(),
            nios: nios.clone(),
            channels: parse_channels(Direction::Rx, channels)?,
            pending: Vec::with_capacity(TRANSFER_SIZE / 2),
        })
    }

    /// Number of channels carried by this stream.
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }

    /// Recommended maximum samples per channel per `read` call.
    pub fn mtu(&self) -> usize {
        TRANSFER_SIZE / 4 / self.channels.len()
    }

    /// Activate the stream (enables the RFIC modules and USB RX path).
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            set_path_enabled(
                &self.nios,
                &self.transport,
                &self.channels,
                Direction::Rx,
                true,
            )
            .await
        })
    }

    /// Deactivate the stream, draining pending transfers first.
    pub fn deactivate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            // Drain any pending input transfers so the endpoint queue is clean on drop
            while self.endpoint.pending() > 0 {
                let _ = usb::next_in(&mut self.endpoint).await;
            }
            set_path_enabled(
                &self.nios,
                &self.transport,
                &self.channels,
                Direction::Rx,
                false,
            )
            .await
        })
    }

    /// Blocking read; `timeout` bounds the wait for data (`None` = forever).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn read(
        &mut self,
        buffers: &mut [&mut [Complex32]],
        timeout: Option<Duration>,
    ) -> Result<usize> {
        maybe_future::block_on(self.read_async(buffers, timeout))
    }

    /// Asynchronous read; `timeout` bounds the wait for data (`None` = forever).
    ///
    /// Fills every buffer completely. Excess data received from a USB transfer
    /// is buffered and returned by later calls.
    pub async fn read_async(
        &mut self,
        buffers: &mut [&mut [Complex32]],
        timeout: Option<Duration>,
    ) -> Result<usize> {
        assert_eq!(
            self.channels.len(),
            buffers.len(),
            "buffer count must match channel count"
        );
        match buffers.len() {
            1 => with_timeout(self.read_single(buffers[0]), timeout).await,
            2 => with_timeout(self.read_dual(buffers), timeout).await,
            _ => Err(Error::ChannelConfig),
        }
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
        while self.endpoint.pending() < MAX_PENDING {
            self.endpoint.submit(Buffer::new(self.in_size));
        }
        let buffer = usb::next_in(&mut self.endpoint).await?;
        if buffer.is_empty() {
            // A zero-length packet mid-stream is unexpected.
            return Err(Error::Timeout);
        }
        extend_i16_le(&mut self.pending, &buffer);
        Ok(())
    }
}

impl Drop for RxStream {
    fn drop(&mut self) {
        teardown(
            self.transport.clone(),
            self.nios.clone(),
            self.channels.clone(),
            Direction::Rx,
        );
    }
}

// ==================== TxStream ====================

/// Transmit sample stream.
///
/// Call [`activate`](TxStream::activate) before writing and
/// [`deactivate`](TxStream::deactivate) when done (deactivate drains queued
/// samples first). Dropping the stream deactivates it best-effort.
pub struct TxStream {
    endpoint: nusb::Endpoint<Bulk, Out>,
    transfer_size: usize,
    transport: UsbTransport,
    nios: NiosCore,
    channels: Vec<Channel>,
}

impl std::fmt::Debug for TxStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TxStream")
            .field("channels", &self.channels)
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
        let endpoint = transport
            .interface()
            .endpoint::<Bulk, Out>(EndpointAddress::SampleOut as u8)?;
        let transfer_size = usb::aligned_size(TRANSFER_SIZE, endpoint.max_packet_size());
        Ok(Self {
            endpoint,
            transfer_size,
            transport: transport.clone(),
            nios: nios.clone(),
            channels: parse_channels(Direction::Tx, channels)?,
        })
    }

    /// Number of channels carried by this stream.
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }

    /// Recommended maximum samples per channel per `write` call.
    pub fn mtu(&self) -> usize {
        TRANSFER_SIZE / 4 / self.channels.len()
    }

    /// Activate the stream (enables the RFIC modules and USB TX path).
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            set_path_enabled(
                &self.nios,
                &self.transport,
                &self.channels,
                Direction::Tx,
                true,
            )
            .await
        })
    }

    /// Deactivate the stream, draining queued samples first so the tail of a
    /// burst is not dropped.
    pub fn deactivate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            while self.endpoint.pending() > 0 {
                usb::next_out(&mut self.endpoint).await?;
            }
            set_path_enabled(
                &self.nios,
                &self.transport,
                &self.channels,
                Direction::Tx,
                false,
            )
            .await
        })
    }

    /// Blocking write; `timeout` bounds the wait for buffer space (`None` = forever).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn write(&mut self, buffers: &[&[Complex32]], timeout: Option<Duration>) -> Result<usize> {
        maybe_future::block_on(self.write_async(buffers, timeout))
    }

    /// Asynchronous write; `timeout` bounds the wait for buffer space (`None` = forever).
    pub async fn write_async(
        &mut self,
        buffers: &[&[Complex32]],
        timeout: Option<Duration>,
    ) -> Result<usize> {
        assert_eq!(
            self.channels.len(),
            buffers.len(),
            "buffer count must match channel count"
        );
        match buffers.len() {
            1 => with_timeout(self.write_single(buffers[0]), timeout).await,
            2 => with_timeout(self.write_dual(buffers), timeout).await,
            _ => Err(Error::ChannelConfig),
        }
    }

    async fn write_single(&mut self, buffer: &[Complex32]) -> Result<usize> {
        let mut raw = vec![0i16; self.mtu() * 2];
        let chunk = buffer.len();
        c32_to_q11(buffer, &mut raw[..chunk * 2]);
        self.write_raw(bytemuck::cast_slice(&raw[..chunk * 2]))
            .await?;
        Ok(chunk)
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
            self.endpoint.submit(bytes[sent..end].to_vec().into());
            sent = end;
            while self.endpoint.pending() >= MAX_PENDING {
                usb::next_out(&mut self.endpoint).await?;
            }
        }
        Ok(())
    }
}

impl Drop for TxStream {
    fn drop(&mut self) {
        teardown(
            self.transport.clone(),
            self.nios.clone(),
            self.channels.clone(),
            Direction::Tx,
        );
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
