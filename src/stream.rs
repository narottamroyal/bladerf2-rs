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
use std::collections::VecDeque;
use std::time::Duration;

/// Size of each USB bulk transfer, in bytes. It is a multiple of every USB
/// packet size, as IN transfers require. Each WebUSB transfer is a round trip
/// through the browser, so the browser needs larger ones to keep up at high
/// sample rates.
#[cfg(not(target_arch = "wasm32"))]
const TRANSFER_SIZE: usize = 16 * 1024;
#[cfg(target_arch = "wasm32")]
const TRANSFER_SIZE: usize = 128 * 1024;
/// Most transfers queued with the OS, to keep the pipe full.
const MAX_PENDING: usize = 16;

/// Longest `deactivate` waits for each queued transfer to finish.
const COLLECT_TIMEOUT: Duration = Duration::from_secs(1);

/// Q11 full-scale: samples span `[-2048, 2047]`.
const Q11_SCALE: f32 = 2048.0;

// ==================== Sample format conversion ====================

fn clamp_q11(value: f32) -> i16 {
    (value * Q11_SCALE)
        .round()
        .clamp(-Q11_SCALE, Q11_SCALE - 1.0) as i16
}

/// Convert one SC16 Q11 sample (little-endian I, then Q) to [`Complex32`].
fn q11_to_c32(sample: &[u8; 4]) -> Complex32 {
    let [i0, i1, q0, q1] = *sample;
    Complex32::new(
        f32::from(i16::from_le_bytes([i0, i1])) / Q11_SCALE,
        f32::from(i16::from_le_bytes([q0, q1])) / Q11_SCALE,
    )
}

/// Convert a [`Complex32`] to one SC16 Q11 sample (little-endian I, then Q),
/// clamping it to the Q11 range.
fn c32_to_q11(sample: Complex32) -> [u8; 4] {
    let [i0, i1] = clamp_q11(sample.re).to_le_bytes();
    let [q0, q1] = clamp_q11(sample.im).to_le_bytes();
    [i0, i1, q0, q1]
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
            async { Ok(usb::next_complete(endpoint).await) },
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
    fn new(transport: &UsbTransport, nios: &NiosCore, channels: &[usize]) -> Result<Self> {
        let (direction, address) = match D::DIR {
            nusb::transfer::Direction::In => (Direction::Rx, EndpointAddress::SampleIn),
            nusb::transfer::Direction::Out => (Direction::Tx, EndpointAddress::SampleOut),
        };
        Ok(Self {
            endpoint: transport.interface().endpoint::<Bulk, D>(address as u8)?,
            transport: transport.clone(),
            nios: nios.clone(),
            direction,
            channels: parse_channels(direction, channels)?,
            active: false,
        })
    }

    /// The length of `buffers`, which must hold one buffer per channel, all of
    /// the same length.
    fn buffer_len<B: AsRef<[Complex32]>>(&self, buffers: &[B]) -> Result<usize> {
        let len = buffers.first().map_or(0, |b| b.as_ref().len());
        if buffers.len() != self.channels.len() || buffers.iter().any(|b| b.as_ref().len() != len) {
            return Err(Error::ChannelConfig);
        }
        Ok(len)
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
/// [`deactivate`](RxStream::deactivate) when done. Samples received but not yet
/// read are kept for the next `read`, so reads of any size lose nothing.
///
/// Dropping an active stream stops it on native targets. In the browser it
/// keeps running until the next stream in the same direction is activated, or
/// the device is opened again.
pub struct RxStream {
    path: Path<In>,
    /// Completed transfers that have not been read in full, oldest first.
    received: VecDeque<Buffer>,
    /// Bytes of the oldest received transfer that have already been read.
    offset: usize,
}

impl std::fmt::Debug for RxStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RxStream")
            .field("channels", &self.path.channels)
            .field("buffered_bytes", &self.buffered())
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
        Ok(Self {
            path: Path::new(transport, nios, channels)?,
            received: VecDeque::new(),
            offset: 0,
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
            self.received.clear();
            self.offset = 0;
            Ok(())
        })
    }

    /// Reads one buffer per channel. `timeout` bounds the wait for data
    /// (`None` = forever).
    ///
    /// Fills every buffer completely. Excess data received from a USB transfer
    /// is kept and returned by later calls, and a timeout discards nothing.
    pub fn read<'a>(
        &'a mut self,
        buffers: &'a mut [&'a mut [Complex32]],
        timeout: Option<Duration>,
    ) -> impl MaybeFuture<Output = Result<usize>> + 'a {
        maybe_future::operation(async move {
            let len = self.path.buffer_len(buffers)?;
            with_timeout(self.receive(len * buffers.len() * 4), timeout).await?;
            self.copy_out(buffers);
            Ok(len)
        })
    }

    /// Unread bytes in the received transfers, counting whole samples only.
    fn buffered(&self) -> usize {
        let received: usize = self.received.iter().map(|t| t.len() / 4 * 4).sum();
        received - self.offset
    }

    /// Waits until the received transfers hold at least `bytes` unread bytes,
    /// keeping up to `MAX_PENDING` transfers queued with the OS. Nothing is
    /// read here, so stopping early loses no samples.
    async fn receive(&mut self, bytes: usize) -> Result<()> {
        while self.buffered() < bytes {
            while self.path.endpoint.pending() < MAX_PENDING {
                self.path.endpoint.submit(Buffer::new(TRANSFER_SIZE));
            }
            let transfer = usb::next_complete(&mut self.path.endpoint).await;
            transfer.status?;
            self.received.push_back(transfer.buffer);
        }
        Ok(())
    }

    /// Converts samples from the received transfers into `buffers`, which
    /// [`receive`](Self::receive) has made sure are available. Sample `k` on the
    /// wire belongs to channel `k % channels` (see the module docs). Transfers
    /// are submitted again once they have been read in full.
    fn copy_out(&mut self, buffers: &mut [&mut [Complex32]]) {
        let channels = buffers.len();
        let total = buffers[0].len() * channels;
        let mut k = 0;
        while k < total {
            let transfer = self.received.front().expect("receive buffered enough");
            let samples = transfer[self.offset..].as_chunks::<4>().0;
            let count = samples.len().min(total - k);
            for (sample, k) in samples[..count].iter().zip(k..) {
                buffers[k % channels][k / channels] = q11_to_c32(sample);
            }
            k += count;
            self.offset += count * 4;

            if self.offset + 4 > transfer.len() {
                let transfer = self.received.pop_front().expect("front exists");
                self.offset = 0;
                if self.path.endpoint.pending() < MAX_PENDING {
                    self.path.endpoint.submit(transfer);
                }
            }
        }
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
        Ok(Self {
            path: Path::new(transport, nios, channels)?,
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
            let len = self.path.buffer_len(buffers)?;
            with_timeout(self.send(buffers), timeout).await?;
            Ok(len)
        })
    }

    /// Converts `buffers` straight into transfers and submits them, waiting
    /// while `MAX_PENDING` transfers are queued with the OS. Sample `k` on the
    /// wire comes from channel `k % channels` (see the module docs).
    async fn send(&mut self, buffers: &[&[Complex32]]) -> Result<()> {
        let channels = buffers.len();
        let total = buffers[0].len() * channels;
        let mut k = 0;
        while k < total {
            let count = (TRANSFER_SIZE / 4).min(total - k);
            let mut transfer = Buffer::new(count * 4);
            let samples = transfer.extend_fill(count * 4, 0).as_chunks_mut::<4>().0;
            for (sample, k) in samples.iter_mut().zip(k..) {
                *sample = c32_to_q11(buffers[k % channels][k / channels]);
            }
            self.path.endpoint.submit(transfer);
            k += count;

            while self.path.endpoint.pending() >= MAX_PENDING {
                usb::next_complete(&mut self.path.endpoint).await.status?;
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
        // I = 0, Q = 2048; I = -2048, Q = 1024 (little-endian).
        assert_eq!(
            q11_to_c32(&[0x00, 0x00, 0x00, 0x08]),
            Complex32::new(0.0, 1.0)
        );
        assert_eq!(
            q11_to_c32(&[0x00, 0xF8, 0x00, 0x04]),
            Complex32::new(-1.0, 0.5)
        );
    }

    #[test]
    fn q11_to_c32_decodes_little_endian() {
        let sample = q11_to_c32(&[0x01, 0x00, 0x00, 0x80]);
        assert_eq!(
            sample,
            Complex32::new(1.0 / Q11_SCALE, -32768.0 / Q11_SCALE)
        );
    }

    #[test]
    fn c32_to_q11_scales_and_clamps() {
        // I = 0, Q = 1024; I = 2047, Q = -2048 (little-endian).
        assert_eq!(
            c32_to_q11(Complex32::new(0.0, 0.5)),
            [0x00, 0x00, 0x00, 0x04]
        );
        assert_eq!(
            c32_to_q11(Complex32::new(2.0, -2.0)),
            [0xFF, 0x07, 0x00, 0xF8]
        );
    }

    #[test]
    fn conversion_roundtrip_for_representable_samples() {
        for sample in [Complex32::new(0.5, -0.25), Complex32::new(-0.5, 0.25)] {
            assert_eq!(q11_to_c32(&c32_to_q11(sample)), sample);
        }
    }
}
