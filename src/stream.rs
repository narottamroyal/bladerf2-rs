//! Sample streaming (SC16 Q11 over USB bulk endpoints).
//!
//! Samples are signed 16-bit I/Q pairs holding 12-bit values, scaled to
//! `[-1.0, 1.0)` by dividing by 2048. Dual-channel streams interleave both
//! channels on the wire (`I0, Q0, I1, Q1, ...`).
//!
//! While timestamps are on (see
//! [`RfLinkSession::set_timestamps`](crate::RfLinkSession::set_timestamps)),
//! RX samples arrive in messages of one FX3 DMA buffer each. Each message
//! starts with a 16-byte header holding the timestamp of its first sample;
//! [`RxStream`] removes the headers and reports the timestamps and any
//! samples the FPGA dropped.
//!
//! Every operation returns a [`MaybeFuture`]: call `.wait()` to block, or
//! `.await` it from async code.

use crate::channel::{Channel, Direction};
use crate::drivers::{fpga, rfic};
use crate::error::{Error, Result};
use crate::maybe_future::{self, with_timeout};
use crate::nios::NiosCore;
use crate::usb::{self, EndpointAddress, UsbTransport};
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

// ==================== Metadata messages ====================

/// Size of a metadata message header.
const HEADER_SIZE: usize = 16;

/// A metadata message header: bytes 0-3 are
/// reserved, 4-11 hold the timestamp of the message's first sample and 12-15
/// the flags, all little-endian.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Header {
    timestamp: u64,
    /// Bit 16 is the J51-1 level at the first sample and bit 0 the TX
    /// underrun latch. The FPGA sets no flag for an RX overrun; it shows only
    /// as a timestamp jump.
    #[cfg_attr(not(test), allow(dead_code))]
    flags: u32,
}

impl Header {
    fn parse(message: &[u8]) -> Self {
        Self {
            timestamp: u64::from_le_bytes(*message[4..].first_chunk().expect("whole header")),
            flags: u32::from_le_bytes(*message[12..].first_chunk().expect("whole header")),
        }
    }
}

/// How samples sit in received RX transfers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Format {
    /// Samples only.
    Raw,
    /// Messages of `message` bytes, each a header followed by samples. A
    /// transfer holds whole messages, so a partial
    /// message at the end of a short transfer is ignored.
    Metadata { message: usize },
}

impl Format {
    /// Sample bytes in a transfer of `len` bytes, counting whole samples only.
    fn payload_len(self, len: usize) -> usize {
        match self {
            Self::Raw => len / 4 * 4,
            Self::Metadata { message } => len / message * (message - HEADER_SIZE),
        }
    }

    /// The contiguous bytes of `transfer` holding sample bytes from `offset`
    /// (counted in sample bytes) up to the end of their message.
    fn segment(self, transfer: &[u8], offset: usize) -> &[u8] {
        match self {
            Self::Raw => &transfer[offset..self.payload_len(transfer.len())],
            Self::Metadata { message } => {
                let payload = message - HEADER_SIZE;
                let base = offset / payload * message;
                &transfer[base + HEADER_SIZE + offset % payload..base + message]
            }
        }
    }

    /// The headers of a transfer's messages.
    fn headers(self, transfer: &[u8]) -> impl Iterator<Item = Header> + '_ {
        let message = match self {
            Self::Raw => None,
            Self::Metadata { message } => Some(message),
        };
        message
            .into_iter()
            .flat_map(|message| transfer.chunks_exact(message))
            .map(Header::parse)
    }

    /// Timestamp of the sample at `offset` (in sample bytes) in `transfer`,
    /// for a stream of `channels` channels, or `None` for raw samples.
    fn timestamp_at(self, transfer: &[u8], offset: usize, channels: usize) -> Option<u64> {
        let Self::Metadata { message } = self else {
            return None;
        };
        let payload = message - HEADER_SIZE;
        let header = Header::parse(&transfer[offset / payload * message..]);
        Some(header.timestamp + (offset % payload / 4 / channels) as u64)
    }
}

/// Counts samples the FPGA dropped, from the jumps between consecutive
/// message timestamps.
#[derive(Debug, Clone, Copy)]
struct LossCounter {
    /// Timestamp advance per message: its samples per channel.
    advance: u64,
    /// Timestamp the next message should carry.
    next: Option<u64>,
    /// Samples per channel lost so far.
    lost: u64,
}

impl LossCounter {
    fn new(format: Format, channels: usize) -> Self {
        let advance = match format {
            Format::Raw => 0,
            Format::Metadata { message } => ((message - HEADER_SIZE) / 4 / channels) as u64,
        };
        Self {
            advance,
            next: None,
            lost: 0,
        }
    }

    fn message(&mut self, header: Header) {
        if let Some(next) = self.next {
            self.lost += header.timestamp.saturating_sub(next);
        }
        self.next = Some(header.timestamp + self.advance);
    }
}

/// Converts samples from the sample bytes of `transfer`, starting at `offset`
/// (in sample bytes), passing each to `put`, until `count` samples have been
/// converted or the transfer is used up. Returns the number converted and
/// advances `offset` past them.
fn take_samples(
    format: Format,
    transfer: &[u8],
    offset: &mut usize,
    count: usize,
    mut put: impl FnMut(usize, Complex32),
) -> usize {
    let end = format.payload_len(transfer.len());
    let mut taken = 0;
    while taken < count && *offset < end {
        let samples = format.segment(transfer, *offset).as_chunks::<4>().0;
        let n = samples.len().min(count - taken);
        for (sample, k) in samples[..n].iter().zip(taken..) {
            put(k, q11_to_c32(sample));
        }
        taken += n;
        *offset += n * 4;
    }
    taken
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
    /// Sample bytes of the oldest received transfer that have already been
    /// read.
    offset: usize,
    /// Format of the received transfers, set on activation.
    format: Format,
    /// Samples lost since activation.
    losses: LossCounter,
    /// Timestamp of the first sample of the last read.
    first_timestamp: Option<u64>,
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
            format: Format::Raw,
            losses: LossCounter::new(Format::Raw, channels.len()),
            first_timestamp: None,
        })
    }

    /// Number of channels carried by this stream.
    pub fn num_channels(&self) -> usize {
        self.path.channels.len()
    }

    /// Recommended maximum samples per channel per `read` call: the samples
    /// per channel in one USB transfer. Once the stream is activated with
    /// timestamps on, this leaves out the message headers.
    pub fn mtu(&self) -> usize {
        self.format.payload_len(TRANSFER_SIZE) / 4 / self.path.channels.len()
    }

    /// Activate the stream (enables the RFIC modules and USB RX path). The
    /// stream receives timestamps if they are on now.
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            self.format = match fpga::metadata_message_size(&self.path.nios).await? {
                Some(message) => Format::Metadata { message },
                None => Format::Raw,
            };
            self.losses = LossCounter::new(self.format, self.num_channels());
            self.first_timestamp = None;
            self.path.start().await
        })
    }

    /// Timestamp of the first sample of the last [`read`](Self::read), in
    /// sample periods (one sample per channel), or `None` if the stream was
    /// activated with timestamps off or nothing has been read.
    pub fn first_timestamp(&self) -> Option<u64> {
        self.first_timestamp
    }

    /// Samples per channel the FPGA dropped since activation, because they
    /// were not read fast enough. Always 0 with timestamps off, where drops
    /// cannot be seen. Counted as messages are received, so it covers samples
    /// received but not yet read.
    pub fn lost(&self) -> u64 {
        self.losses.lost
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

    /// Unread sample bytes in the received transfers, counting whole samples
    /// only.
    fn buffered(&self) -> usize {
        let received: usize = self
            .received
            .iter()
            .map(|t| self.format.payload_len(t.len()))
            .sum();
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
            for header in self.format.headers(&transfer.buffer) {
                self.losses.message(header);
            }
            self.received.push_back(transfer.buffer);
        }
        Ok(())
    }

    /// Converts samples from the received transfers into `buffers`, which
    /// [`receive`](Self::receive) has made sure are available, and records the
    /// first one's timestamp. Transfers are submitted again once they have
    /// been read in full.
    fn copy_out(&mut self, buffers: &mut [&mut [Complex32]]) {
        let endpoint = &mut self.path.endpoint;
        self.first_timestamp = copy_samples(
            self.format,
            &mut self.received,
            &mut self.offset,
            buffers,
            |transfer| {
                if endpoint.pending() < MAX_PENDING {
                    endpoint.submit(transfer);
                }
            },
        );
    }
}

/// Fills `buffers` from the `received` transfers, which must hold enough
/// samples, starting at sample byte `offset` of the oldest. Sample `k` on the
/// wire belongs to channel `k % channels` (see the module docs). Transfers read
/// in full are removed and passed to `done`. Returns the first sample's
/// timestamp, if the format has timestamps.
fn copy_samples<B: std::ops::Deref<Target = [u8]>>(
    format: Format,
    received: &mut VecDeque<B>,
    offset: &mut usize,
    buffers: &mut [&mut [Complex32]],
    mut done: impl FnMut(B),
) -> Option<u64> {
    let channels = buffers.len();
    let total = buffers[0].len() * channels;
    let mut first_timestamp = None;
    let mut k = 0;
    while k < total {
        let transfer = &**received.front().expect("enough samples received");
        let end = format.payload_len(transfer.len());
        if k == 0 && *offset < end {
            first_timestamp = format.timestamp_at(transfer, *offset, channels);
        }
        let start = k;
        k += take_samples(format, transfer, offset, total - k, |i, sample| {
            buffers[(start + i) % channels][(start + i) / channels] = sample;
        });

        if *offset == end {
            done(received.pop_front().expect("front exists"));
            *offset = 0;
        }
    }
    first_timestamp
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

    /// Activate the stream (enables the RFIC modules and USB TX path). Fails
    /// with [`Error::TxTimestamps`] while timestamps are on.
    pub fn activate(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        maybe_future::operation(async move {
            if fpga::metadata_message_size(&self.path.nios)
                .await?
                .is_some()
            {
                return Err(Error::TxTimestamps);
            }
            self.path.start().await
        })
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

    /// A metadata message of `message` bytes: a header with `timestamp` and
    /// `flags`, then samples whose I values count up from `first`.
    fn message(message: usize, timestamp: u64, flags: u32, first: i16) -> Vec<u8> {
        let mut bytes = 0x1234_4321_u32.to_le_bytes().to_vec();
        bytes.extend(timestamp.to_le_bytes());
        bytes.extend(flags.to_le_bytes());
        for i in 0..(message - HEADER_SIZE) / 4 {
            bytes.extend((first + i as i16).to_le_bytes());
            bytes.extend(0_i16.to_le_bytes());
        }
        bytes
    }

    #[test]
    fn header_holds_timestamp_and_flags() {
        let bytes = message(8192, 0x0102_0304_0506_0708, 0x0001_0001, 0);
        assert_eq!(&bytes[4..12], &[8, 7, 6, 5, 4, 3, 2, 1]);
        assert_eq!(
            Header::parse(&bytes),
            Header {
                timestamp: 0x0102_0304_0506_0708,
                flags: 0x0001_0001,
            }
        );
    }

    #[test]
    fn metadata_payload_excludes_headers() {
        let format = Format::Metadata { message: 8192 };
        assert_eq!(format.payload_len(16384), 2 * 8176);
        assert_eq!(Format::Raw.payload_len(16386), 16384);
    }

    /// Samples are taken across message and transfer boundaries with every
    /// header removed, and each read's first timestamp is that of its first
    /// sample, per channel.
    #[test]
    fn headers_are_stripped_across_messages() {
        const MESSAGE: usize = 64; // 12 samples after the header
        let format = Format::Metadata { message: MESSAGE };
        let mut transfer = message(MESSAGE, 100, 0, 0);
        transfer.extend(message(MESSAGE, 106, 0, 12));
        let next = message(MESSAGE, 112, 0, 24);
        let transfers = [transfer, next];

        let mut out = Vec::new();
        let mut offset = 4 * 10; // part-way into the first message
        assert_eq!(format.timestamp_at(&transfers[0], offset, 2), Some(105));
        let taken = take_samples(format, &transfers[0], &mut offset, 100, |k, sample| {
            assert_eq!(k, out.len());
            out.push(sample.re);
        });
        assert_eq!(taken, 14);
        assert_eq!(offset, format.payload_len(transfers[0].len()));

        let mut offset = 0;
        assert_eq!(format.timestamp_at(&transfers[1], offset, 2), Some(112));
        take_samples(format, &transfers[1], &mut offset, 4, |_, sample| {
            out.push(sample.re)
        });
        assert_eq!(offset, 16);
        assert_eq!(format.timestamp_at(&transfers[1], offset, 2), Some(114));

        let expected: Vec<f32> = (10..28).map(|i| i as f32 / Q11_SCALE).collect();
        assert_eq!(out, expected);
    }

    /// One read spanning two queued transfers takes its first timestamp from
    /// the read's first sample, hands back each transfer read in full, and
    /// leaves the rest for the next read. Losses count across transfers.
    #[test]
    fn reads_span_transfers() {
        const MESSAGE: usize = 64; // 6 samples per channel per message
        let format = Format::Metadata { message: MESSAGE };
        let first = [message(MESSAGE, 0, 0, 0), message(MESSAGE, 6, 0, 12)].concat();
        let second = [message(MESSAGE, 20, 0, 24), message(MESSAGE, 26, 0, 36)].concat();
        let mut losses = LossCounter::new(format, 2);
        for header in format.headers(&first).chain(format.headers(&second)) {
            losses.message(header);
        }
        assert_eq!(losses.lost, 8);

        let mut received = VecDeque::from([first, second]);
        let mut offset = 0;
        let mut done = 0;
        let (mut rx0, mut rx1) = ([Complex32::ZERO; 4], [Complex32::ZERO; 4]);
        let first_ts = copy_samples(
            format,
            &mut received,
            &mut offset,
            &mut [&mut rx0, &mut rx1],
            |_| done += 1,
        );
        assert_eq!(first_ts, Some(0));

        let (mut rx0, mut rx1) = ([Complex32::ZERO; 10], [Complex32::ZERO; 10]);
        let next_ts = copy_samples(
            format,
            &mut received,
            &mut offset,
            &mut [&mut rx0, &mut rx1],
            |_| done += 1,
        );
        assert_eq!(next_ts, Some(4));
        assert_eq!(done, 1);
        assert_eq!(received.len(), 1);
        assert_eq!(offset, 4 * 4);
        // Channel 0 takes the even samples: 8, 10, then 12.. from message 2.
        let i: Vec<f32> = rx0.iter().map(|s| s.re * Q11_SCALE).collect();
        assert_eq!(
            i,
            [8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0]
        );
        assert_eq!(format.timestamp_at(&received[0], offset, 2), Some(22));
    }

    #[test]
    fn raw_samples_are_taken_whole() {
        let transfer: Vec<u8> = (0..6_i16)
            .flat_map(|i| [i.to_le_bytes(), [0, 0]].concat())
            .collect();
        let mut offset = 4;
        let mut out = Vec::new();
        let taken = take_samples(Format::Raw, &transfer[..23], &mut offset, 10, |_, s| {
            out.push(s.re)
        });
        assert_eq!(taken, 4);
        assert_eq!(out, [1.0, 2.0, 3.0, 4.0].map(|i: f32| i / Q11_SCALE));
        assert_eq!(Format::Raw.timestamp_at(&transfer, 0, 1), None);
    }

    /// Two channels advance the timestamp by 1022 per 8 KiB message; a later
    /// header counts the jump beyond that as lost samples.
    #[test]
    fn jumps_count_as_lost_samples() {
        let format = Format::Metadata { message: 8192 };
        let mut losses = LossCounter::new(format, 2);
        assert_eq!(losses.advance, 1022);
        assert_eq!(LossCounter::new(format, 1).advance, 2044);

        let mut transfer = message(8192, 5000, 0, 0);
        transfer.extend(message(8192, 6022, 0, 0));
        transfer.extend(message(8192, 7144, 0, 0));
        for header in format.headers(&transfer) {
            losses.message(header);
        }
        assert_eq!(losses.lost, 100);
        assert_eq!(losses.next, Some(8166));
        assert_eq!(Format::Raw.headers(&transfer).count(), 0);
    }

    #[test]
    fn conversion_roundtrip_for_representable_samples() {
        for sample in [Complex32::new(0.5, -0.25), Complex32::new(-0.5, 0.25)] {
            assert_eq!(q11_to_c32(&c32_to_q11(sample)), sample);
        }
    }
}
