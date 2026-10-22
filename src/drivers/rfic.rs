//! AD9361 RFIC control via the NIOS protocol (frequency, gain, sample rate,
//! bandwidth, filters, channel enable).

use std::ops::RangeInclusive;
use std::time::Duration;

use crate::channel::{Channel, Direction};
use crate::error::{Error, Result};
use crate::maybe_future;
use crate::nios::{Nios16x64, NiosCore};

const CHANNEL_ALL: u16 = 0xFFFF;

#[repr(u16)]
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    Status = 0,
    Init = 1,
    Enable = 2,
    SampleRate = 3,
    Frequency = 4,
    Bandwidth = 5,
    GainMode = 6,
    Gain = 7,
    Rssi = 8,
    Filter = 9,
    TxMute = 10,
    FastLock = 11,
}

// ==================== Initialization ====================

#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum State {
    #[default]
    Off = 0,
    On = 1,
    Standby = 2,
}

impl From<State> for u64 {
    fn from(state: State) -> u64 {
        state as u64
    }
}

impl TryFrom<u64> for State {
    type Error = Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            x if x == State::Off.into() => Ok(State::Off),
            x if x == State::On.into() => Ok(State::On),
            x if x == State::Standby.into() => Ok(State::Standby),
            _ => Err(Error::ValueConversion),
        }
    }
}

async fn get_state(nios: &NiosCore) -> Result<State> {
    read(nios, CHANNEL_ALL, Command::Init).await?.try_into()
}

/// Put the RFIC into the "On" state if it is off or in standby.
pub async fn init(nios: &NiosCore) -> Result<()> {
    match get_state(nios).await? {
        State::Off | State::Standby => {
            write(nios, CHANNEL_ALL, Command::Init, State::On.into()).await
        }
        State::On => Ok(()),
    }
}

// ==================== Bandwidth ====================

/// Supported bandwidth range in Hz.
pub fn get_bandwidth_range() -> RangeInclusive<u64> {
    200_000..=56_000_000
}

/// Current bandwidth for a direction, in Hz.
pub async fn get_bandwidth(nios: &NiosCore, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Bandwidth).await
}

/// Set the bandwidth for a direction, in Hz.
pub async fn set_bandwidth(nios: &NiosCore, direction: Direction, bandwidth: u64) -> Result<()> {
    if !get_bandwidth_range().contains(&bandwidth) {
        return Err(Error::OutOfRange);
    }
    write(nios, direction, Command::Bandwidth, bandwidth).await
}

// ==================== Frequency ====================

/// Supported frequency range for a direction, in Hz.
pub fn get_frequency_range(direction: Direction) -> RangeInclusive<u64> {
    match direction {
        Direction::Rx => 70_000_000..=6_000_000_000,
        Direction::Tx => 47_000_000..=6_000_000_000,
    }
}

/// Current frequency for a direction, in Hz.
pub async fn get_frequency(nios: &NiosCore, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Frequency).await
}

/// Set the frequency for a direction, in Hz.
pub async fn set_frequency(nios: &NiosCore, direction: Direction, frequency: u64) -> Result<()> {
    if !get_frequency_range(direction).contains(&frequency) {
        return Err(Error::OutOfRange);
    }
    write(nios, direction, Command::Frequency, frequency).await
}

// ==================== Gain ====================

/// Gain range information, dependent on frequency.
#[derive(Debug, Clone)]
pub struct GainRange {
    /// Frequency range this entry applies to.
    pub freq_range: RangeInclusive<u64>,
    /// Valid gain range in dB.
    pub gain_range: RangeInclusive<f64>,
    /// Offset between user-facing dB and the raw RFIC value.
    pub offset: f64,
}

const RX_RANGES: &[GainRange] = &[
    GainRange {
        freq_range: 0..=1_300_000_000,
        gain_range: -16.0..=60.0,
        offset: -17.0,
    },
    GainRange {
        freq_range: 1_300_000_001..=4_000_000_000,
        gain_range: -15.0..=60.0,
        offset: -11.0,
    },
    GainRange {
        freq_range: 4_000_000_001..=6_000_000_000,
        gain_range: -12.0..=60.0,
        offset: -2.0,
    },
];

const TX_RANGE: GainRange = GainRange {
    freq_range: 47_000_000..=6_000_000_000,
    gain_range: -23.75..=66.0,
    offset: 66.0,
};

/// Gain range for a direction, based on the current frequency.
pub async fn gain_range(nios: &NiosCore, direction: Direction) -> Result<&'static GainRange> {
    let frequency = get_frequency(nios, direction).await?;
    let range = match direction {
        Direction::Rx => RX_RANGES
            .iter()
            .find(|r| r.freq_range.contains(&frequency))
            .ok_or(Error::OutOfRange)?,
        Direction::Tx => &TX_RANGE,
    };
    Ok(range)
}

/// Current gain for a channel, in dB.
pub async fn get_gain(nios: &NiosCore, channel: Channel) -> Result<f64> {
    let mut gain = read(nios, channel, Command::Gain).await? as f64;
    if channel.direction() == Direction::Tx {
        // TX gain register is in millidecibels with an inverted sign
        gain *= -1e-3;
    }
    gain += gain_range(nios, channel.direction()).await?.offset;
    Ok(gain)
}

/// Set the gain for a channel, in dB.
pub async fn set_gain(nios: &NiosCore, channel: Channel, mut gain: f64) -> Result<()> {
    let range = gain_range(nios, channel.direction()).await?;
    if !range.gain_range.contains(&gain) {
        return Err(Error::OutOfRange);
    }

    gain -= range.offset;
    if channel.direction() == Direction::Tx {
        gain *= -1e3;
    }

    // RX gain writes require manual gain mode
    if channel.direction() == Direction::Rx
        && get_gain_mode(nios, channel).await? != GainMode::Manual
    {
        set_gain_mode(nios, channel, GainMode::Manual).await?;
    }

    write(nios, channel, Command::Gain, gain.round() as u64).await?;

    if channel.direction() == Direction::Rx {
        apply_rx_gain(nios).await?;
    }
    Ok(())
}

/// Make a freshly written RX gain take effect (and therefore read back)
/// immediately.
///
/// The RFIC only applies a new RX gain while an RX channel is enabled; with
/// every RX channel off, [`get_gain`] would keep returning the previous value
/// until the next stream starts. Briefly enabling and disabling an RX channel
/// makes the firmware apply it.
///
/// If an RX channel is already enabled the enable state is left alone, since
/// toggling a channel next to a running stream could disturb it.
async fn apply_rx_gain(nios: &NiosCore) -> Result<()> {
    for channel in [Channel::Rx0, Channel::Rx1] {
        if read(nios, channel, Command::Enable).await? != 0 {
            return Ok(());
        }
    }
    let enabled = enable_module(nios, Channel::Rx0, true).await;
    let disabled = enable_module(nios, Channel::Rx0, false).await;
    enabled.and(disabled)
}

// ==================== Gain mode (RX AGC) ====================

/// AGC mode for RX channels.
#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GainMode {
    /// Manual gain control.
    Manual = 1,
    /// AGC with fast attack.
    FastAttack = 2,
    /// AGC with slow attack.
    #[default]
    SlowAttack = 3,
    /// AGC with hybrid attack.
    Hybrid = 4,
}

impl From<GainMode> for u64 {
    fn from(value: GainMode) -> Self {
        value as u64
    }
}

impl TryFrom<u64> for GainMode {
    type Error = Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            1 => Ok(Self::Manual),
            2 => Ok(Self::FastAttack),
            3 => Ok(Self::SlowAttack),
            4 => Ok(Self::Hybrid),
            _ => Err(Error::ValueConversion),
        }
    }
}

/// Current AGC mode for an RX channel.
pub async fn get_gain_mode(nios: &NiosCore, channel: Channel) -> Result<GainMode> {
    match channel.direction() {
        Direction::Rx => read(nios, channel, Command::GainMode).await?.try_into(),
        Direction::Tx => Err(Error::OutOfRange),
    }
}

/// Set the AGC mode for an RX channel.
pub async fn set_gain_mode(nios: &NiosCore, channel: Channel, mode: GainMode) -> Result<()> {
    match channel.direction() {
        Direction::Rx => write(nios, channel, Command::GainMode, mode.into()).await,
        Direction::Tx => Err(Error::OutOfRange),
    }
}

// ==================== Sample rate ====================

/// Supported sample rate range in Hz.
pub fn get_sample_rate_range() -> RangeInclusive<u64> {
    520_834..=61_440_000
}

/// Current sample rate in Hz (all channels share one rate).
pub async fn get_sample_rate(nios: &NiosCore) -> Result<u64> {
    read(nios, Channel::Rx0, Command::SampleRate).await
}

/// Set the sample rate in Hz.
///
/// Crossing between the low-rate and high-rate regimes goes through an
/// intermediate safe rate, and the RX 4x decimation / TX interpolation filters
/// are configured to match.
pub async fn set_sample_rate(nios: &NiosCore, sample_rate: u64) -> Result<()> {
    const LOW_RATE_THRESHOLD: u64 = 2_083_334;
    const HIGH_RATE_THRESHOLD: u64 = 40_000_000;
    const INTERMEDIATE_SAFE_RATE: u64 = 30_000_000;

    if !get_sample_rate_range().contains(&sample_rate) {
        return Err(Error::OutOfRange);
    }

    // Work around a limitation that prevents direct changes between high and
    // low sample rates
    let current = get_sample_rate(nios).await?;
    if (current > HIGH_RATE_THRESHOLD && sample_rate < LOW_RATE_THRESHOLD)
        || (sample_rate > HIGH_RATE_THRESHOLD && current < LOW_RATE_THRESHOLD)
    {
        write(
            nios,
            Channel::Rx0,
            Command::SampleRate,
            INTERMEDIATE_SAFE_RATE,
        )
        .await?;
    }

    // 4x RX decimation / TX interpolation is required for low sample rates
    let (rx_mode, tx_mode) = if sample_rate <= LOW_RATE_THRESHOLD {
        (FilterMode::Ratio4, FilterMode::Ratio4)
    } else {
        (FilterMode::Ratio1, FilterMode::Bypass)
    };

    let filters = async {
        set_filter(nios, Direction::Rx, rx_mode).await?;
        set_filter(nios, Direction::Tx, tx_mode).await
    }
    .await;

    // A failed filter write can leave one direction in 4x mode and the other
    // not. Put both back to their defaults before reporting the error.
    if let Err(error) = filters {
        set_filter(nios, Direction::Rx, FilterMode::Ratio1).await?;
        set_filter(nios, Direction::Tx, FilterMode::Bypass).await?;
        return Err(error);
    }

    write(nios, Channel::Rx0, Command::SampleRate, sample_rate).await
}

// ==================== Filters ====================

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FilterMode {
    Bypass = 0,
    Custom = 1,
    Ratio1 = 2, // RX decimate by 1 / TX interpolate by 1
    Ratio2 = 3, // RX decimate by 2 / TX interpolate by 2
    Ratio4 = 4, // RX decimate by 4 / TX interpolate by 4
}

impl From<FilterMode> for u64 {
    fn from(value: FilterMode) -> Self {
        value as u64
    }
}

impl TryFrom<u64> for FilterMode {
    type Error = Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            0 => Ok(Self::Bypass),
            1 => Ok(Self::Custom),
            2 => Ok(Self::Ratio1),
            3 => Ok(Self::Ratio2),
            4 => Ok(Self::Ratio4),
            _ => Err(Error::ValueConversion),
        }
    }
}

async fn get_filter(nios: &NiosCore, direction: Direction) -> Result<FilterMode> {
    read(nios, direction, Command::Filter).await?.try_into()
}

async fn set_filter(nios: &NiosCore, direction: Direction, mode: FilterMode) -> Result<()> {
    if get_filter(nios, direction).await? == mode {
        return Ok(());
    }
    write(nios, direction, Command::Filter, mode.into()).await
}

// ==================== Channel enable ====================

async fn set_tx_mute(nios: &NiosCore, channel: Channel, mute: bool) -> Result<()> {
    match channel.direction() {
        Direction::Tx => write(nios, channel, Command::TxMute, mute.into()).await,
        Direction::Rx => Err(Error::OutOfRange),
    }
}

/// Enable or disable a channel module (mutes TX while disabled).
pub async fn enable_module(nios: &NiosCore, channel: Channel, enable: bool) -> Result<()> {
    if channel.direction() == Direction::Tx {
        set_tx_mute(nios, channel, !enable).await?;
    }
    write(nios, channel, Command::Enable, enable.into()).await
}

// ==================== Read / write helpers ====================

fn address(channel: impl Into<u16>, command: Command) -> u16 {
    ((channel.into() & 0xF) << 8) | ((command as u16) & 0xFF)
}

async fn read(nios: &NiosCore, channel: impl Into<u16>, command: Command) -> Result<u64> {
    nios.read(Nios16x64::RfIc, address(channel, command)).await
}

/// Poll the RFIC status register until the write queue drains.
async fn wait(nios: &NiosCore) -> Result<()> {
    for _ in 0..30 {
        let write_queue = (read(nios, CHANNEL_ALL, Command::Status).await? >> 8) & 0xFF;
        if write_queue == 0 {
            return Ok(());
        }
        maybe_future::sleep(Duration::from_micros(100)).await;
    }
    Err(Error::Timeout)
}

async fn write(
    nios: &NiosCore,
    channel: impl Into<u16>,
    command: Command,
    data: u64,
) -> Result<()> {
    nios.write(Nios16x64::RfIc, address(channel, command), data)
        .await?;
    wait(nios).await
}
