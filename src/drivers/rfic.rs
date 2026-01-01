use std::ops::RangeInclusive;
use std::thread::sleep;
use std::time::Duration;

use crate::nios::{Nios, Nios16x64};
use crate::{BladeRf2Error, Channel, Direction, Result};

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

// ==================== INITIALIZATION ====================

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
    type Error = BladeRf2Error;

    fn try_from(value: u64) -> std::result::Result<Self, Self::Error> {
        match value {
            x if x == State::Off.into() => Ok(State::Off),
            x if x == State::On.into() => Ok(State::On),
            x if x == State::Standby.into() => Ok(State::Standby),
            _ => Err(BladeRf2Error::ValueConversion),
        }
    }
}

fn get_state(nios: &mut Nios) -> Result<State> {
    read(nios, CHANNEL_ALL, Command::Init)?.try_into()
}

pub fn init(nios: &mut Nios) -> Result<()> {
    let state = get_state(nios)?;
    match state {
        State::Off | State::Standby => write(nios, CHANNEL_ALL, Command::Init, State::On.into()),
        State::On => Ok(()),
    }
}

// ==================== BANDWIDTH ====================

pub fn get_bandwidth_range() -> RangeInclusive<u64> {
    200_000..=56_000_000
}

pub fn get_bandwidth(nios: &mut Nios, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Bandwidth)
}

pub fn set_bandwidth(nios: &mut Nios, direction: Direction, bandwidth: u64) -> Result<()> {
    if !get_bandwidth_range().contains(&bandwidth) {
        return Err(BladeRf2Error::OutOfRange);
    }
    write(nios, direction, Command::Bandwidth, bandwidth)
}

// ==================== FREQUENCY ====================

pub fn get_frequency_range(direction: Direction) -> RangeInclusive<u64> {
    match direction {
        Direction::Rx => 70_000_000..=6_000_000_000,
        Direction::Tx => 47_000_000..=6_000_000_000,
    }
}

pub fn get_frequency(nios: &mut Nios, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Frequency)
}

pub fn set_frequency(nios: &mut Nios, direction: Direction, frequency: u64) -> Result<()> {
    if !get_frequency_range(direction).contains(&frequency) {
        return Err(BladeRf2Error::OutOfRange);
    }
    write(nios, direction, Command::Frequency, frequency)
}

// ==================== GAIN ====================

pub struct GainRange {
    pub freq_range: std::ops::RangeInclusive<u64>,
    pub gain_range: std::ops::RangeInclusive<f64>,
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

fn gain_range(nios: &mut Nios, direction: Direction) -> Result<&GainRange> {
    let frequency = get_frequency(nios, direction)?;
    let range = match direction {
        Direction::Rx => RX_RANGES
            .iter()
            .find(|r| r.freq_range.contains(&frequency))
            .ok_or(BladeRf2Error::OutOfRange)?,
        Direction::Tx => &TX_RANGE,
    };
    Ok(range)
}

pub fn get_gain(nios: &mut Nios, channel: Channel) -> Result<f64> {
    let mut gain = read(nios, channel, Command::Gain)? as f64;
    if Direction::from(channel) == Direction::Tx {
        gain *= -1e-3;
    }
    gain += gain_range(nios, channel.into())?.offset;
    Ok(gain)
}

pub fn set_gain(nios: &mut Nios, channel: Channel, mut gain: f64) -> Result<()> {
    let range = gain_range(nios, channel.into())?;
    if !range.gain_range.contains(&gain) {
        return Err(BladeRf2Error::OutOfRange);
    }

    gain -= range.offset;
    if Direction::from(channel) == Direction::Tx {
        gain *= -1e3
    }

    if Direction::from(channel) == Direction::Rx
        && get_gain_mode(nios, channel)? != GainMode::Manual
    {
        set_gain_mode(nios, channel, GainMode::Manual)?;
    }

    write(nios, channel, Command::Gain, gain.round() as u64)
}

// ==================== RX GAIN MODE ====================

#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GainMode {
    /// Manual Gain Control.
    /// User must set gain values manually; internal RFIC AGC logic is disabled.
    Manual = 1,

    /// AGC with Fast Attack (Advanced).
    /// Quickly reacts to strong signal bursts to prevent clipping.
    FastAttack = 2,

    /// AGC with Slow Attack (Default).
    /// Provides smoother gain transitions for steady signals.
    #[default]
    SlowAttack = 3,

    /// AGC with Hybrid Attack (Advanced).
    /// A combination of fast and slow attack characteristics.
    Hybrid = 4,
}

impl From<GainMode> for u64 {
    fn from(value: GainMode) -> Self {
        value as u64
    }
}

impl TryFrom<u64> for GainMode {
    type Error = BladeRf2Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            1 => Ok(Self::Manual),
            2 => Ok(Self::FastAttack),
            3 => Ok(Self::SlowAttack),
            4 => Ok(Self::Hybrid),
            _ => Err(BladeRf2Error::ValueConversion),
        }
    }
}

pub fn get_gain_mode(nios: &mut Nios, channel: Channel) -> Result<GainMode> {
    match channel.into() {
        Direction::Rx => read(nios, channel, Command::GainMode)?.try_into(),
        _ => Err(BladeRf2Error::OutOfRange),
    }
}

pub fn set_gain_mode(nios: &mut Nios, channel: Channel, mode: GainMode) -> Result<()> {
    match channel.into() {
        Direction::Rx => write(nios, channel, Command::GainMode, mode.into()),
        _ => Err(BladeRf2Error::OutOfRange),
    }
}

// ==================== SAMPLE RATE ====================

pub fn get_sample_rate_range() -> RangeInclusive<u64> {
    520_834..=61_440_000
}

pub fn get_sample_rate(nios: &mut Nios) -> Result<u64> {
    // All channels share the same sample rate
    read(nios, Channel::Rx0, Command::SampleRate)
}

pub fn set_sample_rate(nios: &mut Nios, sample_rate: u64) -> Result<()> {
    const LOW_RATE_THRESHOLD: u64 = 2_083_334;
    const HIGH_RATE_THRESHOLD: u64 = 40_000_000;
    const INTERMEDIATE_SAFE_RATE: u64 = 30_000_000;

    if !get_sample_rate_range().contains(&sample_rate) {
        return Err(BladeRf2Error::OutOfRange);
    }

    // Work around bug that prevents changes between high and low sample rates
    let current = get_sample_rate(nios)?;
    if (current > HIGH_RATE_THRESHOLD && sample_rate < LOW_RATE_THRESHOLD)
        || (sample_rate > HIGH_RATE_THRESHOLD && current < LOW_RATE_THRESHOLD)
    {
        write(
            nios,
            Channel::Rx0,
            Command::SampleRate,
            INTERMEDIATE_SAFE_RATE,
        )?;
    }

    // 4x RX decimation / TX interpolation required for low sample rates
    let (rx_mode, tx_mode) = if sample_rate <= LOW_RATE_THRESHOLD {
        (FilterMode::Ratio4, FilterMode::Ratio4)
    } else {
        (FilterMode::Ratio1, FilterMode::Bypass)
    };

    // Configure filters
    set_filter(nios, Direction::Rx, rx_mode)?;
    set_filter(nios, Direction::Tx, tx_mode)?;

    // All channels share the same sample rate
    write(nios, Channel::Rx0, Command::SampleRate, sample_rate)
}

// ==================== INTERPOLATING / DECIMATING FILTERS ====================

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FilterMode {
    Bypass = 0,
    Custom = 1,
    Ratio1 = 2, // Decimate by 1 (RX) or Interpolate by 1 (TX)
    Ratio2 = 3, // Decimate by 2 (RX) or Interpolate by 2 (TX)
    Ratio4 = 4, // Decimate by 4 (RX) or Interpolate by 4 (TX)
}

impl From<FilterMode> for u64 {
    fn from(value: FilterMode) -> u64 {
        value as u64
    }
}

impl TryFrom<u64> for FilterMode {
    type Error = BladeRf2Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            0 => Ok(Self::Bypass),
            1 => Ok(Self::Custom),
            2 => Ok(Self::Ratio1),
            3 => Ok(Self::Ratio2),
            4 => Ok(Self::Ratio4),
            _ => Err(BladeRf2Error::ValueConversion),
        }
    }
}

fn get_filter(nios: &mut Nios, direction: Direction) -> Result<FilterMode> {
    read(nios, direction, Command::Filter)?.try_into()
}

fn set_filter(nios: &mut Nios, direction: Direction, mode: FilterMode) -> Result<()> {
    if get_filter(nios, direction)? == mode {
        return Ok(());
    }
    write(nios, direction, Command::Filter, mode.into())
}

// ==================== ENABLE MODULE ====================

fn set_tx_mute(nios: &mut Nios, channel: Channel, mute: bool) -> Result<()> {
    match channel.into() {
        Direction::Tx => write(nios, channel, Command::TxMute, mute.into()),
        Direction::Rx => Err(BladeRf2Error::OutOfRange),
    }
}

pub fn enable_module(nios: &mut Nios, channel: Channel, enable: bool) -> Result<()> {
    if Direction::from(channel) == Direction::Tx {
        set_tx_mute(nios, channel, !enable)?;
    }
    write(nios, channel, Command::Enable, enable.into())
}

// ==================== READ / WRITE ====================

fn address(channel: impl Into<u16>, command: Command) -> u16 {
    let channel = (channel.into() & 0xF) << 8;
    let command = (command as u16) & 0xFF;
    channel | command
}

fn read(nios: &mut Nios, channel: impl Into<u16>, command: Command) -> Result<u64> {
    let address = address(channel, command);
    nios.read(Nios16x64::RfIc, address)
}

fn wait(nios: &mut Nios) -> Result<()> {
    for _ in 0..30 {
        let write_queue = (read(nios, CHANNEL_ALL, Command::Status)? >> 8) & 0xFF;
        if write_queue == 0 {
            return Ok(());
        }
        sleep(Duration::from_micros(100));
    }
    Err(BladeRf2Error::Timeout)
}

fn write(nios: &mut Nios, channel: impl Into<u16>, command: Command, data: u64) -> Result<()> {
    let address = address(channel, command);
    nios.write(Nios16x64::RfIc, address, data)?;
    wait(nios)
}
