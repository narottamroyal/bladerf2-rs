//! AD9361 RFIC control via the NIOS protocol (frequency, gain, sample rate,
//! bandwidth, filters, channel enable).

use std::ops::RangeInclusive;
use std::time::Duration;

use crate::channel::{Channel, Direction};
use crate::error::{Error, Result};
use crate::maybe_future;
use crate::nios::{Nios16x64, NiosCore};

const CHANNEL_ALL: u16 = 0xFFFF;

/// Status word bit: the last job in the write queue succeeded.
const STATUS_LAST_JOB_OK: u64 = 1 << 1;

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

impl Command {
    fn name(self) -> &'static str {
        match self {
            Self::Status => "status",
            Self::Init => "init",
            Self::Enable => "enable",
            Self::SampleRate => "sample rate",
            Self::Frequency => "frequency",
            Self::Bandwidth => "bandwidth",
            Self::GainMode => "gain mode",
            Self::Gain => "gain",
            Self::Rssi => "RSSI",
            Self::Filter => "filter",
            Self::TxMute => "TX mute",
            Self::FastLock => "fast lock",
        }
    }
}

// ==================== Initialization ====================

#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Off = 0,
    On = 1,
    Standby = 2,
}

impl TryFrom<u64> for State {
    type Error = Error;

    fn try_from(value: u64) -> Result<Self> {
        match value {
            0 => Ok(Self::Off),
            1 => Ok(Self::On),
            2 => Ok(Self::Standby),
            _ => Err(Error::ValueConversion),
        }
    }
}

async fn state(nios: &NiosCore) -> Result<State> {
    read(nios, CHANNEL_ALL, Command::Init).await?.try_into()
}

/// Put the RFIC into the "On" state if it is off or in standby.
pub(crate) async fn init(nios: &NiosCore) -> Result<()> {
    match state(nios).await? {
        State::Off | State::Standby => {
            write(nios, CHANNEL_ALL, Command::Init, State::On as u64).await
        }
        State::On => Ok(()),
    }
}

// ==================== Bandwidth ====================

/// Supported bandwidth range in Hz.
pub(crate) fn bandwidth_range() -> RangeInclusive<u64> {
    200_000..=56_000_000
}

/// Current bandwidth for a direction, in Hz.
pub(crate) async fn bandwidth(nios: &NiosCore, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Bandwidth).await
}

/// Set the bandwidth for a direction, in Hz.
pub(crate) async fn set_bandwidth(
    nios: &NiosCore,
    direction: Direction,
    bandwidth: u64,
) -> Result<()> {
    if !bandwidth_range().contains(&bandwidth) {
        return Err(Error::OutOfRange);
    }
    write(nios, direction, Command::Bandwidth, bandwidth).await
}

// ==================== Frequency ====================

/// Supported frequency range for a direction, in Hz.
pub(crate) fn frequency_range(direction: Direction) -> RangeInclusive<u64> {
    match direction {
        Direction::Rx => 70_000_000..=6_000_000_000,
        Direction::Tx => 47_000_000..=6_000_000_000,
    }
}

/// Current frequency for a direction, in Hz.
pub(crate) async fn frequency(nios: &NiosCore, direction: Direction) -> Result<u64> {
    read(nios, direction, Command::Frequency).await
}

/// Set the frequency for a direction, in Hz.
pub(crate) async fn set_frequency(
    nios: &NiosCore,
    direction: Direction,
    frequency: u64,
) -> Result<()> {
    if !frequency_range(direction).contains(&frequency) {
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
pub(crate) async fn gain_range(
    nios: &NiosCore,
    direction: Direction,
) -> Result<&'static GainRange> {
    let frequency = frequency(nios, direction).await?;
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
pub(crate) async fn gain(nios: &NiosCore, channel: Channel) -> Result<f64> {
    let mut gain = read(nios, channel, Command::Gain).await? as f64;
    if channel.direction() == Direction::Tx {
        // TX gain register is in millidecibels with an inverted sign
        gain *= -1e-3;
    }
    gain += gain_range(nios, channel.direction()).await?.offset;
    Ok(gain)
}

/// Set the gain for a channel, in dB.
pub(crate) async fn set_gain(nios: &NiosCore, channel: Channel, mut gain: f64) -> Result<()> {
    let range = gain_range(nios, channel.direction()).await?;
    if !range.gain_range.contains(&gain) {
        return Err(Error::OutOfRange);
    }

    gain -= range.offset;
    if channel.direction() == Direction::Tx {
        gain *= -1e3;
    }

    // RX gain writes require manual gain mode
    if channel.direction() == Direction::Rx && gain_mode(nios, channel).await? != GainMode::Manual {
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
/// every RX channel off, [`gain`] would keep returning the previous value
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
pub(crate) async fn gain_mode(nios: &NiosCore, channel: Channel) -> Result<GainMode> {
    match channel.direction() {
        Direction::Rx => read(nios, channel, Command::GainMode).await?.try_into(),
        Direction::Tx => Err(Error::OutOfRange),
    }
}

/// Set the AGC mode for an RX channel.
pub(crate) async fn set_gain_mode(nios: &NiosCore, channel: Channel, mode: GainMode) -> Result<()> {
    match channel.direction() {
        Direction::Rx => write(nios, channel, Command::GainMode, mode.into()).await,
        Direction::Tx => Err(Error::OutOfRange),
    }
}

// ==================== Sample rate ====================

/// Supported sample rate range in Hz.
pub(crate) fn sample_rate_range() -> RangeInclusive<u64> {
    520_834..=61_440_000
}

/// Current sample rate in Hz (all channels share one rate).
pub(crate) async fn sample_rate(nios: &NiosCore) -> Result<u64> {
    read(nios, Channel::Rx0, Command::SampleRate).await
}

/// Highest sample rate that needs the 4x filters, in Hz.
const LOW_RATE_THRESHOLD: u64 = 2_083_334;
/// Rates above this cannot be reached directly from the low range.
const HIGH_RATE_THRESHOLD: u64 = 40_000_000;
/// Rate used to cross between the low range and rates above 40 MS/s.
const INTERMEDIATE_SAFE_RATE: u64 = 30_000_000;

/// Whether `rate` is in the low range, which needs the 4x filters.
fn is_low_rate(rate: u64) -> bool {
    rate <= LOW_RATE_THRESHOLD
}

/// One step of a sample rate change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Write the RX and TX filters (only those that differ).
    Filters(FilterMode, FilterMode),
    /// Write the sample rate.
    Rate(u64),
}

const DEFAULT_FILTERS: Step = Step::Filters(FilterMode::Ratio1, FilterMode::Bypass);
const LOW_RATE_FILTERS: Step = Step::Filters(FilterMode::Ratio4, FilterMode::Ratio4);

/// The steps of a sample rate change.
#[derive(Debug, PartialEq, Eq)]
struct Plan {
    steps: Vec<Step>,
    /// After a failed step, put the default filters back if the radio is left
    /// at a rate above the low range. Set when the change crosses the low-rate
    /// boundary or may start from a stuck state.
    restore_on_failure: bool,
}

/// Plan a change from `current` rate and RX filter to `rate`, in the order the
/// RFIC needs.
///
/// Entering the low range sets the 4x filters before the rate; leaving it
/// writes the rate first and then the default filters (writing the filters
/// first makes the RFIC refuse the next move back into the range). Crossing
/// between the low range and rates above 40 MS/s goes through an intermediate
/// safe rate. A high rate with RX at 4x (a stuck state) gets the default
/// filters back first, before any intermediate rate.
fn plan_sample_rate(current: u64, current_rx: FilterMode, rate: u64) -> Plan {
    let (was_low, is_low) = (is_low_rate(current), is_low_rate(rate));
    let mut steps = Vec::new();
    if !was_low && is_low && current_rx == FilterMode::Ratio4 {
        steps.push(DEFAULT_FILTERS);
    }
    if (current > HIGH_RATE_THRESHOLD && is_low) || (rate > HIGH_RATE_THRESHOLD && was_low) {
        steps.push(Step::Rate(INTERMEDIATE_SAFE_RATE));
    }
    match (was_low, is_low) {
        (true, false) => {
            steps.push(Step::Rate(rate));
            steps.push(DEFAULT_FILTERS);
        }
        (false, true) => {
            steps.push(LOW_RATE_FILTERS);
            steps.push(Step::Rate(rate));
        }
        (true, true) => {
            steps.push(LOW_RATE_FILTERS);
            steps.push(Step::Rate(rate));
        }
        (false, false) => {
            steps.push(DEFAULT_FILTERS);
            steps.push(Step::Rate(rate));
        }
    }
    Plan {
        steps,
        restore_on_failure: was_low != is_low,
    }
}

/// Set the sample rate in Hz.
///
/// The steps are in [`plan_sample_rate`]'s order. If a step fails after the
/// radio may have been left at a rate above the low range with 4x filters, the
/// default filters are restored on a best-effort basis and the original error
/// is returned.
pub(crate) async fn set_sample_rate(nios: &NiosCore, rate: u64) -> Result<()> {
    if !sample_rate_range().contains(&rate) {
        return Err(Error::OutOfRange);
    }

    let current = sample_rate(nios).await?;
    let current_rx = filter(nios, Direction::Rx).await?;
    let plan = plan_sample_rate(current, current_rx, rate);

    for step in plan.steps {
        let result = match step {
            Step::Filters(rx, tx) => set_filters(nios, rx, tx).await,
            Step::Rate(rate) => write(nios, Channel::Rx0, Command::SampleRate, rate).await,
        };
        if let Err(error) = result {
            if plan.restore_on_failure {
                // Best effort: the original error is the one to report.
                let high = sample_rate(nios).await.is_ok_and(|rate| !is_low_rate(rate));
                if high {
                    let _ = set_filters(nios, FilterMode::Ratio1, FilterMode::Bypass).await;
                }
            }
            return Err(error);
        }
    }
    Ok(())
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

async fn filter(nios: &NiosCore, direction: Direction) -> Result<FilterMode> {
    read(nios, direction, Command::Filter).await?.try_into()
}

async fn set_filter(nios: &NiosCore, direction: Direction, mode: FilterMode) -> Result<()> {
    if filter(nios, direction).await? == mode {
        return Ok(());
    }
    write(nios, direction, Command::Filter, mode.into()).await
}

async fn set_filters(nios: &NiosCore, rx: FilterMode, tx: FilterMode) -> Result<()> {
    set_filter(nios, Direction::Rx, rx).await?;
    set_filter(nios, Direction::Tx, tx).await
}

// ==================== Channel enable ====================

/// Enable or disable a channel module, muting TX channels while disabled.
pub(crate) async fn enable_module(nios: &NiosCore, channel: Channel, enable: bool) -> Result<()> {
    if channel.direction() == Direction::Tx {
        write(nios, channel, Command::TxMute, (!enable).into()).await?;
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

/// Poll the RFIC status register until the write queue drains, and return the
/// status word.
///
/// Bit 0 is set once the RFIC is initialised, bit 1 is set when the last job
/// in the write queue succeeded, and bits 15:8 hold the write queue length.
async fn wait(nios: &NiosCore) -> Result<u64> {
    for _ in 0..30 {
        let status = read(nios, CHANNEL_ALL, Command::Status).await?;
        if (status >> 8) & 0xFF == 0 {
            return Ok(status);
        }
        maybe_future::sleep(Duration::from_micros(100)).await;
    }
    Err(Error::Timeout)
}

/// Queue a write and wait for the RFIC to run it. Fails with
/// [`Error::RficCommandFailed`] when the RFIC refused the command.
async fn write(
    nios: &NiosCore,
    channel: impl Into<u16>,
    command: Command,
    data: u64,
) -> Result<()> {
    nios.write(Nios16x64::RfIc, address(channel, command), data)
        .await?;
    if wait(nios).await? & STATUS_LAST_JOB_OK == 0 {
        return Err(Error::RficCommandFailed(command.name()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const R4: FilterMode = FilterMode::Ratio4;
    const R1: FilterMode = FilterMode::Ratio1;
    const INTERMEDIATE: Step = Step::Rate(INTERMEDIATE_SAFE_RATE);

    fn plan(current: u64, rx: FilterMode, rate: u64) -> (Vec<Step>, bool) {
        let plan = plan_sample_rate(current, rx, rate);
        (plan.steps, plan.restore_on_failure)
    }

    #[test]
    fn plan_entering_low_sets_filters_before_the_rate() {
        let expected = vec![LOW_RATE_FILTERS, Step::Rate(2_000_000)];
        assert_eq!(plan(4_000_000, R1, 2_000_000), (expected.clone(), true));
        assert_eq!(plan(8_000_000, R1, 2_000_000), (expected, true));
    }

    #[test]
    fn plan_entering_low_from_above_40_msps_steps_through_30_msps() {
        assert_eq!(
            plan(61_440_000, R1, 2_000_000),
            (
                vec![INTERMEDIATE, LOW_RATE_FILTERS, Step::Rate(2_000_000)],
                true
            )
        );
    }

    #[test]
    fn plan_leaving_low_writes_the_rate_before_the_filters() {
        assert_eq!(
            plan(2_000_000, R4, 4_000_000),
            (vec![Step::Rate(4_000_000), DEFAULT_FILTERS], true)
        );
        assert_eq!(
            plan(2_000_000, R4, 61_440_000),
            (
                vec![INTERMEDIATE, Step::Rate(61_440_000), DEFAULT_FILTERS],
                true
            )
        );
    }

    #[test]
    fn plan_within_a_range_keeps_the_filters_for_that_range() {
        assert_eq!(
            plan(2_000_000, R4, 1_000_000),
            (vec![LOW_RATE_FILTERS, Step::Rate(1_000_000)], false)
        );
        assert_eq!(
            plan(4_000_000, R1, 10_000_000),
            (vec![DEFAULT_FILTERS, Step::Rate(10_000_000)], false)
        );
    }

    #[test]
    fn plan_recovers_from_a_high_rate_with_rx_4x() {
        assert_eq!(
            plan(4_000_000, R4, 2_000_000),
            (
                vec![DEFAULT_FILTERS, LOW_RATE_FILTERS, Step::Rate(2_000_000)],
                true
            )
        );
    }

    #[test]
    fn plan_recovers_from_the_stuck_state_above_40_msps_before_the_intermediate_rate() {
        assert_eq!(
            plan(61_440_000, R4, 2_000_000),
            (
                vec![
                    DEFAULT_FILTERS,
                    INTERMEDIATE,
                    LOW_RATE_FILTERS,
                    Step::Rate(2_000_000)
                ],
                true
            )
        );
    }

    #[test]
    fn plan_threshold_is_inclusive() {
        let (steps, _) = plan(4_000_000, R1, 2_083_334);
        assert_eq!(steps, vec![LOW_RATE_FILTERS, Step::Rate(2_083_334)]);
        let (steps, _) = plan(2_083_334, R4, 2_083_335);
        assert_eq!(steps, vec![Step::Rate(2_083_335), DEFAULT_FILTERS]);
        // The 30 MS/s step uses the same predicate.
        let (steps, _) = plan(2_083_334, R4, 50_000_000);
        assert_eq!(steps[0], INTERMEDIATE);
    }
}
