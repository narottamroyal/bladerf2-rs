//! Blocking receive example: device information, settings, and RX.
//!
//! Prints device information, changes every setting and reads it back, then
//! receives a few buffers of samples. Needs no async runtime.
//!
//! `rx_sync`, `rx_async` and `rx_web` are intentionally near-identical; only
//! how the device is opened and how operations are waited on differ.
//!
//! ```console
//! $ cargo run --release --example rx_sync
//! ```

use bladerf2::{BladeRf2, Channel, Direction, GainMode, MaybeFuture};
use num_complex::Complex32;
use std::time::Duration;

macro_rules! out {
    ($($arg:tt)*) => {
        println!($($arg)*)
    };
}

/// Receive frequency.
const FREQUENCY: u64 = 100_000_000;
/// Sample rate. The board boots at 30.72 Msps; pick a rate the host can sustain.
const SAMPLE_RATE: u64 = 2_000_000;
/// Analog filter bandwidth.
const BANDWIDTH: u64 = 1_500_000;
/// Manual receive gain.
const GAIN_DB: f64 = 20.0;
/// Number of buffers to receive.
const BUFFERS: usize = 8;
/// Per-read timeout.
const TIMEOUT: Duration = Duration::from_secs(2);

/// Hz -> MHz for printing.
fn mhz(hz: u64) -> f64 {
    hz as f64 / 1e6
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // ---- Open the device ----------------------------------------------------
    let mut dev = BladeRf2::open_any().wait()?;

    // ---- Device information ------------------------------------------------
    out!("== Device ==");
    out!("serial:        {}", dev.serial()?);
    out!("FPGA version:  {}", dev.fpga_version().wait()?);
    out!("VCTCXO trim:   {:#06x}", dev.vctcxo_trim());

    // ---- Start an RF-link session ------------------------------------------
    // All configuration goes through the session. It borrows the device mutably,
    // so only one can exist at a time.
    let rf = dev.rf_link_session();
    out!("rx source:     {:?}", rf.rx_source().wait()?);

    // ---- Supported ranges ---------------------------------------------------
    out!("\n== Supported ranges ==");
    let frequency = rf.frequency_range(Direction::Rx);
    out!(
        "frequency:     {:.1} - {:.1} MHz",
        mhz(*frequency.start()),
        mhz(*frequency.end())
    );
    let sample_rate = rf.sample_rate_range();
    out!(
        "sample rate:   {:.3} - {:.3} MS/s",
        mhz(*sample_rate.start()),
        mhz(*sample_rate.end())
    );
    let bandwidth = rf.bandwidth_range();
    out!(
        "bandwidth:     {:.3} - {:.3} MHz",
        mhz(*bandwidth.start()),
        mhz(*bandwidth.end())
    );
    let gain = rf.gain_range(Direction::Rx).wait()?;
    out!(
        "gain:          {:.1} - {:.1} dB",
        gain.gain_range.start(),
        gain.gain_range.end()
    );

    // ---- Change settings and read them back ---------------------------------
    // Each line shows: value before -> value requested -> value read back.
    out!("\n== Settings (before -> requested -> after) ==");

    let before = rf.frequency(Direction::Rx).wait()?;
    rf.set_frequency(Direction::Rx, FREQUENCY).wait()?;
    let after = rf.frequency(Direction::Rx).wait()?;
    out!(
        "frequency:     {:.3} -> {:.3} -> {:.3} MHz",
        mhz(before),
        mhz(FREQUENCY),
        mhz(after)
    );

    let before = rf.sample_rate().wait()?;
    rf.set_sample_rate(SAMPLE_RATE).wait()?;
    let after = rf.sample_rate().wait()?;
    out!(
        "sample rate:   {:.3} -> {:.3} -> {:.3} MS/s",
        mhz(before),
        mhz(SAMPLE_RATE),
        mhz(after)
    );

    let before = rf.bandwidth(Direction::Rx).wait()?;
    rf.set_bandwidth(Direction::Rx, BANDWIDTH).wait()?;
    let after = rf.bandwidth(Direction::Rx).wait()?;
    out!(
        "bandwidth:     {:.3} -> {:.3} -> {:.3} MHz",
        mhz(before),
        mhz(BANDWIDTH),
        mhz(after)
    );

    let before = rf.gain_mode(Channel::Rx0).wait()?;
    rf.set_gain_mode(Channel::Rx0, GainMode::Manual).wait()?;
    let after = rf.gain_mode(Channel::Rx0).wait()?;
    out!(
        "gain mode:     {before:?} -> {:?} -> {after:?}",
        GainMode::Manual
    );

    let before = rf.gain(Channel::Rx0).wait()?;
    rf.set_gain(Channel::Rx0, GAIN_DB).wait()?;
    let after = rf.gain(Channel::Rx0).wait()?;
    out!("gain:          {before:.1} -> {GAIN_DB:.1} -> {after:.1} dB");

    // ---- Receive ------------------------------------------------------------
    let mut rx = rf.rx_stream(&[0])?;
    let mut samples = vec![Complex32::ZERO; rx.mtu()];
    out!("\n== Receive {BUFFERS} x {} samples ==", samples.len());
    rx.activate().wait()?;
    for i in 0..BUFFERS {
        let n = rx.read(&mut [&mut samples], Some(TIMEOUT)).wait()?;
        let power = samples[..n].iter().map(|s| s.norm_sqr()).sum::<f32>() / n as f32;
        let peak = samples[..n].iter().map(|s| s.norm()).fold(0.0, f32::max);
        out!(
            "buffer {i}: {n} samples, mean power {:.1} dBFS, peak {peak:.3}",
            10.0 * power.log10()
        );
    }
    // Always deactivate explicitly; dropping an active stream is best-effort only.
    rx.deactivate().wait()?;
    out!("done");
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    run()
}
