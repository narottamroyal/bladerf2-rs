//! Hardware tests for a bladeRF 2.0 micro.
//!
//! These need the device attached, so they are compiled only with the
//! `hardware-tests` feature:
//!
//! ```console
//! $ cargo test --features hardware-tests
//! ```
//!
//! Set `BLADERF2_SERIAL` to a serial-number prefix to choose one device when
//! several are attached. Every test is marked `#[serial]`, so hardware tests
//! never run concurrently with each other, whatever `--test-threads` is.

#![cfg(all(feature = "hardware-tests", not(target_arch = "wasm32")))]

use std::time::{Duration, Instant};

use approx::abs_diff_eq;
use bladerf2::{BladeRf2, Channel, Direction, GainMode, MaybeFuture, RxStream, TxStream, Version};
use num_complex::Complex32;
use serial_test::serial;

/// Generous per-read timeout, so a dead stream fails the test rather than hanging it.
const TIMEOUT: Option<Duration> = Some(Duration::from_secs(3));

/// Sample rate the board powers up at.
const POWER_UP_SAMPLE_RATE: u64 = 30_720_000;

/// Fraction of the nominal sample rate that a streaming test must sustain.
///
/// Measured rates are within 0.01% of nominal, so this leaves a wide margin
/// while still failing when the host cannot keep up.
const THROUGHPUT_MIN_RATIO: f64 = 0.99;

/// Sample rates for the throughput tests: the minimum, either side of the
/// 2.08 MS/s low-rate filter switch, the power-up rate, either side of the
/// 40 MS/s switch, and the maximum.
fn throughput_rates(range: &std::ops::RangeInclusive<u64>) -> [u64; 7] {
    [
        *range.start(),
        2_000_000,
        10_000_000,
        POWER_UP_SAMPLE_RATE,
        40_000_000,
        50_000_000,
        *range.end(),
    ]
}

/// How long each throughput measurement runs.
const THROUGHPUT_WINDOW: Duration = Duration::from_millis(500);

/// Measures how many samples per second `transfer` sustains. `transfer` moves
/// one buffer and returns how many samples it moved. The first call is
/// excluded as warm-up, so stream start-up time does not count against the
/// measurement.
fn measure_throughput(mut transfer: impl FnMut() -> usize) -> f64 {
    transfer();
    let start = Instant::now();
    let mut total = 0;
    while start.elapsed() < THROUGHPUT_WINDOW {
        total += transfer();
    }
    total as f64 / start.elapsed().as_secs_f64()
}

/// Asserts that `measured` is close enough to the nominal sample `rate`.
fn assert_throughput(direction: &str, rate: u64, measured: f64) {
    assert!(
        measured >= THROUGHPUT_MIN_RATIO * rate as f64,
        "{direction} at {rate} S/s: measured {measured:.0} S/s"
    );
}

/// Allowed error on each component of a looped-back sample: 2^-12.
const LOOPBACK_EPSILON: f32 = 1.0 / 4096.0;

/// Asserts that each received sample matches the sample sent.
fn assert_loopback(sent: &[Complex32], received: &[Complex32]) {
    assert_eq!(sent.len(), received.len());
    for (i, (a, b)) in sent.iter().zip(received).enumerate() {
        assert!(
            abs_diff_eq!(*a, *b, epsilon = LOOPBACK_EPSILON),
            "sample {i}: sent {a}, received {b}"
        );
    }
}

/// Encodes a sample index exactly: the low 10 bits in I, the rest in Q.
fn encode(index: usize) -> Complex32 {
    Complex32::new(
        (index % 1024) as f32 / 2048.0,
        (index / 1024) as f32 / 2048.0,
    )
}

/// Reads one buffer, failing unless it is filled completely.
fn read_full(rx: &mut RxStream, buf: &mut [Complex32]) {
    let len = buf.len();
    assert_eq!(rx.read(&mut [buf], TIMEOUT).unwrap(), len);
}

/// Writes one buffer, failing unless all of it is accepted.
fn write_full(tx: &mut TxStream, buf: &[Complex32]) {
    assert_eq!(tx.write(&[buf], TIMEOUT).unwrap(), buf.len());
}

/// Calls `step` repeatedly for `window`.
fn run_for(window: Duration, mut step: impl FnMut()) {
    let start = Instant::now();
    while start.elapsed() < window {
        step();
    }
}

/// Turns loopback off when dropped, so a failing test cannot leave it on for the next one.
struct Loopback<'r, 'd>(&'r bladerf2::RfLinkSession<'d>);

impl Drop for Loopback<'_, '_> {
    fn drop(&mut self) {
        let _ = self.0.set_loopback(false).wait();
    }
}

/// Open the device and put it in a known state.
///
/// A test that fails part-way can leave the radio in loopback, where RX gets
/// no data unless TX is running. Resetting here stops one failure from
/// cascading into the tests that run after it.
fn open() -> BladeRf2 {
    let mut dev = match std::env::var("BLADERF2_SERIAL") {
        Ok(prefix) => BladeRf2::open_serial(&prefix),
        Err(_) => BladeRf2::open_any(),
    }
    .expect("open bladeRF 2.0 micro");
    {
        let rf = dev.rf_link_session();
        rf.set_loopback(false).wait().unwrap();
        rf.set_sample_rate(2_000_000).wait().unwrap();
    }
    dev
}

#[test]
#[serial]
fn enumerate_finds_a_device() {
    assert!(!BladeRf2::enumerate().unwrap().is_empty());
}

#[test]
#[serial]
fn fpga_version() {
    let dev = open();
    assert_eq!(
        dev.fpga_version().wait().unwrap(),
        Version {
            major: 0,
            minor: 16,
            patch: 0
        }
    );
}

#[test]
#[serial]
fn bandwidth() {
    const BW1: u64 = 1_000_000;
    const BW2: u64 = 40_000_000;
    let mut dev = open();
    let rf = dev.rf_link_session();

    rf.set_bandwidth(Direction::Rx, BW1).wait().unwrap();
    rf.set_bandwidth(Direction::Tx, BW2).wait().unwrap();
    assert_eq!(rf.bandwidth(Direction::Rx).wait().unwrap(), BW1);
    assert_eq!(rf.bandwidth(Direction::Tx).wait().unwrap(), BW2);

    rf.set_bandwidth(Direction::Rx, BW2).wait().unwrap();
    rf.set_bandwidth(Direction::Tx, BW1).wait().unwrap();
    assert_eq!(rf.bandwidth(Direction::Rx).wait().unwrap(), BW2);
    assert_eq!(rf.bandwidth(Direction::Tx).wait().unwrap(), BW1);
}

#[test]
#[serial]
fn frequency() {
    const FREQ1: u64 = 920_000_000;
    const FREQ2: u64 = 433_000_000;
    let mut dev = open();
    let rf = dev.rf_link_session();

    rf.set_frequency(Direction::Rx, FREQ1).wait().unwrap();
    rf.set_frequency(Direction::Tx, FREQ2).wait().unwrap();
    assert!(rf.frequency(Direction::Rx).wait().unwrap().abs_diff(FREQ1) <= 5);
    assert!(rf.frequency(Direction::Tx).wait().unwrap().abs_diff(FREQ2) <= 5);

    rf.set_frequency(Direction::Rx, FREQ2).wait().unwrap();
    rf.set_frequency(Direction::Tx, FREQ1).wait().unwrap();
    assert!(rf.frequency(Direction::Rx).wait().unwrap().abs_diff(FREQ2) <= 5);
    assert!(rf.frequency(Direction::Tx).wait().unwrap().abs_diff(FREQ1) <= 5);

    // Just outside each end of each range: rejected, and the previous
    // frequency is kept.
    for direction in [Direction::Rx, Direction::Tx] {
        let range = rf.frequency_range(direction);
        let current = rf.frequency(direction).wait().unwrap();
        if let Some(below) = range.start().checked_sub(1) {
            assert!(rf.set_frequency(direction, below).wait().is_err());
        }
        assert!(rf.set_frequency(direction, range.end() + 1).wait().is_err());
        assert!(rf.frequency(direction).wait().unwrap().abs_diff(current) <= 5);
    }
}

#[test]
#[serial]
fn gain() {
    // Gains read back as set, both with no stream running and with one active.
    let mut dev = open();
    let rf = dev.rf_link_session();

    let check = |rx0: f64, rx1: f64, tx0: f64, tx1: f64| {
        rf.set_gain(Channel::Rx0, rx0).wait().unwrap();
        rf.set_gain(Channel::Rx1, rx1).wait().unwrap();
        rf.set_gain(Channel::Tx0, tx0).wait().unwrap();
        rf.set_gain(Channel::Tx1, tx1).wait().unwrap();

        assert_eq!(rf.gain(Channel::Rx0).wait().unwrap(), rx0);
        assert_eq!(rf.gain(Channel::Rx1).wait().unwrap(), rx1);
        assert_eq!(rf.gain(Channel::Tx0).wait().unwrap(), tx0);
        assert_eq!(rf.gain(Channel::Tx1).wait().unwrap(), tx1);
    };

    check(50.0, 40.0, 30.0, 20.0);
    check(20.0, 30.0, 40.0, 50.0);

    let mut stream = rf.rx_stream(&[0]).unwrap();
    stream.activate().wait().unwrap();
    check(50.0, 40.0, 30.0, 20.0);
    check(20.0, 30.0, 40.0, 50.0);

    // Back to the default gains.
    check(60.0, 60.0, 56.0, 56.0);

    stream.deactivate().wait().unwrap();
}

#[test]
#[serial]
fn gain_mode() {
    let mut dev = open();
    let rf = dev.rf_link_session();

    rf.set_gain_mode(Channel::Rx0, GainMode::SlowAttack)
        .wait()
        .unwrap();
    rf.set_gain_mode(Channel::Rx1, GainMode::Manual)
        .wait()
        .unwrap();
    assert_eq!(
        rf.gain_mode(Channel::Rx0).wait().unwrap(),
        GainMode::SlowAttack
    );
    assert_eq!(rf.gain_mode(Channel::Rx1).wait().unwrap(), GainMode::Manual);

    rf.set_gain_mode(Channel::Rx0, GainMode::FastAttack)
        .wait()
        .unwrap();
    rf.set_gain_mode(Channel::Rx1, GainMode::Hybrid)
        .wait()
        .unwrap();
    assert_eq!(
        rf.gain_mode(Channel::Rx0).wait().unwrap(),
        GainMode::FastAttack
    );
    assert_eq!(rf.gain_mode(Channel::Rx1).wait().unwrap(), GainMode::Hybrid);

    rf.set_gain_mode(Channel::Rx0, GainMode::default())
        .wait()
        .unwrap();
    rf.set_gain_mode(Channel::Rx1, GainMode::default())
        .wait()
        .unwrap();
    assert_eq!(
        rf.gain_mode(Channel::Rx0).wait().unwrap(),
        GainMode::SlowAttack
    );
    assert_eq!(
        rf.gain_mode(Channel::Rx1).wait().unwrap(),
        GainMode::SlowAttack
    );
}

#[test]
#[serial]
fn sample_rate() {
    let mut dev = open();
    let rf = dev.rf_link_session();
    let range = rf.sample_rate_range();

    // Both ends of the range, and values either side of the low/high rate
    // switches (about 2.08 and 40 MS/s).
    for rate in [
        *range.start(),
        2_083_332,
        2_083_334,
        2_500_000,
        25_000_000,
        40_000_000,
        40_000_001,
        50_000_000,
        *range.end(),
    ] {
        rf.set_sample_rate(rate).wait().unwrap();
        let actual = rf.sample_rate().wait().unwrap();
        assert!(
            actual.abs_diff(rate) <= 1,
            "requested {rate} Hz, got {actual} Hz"
        );
    }

    // Out of range: rejected, and the previous rate is kept.
    rf.set_sample_rate(10_000_000).wait().unwrap();
    assert!(rf.set_sample_rate(range.start() - 1).wait().is_err());
    assert!(rf.set_sample_rate(range.end() + 1).wait().is_err());
    assert_eq!(rf.sample_rate().wait().unwrap(), 10_000_000);

    // Setting the current rate during a stream must not disturb it.
    let mut rx = rf.rx_stream(&[0]).unwrap();
    rx.activate().wait().unwrap();
    let mut buf = vec![Complex32::ZERO; rx.mtu()];
    read_full(&mut rx, &mut buf);

    rf.set_sample_rate(10_000_000).wait().unwrap();
    read_full(&mut rx, &mut buf);
    assert_eq!(rf.sample_rate().wait().unwrap(), 10_000_000);

    rx.deactivate().wait().unwrap();
}

#[test]
#[serial]
fn throughput() {
    let mut dev = open();
    let rf = dev.rf_link_session();

    for rate in throughput_rates(&rf.sample_rate_range()) {
        rf.set_sample_rate(rate).wait().unwrap();

        let mut rx = rf.rx_stream(&[0]).unwrap();
        rx.activate().wait().unwrap();
        let mut rx_buf = vec![Complex32::ZERO; rx.mtu()];
        let received =
            measure_throughput(|| rx.read(&mut [rx_buf.as_mut_slice()], TIMEOUT).unwrap());
        rx.deactivate().wait().unwrap();
        assert_throughput("receive", rate, received);

        // In loopback nothing is transmitted. `write` blocks while the radio
        // consumes queued data, so the rate it accepts is the transmit rate.
        rf.set_loopback(true).wait().unwrap();
        let loopback = Loopback(&rf);
        let mut tx = rf.tx_stream(&[0]).unwrap();
        tx.activate().wait().unwrap();
        let tx_buf = vec![Complex32::ZERO; tx.mtu()];
        let transmitted = measure_throughput(|| tx.write(&[tx_buf.as_slice()], TIMEOUT).unwrap());
        tx.deactivate().wait().unwrap();
        drop(loopback);
        assert_throughput("transmit", rate, transmitted);
    }
}

#[test]
#[serial]
fn siso_loopback() {
    // Each block carries sample indices. Every sample must come back exactly
    // as sent, which checks both the values and their order across many blocks.
    let mut dev = open();
    let rf = dev.rf_link_session();
    rf.set_gain(Channel::Rx0, 0.0).wait().unwrap();

    rf.set_loopback(true).wait().unwrap();
    let loopback = Loopback(&rf);
    assert!(rf.loopback().wait().unwrap());

    let mut rx = rf.rx_stream(&[0]).unwrap();
    let mut tx = rf.tx_stream(&[0]).unwrap();
    tx.activate().wait().unwrap();
    rx.activate().wait().unwrap();

    let n = rx.mtu();
    let mut rx_data = vec![Complex32::ZERO; n];
    for block in 0..16 {
        let tx_data: Vec<_> = (0..n).map(|i| encode(block * n + i)).collect();
        assert_eq!(tx.write(&[tx_data.as_slice()], TIMEOUT).unwrap(), n);
        assert_eq!(rx.read(&mut [rx_data.as_mut_slice()], TIMEOUT).unwrap(), n);
        assert_loopback(&tx_data, &rx_data);
    }

    rx.deactivate().wait().unwrap();
    tx.deactivate().wait().unwrap();
    drop(loopback);
    assert!(!rf.loopback().wait().unwrap());
}

#[test]
#[ignore = "unresolved issue: in MIMO loopback the last 9 samples of every 1024-sample block are wrong"]
#[serial]
fn mimo_loopback() {
    let mut dev = open();
    let rf = dev.rf_link_session();
    rf.set_gain(Channel::Rx0, 0.0).wait().unwrap();
    rf.set_gain(Channel::Rx1, 0.0).wait().unwrap();

    rf.set_loopback(true).wait().unwrap();
    let loopback = Loopback(&rf);
    assert!(rf.loopback().wait().unwrap());

    let mut rx = rf.rx_stream(&[0, 1]).unwrap();
    let mut tx = rf.tx_stream(&[0, 1]).unwrap();
    tx.activate().wait().unwrap();
    rx.activate().wait().unwrap();

    let n = rx.mtu();
    let mut tx0 = vec![Complex32::ZERO; n];
    let mut tx1 = vec![Complex32::ZERO; n];
    let mut rx0 = vec![Complex32::ZERO; n];
    let mut rx1 = vec![Complex32::ZERO; n];

    for pow in 1..16 {
        for i in 0..n {
            let phase = 2.0 * std::f32::consts::PI * (i as f32) / 2.0_f32.powi(pow);
            tx0[i] = (Complex32::I * phase).exp() * 0.5;
            let phase = 2.0 * std::f32::consts::PI * (i as f32) / 2.0_f32.powi(pow * 2);
            tx1[i] = (Complex32::I * phase).exp() * -0.25;
        }

        assert_eq!(
            tx.write(&[tx0.as_slice(), tx1.as_slice()], TIMEOUT)
                .unwrap(),
            n
        );
        assert_eq!(
            rx.read(&mut [rx0.as_mut_slice(), rx1.as_mut_slice()], TIMEOUT)
                .unwrap(),
            n
        );
        assert_loopback(&tx0, &rx0);
        assert_loopback(&tx1, &rx1);
    }

    rx.deactivate().wait().unwrap();
    tx.deactivate().wait().unwrap();
    drop(loopback);
    assert!(!rf.loopback().wait().unwrap());
}

#[test]
#[serial]
fn bias_tee() {
    // Check the DC voltage on the antenna port with a meter during the pauses.
    let mut dev = open();
    let rf = dev.rf_link_session();

    for direction in [Direction::Rx, Direction::Tx] {
        let original = rf.bias_tee(direction).wait().unwrap();

        rf.set_bias_tee(direction, true).wait().unwrap();
        assert!(rf.bias_tee(direction).wait().unwrap());
        std::thread::sleep(Duration::from_secs(1));

        rf.set_bias_tee(direction, false).wait().unwrap();
        assert!(!rf.bias_tee(direction).wait().unwrap());

        rf.set_bias_tee(direction, original).wait().unwrap();
        assert_eq!(rf.bias_tee(direction).wait().unwrap(), original);
    }
}

#[test]
#[serial]
fn pll_refclk() {
    let mut dev = open();
    let rf = dev.rf_link_session();
    let range = rf.pll_refclk_range();
    let original = rf.pll_refclk().wait().unwrap();

    // Frequencies the dividers can hit exactly, including both range limits.
    for hz in [*range.start(), 19_200_000, *range.end(), 10_000_000] {
        rf.set_pll_refclk(hz).wait().unwrap();
        assert_eq!(rf.pll_refclk().wait().unwrap(), hz);
    }

    // Other frequencies are quantized by the integer dividers.
    for hz in [12_345_678, 26_000_000, 100_000_001] {
        rf.set_pll_refclk(hz).wait().unwrap();
        let actual = rf.pll_refclk().wait().unwrap();
        assert!(
            actual.abs_diff(hz) <= hz / 100_000,
            "{hz} Hz was programmed as {actual} Hz"
        );
    }

    // Outside the range: rejected, and the programmed value is unchanged.
    rf.set_pll_refclk(original).wait().unwrap();
    assert!(rf.set_pll_refclk(*range.start() - 1).wait().is_err());
    assert!(rf.set_pll_refclk(*range.end() + 1).wait().is_err());
    assert_eq!(rf.pll_refclk().wait().unwrap(), original);
}

#[test]
#[serial]
fn pll() {
    // Turning the PLL on releases the VCTCXO DAC. Restore the original state
    // afterwards, and check that receiving still works.
    //
    // Lock needs a reference on REFIN (J95) at the programmed frequency. Set
    // BLADERF2_REFIN when one is connected to require lock; otherwise the
    // lock state is only read.
    let mut dev = open();
    let rf = dev.rf_link_session();
    let original = rf.pll_enable().wait().unwrap();

    rf.set_pll_enable(true).wait().unwrap();
    assert!(rf.pll_enable().wait().unwrap());
    std::thread::sleep(Duration::from_secs(1));
    let locked = rf.pll_lock_state().wait().unwrap();
    if std::env::var_os("BLADERF2_REFIN").is_some() {
        assert!(locked, "PLL did not lock to REFIN");
    }

    rf.set_pll_enable(false).wait().unwrap();
    assert!(!rf.pll_enable().wait().unwrap());

    rf.set_pll_enable(original).wait().unwrap();
    assert_eq!(rf.pll_enable().wait().unwrap(), original);

    let mut rx = rf.rx_stream(&[0]).unwrap();
    rx.activate().wait().unwrap();
    let mut buf = vec![Complex32::ZERO; rx.mtu()];
    read_full(&mut rx, &mut buf);
    rx.deactivate().wait().unwrap();
}

#[test]
#[serial]
fn drop_while_streaming() {
    // Dropping an active stream at the highest rate must leave the device
    // usable, as happens when a test panics mid-stream.
    let mut dev = open();
    let rf = dev.rf_link_session();
    rf.set_sample_rate(*rf.sample_rate_range().end())
        .wait()
        .unwrap();

    let mut rx = rf.rx_stream(&[0]).unwrap();
    rx.activate().wait().unwrap();
    let mut buf = vec![Complex32::ZERO; rx.mtu()];
    run_for(THROUGHPUT_WINDOW, || read_full(&mut rx, &mut buf));
    drop(rx);

    let mut rx = rf.rx_stream(&[0]).unwrap();
    rx.activate().wait().unwrap();
    read_full(&mut rx, &mut buf);
    rx.deactivate().wait().unwrap();

    // TX in loopback, so nothing is transmitted.
    let loopback = Loopback(&rf);
    rf.set_loopback(true).wait().unwrap();
    let mut tx = rf.tx_stream(&[0]).unwrap();
    tx.activate().wait().unwrap();
    let tx_buf = vec![Complex32::ZERO; tx.mtu()];
    run_for(THROUGHPUT_WINDOW, || write_full(&mut tx, &tx_buf));
    drop(tx);

    let mut tx = rf.tx_stream(&[0]).unwrap();
    tx.activate().wait().unwrap();
    write_full(&mut tx, &tx_buf);
    tx.deactivate().wait().unwrap();
    drop(loopback);
}
