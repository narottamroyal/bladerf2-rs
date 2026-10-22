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

use std::time::Duration;

use bladerf2::{BladeRf2, Channel, Direction, GainMode, MaybeFuture, Version};
use num_complex::Complex32;
use serial_test::serial;

/// Generous per-read timeout, so a dead stream fails the test rather than hanging it.
const TIMEOUT: Option<Duration> = Some(Duration::from_secs(3));

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
    let rx = rf.frequency(Direction::Rx).wait().unwrap();
    let tx = rf.frequency(Direction::Tx).wait().unwrap();
    assert!((rx as f64 - FREQ1 as f64).abs() < 5.0);
    assert!((tx as f64 - FREQ2 as f64).abs() < 5.0);

    rf.set_frequency(Direction::Rx, FREQ2).wait().unwrap();
    rf.set_frequency(Direction::Tx, FREQ1).wait().unwrap();
    let rx = rf.frequency(Direction::Rx).wait().unwrap();
    let tx = rf.frequency(Direction::Tx).wait().unwrap();
    assert!((rx as f64 - FREQ2 as f64).abs() < 5.0);
    assert!((tx as f64 - FREQ1 as f64).abs() < 5.0);
}

#[test]
#[serial]
fn gain_with_active_stream() {
    let mut dev = open();
    let rf = dev.rf_link_session();
    let mut stream = rf.rx_stream(&[0]).unwrap();
    stream.activate().wait().unwrap();

    let test_gains = |rx0: f64, rx1: f64, tx0: f64, tx1: f64| {
        rf.set_gain(Channel::Rx0, rx0).wait().unwrap();
        rf.set_gain(Channel::Rx1, rx1).wait().unwrap();
        rf.set_gain(Channel::Tx0, tx0).wait().unwrap();
        rf.set_gain(Channel::Tx1, tx1).wait().unwrap();

        assert_eq!(rf.gain(Channel::Rx0).wait().unwrap(), rx0);
        assert_eq!(rf.gain(Channel::Rx1).wait().unwrap(), rx1);
        assert_eq!(rf.gain(Channel::Tx0).wait().unwrap(), tx0);
        assert_eq!(rf.gain(Channel::Tx1).wait().unwrap(), tx1);
    };

    test_gains(50.0, 40.0, 30.0, 20.0);
    test_gains(20.0, 30.0, 40.0, 50.0);

    // Back to the default gains.
    test_gains(60.0, 60.0, 56.0, 56.0);

    stream.deactivate().wait().unwrap();
}

#[test]
#[serial]
fn gain_reads_back_without_a_stream() {
    let mut dev = open();
    let rf = dev.rf_link_session();

    for gain in [50.0, 20.0, 35.0] {
        rf.set_gain(Channel::Rx0, gain).wait().unwrap();
        assert_eq!(rf.gain(Channel::Rx0).wait().unwrap(), gain);
    }
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

    // Includes the switches between high and low sample rates that once
    // misbehaved.
    for rate in [25_000_000, 50_000_000, 2_083_334, 40_000_001, 2_083_332] {
        rf.set_sample_rate(rate).wait().unwrap();
        assert_eq!(rf.sample_rate().wait().unwrap(), rate);
    }
}

#[test]
#[serial]
fn siso_loopback() {
    let mut dev = open();
    let rf = dev.rf_link_session();
    rf.set_gain(Channel::Rx0, 0.0).wait().unwrap();

    rf.set_loopback(true).wait().unwrap();
    let _loopback = Loopback(&rf);
    assert!(rf.loopback().wait().unwrap());

    let mut rx = rf.rx_stream(&[0]).unwrap();
    let mut tx = rf.tx_stream(&[0]).unwrap();
    tx.activate().wait().unwrap();
    rx.activate().wait().unwrap();

    let n = rx.mtu();
    let mut tx_data = vec![Complex32::ZERO; n];
    let mut rx_data = vec![Complex32::ZERO; n];

    for pow in 1..16 {
        // Complex sinusoids at a range of frequencies.
        for (i, sample) in tx_data.iter_mut().enumerate() {
            let phase = 2.0 * std::f32::consts::PI * (i as f32) / 2.0_f32.powi(pow);
            *sample = (Complex32::I * phase).exp() * 0.5;
        }

        tx.write(&[tx_data.as_slice()], TIMEOUT).unwrap();
        rx.read(&mut [rx_data.as_mut_slice()], TIMEOUT).unwrap();

        const MAX_ERROR: f32 = 1.0 / 4096.0; // 2^-12
        for (a, b) in tx_data.iter().zip(rx_data.iter()) {
            let delta = a - b;
            assert!(delta.re.abs() <= MAX_ERROR, "pow {pow}: {delta}");
            assert!(delta.im.abs() <= MAX_ERROR, "pow {pow}: {delta}");
        }
    }

    rx.deactivate().wait().unwrap();
    tx.deactivate().wait().unwrap();
    drop(_loopback);
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
    let _loopback = Loopback(&rf);
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

        tx.write(&[tx0.as_slice(), tx1.as_slice()], TIMEOUT)
            .unwrap();
        rx.read(&mut [rx0.as_mut_slice(), rx1.as_mut_slice()], TIMEOUT)
            .unwrap();

        const MAX_ERROR: f32 = 1.0 / 4096.0; // 2^-12
        for (a, b) in tx0.iter().zip(rx0.iter()) {
            let delta = a - b;
            assert!(delta.re.abs() <= MAX_ERROR, "ch0 pow {pow}: {delta}");
            assert!(delta.im.abs() <= MAX_ERROR, "ch0 pow {pow}: {delta}");
        }
        for (a, b) in tx1.iter().zip(rx1.iter()) {
            let delta = a - b;
            assert!(delta.re.abs() <= MAX_ERROR, "ch1 pow {pow}: {delta}");
            assert!(delta.im.abs() <= MAX_ERROR, "ch1 pow {pow}: {delta}");
        }
    }

    rx.deactivate().wait().unwrap();
    tx.deactivate().wait().unwrap();
    drop(_loopback);
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
fn pll_enable() {
    // Turning the PLL on releases the VCTCXO DAC. Restore the original state
    // afterwards, and check that receiving still works.
    let mut dev = open();
    let rf = dev.rf_link_session();
    let original = rf.pll_enable().wait().unwrap();

    rf.set_pll_enable(true).wait().unwrap();
    assert!(rf.pll_enable().wait().unwrap());
    rf.set_pll_enable(false).wait().unwrap();
    assert!(!rf.pll_enable().wait().unwrap());

    rf.set_pll_enable(original).wait().unwrap();
    assert_eq!(rf.pll_enable().wait().unwrap(), original);

    let mut rx = rf.rx_stream(&[0]).unwrap();
    rx.activate().wait().unwrap();
    let mut buf = vec![Complex32::ZERO; rx.mtu()];
    rx.read(&mut [buf.as_mut_slice()], TIMEOUT).unwrap();
    rx.deactivate().wait().unwrap();
}

#[test]
#[serial]
fn pll_lock_state() {
    // Lock needs a reference on REFIN (J95) at the programmed frequency. Set
    // BLADERF2_REFIN when one is connected to require lock; otherwise the
    // state is only read.
    let mut dev = open();
    let rf = dev.rf_link_session();
    let original = rf.pll_enable().wait().unwrap();

    rf.set_pll_enable(true).wait().unwrap();
    std::thread::sleep(Duration::from_secs(1));
    let locked = rf.pll_lock_state().wait().unwrap();
    rf.set_pll_enable(original).wait().unwrap();

    if std::env::var_os("BLADERF2_REFIN").is_some() {
        assert!(locked, "PLL did not lock to REFIN");
    }
}
