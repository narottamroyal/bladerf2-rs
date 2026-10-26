//! Hardware tests for a bladeRF 2.0 micro.
//!
//! Each test is one async body, run three ways by `hardware_test!`:
//! - `blocking`: native, on the blocking path that `.wait()` uses;
//! - `asynchronous`: native, with real async USB (needs the `smol` feature);
//! - `browser`: wasm, in headless Chromium over WebUSB.
//!
//! See the README's Testing section for how to run them.

#![cfg(feature = "hardware-tests")]

use std::time::Duration;

use approx::abs_diff_eq;
use bladerf2::{BladeRf2, Channel, Direction, GainMode, RxSource, RxStream, TxStream, Version};
use futures_timer::Delay;
use num_complex::Complex32;
use web_time::Instant;

#[cfg(target_arch = "wasm32")]
wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

/// The browser runs tests concurrently (`--test-threads` is not supported), and
/// one device can only be open once. Browser tests take turns on this lock.
#[cfg(target_arch = "wasm32")]
static DEVICE: async_lock::Mutex<()> = async_lock::Mutex::new(());

/// Generous per-read timeout, so a dead stream fails the test rather than hanging it.
const TIMEOUT: Option<Duration> = Some(Duration::from_secs(3));
/// Sample rate the board powers up at.
const POWER_UP_SAMPLE_RATE: u64 = 30_720_000;
/// Fraction of the nominal sample rate that a throughput test must sustain.
const THROUGHPUT_MIN_RATIO: f64 = 0.99;
/// How long each throughput measurement runs.
const THROUGHPUT_WINDOW: Duration = Duration::from_millis(500);
/// Allowed error on each component of a looped-back sample: 2^-12.
const LOOPBACK_EPSILON: f32 = 1.0 / 4096.0;

/// Defines a hardware test once, as an async body that gets the device (see
/// [`with_device`]), and runs it in each mode. An `#[ignore]` applies to every
/// mode.
macro_rules! hardware_test {
    // `$reason` is a `tt` so that wasm-bindgen-test can see the `ignore`.
    ($(#[ignore = $reason:tt])? fn $name:ident($dev:ident) $body:block) => {
        mod $name {
            use super::*;

            async fn body($dev: &mut BladeRf2) $body

            #[cfg(not(target_arch = "wasm32"))]
            #[test]
            $(#[ignore = $reason])?
            fn blocking() {
                bladerf2::block_on(with_device(body))
            }

            #[cfg(all(not(target_arch = "wasm32"), feature = "smol"))]
            #[test]
            $(#[ignore = $reason])?
            fn asynchronous() {
                smol::block_on(with_device(body))
            }

            #[cfg(target_arch = "wasm32")]
            #[wasm_bindgen_test::wasm_bindgen_test]
            $(#[ignore = $reason])?
            async fn browser() {
                let _turn = DEVICE.lock().await;
                with_device(body).await
            }
        }
    };
}

/// Opens the device, resets it (RX from the RFIC, 2 MS/s), and runs `test` on it.
async fn with_device(test: impl AsyncFnOnce(&mut BladeRf2)) {
    let mut dev = match std::env::var("BLADERF2_SERIAL") {
        Ok(prefix) => BladeRf2::open_serial(&prefix).await,
        Err(_) => BladeRf2::open_any().await,
    }
    .expect("open bladeRF 2.0 micro");
    {
        let rf = dev.rf_link_session();
        rf.set_rx_source(RxSource::Rfic).await.unwrap();
        rf.set_sample_rate(2_000_000).await.unwrap();
    }
    test(&mut dev).await;
}

/// Reads one buffer, failing unless it is filled completely.
async fn read_full(rx: &mut RxStream, buf: &mut [Complex32]) {
    let len = buf.len();
    assert_eq!(rx.read(&mut [buf], TIMEOUT).await.unwrap(), len);
}

/// Writes one buffer, failing unless all of it is accepted.
async fn write_full(tx: &mut TxStream, buf: &[Complex32]) {
    assert_eq!(tx.write(&[buf], TIMEOUT).await.unwrap(), buf.len());
}

/// Calls `step` repeatedly for `window`.
async fn run_for(window: Duration, mut step: impl AsyncFnMut()) {
    let start = Instant::now();
    while start.elapsed() < window {
        step().await;
    }
}

/// Samples per second that `transfer` sustains. `transfer` moves one buffer
/// and returns how many samples it moved. The first call is warm-up.
async fn measure_throughput(mut transfer: impl AsyncFnMut() -> usize) -> f64 {
    transfer().await;
    let start = Instant::now();
    let mut total = 0;
    while start.elapsed() < THROUGHPUT_WINDOW {
        total += transfer().await;
    }
    total as f64 / start.elapsed().as_secs_f64()
}

/// Asserts that `measure` sustains close enough to the nominal sample `rate`.
/// It gets up to three attempts, so that a one-off stall of the host (a busy
/// browser, say) does not fail the test, but a rate that is always low does.
async fn assert_throughput(direction: &str, rate: u64, mut measure: impl AsyncFnMut() -> f64) {
    let mut best = 0.0_f64;
    for _ in 0..3 {
        best = best.max(measure().await);
        if best >= THROUGHPUT_MIN_RATIO * rate as f64 {
            return;
        }
    }
    panic!("{direction} at {rate} S/s: measured {best:.0} S/s");
}

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

/// Encodes a sample index exactly: the low 10 bits in I, the rest in Q.
fn encode(index: usize) -> Complex32 {
    Complex32::new(
        (index % 1024) as f32 / 2048.0,
        (index / 1024) as f32 / 2048.0,
    )
}

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

hardware_test! {
    fn enumerate_finds_a_device(_dev) {
        assert!(!BladeRf2::list_devices().await.unwrap().is_empty());
    }
}

hardware_test! {
    fn fpga_version(dev) {
        assert_eq!(
            dev.fpga_version().await.unwrap(),
            Version {
                major: 0,
                minor: 16,
                patch: 0
            }
        );
    }
}

hardware_test! {
    fn bandwidth(dev) {
        const BW1: u64 = 1_000_000;
        const BW2: u64 = 40_000_000;
        let rf = dev.rf_link_session();

        rf.set_bandwidth(Direction::Rx, BW1).await.unwrap();
        rf.set_bandwidth(Direction::Tx, BW2).await.unwrap();
        assert_eq!(rf.bandwidth(Direction::Rx).await.unwrap(), BW1);
        assert_eq!(rf.bandwidth(Direction::Tx).await.unwrap(), BW2);

        rf.set_bandwidth(Direction::Rx, BW2).await.unwrap();
        rf.set_bandwidth(Direction::Tx, BW1).await.unwrap();
        assert_eq!(rf.bandwidth(Direction::Rx).await.unwrap(), BW2);
        assert_eq!(rf.bandwidth(Direction::Tx).await.unwrap(), BW1);
    }
}

hardware_test! {
    fn frequency(dev) {
        const FREQ1: u64 = 920_000_000;
        const FREQ2: u64 = 433_000_000;
        let rf = dev.rf_link_session();

        rf.set_frequency(Direction::Rx, FREQ1).await.unwrap();
        rf.set_frequency(Direction::Tx, FREQ2).await.unwrap();
        assert!(rf.frequency(Direction::Rx).await.unwrap().abs_diff(FREQ1) <= 5);
        assert!(rf.frequency(Direction::Tx).await.unwrap().abs_diff(FREQ2) <= 5);

        rf.set_frequency(Direction::Rx, FREQ2).await.unwrap();
        rf.set_frequency(Direction::Tx, FREQ1).await.unwrap();
        assert!(rf.frequency(Direction::Rx).await.unwrap().abs_diff(FREQ2) <= 5);
        assert!(rf.frequency(Direction::Tx).await.unwrap().abs_diff(FREQ1) <= 5);

        // Just outside each end of each range: rejected, and the previous
        // frequency is kept.
        for direction in [Direction::Rx, Direction::Tx] {
            let range = rf.frequency_range(direction);
            let current = rf.frequency(direction).await.unwrap();
            if let Some(below) = range.start().checked_sub(1) {
                assert!(rf.set_frequency(direction, below).await.is_err());
            }
            assert!(rf.set_frequency(direction, range.end() + 1).await.is_err());
            assert!(rf.frequency(direction).await.unwrap().abs_diff(current) <= 5);
        }
    }
}

hardware_test! {
    fn abandoned_request(dev) {
        // A request dropped before its response arrives must not leave that
        // response to be taken as the answer to the next request.
        use std::future::IntoFuture;
        const FREQUENCY: u64 = 915_000_000;
        let rf = dev.rf_link_session();
        rf.set_frequency(Direction::Rx, FREQUENCY).await.unwrap();

        {
            let mut read = std::pin::pin!(rf.sample_rate().into_future());
            // Poll once, so the request is sent, then drop it.
            std::future::poll_fn(|cx| {
                let _ = read.as_mut().poll(cx);
                std::task::Poll::Ready(())
            })
            .await;
        }

        for _ in 0..3 {
            let frequency = rf.frequency(Direction::Rx).await.unwrap();
            assert!(frequency.abs_diff(FREQUENCY) <= 5, "read {frequency} Hz");
        }
    }
}

hardware_test! {
    fn gain(dev) {
        let rf = dev.rf_link_session();

        let check = async |rx0: f64, rx1: f64, tx0: f64, tx1: f64| {
            rf.set_gain(Channel::Rx0, rx0).await.unwrap();
            rf.set_gain(Channel::Rx1, rx1).await.unwrap();
            rf.set_gain(Channel::Tx0, tx0).await.unwrap();
            rf.set_gain(Channel::Tx1, tx1).await.unwrap();

            assert_eq!(rf.gain(Channel::Rx0).await.unwrap(), rx0);
            assert_eq!(rf.gain(Channel::Rx1).await.unwrap(), rx1);
            assert_eq!(rf.gain(Channel::Tx0).await.unwrap(), tx0);
            assert_eq!(rf.gain(Channel::Tx1).await.unwrap(), tx1);
        };

        // With no stream running, then with one active.
        check(50.0, 40.0, 30.0, 20.0).await;
        check(20.0, 30.0, 40.0, 50.0).await;

        let mut stream = rf.rx_stream(&[0]).unwrap();
        stream.activate().await.unwrap();
        check(50.0, 40.0, 30.0, 20.0).await;
        check(20.0, 30.0, 40.0, 50.0).await;

        // Back to the default gains.
        check(60.0, 60.0, 56.0, 56.0).await;

        stream.deactivate().await.unwrap();
    }
}

hardware_test! {
    fn gain_mode(dev) {
        let rf = dev.rf_link_session();

        rf.set_gain_mode(Channel::Rx0, GainMode::SlowAttack).await.unwrap();
        rf.set_gain_mode(Channel::Rx1, GainMode::Manual).await.unwrap();
        assert_eq!(rf.gain_mode(Channel::Rx0).await.unwrap(), GainMode::SlowAttack);
        assert_eq!(rf.gain_mode(Channel::Rx1).await.unwrap(), GainMode::Manual);

        rf.set_gain_mode(Channel::Rx0, GainMode::FastAttack).await.unwrap();
        rf.set_gain_mode(Channel::Rx1, GainMode::Hybrid).await.unwrap();
        assert_eq!(rf.gain_mode(Channel::Rx0).await.unwrap(), GainMode::FastAttack);
        assert_eq!(rf.gain_mode(Channel::Rx1).await.unwrap(), GainMode::Hybrid);

        rf.set_gain_mode(Channel::Rx0, GainMode::default()).await.unwrap();
        rf.set_gain_mode(Channel::Rx1, GainMode::default()).await.unwrap();
        assert_eq!(rf.gain_mode(Channel::Rx0).await.unwrap(), GainMode::SlowAttack);
        assert_eq!(rf.gain_mode(Channel::Rx1).await.unwrap(), GainMode::SlowAttack);
    }
}

hardware_test! {
    fn sample_rate(dev) {
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
            rf.set_sample_rate(rate).await.unwrap();
            // The rate comes from a rational divider, so the result can be 1 Hz
            // off depending on the previous rate.
            let actual = rf.sample_rate().await.unwrap();
            assert!(actual.abs_diff(rate) <= 1, "requested {rate} Hz, got {actual} Hz");
        }

        // Out of range: rejected, and the previous rate is kept.
        rf.set_sample_rate(10_000_000).await.unwrap();
        assert!(rf.set_sample_rate(range.start() - 1).await.is_err());
        assert!(rf.set_sample_rate(range.end() + 1).await.is_err());
        assert_eq!(rf.sample_rate().await.unwrap(), 10_000_000);

        // Setting the current rate during a stream must not disturb it.
        let mut rx = rf.rx_stream(&[0]).unwrap();
        rx.activate().await.unwrap();
        let mut buf = vec![Complex32::ZERO; rx.mtu()];
        read_full(&mut rx, &mut buf).await;

        rf.set_sample_rate(10_000_000).await.unwrap();
        read_full(&mut rx, &mut buf).await;
        assert_eq!(rf.sample_rate().await.unwrap(), 10_000_000);

        rx.deactivate().await.unwrap();
    }
}

hardware_test! {
    fn throughput(dev) {
        let rf = dev.rf_link_session();

        for rate in throughput_rates(&rf.sample_rate_range()) {
            rf.set_sample_rate(rate).await.unwrap();

            assert_throughput("receive", rate, async || {
                let mut rx = rf.rx_stream(&[0]).unwrap();
                rx.activate().await.unwrap();
                let mut buf = vec![Complex32::ZERO; rx.mtu()];
                let received = measure_throughput(async || {
                    rx.read(&mut [buf.as_mut_slice()], TIMEOUT).await.unwrap()
                })
                .await;
                rx.deactivate().await.unwrap();
                received
            })
            .await;

            // `write` blocks while the radio consumes queued data, so the rate it
            // accepts is the transmit rate. This transmits zeros.
            assert_throughput("transmit", rate, async || {
                let mut tx = rf.tx_stream(&[0]).unwrap();
                tx.activate().await.unwrap();
                let buf = vec![Complex32::ZERO; tx.mtu()];
                let transmitted = measure_throughput(async || {
                    tx.write(&[buf.as_slice()], TIMEOUT).await.unwrap()
                })
                .await;
                tx.deactivate().await.unwrap();
                transmitted
            })
            .await;
        }
    }
}

hardware_test! {
    fn siso_loopback(dev) {
        // Each block carries sample indices, so every sample must come back exactly
        // as sent, in order, across many blocks.
        let rf = dev.rf_link_session();
        rf.set_gain(Channel::Rx0, 0.0).await.unwrap();

        rf.set_rx_source(RxSource::Loopback).await.unwrap();
        assert_eq!(rf.rx_source().await.unwrap(), RxSource::Loopback);

        let mut rx = rf.rx_stream(&[0]).unwrap();
        let mut tx = rf.tx_stream(&[0]).unwrap();
        // Twice, so stopping must leave both streams usable.
        for _ in 0..2 {
            tx.activate().await.unwrap();
            rx.activate().await.unwrap();

            let n = rx.mtu();
            let mut rx_data = vec![Complex32::ZERO; n];
            for block in 0..16 {
                let tx_data: Vec<_> = (0..n).map(|i| encode(block * n + i)).collect();
                write_full(&mut tx, &tx_data).await;
                read_full(&mut rx, &mut rx_data).await;
                assert_loopback(&tx_data, &rx_data);
            }

            rx.deactivate().await.unwrap();
            tx.deactivate().await.unwrap();
        }
        rf.set_rx_source(RxSource::Rfic).await.unwrap();
        assert_eq!(rf.rx_source().await.unwrap(), RxSource::Rfic);
    }
}

hardware_test! {
    #[ignore = "FPGA 0.16.0 loses samples in MIMO digital loopback rx_counter covers MIMO RX"]
    fn mimo_loopback(dev) {
        let rf = dev.rf_link_session();
        rf.set_gain(Channel::Rx0, 0.0).await.unwrap();
        rf.set_gain(Channel::Rx1, 0.0).await.unwrap();

        rf.set_rx_source(RxSource::Loopback).await.unwrap();
        assert_eq!(rf.rx_source().await.unwrap(), RxSource::Loopback);

        let mut rx = rf.rx_stream(&[0, 1]).unwrap();
        let mut tx = rf.tx_stream(&[0, 1]).unwrap();
        tx.activate().await.unwrap();
        rx.activate().await.unwrap();

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

            assert_eq!(tx.write(&[tx0.as_slice(), tx1.as_slice()], TIMEOUT).await.unwrap(), n);
            assert_eq!(
                rx.read(&mut [rx0.as_mut_slice(), rx1.as_mut_slice()], TIMEOUT)
                    .await
                    .unwrap(),
                n
            );
            assert_loopback(&tx0, &rx0);
            assert_loopback(&tx1, &rx1);
        }

        rx.deactivate().await.unwrap();
        tx.deactivate().await.unwrap();
        rf.set_rx_source(RxSource::Rfic).await.unwrap();
        assert_eq!(rf.rx_source().await.unwrap(), RxSource::Rfic);
    }
}

hardware_test! {
    fn rx_counter(dev) {
        // The FPGA's test counter checks the whole RX path, including splitting
        // two channels, which MIMO loopback cannot (see `mimo_loopback`).
        let rf = dev.rf_link_session();
        rf.set_rx_source(RxSource::Counter).await.unwrap();
        assert_eq!(rf.rx_source().await.unwrap(), RxSource::Counter);

        for channels in [&[0][..], &[1], &[0, 1]] {
            let mut rx = rf.rx_stream(channels).unwrap();
            rx.activate().await.unwrap();
            let mtu = rx.mtu();
            let mut buffers = vec![vec![Complex32::ZERO; mtu]; channels.len()];
            // The counter restarts at 256 whenever RX is enabled.
            let mut expected = 256;
            // Reads of varying sizes span transfers and leave samples for the
            // next read. Every fourth read has no time to wait: whether or not
            // it times out, the counter must carry on without a gap.
            let lens = [mtu, 1000, 1, 2047, mtu / 2 + 3];
            for (read, len) in lens.into_iter().cycle().take(32).enumerate() {
                let timeout = if read % 4 == 3 { Some(Duration::ZERO) } else { TIMEOUT };
                let mut refs: Vec<_> = buffers.iter_mut().map(|b| &mut b[..len]).collect();
                match rx.read(&mut refs, timeout).await {
                    Ok(n) => assert_eq!(n, len),
                    Err(bladerf2::Error::Timeout) => continue,
                    Err(error) => panic!("read failed: {error}"),
                }
                for i in 0..len {
                    let sample = buffers[0][i];
                    let value = (sample.re * 2048.0).round() as i32;
                    assert_eq!(value, expected, "channels {channels:?}, sample {i}");
                    assert_eq!((sample.im * 2048.0).round() as i32, -value);
                    for buffer in &buffers[1..] {
                        assert_eq!(buffer[i], sample, "channels differ at sample {i}");
                    }
                    expected = if value == 2047 { -2047 } else { value + 1 };
                }
            }
            rx.deactivate().await.unwrap();
        }

        rf.set_rx_source(RxSource::Rfic).await.unwrap();
    }
}

hardware_test! {
    fn bias_tee(dev) {
        // Check the DC voltage on the antenna port with a meter during the pauses.
        let rf = dev.rf_link_session();

        for direction in [Direction::Rx, Direction::Tx] {
            let original = rf.bias_tee(direction).await.unwrap();

            rf.set_bias_tee(direction, true).await.unwrap();
            assert!(rf.bias_tee(direction).await.unwrap());
            Delay::new(Duration::from_secs(1)).await;

            rf.set_bias_tee(direction, false).await.unwrap();
            assert!(!rf.bias_tee(direction).await.unwrap());

            rf.set_bias_tee(direction, original).await.unwrap();
            assert_eq!(rf.bias_tee(direction).await.unwrap(), original);
        }
    }
}

hardware_test! {
    fn pll_refclk(dev) {
        let rf = dev.rf_link_session();
        let range = rf.pll_refclk_range();
        let original = rf.pll_refclk().await.unwrap();

        // Frequencies the dividers can hit exactly, including both range limits.
        for hz in [*range.start(), 19_200_000, *range.end(), 10_000_000] {
            rf.set_pll_refclk(hz).await.unwrap();
            assert_eq!(rf.pll_refclk().await.unwrap(), hz);
        }

        // Other frequencies are quantized by the integer dividers.
        for hz in [12_345_678, 26_000_000, 100_000_001] {
            rf.set_pll_refclk(hz).await.unwrap();
            let actual = rf.pll_refclk().await.unwrap();
            assert!(
                actual.abs_diff(hz) <= hz / 100_000,
                "{hz} Hz was programmed as {actual} Hz"
            );
        }

        // Outside the range: rejected, and the programmed value is unchanged.
        rf.set_pll_refclk(original).await.unwrap();
        assert!(rf.set_pll_refclk(range.start() - 1).await.is_err());
        assert!(rf.set_pll_refclk(range.end() + 1).await.is_err());
        assert_eq!(rf.pll_refclk().await.unwrap(), original);

        // Restore the frequency the device started with.
        rf.set_pll_refclk(original).await.unwrap();
        assert_eq!(rf.pll_refclk().await.unwrap(), original);
    }
}

hardware_test! {
    fn pll(dev) {
        // Turning the PLL on releases the VCTCXO DAC. Restore the original state
        // afterwards, and check that receiving still works.
        //
        // Lock needs a reference on REFIN (J95) at the programmed frequency. Set
        // BLADERF2_REFIN when one is connected to require lock; otherwise the
        // lock state is only read.
        let rf = dev.rf_link_session();
        let original = rf.pll_enable().await.unwrap();

        rf.set_pll_enable(true).await.unwrap();
        assert!(rf.pll_enable().await.unwrap());
        Delay::new(Duration::from_secs(1)).await;
        let locked = rf.pll_lock_state().await.unwrap();
        if std::env::var_os("BLADERF2_REFIN").is_some() {
            assert!(locked, "PLL did not lock to REFIN");
        }

        rf.set_pll_enable(false).await.unwrap();
        assert!(!rf.pll_enable().await.unwrap());

        rf.set_pll_enable(original).await.unwrap();
        assert_eq!(rf.pll_enable().await.unwrap(), original);

        let mut rx = rf.rx_stream(&[0]).unwrap();
        rx.activate().await.unwrap();
        let mut buf = vec![Complex32::ZERO; rx.mtu()];
        read_full(&mut rx, &mut buf).await;
        rx.deactivate().await.unwrap();
    }
}

hardware_test! {
    fn drop_while_streaming(dev) {
        // Dropping an active stream at the highest rate must leave the device
        // usable, as happens when a test panics mid-stream.
        let rf = dev.rf_link_session();
        rf.set_sample_rate(*rf.sample_rate_range().end()).await.unwrap();

        let mut rx = rf.rx_stream(&[0]).unwrap();
        rx.activate().await.unwrap();
        let mut buf = vec![Complex32::ZERO; rx.mtu()];
        run_for(THROUGHPUT_WINDOW, async || read_full(&mut rx, &mut buf).await).await;
        drop(rx);

        let mut rx = rf.rx_stream(&[0]).unwrap();
        rx.activate().await.unwrap();
        read_full(&mut rx, &mut buf).await;
        rx.deactivate().await.unwrap();

        // This transmits zeros.
        let mut tx = rf.tx_stream(&[0]).unwrap();
        tx.activate().await.unwrap();
        let tx_buf = vec![Complex32::ZERO; tx.mtu()];
        run_for(THROUGHPUT_WINDOW, async || write_full(&mut tx, &tx_buf).await).await;
        drop(tx);

        let mut tx = rf.tx_stream(&[0]).unwrap();
        tx.activate().await.unwrap();
        write_full(&mut tx, &tx_buf).await;
        tx.deactivate().await.unwrap();
    }
}
