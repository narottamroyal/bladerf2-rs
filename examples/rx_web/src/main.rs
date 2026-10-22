//! WebUSB receive example: device information, settings, and RX.
//!
//! Prints device information, changes every setting and reads it back, then
//! receives a few buffers of samples. A button on the page starts the run
//! (WebUSB needs a user gesture), and output goes to the page and the browser
//! console.
//!
//! `rx_sync`, `rx_async` and `rx_web` are intentionally near-identical; only
//! how the device is opened and how operations are waited on differ.
//!
//! ```console
//! $ cargo install trunk   # once
//! $ cd examples/rx_web
//! $ trunk serve --open
//! ```
//!
//! Use a Chromium-based browser (Chrome, Edge, ...), which supports WebUSB.
//! `localhost` counts as a secure context, so no certificate is needed.

use bladerf2::{BladeRf2, Channel, Direction, GainMode};
use num_complex::Complex32;
use std::time::Duration;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::Closure;
use wasm_bindgen_futures::{JsFuture, spawn_local};

/// Print a line to the page and the browser console.
fn print(line: &str) {
    web_sys::console::log_1(&line.into());
    let output = web_sys::window()
        .and_then(|window| window.document())
        .and_then(|document| document.get_element_by_id("output"));
    if let Some(output) = output {
        output.set_text_content(Some(&format!(
            "{}{line}\n",
            output.text_content().unwrap_or_default()
        )));
    }
}

macro_rules! out {
    ($($arg:tt)*) => {
        print(&format!($($arg)*))
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

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    // ---- Open the device ----------------------------------------------------
    // The browser only lists devices the user has granted, and the permission
    // prompt needs a user gesture (here: the button click).
    let usb = web_sys::window().ok_or("no window")?.navigator().usb();
    let filter = web_sys::UsbDeviceFilter::new();
    filter.set_vendor_id(BladeRf2::VID);
    filter.set_product_id(BladeRf2::PID);
    let options = web_sys::UsbDeviceRequestOptions::new(&[filter]);
    JsFuture::from(usb.request_device(&options))
        .await
        .map_err(|e| format!("device request failed: {e:?}"))?;

    let info = BladeRf2::list_devices()
        .await?
        .into_iter()
        .next()
        .ok_or("no bladeRF 2.0 granted")?;
    let mut dev = BladeRf2::open_device(info).await?;

    // ---- Device information ------------------------------------------------
    out!("== Device ==");
    out!("serial:        {}", dev.serial()?);
    out!("FPGA version:  {}", dev.fpga_version().await?);
    out!("VCTCXO trim:   {:#06x}", dev.vctcxo_trim());

    // ---- Start an RF-link session ------------------------------------------
    // All configuration goes through the session. It borrows the device mutably,
    // so only one can exist at a time.
    let rf = dev.rf_link_session();
    out!("loopback:      {}", rf.loopback().await?);

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
    let gain = rf.gain_range(Direction::Rx).await?;
    out!(
        "gain:          {:.1} - {:.1} dB",
        gain.gain_range.start(),
        gain.gain_range.end()
    );

    // ---- Change settings and read them back ---------------------------------
    // Each line shows: value before -> value requested -> value read back.
    out!("\n== Settings (before -> requested -> after) ==");

    let before = rf.frequency(Direction::Rx).await?;
    rf.set_frequency(Direction::Rx, FREQUENCY).await?;
    let after = rf.frequency(Direction::Rx).await?;
    out!(
        "frequency:     {:.3} -> {:.3} -> {:.3} MHz",
        mhz(before),
        mhz(FREQUENCY),
        mhz(after)
    );

    let before = rf.sample_rate().await?;
    rf.set_sample_rate(SAMPLE_RATE).await?;
    let after = rf.sample_rate().await?;
    out!(
        "sample rate:   {:.3} -> {:.3} -> {:.3} MS/s",
        mhz(before),
        mhz(SAMPLE_RATE),
        mhz(after)
    );

    let before = rf.bandwidth(Direction::Rx).await?;
    rf.set_bandwidth(Direction::Rx, BANDWIDTH).await?;
    let after = rf.bandwidth(Direction::Rx).await?;
    out!(
        "bandwidth:     {:.3} -> {:.3} -> {:.3} MHz",
        mhz(before),
        mhz(BANDWIDTH),
        mhz(after)
    );

    let before = rf.gain_mode(Channel::Rx0).await?;
    rf.set_gain_mode(Channel::Rx0, GainMode::Manual).await?;
    let after = rf.gain_mode(Channel::Rx0).await?;
    out!(
        "gain mode:     {before:?} -> {:?} -> {after:?}",
        GainMode::Manual
    );

    let before = rf.gain(Channel::Rx0).await?;
    rf.set_gain(Channel::Rx0, GAIN_DB).await?;
    let after = rf.gain(Channel::Rx0).await?;
    out!("gain:          {before:.1} -> {GAIN_DB:.1} -> {after:.1} dB");

    // ---- Receive ------------------------------------------------------------
    let mut rx = rf.rx_stream(&[0])?;
    let mut samples = vec![Complex32::ZERO; rx.mtu()];
    out!("\n== Receive {BUFFERS} x {} samples ==", samples.len());
    rx.activate().await?;
    for i in 0..BUFFERS {
        let n = rx.read_async(&mut [&mut samples], Some(TIMEOUT)).await?;
        let power = samples[..n].iter().map(|s| s.norm_sqr()).sum::<f32>() / n as f32;
        let peak = samples[..n].iter().map(|s| s.norm()).fold(0.0, f32::max);
        out!(
            "buffer {i}: {n} samples, mean power {:.1} dBFS, peak {peak:.3}",
            10.0 * power.log10()
        );
    }
    // Always deactivate explicitly; dropping an active stream is best-effort only.
    rx.deactivate().await?;
    out!("done");
    Ok(())
}

fn main() {
    let document = web_sys::window()
        .and_then(|window| window.document())
        .expect("no document");
    let body = document.body().expect("no body");

    let output = document.create_element("pre").expect("create <pre>");
    output.set_id("output");
    let button = document.create_element("button").expect("create <button>");
    button.set_text_content(Some("Connect bladeRF 2.0 micro"));

    let on_click = Closure::<dyn FnMut()>::new(|| {
        spawn_local(async {
            if let Err(error) = run().await {
                out!("error: {error}");
            }
        });
    });
    button
        .add_event_listener_with_callback("click", on_click.as_ref().unchecked_ref())
        .expect("add click listener");
    on_click.forget();

    body.append_child(&button).expect("append button");
    body.append_child(&output).expect("append output");
}
