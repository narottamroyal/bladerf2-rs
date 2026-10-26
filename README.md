# bladerf2

[![CI](https://github.com/narottamroyal/bladerf2-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/narottamroyal/bladerf2-rs/actions/workflows/ci.yml)

A pure Rust driver for the [bladeRF 2.0 micro](https://www.nuand.com/bladerf-2-0-micro/),
using [`nusb`](https://crates.io/crates/nusb) for USB. It needs no native libraries, and
runs on any platform that `nusb` supports as well as in the browser
(`wasm32-unknown-unknown`) over WebUSB.

- Every device operation returns a `MaybeFuture`: call `.wait()` to block, or `.await` it.
- Radio configuration goes through an `RfLinkSession`, which borrows the device
  mutably, so only one can exist at a time.
- `RxStream` and `TxStream` own their USB endpoints, so streaming does not wait
  on configuration calls.

## Usage

```toml
[dependencies]
bladerf2 = "0.1"
```

### Blocking

```rust
use bladerf2::{BladeRf2, Direction, MaybeFuture};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut dev = BladeRf2::open_any().wait()?;
    println!("serial {}, FPGA {}", dev.serial()?, dev.fpga_version().wait()?);

    let rf = dev.rf_link_session();
    rf.set_frequency(Direction::Rx, 100_000_000).wait()?;
    rf.set_sample_rate(2_000_000).wait()?;

    let mut rx = rf.rx_stream(&[0])?;
    rx.activate().wait()?;
    let mut samples = vec![num_complex::Complex32::ZERO; rx.mtu()];
    rx.read(&mut [&mut samples], None).wait()?;
    rx.deactivate().wait()?;
    Ok(())
}
```

### Async

```rust
let info = BladeRf2::list_devices().await?.into_iter().next().ok_or("no device")?;
let mut dev = BladeRf2::open_device(info).await?;
let rf = dev.rf_link_session();
rf.set_frequency(Direction::Rx, 100_000_000).await?;
```

On native targets, `.await` needs the `smol` or `tokio` feature so that `nusb` can
run blocking USB calls on a runtime. The blocking API and the browser need neither.

### In the browser

The browser only lists devices that the user has granted to the page. Ask for
one with `navigator.usb.requestDevice` from a user gesture, such as a button
click, then use `BladeRf2::list_devices()` and `BladeRf2::open_device(info)` as
above. The `rx_web` example shows the whole flow.

## Examples

Each example prints the device information, lists the supported ranges, changes
each setting and reads it back, and receives a few buffers of samples.

| Example | Run with |
| --- | --- |
| `rx_sync` | `cargo run --release --example rx_sync` |
| `rx_async` | `cargo run --release --features smol --example rx_async` |
| `rx_web` | `cd examples/rx_web && trunk serve --open` (needs [`trunk`](https://trunkrs.dev) and a Chromium-based browser) |

## Things to know

- **USB 3.0.** Streaming needs a SuperSpeed connection. On native targets,
  opening a device on a slower link fails with `Error::UsbSpeed`. WebUSB cannot
  report the link speed, so the browser does not check it.
- **Permissions.** On Linux, allow access to the device with a udev rule in
  `/etc/udev/rules.d/`; the browser needs the same access:
  `SUBSYSTEM=="usb", ATTRS{idVendor}=="2cf0", ATTRS{idProduct}=="5250", MODE="0666"`.
  On Windows, the device needs a WinUSB driver, which you can install with Zadig.
- **Sample rate.** The radio powers up at 30.72 MS/s. Set a rate you can sustain
  before streaming.
- **RX gain.** The radio only applies a new RX gain while an RX channel is
  enabled. With none enabled, `set_gain()` briefly enables one so that `gain()`
  reads the new value back.
- **Deactivate streams.** On native targets, dropping an active stream stops it.
  In the browser it keeps running until the next stream in that direction is
  activated, or the device is opened again.
- **MIMO loopback.** FPGA 0.16.0 loses samples in MIMO digital loopback. SISO
  loopback is unaffected.

## Testing

Unit tests need no hardware:

```sh
cargo test
```

Hardware tests need a bladeRF 2.0 micro and the `hardware-tests` feature.
Because they share the device, they must run one at a time. Each test runs in up
to three modes: `blocking`, `asynchronous` (which needs the `smol` feature) and
`browser`.

Blocking:

```sh
cargo test --features hardware-tests -- --test-threads=1
```

Async:

```sh
cargo test --features hardware-tests,smol -- --test-threads=1 asynchronous
```

Both:

```sh
cargo test --features hardware-tests,smol -- --test-threads=1
```

To choose a device when several are attached, set `BLADERF2_SERIAL` to a prefix
of its serial number:

```sh
BLADERF2_SERIAL=c504 cargo test --features hardware-tests -- --test-threads=1
```

### Browser tests

The browser tests run in headless Chromium, and have only been validated on
Linux. They need `chromium` and `chromedriver`; `chromedriver` must be on `PATH`,
or set `CHROMEDRIVER` to its location.

Before the first run, install a Chromium policy that lets the test page use the
device without a permission prompt:

```sh
sudo scripts/browser-policy.sh install
```

Run the tests:

```sh
cargo test --target wasm32-unknown-unknown --features hardware-tests --test hardware
```

Remove the policy:

```sh
sudo scripts/browser-policy.sh remove
```

The policy only applies to the bladeRF 2.0 micro on `http://127.0.0.1:8123`, the
test server's address, so port 8123 must be free while the tests run. If a test
panics, the remaining browser tests time out.

## License

Licensed under either of Apache License, Version 2.0 or MIT license at your option.
