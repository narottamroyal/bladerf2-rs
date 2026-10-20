# bladerf2

A pure Rust driver for the [bladeRF 2.0 micro](https://www.nuand.com/bladerf-2-0-micro/)
software-defined radio.

The driver talks to the radio directly over USB with [`nusb`](https://crates.io/crates/nusb).
There is no `libbladeRF` or other native library to install, and the same code
runs in the browser through WebUSB.

- **Blocking or async.** Every device operation returns a `MaybeFuture`: call
  `.wait()` to block (no async runtime needed) or `.await` it.
- **Native and WebAssembly.** Any platform supported by `nusb` (developed and
  tested on Linux), and `wasm32-unknown-unknown` through WebUSB.
- **Safe by construction.** All RF configuration goes through an `RfLinkSession`
  that borrows the device mutably, so the compiler guarantees only one session
  is active at a time.
- **Streaming.** `RxStream` and `TxStream` own their USB endpoints, so sample
  I/O never contends with configuration calls.

It is also the bladeRF 2.0 backend of [seify](https://github.com/FutureSDR/seify)
and therefore usable from [FutureSDR](https://www.futuresdr.org).

## Usage

```toml
[dependencies]
bladerf2 = "0.2"
```

### Blocking

```rust
use bladerf2::{BladeRf2, Direction, MaybeFuture};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut dev = BladeRf2::open_any()?;
    println!("serial {}, FPGA {}", dev.serial()?, dev.fpga_version().wait()?);

    let rf = dev.rf_link_session();
    rf.set_frequency(Direction::Rx, 100_000_000).wait()?;
    rf.set_sample_rate(2_000_000).wait()?;

    let mut rx = rf.rx_stream(&[0])?;
    rx.activate().wait()?;
    let mut samples = vec![num_complex::Complex32::ZERO; rx.mtu()];
    rx.read(&mut [&mut samples], None)?;
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

On native targets, `.await` needs one of the `smol` or `tokio` cargo features so
nusb can run blocking USB calls on a runtime; the blocking API and WebAssembly
need neither.

### In the browser

Browsers only expose USB devices the user has granted. Ask for permission with
`navigator.usb.requestDevice` from a user gesture (a button click), then use
`BladeRf2::list_devices()` and `BladeRf2::open_device(info)` as above: they
return the granted devices. The `rx_web` example shows the complete flow.

## Examples

`rx_sync`, `rx_async` and `rx_web` are deliberately near-identical. Each one
prints the serial number, FPGA version, VCTCXO trim, timestamp and loopback
state; lists the supported frequency, sample rate, bandwidth and gain ranges;
changes each setting and reads it back; and receives a few buffers of samples.

| Example | Run with |
| --- | --- |
| `rx_sync` | `cargo run --release --example rx_sync` |
| `rx_async` | `cargo run --release --features smol --example rx_async` |
| `rx_web` | `cd examples/rx_web && trunk serve --open` (needs [`trunk`](https://trunkrs.dev); use a Chromium-based browser) |

## Things to know

- **Sample rate at power-up.** The radio boots at 30.72 Msps. Set a rate you can
  sustain before streaming.
- **Setting RX gain.** The radio only applies a new RX gain while an RX channel
  is enabled. When none is running, `set_gain()` briefly (about 2 ms) enables and
  disables the RX path so that `gain()` reads the new value straight back.
- **Always `deactivate` streams.** Dropping an active stream deactivates it on a
  best-effort basis only (asynchronously on wasm), so deactivate explicitly.
- **USB 3.0.** Streaming needs a SuperSpeed connection. Where the platform
  reports the link speed, opening a device on a slower link fails with
  `Error::UsbSpeed`. WebUSB does not report it.

### Permissions

- **Linux:** allow access to the device with a udev rule, for example
  `SUBSYSTEM=="usb", ATTRS{idVendor}=="2cf0", ATTRS{idProduct}=="5250", MODE="0666"`
  in `/etc/udev/rules.d/`, or install Nuand's rules. The same permission is
  needed for WebUSB in the browser.
- **Windows:** the device needs a WinUSB-compatible driver (for example via Zadig).

## Status

Receive is exercised by the examples. The transmit path (`TxStream`) is
available but not covered by them.

## License

Licensed under either of Apache License, Version 2.0 or MIT license at your option.
