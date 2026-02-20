# sts-scs-motor-controller

Async motor controller architecture for **Feetech STS3215** and **Feetech SCS0009**.

This project mirrors the queue + control-loop pattern used in Reachy Mini's motor controller, but targets Feetech protocol families.

## Status

- ✅ Tokio-based control loop + command queue
- ✅ Setup checks on startup:
  - missing IDs scan
  - optional voltage threshold check
- ✅ Error model + health snapshot
- ✅ Feetech transport wired via `rustypot`:
  - `Sts3215Controller`
  - `Scs0009Controller`
- ✅ Optional Python bindings (feature: `python`)
- ✅ CI workflow for build + tests

## Design

- One background control loop owns bus access
- Commands are sent through an async channel
- Periodic read loop runs at configurable frequency
- Last known state and last error are exposed for monitoring

## Quick start

```bash
git clone https://github.com/koenvanwijk/sts-scs-motor-controller.git
cd sts-scs-motor-controller
cargo check
cargo test
```

## Minimal usage (STS3215)

```rust
use std::time::Duration;
use sts_scs_motor_controller::{
    start_control_loop, ControlLoopConfig, FeetechTransport, MotorId,
};

let transport = FeetechTransport::new_sts3215(
    "/dev/tty_pink_follower_so101", // or your tty device
    1_000_000,
    Duration::from_millis(20),
)?;

let ids = vec![MotorId(1), MotorId(2), MotorId(3), MotorId(4), MotorId(5), MotorId(6)];
let handle = start_control_loop(transport, ids, ControlLoopConfig::default())?;

let snap = handle.last_snapshot()?;
println!("positions: {:?}", snap.positions);
```

## Python binding

Enable with feature flag:

```bash
cargo build --features python
```

Exposed class: `FeetechPyController`
- `new_sts3215(serial_port, ids, baudrate=1_000_000, timeout_ms=20)`
- `get_last_positions()`
- `set_goal_positions(ids, positions)`
- `set_torque(ids, enabled)`
- `close()`

## Conversion helpers (4096 ticks/rev + configurable zero)

`EncoderConversion` is included for 12-bit encoders:

```rust
use sts_scs_motor_controller::EncoderConversion;

let conv = EncoderConversion::new(1234); // configurable logical zero
let rad = conv.tick_to_rad(3500);
let tick = conv.rad_to_tick(rad);
```

- resolution: `4096` ticks per full revolution
- supports shifted zero-point (`zero_tick`)
- angle normalization is wrap-safe

## Notes

- The library assumes **single owner of the serial bus** (through the control loop).
- Default tested baudrate for current setup: **1_000_000**.
- Startup checks include missing-ID scan and optional voltage threshold check (default threshold `45`, i.e. ~4.5V in 0.1V units).
- If CI fails on Linux with serial dependencies, ensure `pkg-config` and `libudev-dev` are installed.

## Roadmap

- Model-specific safety limits and conversion helpers
- Better diagnostics/recovery helpers
- Optional Python bindings (PyO3/maturin)

## License

MIT
