# sts-scs-motor-controller

Async motor controller architecture for **Feetech STS3215** and **Feetech SCS0009**.

This project mirrors the queue + control-loop pattern used in Reachy Mini's motor controller, but targets Feetech protocol families.

## Status

- ✅ Tokio-based control loop + command queue
- ✅ Error model + health snapshot
- ✅ Feetech transport wired via `rustypot`:
  - `Sts3215Controller`
  - `Scs0009Controller`
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

## Notes

- The library assumes **single owner of the serial bus** (through the control loop).
- Default tested baudrate for current setup: **1_000_000**.
- If CI fails on Linux with serial dependencies, ensure `pkg-config` and `libudev-dev` are installed.

## Roadmap

- Model-specific safety limits and conversion helpers
- Better diagnostics/recovery helpers
- Optional Python bindings (PyO3/maturin)

## License

MIT
