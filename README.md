# sts-scs-motor-controller

Async motor controller architecture for **Feetech STS3215** and **Feetech SCS0009**.

This project mirrors the queue + control-loop pattern used in Reachy Mini's motor controller, but targets Feetech protocol families.

## Status

- ✅ Project scaffolded
- ✅ Tokio-based control loop + command queue
- ✅ Error model and health state
- ✅ Feetech transport wired via `rustypot` (`Sts3215Controller` / `Scs0009Controller`)

## Design

- One background control loop owns bus access
- Commands are sent through an async channel
- Periodic read loop runs at configurable frequency
- Last known state and last error are exposed for monitoring

## Quick start

```bash
git clone https://github.com/kwijk/sts-scs-motor-controller.git
cd sts-scs-motor-controller
# build once Rust toolchain is installed
cargo check
```

## Next steps

1. Implement `MotorTransport` for your Feetech serial protocol implementation.
2. Wire model-specific register maps for STS3215 and SCS0009.
3. Add Python bindings (PyO3/maturin) if needed.

## License

MIT
