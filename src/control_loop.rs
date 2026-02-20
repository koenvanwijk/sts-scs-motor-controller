use std::{sync::{Arc, Mutex}, time::{Duration, SystemTime, UNIX_EPOCH}};

use tokio::{runtime::Runtime, sync::mpsc, time};
use tracing::warn;

use crate::{error::MotorError, model::MotorId, transport::MotorTransport};

#[derive(Debug, Clone, Copy)]
pub struct ControlLoopConfig {
    pub read_period: Duration,
    pub channel_capacity: usize,
    pub startup_voltage_threshold: Option<u8>,
    pub startup_voltage_timeout: Duration,
}

impl Default for ControlLoopConfig {
    fn default() -> Self {
        Self {
            read_period: Duration::from_millis(10),
            channel_capacity: 128,
            startup_voltage_threshold: Some(45),
            startup_voltage_timeout: Duration::from_secs(3),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MotorSnapshot {
    pub ids: Vec<MotorId>,
    pub positions: Vec<f64>,
    pub timestamp_s: f64,
}

#[derive(Debug, Clone)]
pub enum MotorCommand {
    SetGoalPositions { ids: Vec<MotorId>, positions: Vec<f64> },
    SetTorque { ids: Vec<MotorId>, enabled: bool },
}

pub struct ControlLoopHandle {
    tx: mpsc::Sender<MotorCommand>,
    last_snapshot: Arc<Mutex<Result<MotorSnapshot, MotorError>>>,
    stop: Arc<Mutex<bool>>,
}

impl ControlLoopHandle {
    pub fn push(&self, cmd: MotorCommand) -> Result<(), MotorError> {
        self.tx.blocking_send(cmd).map_err(|_| MotorError::Closed)
    }

    pub fn last_snapshot(&self) -> Result<MotorSnapshot, MotorError> {
        self.last_snapshot.lock().map_err(|_| MotorError::Closed)?.clone()
    }

    pub fn close(&self) {
        if let Ok(mut stop) = self.stop.lock() {
            *stop = true;
        }
    }
}

pub fn start_control_loop<T: MotorTransport>(
    mut transport: T,
    all_ids: Vec<MotorId>,
    cfg: ControlLoopConfig,
) -> Result<ControlLoopHandle, MotorError> {
    let missing = transport.scan_missing_ids(&all_ids)?;
    if !missing.is_empty() {
        return Err(MotorError::MissingMotors(missing.into_iter().map(|m| m.0).collect()));
    }

    if let Some(min_v) = cfg.startup_voltage_threshold {
        let t0 = std::time::Instant::now();
        loop {
            match transport.read_voltages(&all_ids) {
                Ok(v) if !v.is_empty() && v.iter().all(|x| *x >= min_v) => break,
                Ok(v) if !v.is_empty() && t0.elapsed() >= cfg.startup_voltage_timeout => {
                    let low = v.into_iter().min().unwrap_or(0);
                    return Err(MotorError::LowVoltage(low));
                }
                Err(MotorError::Unsupported) => break,
                Err(_) if t0.elapsed() >= cfg.startup_voltage_timeout => {
                    return Err(MotorError::Communication)
                }
                _ => std::thread::sleep(Duration::from_millis(100)),
            }
        }
    }

    let (tx, mut rx) = mpsc::channel::<MotorCommand>(cfg.channel_capacity);
    let stop = Arc::new(Mutex::new(false));
    let stop_clone = Arc::clone(&stop);

    let snapshot = Arc::new(Mutex::new(Err(MotorError::Communication)));
    let snapshot_clone = Arc::clone(&snapshot);

    std::thread::spawn(move || {
        Runtime::new().expect("tokio runtime").block_on(async move {
            let mut interval = time::interval(cfg.read_period);

            loop {
                tokio::select! {
                    Some(cmd) = rx.recv() => {
                        let res = match cmd {
                            MotorCommand::SetGoalPositions { ids, positions } => transport.write_goal_positions(&ids, &positions),
                            MotorCommand::SetTorque { ids, enabled } => transport.set_torque(&ids, enabled),
                        };
                        if let Err(e) = res {
                            warn!("command failed: {e}");
                        }
                    }
                    _ = interval.tick() => {
                        let read = transport.read_positions(&all_ids).map(|positions| {
                            let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or(Duration::ZERO).as_secs_f64();
                            MotorSnapshot { ids: all_ids.clone(), positions, timestamp_s: ts }
                        });
                        if let Ok(mut guard) = snapshot_clone.lock() {
                            *guard = read;
                        }
                    }
                }

                if *stop_clone.lock().unwrap_or_else(|p| p.into_inner()) {
                    break;
                }
            }
        });
    });

    Ok(ControlLoopHandle { tx, last_snapshot: snapshot, stop })
}
