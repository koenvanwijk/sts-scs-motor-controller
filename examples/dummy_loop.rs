use std::{thread, time::Duration};

use sts_scs_motor_controller::{
    control_loop::start_control_loop, ControlLoopConfig, MotorCommand, MotorId, MotorTransport,
    MotorError,
};

struct DummyTransport;

impl MotorTransport for DummyTransport {
    fn scan_missing_ids(&mut self, _all_ids: &[MotorId]) -> Result<Vec<MotorId>, MotorError> {
        Ok(vec![])
    }

    fn read_positions(&mut self, all_ids: &[MotorId]) -> Result<Vec<f64>, MotorError> {
        Ok(vec![0.0; all_ids.len()])
    }

    fn write_goal_positions(&mut self, _ids: &[MotorId], _positions: &[f64]) -> Result<(), MotorError> {
        Ok(())
    }

    fn set_torque(&mut self, _ids: &[MotorId], _enable: bool) -> Result<(), MotorError> {
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    let ids = vec![MotorId(1), MotorId(2)];
    let handle = start_control_loop(DummyTransport, ids.clone(), ControlLoopConfig::default())?;
    handle.push(MotorCommand::SetTorque { ids: ids.clone(), enabled: true })?;
    handle.push(MotorCommand::SetGoalPositions { ids, positions: vec![10.0, -10.0] })?;
    thread::sleep(Duration::from_millis(50));
    let snap = handle.last_snapshot()?;
    println!("snapshot at {}: {:?}", snap.timestamp_s, snap.positions);
    handle.close();
    Ok(())
}
