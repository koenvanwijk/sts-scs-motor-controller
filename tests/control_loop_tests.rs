use std::{thread, time::Duration};

use sts_scs_motor_controller::{
    start_control_loop, ControlLoopConfig, MotorCommand, MotorError, MotorId, MotorTransport,
};

#[derive(Default)]
struct DummyTransport {
    torque_enabled: bool,
    last_positions: Vec<f64>,
}

impl MotorTransport for DummyTransport {
    fn scan_missing_ids(&mut self, _all_ids: &[MotorId]) -> Result<Vec<MotorId>, MotorError> {
        Ok(vec![])
    }

    fn read_positions(&mut self, all_ids: &[MotorId]) -> Result<Vec<f64>, MotorError> {
        if self.last_positions.len() != all_ids.len() {
            self.last_positions = vec![0.0; all_ids.len()];
        }
        Ok(self.last_positions.clone())
    }

    fn write_goal_positions(
        &mut self,
        ids: &[MotorId],
        positions: &[f64],
    ) -> Result<(), MotorError> {
        if ids.len() != positions.len() {
            return Err(MotorError::InvalidResponse);
        }
        self.last_positions = positions.to_vec();
        Ok(())
    }

    fn set_torque(&mut self, _ids: &[MotorId], enable: bool) -> Result<(), MotorError> {
        self.torque_enabled = enable;
        Ok(())
    }
}

#[test]
fn control_loop_updates_snapshot() {
    let ids = vec![MotorId(1), MotorId(2), MotorId(3)];
    let cfg = ControlLoopConfig {
        read_period: Duration::from_millis(5),
        channel_capacity: 32,
    };

    let handle = start_control_loop(DummyTransport::default(), ids.clone(), cfg).unwrap();

    handle
        .push(MotorCommand::SetGoalPositions {
            ids: ids.clone(),
            positions: vec![1.0, 2.0, 3.0],
        })
        .unwrap();

    thread::sleep(Duration::from_millis(30));

    let snap = handle.last_snapshot().unwrap();
    assert_eq!(snap.ids.len(), 3);
    assert_eq!(snap.positions, vec![1.0, 2.0, 3.0]);
    assert!(snap.timestamp_s > 0.0);

    handle.close();
}

#[test]
fn control_loop_rejects_missing_motors() {
    struct MissingTransport;
    impl MotorTransport for MissingTransport {
        fn scan_missing_ids(&mut self, all_ids: &[MotorId]) -> Result<Vec<MotorId>, MotorError> {
            Ok(vec![all_ids[0]])
        }
        fn read_positions(&mut self, _all_ids: &[MotorId]) -> Result<Vec<f64>, MotorError> {
            Ok(vec![])
        }
        fn write_goal_positions(
            &mut self,
            _ids: &[MotorId],
            _positions: &[f64],
        ) -> Result<(), MotorError> {
            Ok(())
        }
        fn set_torque(&mut self, _ids: &[MotorId], _enable: bool) -> Result<(), MotorError> {
            Ok(())
        }
    }

    let err = start_control_loop(
        MissingTransport,
        vec![MotorId(1), MotorId(2)],
        ControlLoopConfig::default(),
    )
    .unwrap_err();

    match err {
        MotorError::MissingMotors(ids) => assert_eq!(ids, vec![1]),
        other => panic!("unexpected error: {other:?}"),
    }
}
