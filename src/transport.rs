use crate::{error::MotorError, model::MotorId};

pub trait MotorTransport: Send + 'static {
    fn scan_missing_ids(&mut self, all_ids: &[MotorId]) -> Result<Vec<MotorId>, MotorError>;
    fn read_positions(&mut self, all_ids: &[MotorId]) -> Result<Vec<f64>, MotorError>;
    fn write_goal_positions(
        &mut self,
        ids: &[MotorId],
        positions: &[f64],
    ) -> Result<(), MotorError>;
    fn set_torque(&mut self, ids: &[MotorId], enable: bool) -> Result<(), MotorError>;

    fn read_voltages(&mut self, _all_ids: &[MotorId]) -> Result<Vec<u8>, MotorError> {
        Err(MotorError::Unsupported)
    }
}
