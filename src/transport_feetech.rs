use std::time::Duration;

use rustypot::servo::feetech::{scs0009::Scs0009Controller, sts3215::Sts3215Controller};

use crate::{error::MotorError, model::MotorId, transport::MotorTransport};

enum FeetechBus {
    Sts3215(Sts3215Controller),
    Scs0009(Scs0009Controller),
}

pub struct FeetechTransport {
    bus: FeetechBus,
}

impl FeetechTransport {
    pub fn new_sts3215(serial_port: &str, baudrate: u32, timeout: Duration) -> Result<Self, MotorError> {
        let io = serialport::new(serial_port, baudrate)
            .timeout(timeout)
            .open()
            .map_err(|_| MotorError::Communication)?;

        let controller = Sts3215Controller::new()
            .with_protocol_v1()
            .with_serial_port(io);

        Ok(Self { bus: FeetechBus::Sts3215(controller) })
    }

    pub fn new_scs0009(serial_port: &str, baudrate: u32, timeout: Duration) -> Result<Self, MotorError> {
        let io = serialport::new(serial_port, baudrate)
            .timeout(timeout)
            .open()
            .map_err(|_| MotorError::Communication)?;

        let controller = Scs0009Controller::new()
            .with_protocol_v1()
            .with_serial_port(io);

        Ok(Self { bus: FeetechBus::Scs0009(controller) })
    }
}

impl MotorTransport for FeetechTransport {
    fn scan_missing_ids(&mut self, all_ids: &[MotorId]) -> Result<Vec<MotorId>, MotorError> {
        let mut missing = Vec::new();

        for id in all_ids {
            let ids = vec![id.0];
            let ok = match &mut self.bus {
                FeetechBus::Sts3215(c) => c.sync_read_id(&ids).is_ok(),
                FeetechBus::Scs0009(c) => c.sync_read_id(&ids).is_ok(),
            };

            if !ok {
                missing.push(*id);
            }
        }

        Ok(missing)
    }

    fn read_positions(&mut self, all_ids: &[MotorId]) -> Result<Vec<f64>, MotorError> {
        let ids: Vec<u8> = all_ids.iter().map(|x| x.0).collect();

        match &mut self.bus {
            FeetechBus::Sts3215(c) => c
                .sync_read_present_position(&ids)
                .map_err(|_| MotorError::Communication),
            FeetechBus::Scs0009(c) => c
                .sync_read_present_position(&ids)
                .map_err(|_| MotorError::Communication),
        }
    }

    fn write_goal_positions(&mut self, ids: &[MotorId], positions: &[f64]) -> Result<(), MotorError> {
        let ids: Vec<u8> = ids.iter().map(|x| x.0).collect();

        if ids.len() != positions.len() {
            return Err(MotorError::InvalidResponse);
        }

        match &mut self.bus {
            FeetechBus::Sts3215(c) => c
                .sync_write_goal_position(&ids, &positions.to_vec())
                .map_err(|_| MotorError::Communication),
            FeetechBus::Scs0009(c) => c
                .sync_write_goal_position(&ids, &positions.to_vec())
                .map_err(|_| MotorError::Communication),
        }
    }

    fn set_torque(&mut self, ids: &[MotorId], enable: bool) -> Result<(), MotorError> {
        let ids: Vec<u8> = ids.iter().map(|x| x.0).collect();
        let values: Vec<u8> = vec![if enable { 1 } else { 0 }; ids.len()];

        match &mut self.bus {
            FeetechBus::Sts3215(c) => c
                .sync_write_torque_enable(&ids, &values)
                .map_err(|_| MotorError::Communication),
            FeetechBus::Scs0009(c) => c
                .sync_write_torque_enable(&ids, &values)
                .map_err(|_| MotorError::Communication),
        }
    }

    fn read_voltages(&mut self, all_ids: &[MotorId]) -> Result<Vec<u8>, MotorError> {
        let ids: Vec<u8> = all_ids.iter().map(|x| x.0).collect();
        match &mut self.bus {
            FeetechBus::Sts3215(c) => c
                .sync_read_present_voltage(&ids)
                .map_err(|_| MotorError::Communication),
            FeetechBus::Scs0009(c) => c
                .sync_read_present_voltage(&ids)
                .map_err(|_| MotorError::Communication),
        }
    }
}
