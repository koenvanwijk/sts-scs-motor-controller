use std::{sync::Mutex, time::Duration};

use pyo3::prelude::*;

use crate::{
    start_control_loop, ControlLoopConfig, ControlLoopHandle, FeetechTransport, MotorCommand,
    MotorError, MotorId,
};

fn to_py_err(e: MotorError) -> PyErr {
    pyo3::exceptions::PyRuntimeError::new_err(e.to_string())
}

#[pyclass]
pub struct FeetechPyController {
    handle: Mutex<Option<ControlLoopHandle>>,
}

#[pymethods]
impl FeetechPyController {
    #[staticmethod]
    pub fn new_sts3215(
        serial_port: String,
        ids: Vec<u8>,
        baudrate: Option<u32>,
        timeout_ms: Option<u64>,
    ) -> PyResult<Self> {
        let transport = FeetechTransport::new_sts3215(
            &serial_port,
            baudrate.unwrap_or(1_000_000),
            Duration::from_millis(timeout_ms.unwrap_or(20)),
        )
        .map_err(to_py_err)?;

        let all_ids = ids.into_iter().map(MotorId).collect::<Vec<_>>();
        let cfg = ControlLoopConfig::default();
        let handle = start_control_loop(transport, all_ids, cfg).map_err(to_py_err)?;

        Ok(Self {
            handle: Mutex::new(Some(handle)),
        })
    }

    pub fn get_last_positions(&self) -> PyResult<Vec<f64>> {
        let guard = self
            .handle
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("controller lock poisoned"))?;
        let handle = guard
            .as_ref()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("controller is closed"))?;
        let snap = handle.last_snapshot().map_err(to_py_err)?;
        Ok(snap.positions)
    }

    pub fn set_goal_positions(&self, ids: Vec<u8>, positions: Vec<f64>) -> PyResult<()> {
        let guard = self
            .handle
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("controller lock poisoned"))?;
        let handle = guard
            .as_ref()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("controller is closed"))?;

        handle
            .push(MotorCommand::SetGoalPositions {
                ids: ids.into_iter().map(MotorId).collect(),
                positions,
            })
            .map_err(to_py_err)
    }

    pub fn set_torque(&self, ids: Vec<u8>, enabled: bool) -> PyResult<()> {
        let guard = self
            .handle
            .lock()
            .map_err(|_| pyo3::exceptions::PyRuntimeError::new_err("controller lock poisoned"))?;
        let handle = guard
            .as_ref()
            .ok_or_else(|| pyo3::exceptions::PyRuntimeError::new_err("controller is closed"))?;

        handle
            .push(MotorCommand::SetTorque {
                ids: ids.into_iter().map(MotorId).collect(),
                enabled,
            })
            .map_err(to_py_err)
    }

    pub fn close(&self) {
        if let Ok(mut guard) = self.handle.lock() {
            if let Some(handle) = guard.as_ref() {
                handle.close();
            }
            *guard = None;
        }
    }
}

#[pymodule]
fn sts_scs_motor_controller_py(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<FeetechPyController>()?;
    Ok(())
}
