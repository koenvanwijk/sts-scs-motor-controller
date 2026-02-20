use thiserror::Error;

#[derive(Debug, Error, Clone)]
pub enum MotorError {
    #[error("serial/bus communication error")]
    Communication,
    #[error("missing motors: {0:?}")]
    MissingMotors(Vec<u8>),
    #[error("invalid response")]
    InvalidResponse,
    #[error("transport closed")]
    Closed,
    #[error("feature unsupported by transport")]
    Unsupported,
    #[error("voltage below threshold: {0}")]
    LowVoltage(u8),
}
