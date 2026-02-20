pub mod control_loop;
pub mod error;
pub mod model;
pub mod transport;

pub use control_loop::{ControlLoopConfig, ControlLoopHandle, MotorCommand, MotorSnapshot};
pub use error::MotorError;
pub use model::{MotorId, ServoModel};
pub use transport::MotorTransport;
