#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MotorId(pub u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoModel {
    Sts3215,
    Scs0009,
}
