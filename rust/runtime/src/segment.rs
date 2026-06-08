/// This discriminant is embedded in the MCU wire protocol (see
/// `dispatch.rs:KINEMATICS_COREXY`) and must never be renumbered without a
/// matching change on both sides of the host/MCU boundary.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    CoreXyAndE = 0,
    CartesianXyzAndE = 1,
}
