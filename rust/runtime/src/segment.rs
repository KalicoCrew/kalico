//! Wire-ABI type stubs retained for `motion-bridge` compatibility.
//!
//! All segment-era machinery (queue, pool, trace, stream lifecycle) has been
//! removed. This module exists solely to export [`KinematicTag`], which
//! `motion-bridge::dispatch` pins as a compile-time ABI constant derived from
//! the MCU-side discriminant. The rest of `segment.rs` — `Segment`, `EMode`,
//! `CurveHandle`, flags — have been deleted with the segment-era data path.

/// Kinematic transform tag — identifies the motor-frame transform the MCU
/// applies to X/Y before evaluating the Bézier pieces.
///
/// This discriminant is embedded in the MCU wire protocol (see
/// `dispatch.rs:KINEMATICS_COREXY`) and must never be renumbered without a
/// matching change on both sides of the host/MCU boundary.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    /// `CoreXY`: host pre-applies A = X+Y / B = X−Y; MCU receives motor-frame
    /// curves in its X/Y slots and drives two motors per logical axis.
    CoreXyAndE = 0,
    /// Cartesian: X, Y, Z, E map 1-to-1 to MCU axis slots.
    CartesianXyzAndE = 1,
}
