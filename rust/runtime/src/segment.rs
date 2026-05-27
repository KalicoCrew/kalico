//! Segment stub — Task 5 placeholder.
//!
//! The full `Segment` type (with kinematics, E-mode, consumer mask logic) has
//! been removed. This stub retains the minimum needed for `state.rs` and
//! `c_segment_queue` to compile until Task 6 introduces the new
//! `PieceRing`-based segment representation.

use crate::config::EMode;
use crate::curve_pool::CurveHandle;

/// Kinematic transform tag.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    CoreXyAndE = 0,
    CartesianXyzAndE = 1,
}

/// Stub segment — Task 6 replaces with the `PieceRing` contract.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub id: u32,
    pub x_handle: CurveHandle,
    pub y_handle: CurveHandle,
    pub z_handle: CurveHandle,
    pub e_handle: CurveHandle,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: KinematicTag,
    /// Extruder evaluation mode: CoupledToXy / Independent / Travel.
    pub e_mode: EMode,
    /// mm of extrusion per mm of XY arc-length (used when e_mode == CoupledToXy).
    pub extrusion_ratio: f32,
    pub flags: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 1],
    pub consumers_remaining: u8,
}

impl Segment {
    /// Compute the consumer-participation bitmask from the four per-axis
    /// handles. A bit is set for axis `i` iff `handles[i]` is not the
    /// UNUSED sentinel.
    pub fn compute_consumers_remaining(
        _kin: KinematicTag,
        x: CurveHandle,
        y: CurveHandle,
        z: CurveHandle,
        e: CurveHandle,
    ) -> u8 {
        let mut mask: u8 = 0;
        if !x.is_unused_sentinel() {
            mask |= 1 << 0;
        }
        if !y.is_unused_sentinel() {
            mask |= 1 << 1;
        }
        if !z.is_unused_sentinel() {
            mask |= 1 << 2;
        }
        if !e.is_unused_sentinel() {
            mask |= 1 << 3;
        }
        mask
    }
}
