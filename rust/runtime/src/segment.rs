//! Segment and CurveHandle types — Task 6 piece-ring model.
//!
//! `CurveHandle` is the wire-format per-axis handle embedded in every
//! `Segment`. It was previously defined in `curve_pool` but lives here
//! because it is purely a segment-wire type; the curve-pool module has been
//! removed (2026-05-28).

use crate::config::EMode;

/// Wire-encoded `(generation << 16) | slot_idx` handle to a piece-ring axis
/// slot. `#[repr(C)]` so `TraceSample` and `Segment` stay ABI-compatible
/// with C consumers.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle {
    /// High 16 bits: generation. Low 16 bits: slot index.
    packed: u32,
}

impl CurveHandle {
    /// Sentinel value meaning "this axis has no curve" (all-ones pattern).
    pub const UNUSED_SENTINEL: Self = Self {
        packed: 0xFFFE_FFFE,
    };

    /// Sentinel for hold segments (differs from UNUSED so the ISR can
    /// distinguish "no work" from "hold position").
    pub const HOLD_SEGMENT_SENTINEL: Self = Self {
        packed: 0xFFFF_FFFF,
    };

    /// Construct from `slot_idx` and `generation`.
    #[inline]
    pub const fn new(slot_idx: u16, generation: u16) -> Self {
        Self {
            packed: ((generation as u32) << 16) | (slot_idx as u32),
        }
    }

    /// Pack to a `u32` for wire transmission.
    #[inline]
    pub const fn pack(self) -> u32 {
        self.packed
    }

    /// Unpack from a wire `u32`.
    #[inline]
    pub const fn unpack(v: u32) -> Self {
        Self { packed: v }
    }

    #[inline]
    pub const fn slot_idx(self) -> u16 {
        self.packed as u16
    }

    #[inline]
    pub const fn generation(self) -> u16 {
        (self.packed >> 16) as u16
    }

    /// Returns true if this handle is the UNUSED sentinel.
    #[inline]
    pub fn is_unused_sentinel(self) -> bool {
        self == Self::UNUSED_SENTINEL
    }
}

/// Kinematic transform tag.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicTag {
    CoreXyAndE = 0,
    CartesianXyzAndE = 1,
}

/// Flag bit: segment carries no motion — the ISR holds position for its
/// duration. Used for dwell / toolchange pauses and similar idle spans.
pub const SEGMENT_FLAG_HOLD_SEGMENT: u8 = 1 << 0;

/// Segment wire type — piece-ring model.
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
