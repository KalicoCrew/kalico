//! `Segment` and `KinematicTag` — runtime per-segment record. Spec §3.1.
//!
//! Distinct from `geometry::Segment`. Step 7 MVP wires the converter at the
//! Layer-3-to-Layer-4 boundary.

use crate::curve_pool::CurveHandle;

/// Selects the kinematic transform applied per tick.
///
/// Step 5 only emits `CoreXyAndE` (`CoreXY` for AB axes + identity for E).
/// `CartesianXyz` and `CartesianXyzAndE` are reserved slots for Step 6+ when
/// the F4x Z-only path lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KinematicTag {
    CoreXyAndE = 0,
    CartesianXyzAndE = 1,
}

/// `HOLD_SEGMENT` marker bit (§6.5). The ISR short-circuits on this bit
/// before looking up the curve handle, so a hold segment never resolves
/// through `CurvePool::lookup`.
pub const SEGMENT_FLAG_HOLD_SEGMENT: u8 = 1 << 0;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Segment {
    /// Stable monotonic identifier set by the producer; appears in trace samples.
    pub id: u32,
    /// Index into the static `CurvePool` (slot + generation; spec §10.1). The
    /// producer guarantees the slot is loaded before pushing this Segment.
    pub curve_handle: CurveHandle,
    /// MCU clock cycles (see spec §4.1 — widened from CYCCNT inside Rust).
    pub t_start: u64,
    /// MCU clock cycles. Invariant: `t_end > t_start + MIN_SEGMENT_CYCLES`.
    pub t_end: u64,
    pub kinematics: KinematicTag,
    /// §6.5 — bit 0 (`SEGMENT_FLAG_HOLD_SEGMENT`) is set on the in-band hold
    /// marker that primes the pipeline ahead of the armed `t_start`. Other
    /// bits reserved for future Step-6+ flags. Step-5 producer-side path
    /// always sets this to zero.
    pub flags: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 2],
}

impl Segment {
    #[inline]
    pub fn duration(&self) -> u64 {
        self.t_end.saturating_sub(self.t_start)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_size_is_under_64_bytes() {
        // Spec §4.7 / §3.1: small POD to minimize SPSC enqueue/dequeue memcpy cost.
        assert!(
            core::mem::size_of::<Segment>() <= 64,
            "Segment grew too large: {} bytes",
            core::mem::size_of::<Segment>()
        );
    }

    #[test]
    fn segment_size_locked_at_32_bytes() {
        // After Step-6 §10.1: id(4) + curve_handle(4) + t_start(8) + t_end(8) +
        // kinematics(1) + flags(1) + _pad(2) = 28 bytes; aligned to 8 (because
        // of u64 fields) → 32 bytes total. Adding fields without bumping
        // _pad will fail this assert.
        assert_eq!(core::mem::size_of::<Segment>(), 32);
    }

    #[test]
    fn segment_duration_returns_t_end_minus_t_start() {
        let seg = Segment {
            id: 1,
            curve_handle: CurveHandle::new(0, 1),
            t_start: 100,
            t_end: 350,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        };
        assert_eq!(seg.duration(), 250);
    }

    #[test]
    fn segment_is_copy_clone() {
        let seg = Segment {
            id: 0,
            curve_handle: CurveHandle::new(0, 1),
            t_start: 0,
            t_end: 100,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        };
        let _ = seg; // copy
        // Verify Clone derive exists; suppress lint since Copy is also derived.
        #[allow(clippy::clone_on_copy)]
        let _ = seg.clone();
    }
}
