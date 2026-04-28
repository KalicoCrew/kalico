//! `Segment` and `KinematicTag` — runtime per-segment record. Spec §3.1.
//!
//! Distinct from `geometry::Segment`. Step 7 MVP wires the converter at the
//! Layer-3-to-Layer-4 boundary.

/// Index into the static `CurvePool` slab (see `curve_pool` module).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle(pub u16);

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

#[derive(Debug, Clone, Copy)]
pub struct Segment {
    /// Stable monotonic identifier set by the producer; appears in trace samples.
    pub id: u32,
    /// Index into the static `CurvePool`. Producer guarantees the slot is loaded
    /// before pushing this Segment.
    pub curve: CurveHandle,
    /// MCU clock cycles (see spec §4.1 — widened from CYCCNT inside Rust).
    pub t_start: u64,
    /// MCU clock cycles. Invariant: `t_end > t_start + MIN_SEGMENT_CYCLES`.
    pub t_end: u64,
    pub kinematics: KinematicTag,
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
        assert!(core::mem::size_of::<Segment>() <= 64,
            "Segment grew too large: {} bytes", core::mem::size_of::<Segment>());
    }

    #[test]
    fn segment_duration_returns_t_end_minus_t_start() {
        let seg = Segment {
            id: 1,
            curve: CurveHandle(0),
            t_start: 100,
            t_end: 350,
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert_eq!(seg.duration(), 250);
    }

    #[test]
    fn segment_is_copy_clone() {
        let seg = Segment {
            id: 0, curve: CurveHandle(0), t_start: 0, t_end: 100,
            kinematics: KinematicTag::CoreXyAndE,
        };
        let _ = seg;     // copy
        // Verify Clone derive exists; suppress lint since Copy is also derived.
        #[allow(clippy::clone_on_copy)]
        let _ = seg.clone();
    }
}
