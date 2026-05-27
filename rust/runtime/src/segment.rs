//! `Segment` and `KinematicTag` — runtime per-segment record. Spec §3.1.
//!
//! Distinct from `geometry::Segment`. Step 7 MVP wires the converter at the
//! Layer-3-to-Layer-4 boundary.

use crate::config::EMode;
use crate::curve_pool::CurveHandle;

/// Bit positions inside [`Segment::consumers_remaining`] mask.
///
/// Each curve handle (x / y / z / e) occupies a u4 nibble; the bits inside
/// the nibble identify which motor still needs the curve.
///
/// **Lockstep simplification (Task 5 MVP).** For the single-segment-active
/// workload that the bench produces today, all four motor producers finish
/// a segment's curves before any of them advances to the next segment.
/// Under that invariant the nibble structure is over-specified — the engine
/// retires the whole segment once any motor reports "done" because the
/// others have already reported "done" within the same producer_step call.
/// The full per-motor bookkeeping arrives when truly-independent per-motor
/// cursors land (post-MVP, see spec §7.2).
///
/// Layout (bit index, motor-bit-within-nibble):
/// - bits 0..3   = x curve consumer mask (motor 0/1/2/3 bit)
/// - bits 4..7   = y curve consumer mask
/// - bits 8..11  = z curve consumer mask
/// - bits 12..15 = e curve consumer mask
pub const CONS_REMAINING_X_SHIFT: u32 = 0;
pub const CONS_REMAINING_Y_SHIFT: u32 = 4;
pub const CONS_REMAINING_Z_SHIFT: u32 = 8;
pub const CONS_REMAINING_E_SHIFT: u32 = 12;

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

/// Per-segment record pushed through the SPSC queue from foreground to ISR.
///
/// Step 7-B: four per-axis curve handles (X, Y, Z, E) replace the single
/// `curve_handle`. `e_mode` selects extruder evaluation strategy;
/// `extrusion_ratio` carries the `extrusion_per_xy_mm` scalar for
/// `CoupledToXy` mode.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Segment {
    /// Stable monotonic identifier set by the producer; appears in trace samples.
    pub id: u32,
    /// Per-axis curve handles into `CurvePool`. The producer guarantees each
    /// slot is loaded before pushing this Segment. Use
    /// `CurveHandle::UNUSED_SENTINEL` for axes with no curve (e.g. Z on a
    /// planar move, or E in Travel mode).
    pub x_handle: CurveHandle,
    pub y_handle: CurveHandle,
    pub z_handle: CurveHandle,
    pub e_handle: CurveHandle,
    /// MCU clock cycles (see spec §4.1 — widened from CYCCNT inside Rust).
    pub t_start: u64,
    /// MCU clock cycles. Invariant: `t_end > t_start + MIN_SEGMENT_CYCLES`.
    pub t_end: u64,
    pub kinematics: KinematicTag,
    /// Extruder mode for this segment. See `EMode` doc.
    pub e_mode: EMode,
    /// §6.5 — bit 0 (`SEGMENT_FLAG_HOLD_SEGMENT`) is set on the in-band hold
    /// marker that primes the pipeline ahead of the armed `t_start`. Other
    /// bits reserved for future Step-6+ flags. Step-5 producer-side path
    /// always sets this to zero.
    pub flags: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 1],
    /// Extrusion ratio (extrusion_per_xy_mm) for `CoupledToXy` mode.
    /// Ignored when `e_mode != CoupledToXy`.
    pub extrusion_ratio: f32,
    /// Per-axis-curve consumer bitmask (spec §3.8 retirement decoupling).
    /// Each axis handle gets one u4 nibble; each bit inside the nibble marks
    /// a motor that still needs that curve. The producer clears its bit on
    /// Newton `SegmentExhausted` for its motor; the Modulated path clears
    /// its bit on wall-clock `t_end` cross. When all four nibbles read 0
    /// the segment's curve slots retire and the segment can be dropped.
    ///
    /// Constructors use [`Segment::compute_consumers_remaining`] to derive
    /// the initial mask from `kinematics` + the four handles (an UNUSED
    /// handle contributes no consumer bits).
    pub consumers_remaining: u16,
}

impl Segment {
    #[inline]
    pub fn duration(&self) -> u64 {
        self.t_end.saturating_sub(self.t_start)
    }

    /// Compute the initial `consumers_remaining` bitmask from the segment's
    /// four curve handles and kinematic transform.
    ///
    /// Each UNUSED handle contributes no consumer bits. By the time a
    /// `Segment` exists, curves are already in motor frame (the CoreXY
    /// transform A=X+Y / B=X−Y is applied by the bridge before the segment
    /// is constructed). The mapping is therefore always Cartesian:
    /// motor 0 ← x slot, motor 1 ← y slot, motor 2 ← z slot, motor 3 ← e slot,
    /// regardless of `kinematics`.
    ///
    /// The producer or the bridge calls this at segment construction time so
    /// the engine never sees a `Segment` with a stale or zero mask while
    /// curve handles are present.
    pub fn compute_consumers_remaining(
        _kinematics: KinematicTag,
        x_handle: CurveHandle,
        y_handle: CurveHandle,
        z_handle: CurveHandle,
        e_handle: CurveHandle,
    ) -> u16 {
        let mut mask: u16 = 0;
        // Motor-frame identity: each motor reads exactly its own slot.
        // kinematics is accepted for API compatibility (used as a wire tag)
        // but does not alter the consumer mapping — the CoreXY transform
        // was applied by the bridge before this segment was built.
        let motor_consumes_x = |i: u8| -> bool { i == 0 };
        let motor_consumes_y = |i: u8| -> bool { i == 1 };
        let motor_consumes_z = |i: u8| -> bool { i == 2 };
        let motor_consumes_e = |i: u8| -> bool { i == 3 };

        // For each axis: if its handle is non-UNUSED, set the bit for every
        // motor that consumes that axis.
        let set_for = |mask: &mut u16, handle: CurveHandle, shift: u32, consumes: &dyn Fn(u8) -> bool| {
            if handle.is_unused_sentinel() {
                return;
            }
            let mut nibble: u16 = 0;
            for motor in 0_u8..4 {
                if consumes(motor) {
                    nibble |= 1_u16 << motor;
                }
            }
            *mask |= (nibble & 0x0F) << shift;
        };
        set_for(&mut mask, x_handle, CONS_REMAINING_X_SHIFT, &motor_consumes_x);
        set_for(&mut mask, y_handle, CONS_REMAINING_Y_SHIFT, &motor_consumes_y);
        set_for(&mut mask, z_handle, CONS_REMAINING_Z_SHIFT, &motor_consumes_z);
        set_for(&mut mask, e_handle, CONS_REMAINING_E_SHIFT, &motor_consumes_e);
        mask
    }

    /// True iff every consumer bit has been cleared (all motors reading any
    /// of this segment's curves have finished). The segment's curve slots
    /// can be retired and the segment dropped from the queue.
    #[inline]
    pub fn consumers_done(&self) -> bool {
        self.consumers_remaining == 0
    }

    /// True iff motor `motor_idx` still has at least one un-cleared consumer
    /// bit in this segment's `consumers_remaining` mask.
    ///
    /// Used by the producer to skip motors that have already finished this
    /// segment's work — without this check, a finished motor whose
    /// `ProducerState` was cleared at `SegmentExhausted` would, on the next
    /// `producer_step` call, re-enter the "is_idle, fetch segment, start
    /// curve, find SegmentExhausted again, mark finished again" path and
    /// spuriously report `made_progress=true` every fire. That produces an
    /// infinite `WorkPending` self-reschedule at `SF_RESCHEDULE_FLOOR=100 µs`
    /// on the C side, pegging the SysTick dispatch loop at 10 kHz and
    /// starving foreground tasks (including `watchdog_reset`) until IWDG fires.
    pub fn motor_has_remaining_work(&self, motor_idx: u8) -> bool {
        let motor_bit: u16 = 1 << motor_idx;
        // Motor-frame identity: each motor reads exactly its own curve slot.
        // The CoreXY transform (A=X+Y, B=X−Y) is applied by the bridge;
        // by the time a Segment is constructed the curves are already motor-frame.
        let consumes_x = motor_idx == 0;
        let consumes_y = motor_idx == 1;
        let consumes_z = motor_idx == 2;
        let consumes_e = motor_idx == 3;
        let mut motor_mask: u16 = 0;
        if consumes_x {
            motor_mask |= motor_bit << CONS_REMAINING_X_SHIFT;
        }
        if consumes_y {
            motor_mask |= motor_bit << CONS_REMAINING_Y_SHIFT;
        }
        if consumes_z {
            motor_mask |= motor_bit << CONS_REMAINING_Z_SHIFT;
        }
        if consumes_e {
            motor_mask |= motor_bit << CONS_REMAINING_E_SHIFT;
        }
        (self.consumers_remaining & motor_mask) != 0
    }
}

#[cfg(test)]
mod tests;
