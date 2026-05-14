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
    /// Each UNUSED handle contributes no consumer bits. The kinematic
    /// transform determines which motors consume which axis curves:
    /// - `CartesianXyzAndE`: motor 0 ← x, 1 ← y, 2 ← z, 3 ← e.
    /// - `CoreXyAndE`: motors 0 (A = X+Y) and 1 (B = X−Y) both consume the
    ///   x AND y curves; motor 2 ← z; motor 3 ← e.
    ///
    /// The producer or the bridge calls this at segment construction time so
    /// the engine never sees a `Segment` with a stale or zero mask while
    /// curve handles are present.
    pub fn compute_consumers_remaining(
        kinematics: KinematicTag,
        x_handle: CurveHandle,
        y_handle: CurveHandle,
        z_handle: CurveHandle,
        e_handle: CurveHandle,
    ) -> u16 {
        let mut mask: u16 = 0;
        // Per-motor consumer presence flags. `motor_idx_consumes_axis(i,
        // axis)` returns true iff motor `i` reads `axis` under the
        // selected kinematics.
        let motor_consumes_x = |i: u8| -> bool {
            match kinematics {
                KinematicTag::CartesianXyzAndE => i == 0,
                KinematicTag::CoreXyAndE => i == 0 || i == 1,
            }
        };
        let motor_consumes_y = |i: u8| -> bool {
            match kinematics {
                KinematicTag::CartesianXyzAndE => i == 1,
                KinematicTag::CoreXyAndE => i == 0 || i == 1,
            }
        };
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
        let consumes_x = match self.kinematics {
            KinematicTag::CartesianXyzAndE => motor_idx == 0,
            KinematicTag::CoreXyAndE => motor_idx == 0 || motor_idx == 1,
        };
        let consumes_y = match self.kinematics {
            KinematicTag::CartesianXyzAndE => motor_idx == 1,
            KinematicTag::CoreXyAndE => motor_idx == 0 || motor_idx == 1,
        };
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
    fn segment_size_under_64_bytes_with_consumers_mask() {
        // Step-7-B base layout (48 B) + consumers_remaining (u16) =
        // ≤ 56 B after natural alignment. The repr(C) field ordering plus
        // tail-padding to 8-byte alignment lands the actual size on the
        // host build at 56 B. The looser ≤ 64 B assertion in
        // [`segment_size_is_under_64_bytes`] stays load-bearing for the
        // SPSC enqueue/dequeue memcpy budget; this exact-size assertion
        // is the canary for accidental field-order regressions.
        assert_eq!(core::mem::size_of::<Segment>(), 56);
    }

    #[test]
    fn segment_duration_returns_t_end_minus_t_start() {
        let seg = Segment {
            id: 1,
            x_handle: CurveHandle::new(0, 1),
            y_handle: CurveHandle::new(1, 1),
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 100,
            t_end: 350,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        assert_eq!(seg.duration(), 250);
    }

    #[test]
    fn motor_has_remaining_work_cartesian() {
        let mut seg = Segment {
            id: 1,
            x_handle: CurveHandle::new(0, 1),
            y_handle: CurveHandle::new(1, 1),
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::new(3, 1),
            t_start: 0,
            t_end: 1000,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        seg.consumers_remaining = Segment::compute_consumers_remaining(
            seg.kinematics,
            seg.x_handle,
            seg.y_handle,
            seg.z_handle,
            seg.e_handle,
        );

        // Cartesian: motor 0 ← X, 1 ← Y, 2 ← Z, 3 ← E. Z is UNUSED so
        // motor 2 has no work even before any clearing.
        assert!(seg.motor_has_remaining_work(0));
        assert!(seg.motor_has_remaining_work(1));
        assert!(!seg.motor_has_remaining_work(2)); // Z is UNUSED
        assert!(seg.motor_has_remaining_work(3));

        // Clear motor 0's X bit; motor 0 no longer has work.
        seg.consumers_remaining &= !(1u16 << CONS_REMAINING_X_SHIFT);
        assert!(!seg.motor_has_remaining_work(0));
        assert!(seg.motor_has_remaining_work(1));
        assert!(seg.motor_has_remaining_work(3));

        // Clear motor 1, motor 3. Now no motor has work; consumers_done.
        seg.consumers_remaining &= !(1u16 << (CONS_REMAINING_Y_SHIFT + 1));
        seg.consumers_remaining &= !(1u16 << (CONS_REMAINING_E_SHIFT + 3));
        assert!(!seg.motor_has_remaining_work(0));
        assert!(!seg.motor_has_remaining_work(1));
        assert!(!seg.motor_has_remaining_work(3));
        assert!(seg.consumers_done());
    }

    #[test]
    fn motor_has_remaining_work_corexy() {
        let mut seg = Segment {
            id: 1,
            x_handle: CurveHandle::new(0, 1),
            y_handle: CurveHandle::new(1, 1),
            z_handle: CurveHandle::new(2, 1),
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: 1000,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        seg.consumers_remaining = Segment::compute_consumers_remaining(
            seg.kinematics,
            seg.x_handle,
            seg.y_handle,
            seg.z_handle,
            seg.e_handle,
        );

        // CoreXY: motors 0 and 1 each consume BOTH X and Y. Motor 2 ← Z,
        // motor 3 ← E (UNUSED here, so no work).
        assert!(seg.motor_has_remaining_work(0));
        assert!(seg.motor_has_remaining_work(1));
        assert!(seg.motor_has_remaining_work(2));
        assert!(!seg.motor_has_remaining_work(3));

        // Clear motor 0 across BOTH X and Y nibbles — that's what
        // clear_motor_bits_in_mask does for CoreXY motor 0 when it finishes.
        seg.consumers_remaining &= !(1u16 << CONS_REMAINING_X_SHIFT);
        seg.consumers_remaining &= !(1u16 << CONS_REMAINING_Y_SHIFT);
        assert!(!seg.motor_has_remaining_work(0));
        // Motor 1 still has its bit in the same nibbles.
        assert!(seg.motor_has_remaining_work(1));
    }

    #[test]
    fn segment_is_copy_clone() {
        let seg = Segment {
            id: 0,
            x_handle: CurveHandle::new(0, 1),
            y_handle: CurveHandle::new(1, 1),
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: 100,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        let _ = seg; // copy
        // Verify Clone derive exists; suppress lint since Copy is also derived.
        #[allow(clippy::clone_on_copy)]
        let _ = seg.clone();
    }
}
