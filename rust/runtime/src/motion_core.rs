// Hardened piece-ring walker — per-axis walk+eval loop shared by the MCU
// stepper engine and any other motion node.
//
// Walk/load split: only the LANDED piece (the one whose window contains `now`)
// is monomialised; walked-past pieces are retired without calling `to_monomial`.
// `to_monomial` is the dominant cost on the hot path; calling it for every
// expired piece is an O(walk-depth) waste — do not revert this split.

use crate::fault_sink::FaultSink;
use crate::piece_ring::{PieceEntry, RingDescriptor};

/// ISR's cached working copy of the currently-armed piece: monomial
/// coefficients plus the piece's MCU-clock window.
#[derive(Debug, Clone, Copy)]
pub struct ArmedPiece {
    pub mono_coeffs: [f32; 4],
    pub vel_coeffs: [f32; 3],
    pub piece_start_cycles: u64,
    pub piece_end_cycles: u64,
}

/// Advance the axis to the correct piece for `now`, returning
/// `(position, velocity)` if an active piece exists.
///
/// Two invariants must be preserved:
/// - Walk/load split: only the landed piece is monomialised.
/// - Fault-check runs BEFORE the `now < end` window return in
///   `get_piece_for_time`; inverting the order silently drops the
///   cold-adoption fault.
#[inline]
pub fn get_position_and_velocity<F: FaultSink>(
    armed: &mut Option<ArmedPiece>,
    ring: &mut RingDescriptor,
    storage: &[PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    axis_idx: usize,
    fault: &F,
) -> Option<(f32, f32)> {
    if let Some(p) = &*armed {
        if now < p.piece_end_cycles {
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_HORNER);
            return Some(eval_horner(
                &p.mono_coeffs,
                &p.vel_coeffs,
                p.piece_start_cycles,
                now,
                cycles_per_second,
            ));
        }
        *armed = None;
        ring.advance_counter();
    }

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_WALK);
    let walk_start = crate::isr_phase::cyccnt();
    let slot = get_piece_for_time(
        ring,
        storage,
        now,
        sample_period_cycles,
        cycles_per_second,
        axis_idx,
        fault,
    )?;
    crate::isr_phase::walk_account(crate::isr_phase::cyccnt().wrapping_sub(walk_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_MONOMIAL);
    let mono_start = crate::isr_phase::cyccnt();
    // SAFETY: `slot` is `ring_offset + tail` from `get_piece_for_time` →
    // `ring.front_slot()`. `configure_axis` guarantees
    // `ring_offset + ring_depth <= storage.len()`, and `tail < ring_depth`
    // always holds (tail advances mod ring_depth). Therefore `slot <
    // storage.len()`.
    #[allow(clippy::indexing_slicing)]
    let p = arm_and_load(armed, &storage[slot], cycles_per_second);
    crate::isr_phase::monomial_account(crate::isr_phase::cyccnt().wrapping_sub(mono_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_HORNER);
    Some(eval_horner(
        &p.mono_coeffs,
        &p.vel_coeffs,
        p.piece_start_cycles,
        now,
        cycles_per_second,
    ))
}

/// Walk the ring to the entry whose window contains `now`, retiring elapsed
/// entries WITHOUT monomialising them. Returns the physical slot index of the
/// landed piece, or `None` when the ring is empty.
///
/// Hard-faults `PieceStartInPast` when a freshly adopted piece's start is
/// more than `drift_budget + sample_period_cycles` in the past.
///
/// CRITICAL: the fault-check runs BEFORE the `now < end` window return so
/// that a cold-adopted front is always checked.
#[inline]
fn get_piece_for_time<F: FaultSink>(
    ring: &mut RingDescriptor,
    storage: &[PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    axis_idx: usize,
    fault: &F,
) -> Option<usize> {
    const MAX_START_IN_PAST_SECS: f32 = 200e-6;
    let drift_budget = (MAX_START_IN_PAST_SECS * cycles_per_second) as u64;
    let fault_tolerance = drift_budget + u64::from(sample_period_cycles);
    loop {
        let slot = ring.front_slot()?;
        // SAFETY: `slot` is `ring_offset + tail` from `front_slot()`.
        // `configure_axis` guarantees `ring_offset + ring_depth <= storage.len()`,
        // and `tail < ring_depth` always holds. Therefore `slot < storage.len()`.
        #[allow(clippy::indexing_slicing)]
        let entry = &storage[slot];
        let deficit_cycles = now.saturating_sub(entry.start_time);
        if deficit_cycles > fault_tolerance {
            let deficit_us = (deficit_cycles as f32 * (1.0e6_f32 / cycles_per_second)) as u32;
            fault.piece_start_in_past(axis_idx, deficit_us);
            return None;
        }
        if now < entry.end_time(cycles_per_second) {
            return Some(slot);
        }
        ring.advance_counter();
    }
}

#[inline]
fn arm_and_load<'a>(
    armed: &'a mut Option<ArmedPiece>,
    entry: &PieceEntry,
    cycles_per_second: f32,
) -> &'a ArmedPiece {
    let (mono, vel) = entry.to_monomial();
    armed.insert(ArmedPiece {
        mono_coeffs: mono,
        vel_coeffs: vel,
        piece_start_cycles: entry.start_time,
        piece_end_cycles: entry.end_time(cycles_per_second),
    })
}

#[inline]
pub fn eval_horner(
    mono: &[f32; 4],
    vel: &[f32; 3],
    piece_start_cycles: u64,
    now: u64,
    cycles_per_second: f32,
) -> (f32, f32) {
    let elapsed_cycles = now.saturating_sub(piece_start_cycles);
    let t = if cycles_per_second > 0.0 {
        elapsed_cycles as f32 / cycles_per_second
    } else {
        0.0_f32
    };
    let p = mono[0] + t * (mono[1] + t * (mono[2] + t * mono[3]));
    let v = vel[0] + t * (vel[1] + t * vel[2]);
    (p, v)
}
