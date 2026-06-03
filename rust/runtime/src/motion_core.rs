//! Hardened piece-ring walker — the single source of the per-axis walk+eval
//! loop shared by the MCU stepper engine and any other motion node (EtherCAT
//! servo, etc.).
//!
//! The public primitives here (`get_position_and_velocity`, `eval_horner`) are
//! marked `#[inline]` so they inline into `Engine::tick` on MCU builds.
//! External nodes call them through generic monomorphization — never through `dyn`.
//!
//! ## Walk / load split (throughput invariant)
//!
//! Walking past elapsed pieces does NOT call `to_monomial`; only the LANDED
//! piece (the one whose window contains `now`) is monomialised. This is the
//! "don't monomialise walked-past pieces" split from commit ddcb69188. It is
//! non-negotiable: `to_monomial` is the dominant cost on the hot path and
//! calling it for every expired piece is an O(walk-depth) waste.

use crate::fault_sink::FaultSink;
use crate::piece_ring::{PieceEntry, RingDescriptor};

/// The ISR's cached working copy of the currently-armed piece: monomial
/// coefficients plus the piece's MCU-clock window. Bundled into one struct so
/// "is a piece loaded?" is `Option<ArmedPiece>::is_some()` — no separate
/// validity flag to keep in sync.
#[derive(Debug, Clone, Copy)]
pub struct ArmedPiece {
    /// Position monomial coefficients (c0, c1, c2, c3).
    pub mono_coeffs: [f32; 4],
    /// Velocity coefficients (vc0, vc1, vc2).
    pub vel_coeffs: [f32; 3],
    /// MCU clock cycle at which the piece starts.
    pub piece_start_cycles: u64,
    /// MCU clock cycle at which the piece ends.
    pub piece_end_cycles: u64,
}

/// Advance the axis to the correct piece for `now`, returning
/// `(position, velocity)` if an active piece exists.
///
/// Two invariants must be preserved:
/// - Walk/load split: only the landed piece is monomialised; walked-past
///   pieces are retired without calling `to_monomial`.
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
    // Branch 1: current armed piece still inside its window (or a future
    // piece held at t=0 by eval_horner's saturating elapsed). Short-circuit
    // before walk/fault so a long hold never sees the cold-adoption check.
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
    let slot = get_piece_for_time(ring, storage, now, sample_period_cycles, cycles_per_second, axis_idx, fault)?;
    crate::isr_phase::walk_account(crate::isr_phase::cyccnt().wrapping_sub(walk_start));

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_MONOMIAL);
    let mono_start = crate::isr_phase::cyccnt();
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

/// Walk the ring to the entry whose window contains `now`, retiring every
/// elapsed front along the way WITHOUT monomialising them. Returns the
/// physical slot index of the landed piece, or `None` when the ring is
/// empty (idle/underrun).
///
/// Hard-faults `PieceStartInPast` when a freshly adopted piece's start is
/// more than `drift_budget + sample_period_cycles` in the past (branch 3).
///
/// CRITICAL: the fault-check runs BEFORE the `now < end` window return so
/// that a cold-adopted front is always checked (inverting the order silently
/// drops the cold-adoption fault).
///
/// Do NOT couple the drift tolerance numerically to pump's `MAX_LEAD_SECS` —
/// that would hide a new magic number.
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
        let slot = ring.front_slot()?; // branch 2: ring empty → underrun
        let entry = &storage[slot];
        // Fault-check BEFORE the window return (preserves cold-adoption fault).
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

/// Monomialise and cache the landed piece.
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

/// Evaluate position and velocity via Horner using the axis's cached
/// coefficients.  Returns `(p_end, v_end)` in mm and mm/s.
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
