//! Minimal non-tautological smoke tests for the runtime crate root.
//!
//! These are not comprehensive integration tests (those live in `tests/`);
//! they are the smallest useful assertions that confirm the public API is
//! wired up correctly.

// Test code: panicking / unwrapping / integer constants are the intended
// failure signals here; the deny-in-production lints don't apply to tests.
#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::integer_division
)]

use crate::fault_sink::FaultSink;
use crate::monomial::bernstein_to_monomial_with_duration;
use crate::motion_core::get_position_and_velocity;
use crate::piece_ring::{PieceEntry, RingDescriptor};

// Minimal FaultSink for smoke tests.
struct PanicFaultSink;
impl FaultSink for PanicFaultSink {
    fn piece_start_in_past(&self, axis_idx: usize, _deficit_us: u32) {
        panic!("unexpected PieceStartInPast fault on axis {axis_idx}");
    }
}

const CLOCK_FREQ: f32 = 520_000_000.0;
const TICK_CYCLES: u32 = 520_000_000_u32 / 40_000_u32; // 13_000

/// The walker must return `None` for an empty ring — no armed piece, no
/// entries. This is the basic idle/underrun path.
#[test]
fn walker_empty_ring_returns_none() {
    let mut ring = RingDescriptor::new_unconfigured();
    let storage: Vec<PieceEntry> = Vec::new();
    let fault = PanicFaultSink;
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        TICK_CYCLES as u64 * 10,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_none(), "empty ring must return None");
}

/// `eval_horner` (indirectly via walker with a piece at t=0) must return
/// (c0, c1) for the linear piece at t=0.
///
/// Bernstein [0.5, 1.0, 1.5, 2.0] over 0.1 s:
///   c0 = 0.5  (P(0) must equal the first control point)
///   c1 = 3*(1.0 - 0.5) / 0.1 = 15 mm/s  (V(0))
#[test]
fn walker_at_t0_returns_c0_and_c1() {
    let duration_s = 0.1_f32;
    let coeffs = [0.5_f32, 1.0, 1.5, 2.0];
    let start = TICK_CYCLES as u64 * 10;

    let entry = PieceEntry {
        start_time: start,
        coeffs,
        duration: duration_s,
        _reserved: 0,
    };

    let mut storage = vec![entry; 4];
    let mut ring = RingDescriptor::new(0, 4);
    ring.push(&mut storage, entry).expect("push must succeed");

    let fault = PanicFaultSink;
    let mut armed = None;

    // now == start → t = 0 s in the piece.
    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        start,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_some(), "piece at t=0 must return Some");
    let (p, v) = res.unwrap();

    // Analytic via bernstein_to_monomial_with_duration.
    let m = bernstein_to_monomial_with_duration(coeffs, duration_s);
    let c0 = m.coeffs[0]; // P(0) = c0
    let c1 = m.vel_coeffs[0]; // V(0) = vc0 = c1

    assert!((p - c0).abs() < 1e-5, "P(0) must equal c0={c0}; got {p}");
    assert!(
        (v - c1).abs() < 1e-3,
        "V(0) must equal c1={c1} mm/s; got {v}"
    );
}
