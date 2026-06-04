//! Sustained-streaming test: proves that the `AxisRing` pump loop correctly
//! retires pieces past one ring depth without deadlocking or spuriously faulting.
//!
//! ## Historical failure mode
//!
//! Early versions of the EtherCAT endpoint stalled after delivering the first
//! ring-full batch: the host pump did not refill because retirement heartbeats
//! were not emitted, or the ring did not expose `retired_count()` advancement,
//! or the pump's occupancy accounting was not driven by retirements. This test
//! catches all of those failure modes at the `AxisRing` API level, without
//! requiring a socket or subprocess.
//!
//! ## Strategy
//!
//! Generate `AXIS_RING_CAPACITY + 20` contiguous 1 ms moving pieces (a linear
//! ramp from 0 → N mm). The loop mimics the host pump:
//!
//! 1. Push as many pieces as the ring will accept (`push_entry` returns Err when
//!    full).
//! 2. Advance the synthetic clock in 1 ms steps, calling `sample(now)` each
//!    step to trigger retirement of elapsed pieces.
//! 3. Whenever `retired_count()` advances (heartbeat signal), push more pieces
//!    to refill the ring.
//! 4. Continue until all N pieces are retired.
//!
//! Assertions:
//! - `retired_count() == N` at termination (all pieces delivered and retired).
//! - `take_fault()` remains `None` throughout (contiguous in-time delivery must
//!   not trip the PieceStartInPast fault).
//! - The loop terminates within a bounded iteration count (no stall).
//!
//! ## Determinism
//!
//! All time is synthetic. No wall-clock sleeps. The test is entirely
//! deterministic and runs without the `hw` feature.

use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, EC_DC_PERIOD_NS};
use runtime::piece_ring::PieceEntry;

/// Build a single 1 ms linear-ramp piece starting at `start_ns` with
/// position advancing from `from_mm` to `to_mm`.
fn ramp_piece(from_mm: f32, to_mm: f32, start_ns: u64) -> PieceEntry {
    // Linear Bernstein form for a ramp [a, a+(b-a)/3, a+2(b-a)/3, b].
    let d = to_mm - from_mm;
    PieceEntry {
        start_time: start_ns,
        coeffs: [from_mm, from_mm + d / 3.0, from_mm + 2.0 * d / 3.0, to_mm],
        duration: 0.001_f32, // 1 ms
        _reserved: 0,
    }
}

/// Sustained streaming of `AXIS_RING_CAPACITY + 20` pieces across the ring
/// depth boundary.
///
/// This is the endpoint-level proof that streaming does not stall after the
/// first ring-full, which is the historical "stopped after first move" failure.
#[test]
fn sustained_streaming_past_ring_depth() {
    const TOTAL: usize = AXIS_RING_CAPACITY + 20;
    const PIECE_DUR_NS: u64 = EC_DC_PERIOD_NS as u64; // 1_000_000 ns

    // Synthetic epoch well clear of 0 to avoid underflow.
    const BASE_NS: u64 = 10_000_000_000_u64; // 10 s

    // Build all pieces up front for deterministic indexing.
    // Each piece advances position by (1/TOTAL) mm so the full stream is 0→1 mm.
    let mm_per_piece = 1.0_f32 / TOTAL as f32;
    let pieces: Vec<PieceEntry> = (0..TOTAL)
        .map(|i| {
            let from = i as f32 * mm_per_piece;
            let to = (i + 1) as f32 * mm_per_piece;
            let start_ns = BASE_NS + i as u64 * PIECE_DUR_NS;
            ramp_piece(from, to, start_ns)
        })
        .collect();

    let mut ring = AxisRing::new();

    // Pump state.
    let mut next_to_push: usize = 0; // index into `pieces`
    let mut last_retired: u32 = 0;

    // Fill the ring as much as possible before the loop.
    while next_to_push < TOTAL {
        if ring.push_entry(pieces[next_to_push]).is_err() {
            break; // ring full
        }
        next_to_push += 1;
    }

    // Advance synthetic clock in 1 ms steps. Each step retires one piece
    // (the piece that started at BASE_NS + (step-1)*PIECE_DUR_NS ends at
    // BASE_NS + step*PIECE_DUR_NS). The loop re-fills the ring whenever
    // retired_count advances (mimicking the pump heartbeat response).
    //
    // Bound the loop to prevent an infinite spin if streaming stalls.
    // Maximum iterations: TOTAL pieces × (capacity fill + retire) × 4 safety margin.
    let max_iterations = TOTAL * 4 + AXIS_RING_CAPACITY * 4;
    let mut now = BASE_NS;
    let mut iterations = 0usize;

    loop {
        assert!(
            iterations < max_iterations,
            "streaming stalled: retired_count={}/{} next_to_push={}/{} after {} iterations",
            ring.retired_count(),
            TOTAL,
            next_to_push,
            TOTAL,
            iterations
        );
        iterations += 1;

        // Sample at the current synthetic clock.
        let _sample = ring.sample(now);

        // Any fault is a test failure: contiguous in-time delivery must not fault.
        assert_eq!(
            ring.take_fault(),
            None,
            "spurious PieceStartInPast fault at iteration {iterations}, now={now}, \
             retired={}/{}",
            ring.retired_count(),
            TOTAL
        );

        // Advance the clock by one DC period.
        now = now.saturating_add(PIECE_DUR_NS);

        // Check for retirement advancement (heartbeat signal) and refill.
        let current_retired = ring.retired_count();
        if current_retired != last_retired {
            last_retired = current_retired;
            // Push as many pieces as the ring will now accept.
            while next_to_push < TOTAL {
                if ring.push_entry(pieces[next_to_push]).is_err() {
                    break; // ring full again
                }
                next_to_push += 1;
            }
        }

        // Termination: all pieces have been pushed AND retired.
        if next_to_push == TOTAL && ring.retired_count() == TOTAL as u32 {
            break;
        }
    }

    // Final assertions.
    assert_eq!(
        ring.retired_count(),
        TOTAL as u32,
        "all {} pieces must be retired; got {}",
        TOTAL,
        ring.retired_count()
    );
    assert_eq!(
        ring.take_fault(),
        None,
        "no fault must remain latched after full stream"
    );
    // The ring should be empty — all pieces expired.
    assert!(
        ring.is_empty(),
        "ring must be empty after all pieces retire"
    );
}
