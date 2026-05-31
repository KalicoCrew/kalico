//! Integration tests for the TIM5 inter-arrival guard in `isr_sample_tick`.
//!
//! Drives `isr_sample_tick` directly with a raw_cyccnt argument, following the
//! style of `tests/piece_tick.rs`.  Tests cover:
//!   - steady cadence: no fault over many ticks;
//!   - starvation: a gap > 2×period latches `TickIntervalExceeded` (-311) and
//!     encodes the gap in ticks in `fault_detail`;
//!   - first tick never faults regardless of raw_cyccnt.

use core::sync::atomic::Ordering;

use runtime::clock::WidenState;
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::piece_ring::PieceEntry;
use runtime::state::{IsrState, SharedState, TOTAL_RING_PIECES};
use runtime::tick::isr_sample_tick;

// 520 MHz clock, 40 kHz ISR → 13_000 cycles per tick.
const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u32 = CLOCK_FREQ / SAMPLE_RATE; // 13_000

fn make_isr() -> IsrState {
    IsrState {
        engine: Engine::new(CLOCK_FREQ, SAMPLE_RATE),
        widen_state: WidenState::default(),
        last_tick_now: None,
    }
}

fn make_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ]
}

// ── Test 1: steady cadence never faults ──────────────────────────────────────

/// Feed `now` advancing by exactly one tick period for 100 ticks.
/// No fault should be latched at any point.
#[test]
fn steady_cadence_no_fault() {
    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();

    // WidenState starts at high=0, last_low=0.  With raw values that never
    // wrap (TICK_CYCLES * 100 = 1_300_000 << u32::MAX), widened_now == raw.
    for i in 0u32..100 {
        let raw = TICK_CYCLES * i;
        isr_sample_tick(&mut isr, &shared, &mut storage, raw);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "no fault expected at tick {i} on a steady cadence"
        );
    }
}

// ── Test 2: starvation latches TickIntervalExceeded ──────────────────────────

/// After one normal baseline tick, inject a raw_cyccnt jump corresponding to
/// > 2 × period.  The guard must:
///   - latch `last_error == -311` (TickIntervalExceeded);
///   - encode the gap in ticks (≥ 3) into `fault_detail`.
#[test]
fn starvation_latches_tick_interval_exceeded() {
    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();

    // Tick 0: baseline (first tick, never faults).
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first tick must not fault"
    );

    // Tick 1: jump by 3 × TICK_CYCLES — exactly 3 ticks worth of gap.
    // gap = 3 * 13_000 = 39_000; period = 13_000; gap > 2 * period → fault.
    let gap_raw: u32 = TICK_CYCLES * 3;
    isr_sample_tick(&mut isr, &shared, &mut storage, gap_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "TickIntervalExceeded must be latched when gap > 2×period"
    );

    // fault_detail must encode the gap in ticks (gap / period = 3).
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        detail, 3,
        "fault_detail must equal gap_ticks (gap / period = 3), got {detail}"
    );
}

// ── Test 3: first tick never faults ──────────────────────────────────────────

/// Even with an extreme raw_cyccnt on the very first call, no fault should fire
/// (there is no previous baseline to compare against).
#[test]
fn first_tick_never_faults() {
    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();

    // Pass a raw value representing a very large accumulated cycle count.
    // Without a prior baseline, last_tick_now is None → guard is skipped.
    let large_raw: u32 = u32::MAX / 2;
    isr_sample_tick(&mut isr, &shared, &mut storage, large_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first tick must never fault regardless of raw_cyccnt"
    );
}

// ── Test 4: exactly-2× gap is within tolerance ───────────────────────────────

/// A gap of exactly 2 × period must NOT fault (the threshold is strictly
/// greater-than).
#[test]
fn gap_exactly_2x_period_is_ok() {
    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();

    // Tick 0: baseline.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);

    // Tick 1: gap = exactly 2 × period.
    let gap_raw: u32 = TICK_CYCLES * 2;
    isr_sample_tick(&mut isr, &shared, &mut storage, gap_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "gap == 2×period must not fault (strictly greater-than threshold)"
    );
}

// ── Test 5: large gap saturates fault_detail at 0xFFFF ───────────────────────

/// A gap of 0x1_0000 (65536) ticks must clip `gap_ticks.min(0xFFFF)` down to
/// 0xFFFF. The clip only fires when `gap / period > 0xFFFF`, so the target must
/// be strictly above 65535 — at 65535 the `.min()` is the identity and the
/// saturation path is never exercised.
#[test]
fn large_gap_saturates_fault_detail() {
    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();

    // Tick 0: baseline at raw = 0.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);

    // 65_536 ticks of gap: 65_536 * 13_000 = 851_968_000 < u32::MAX, so it fits
    // a single raw_cyccnt with no wrap. gap / period = 65_536, `as u32` = 65_536,
    // `.min(0xFFFF)` clips to 0xFFFF — the saturation path under test.
    let gap_ticks_target: u32 = 0x1_0000; // 65_536, strictly > 0xFFFF
    let gap_raw: u32 = gap_ticks_target * TICK_CYCLES; // 851_968_000, fits u32
    isr_sample_tick(&mut isr, &shared, &mut storage, gap_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "large gap must latch TickIntervalExceeded"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        detail, 0xFFFF,
        "fault_detail must saturate at 0xFFFF for a {gap_ticks_target}-tick gap, got {detail}"
    );
}
