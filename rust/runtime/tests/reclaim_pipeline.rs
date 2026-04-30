//! Foreground `SEGMENT_END` trace-drain -> curve-pool reclaim. Spec §10.4.
//!
//! All tests use the multi-handle retirement path: register 4 per-axis
//! handles (or UNUSED sentinels) in a `RetirementTable`, then drain via
//! `drain_and_reclaim` and verify the correct slots are freed.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::reclaim::{RetirementTable, drain_and_reclaim};
use runtime::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};

/// Scalar degree-1 linear curve helpers.
fn linear_knots() -> [f32; 4] {
    [0.0, 0.0, 1.0, 1.0]
}
fn linear_cps() -> [f32; 2] {
    [0.0, 10.0]
}

fn segment_end_sample(seg_id: u32, x_handle: CurveHandle) -> TraceSample {
    TraceSample {
        tick: 100,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_z: 0.0,
        motor_e: 0.0,
        segment_id: seg_id,
        curve_handle: x_handle,
        flags: TRACE_FLAG_SEGMENT_END,
        _pad: [0; 7],
    }
}

/// Build a 4-handle array with valid handle at index 0 and UNUSED for the rest.
fn single_handle_array(h: CurveHandle) -> [CurveHandle; 4] {
    [h, CurveHandle::UNUSED_SENTINEL, CurveHandle::UNUSED_SENTINEL, CurveHandle::UNUSED_SENTINEL]
}

/// Build a 4-handle array for X, Y, Z with UNUSED for E.
fn xyz_handle_array(hx: CurveHandle, hy: CurveHandle, hz: CurveHandle) -> [CurveHandle; 4] {
    [hx, hy, hz, CurveHandle::UNUSED_SENTINEL]
}

#[test]
fn reclaim_advances_last_retired_gen() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    // Second alloc on same slot fails — slot is still busy.
    assert!(pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .is_none());

    let mut table = RetirementTable::new();
    table.register(1, single_handle_array(h1));

    let mut samples = vec![segment_end_sample(1, h1)];
    let drained = drain_and_reclaim(&pool, &table, || samples.pop(), 16);
    assert_eq!(drained, 1);
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
            .is_some(),
        "alloc should succeed after retire"
    );
}

#[test]
fn fifo_ordering_implies_prior_gens_retired() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap(); // gen=1
    pool.confirm_retired(h1);
    let h2 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap(); // gen=2

    let mut table = RetirementTable::new();
    table.register(2, single_handle_array(h2));

    // Trace stream emits gen=2 SEGMENT_END.
    let mut samples = vec![segment_end_sample(2, h2)];
    drain_and_reclaim(&pool, &table, || samples.pop(), 16);

    // After SEGMENT_END(gen=2), slot is reusable.
    let h3 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert_eq!(h3.generation, 3);
}

#[test]
fn drain_respects_limit() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();

    let mut table = RetirementTable::new();
    table.register(1, single_handle_array(h1));

    let mut samples = vec![
        segment_end_sample(1, h1),
        segment_end_sample(1, h1),
        segment_end_sample(1, h1),
    ];
    // Drain only 2 of 3; reclaim still happens for the two we drained.
    let drained = drain_and_reclaim(&pool, &table, || samples.pop(), 2);
    assert_eq!(drained, 2);
    assert_eq!(samples.len(), 1, "one sample left undrained");
}

#[test]
fn non_segment_end_samples_skip_reclaim() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();

    let mut table = RetirementTable::new();
    table.register(1, single_handle_array(h1));

    let mut samples = vec![TraceSample {
        tick: 1,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_z: 0.0,
        motor_e: 0.0,
        segment_id: 1,
        curve_handle: h1,
        flags: 0, // NOT a SEGMENT_END
        _pad: [0; 7],
    }];
    drain_and_reclaim(&pool, &table, || samples.pop(), 16);
    // Slot still busy because SEGMENT_END was never observed.
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
            .is_none(),
        "non-SEGMENT_END samples must not advance reclaim"
    );
}

/// Multi-handle: load X (slot 0), Y (slot 1), Z (slot 2) as 3 separate scalar
/// curves, register all 3 + UNUSED for E. Drain one SEGMENT_END → all 3 freed.
#[test]
fn multi_handle_xyz_all_retired_on_segment_end() {
    let pool = CurvePool::new();

    let hx = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .expect("X alloc");
    let hy = pool
        .try_alloc_and_load(1, 1, &linear_knots(), &linear_cps())
        .expect("Y alloc");
    let hz = pool
        .try_alloc_and_load(2, 1, &linear_knots(), &linear_cps())
        .expect("Z alloc");

    // All 3 slots are busy.
    assert!(pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps()).is_none(), "X busy");
    assert!(pool.try_alloc_and_load(1, 1, &linear_knots(), &linear_cps()).is_none(), "Y busy");
    assert!(pool.try_alloc_and_load(2, 1, &linear_knots(), &linear_cps()).is_none(), "Z busy");

    let mut table = RetirementTable::new();
    table.register(42, xyz_handle_array(hx, hy, hz));

    // Trace emits SEGMENT_END for segment 42 carrying only the X handle (diagnostics).
    let mut samples = vec![segment_end_sample(42, hx)];
    let drained = drain_and_reclaim(&pool, &table, || samples.pop(), 16);
    assert_eq!(drained, 1);

    // All 3 slots must now be free.
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps()).is_some(),
        "X slot freed"
    );
    assert!(
        pool.try_alloc_and_load(1, 1, &linear_knots(), &linear_cps()).is_some(),
        "Y slot freed"
    );
    assert!(
        pool.try_alloc_and_load(2, 1, &linear_knots(), &linear_cps()).is_some(),
        "Z slot freed"
    );
}

/// HOLD_SEGMENT_SENTINEL handles are skipped — they must not be passed to
/// `confirm_retired` (they carry slot_idx=u16::MAX which would be out-of-bounds).
#[test]
fn hold_sentinel_in_retirement_table_skipped() {
    let pool = CurvePool::new();
    let hx = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .expect("X alloc");

    let mut table = RetirementTable::new();
    // Mix real handle + HOLD sentinel + two UNUSED sentinels.
    table.register(
        7,
        [
            hx,
            CurveHandle::HOLD_SEGMENT_SENTINEL,
            CurveHandle::UNUSED_SENTINEL,
            CurveHandle::UNUSED_SENTINEL,
        ],
    );

    let mut samples = vec![segment_end_sample(7, hx)];
    // Must not panic or assert-fail on the sentinel.
    let drained = drain_and_reclaim(&pool, &table, || samples.pop(), 16);
    assert_eq!(drained, 1);
    // Slot 0 (hx) freed; sentinel slots were skipped cleanly.
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps()).is_some(),
        "real slot freed"
    );
}

/// Segment_id not in table → no reclaim. The slot remains busy.
#[test]
fn missing_segment_id_no_reclaim() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();

    let mut table = RetirementTable::new();
    // Register under a different id than the one in the trace sample.
    table.register(99, single_handle_array(h1));

    let mut samples = vec![segment_end_sample(55, h1)]; // id=55 not in table
    drain_and_reclaim(&pool, &table, || samples.pop(), 16);

    // Slot is still busy — no registration for id=55.
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps()).is_none(),
        "slot must remain busy when segment_id is not in table"
    );
}
