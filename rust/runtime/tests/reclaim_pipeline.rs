//! Foreground `SEGMENT_END` trace-drain -> curve-pool reclaim. Spec §10.4.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::curve_pool::CurvePool;
use runtime::reclaim::drain_and_reclaim;
use runtime::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};

/// Scalar degree-1 linear curve helpers.
fn linear_knots() -> [f32; 4] {
    [0.0, 0.0, 1.0, 1.0]
}
fn linear_cps() -> [f32; 2] {
    [0.0, 10.0]
}

fn segment_end_sample(seg_id: u32, handle: runtime::curve_pool::CurveHandle) -> TraceSample {
    TraceSample {
        tick: 100,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_e: 0.0,
        segment_id: seg_id,
        curve_handle: handle,
        flags: TRACE_FLAG_SEGMENT_END,
        _pad: [0; 3],
    }
}

#[test]
fn reclaim_advances_last_retired_gen() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    assert!(pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .is_none());

    let mut samples = vec![segment_end_sample(1, h1)];
    let drained = drain_and_reclaim(&pool, || samples.pop(), 16);
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
    // Allocate, retire, allocate, retire -- drain in order.
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap(); // gen=1
    pool.confirm_retired(h1);
    let h2 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap(); // gen=2

    // Trace stream emits gen=2 SEGMENT_END.
    let mut samples = vec![segment_end_sample(2, h2)];
    drain_and_reclaim(&pool, || samples.pop(), 16);

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
    let mut samples = vec![
        segment_end_sample(1, h1),
        segment_end_sample(1, h1),
        segment_end_sample(1, h1),
    ];
    // Drain only 2 of 3; reclaim still happens for the two we drained.
    let drained = drain_and_reclaim(&pool, || samples.pop(), 2);
    assert_eq!(drained, 2);
    assert_eq!(samples.len(), 1, "one sample left undrained");
}

#[test]
fn non_segment_end_samples_skip_reclaim() {
    let pool = CurvePool::new();
    let h1 = pool
        .try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
        .unwrap();
    let mut samples = vec![TraceSample {
        tick: 1,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_e: 0.0,
        segment_id: 1,
        curve_handle: h1,
        flags: 0, // NOT a SEGMENT_END
        _pad: [0; 3],
    }];
    drain_and_reclaim(&pool, || samples.pop(), 16);
    // Slot still busy because SEGMENT_END was never observed.
    assert!(
        pool.try_alloc_and_load(0, 1, &linear_knots(), &linear_cps())
            .is_none(),
        "non-SEGMENT_END samples must not advance reclaim"
    );
}
