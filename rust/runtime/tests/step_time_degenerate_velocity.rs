//! Degenerate-velocity Newton-seed regression tests.
//!
//! Today the runtime bails as `SegmentExhausted` the moment it sees
//! `|v(t_curr)| < EPS_VELOCITY`. For an accel-from-rest segment, `v(0) = 0`
//! *exactly* — so the old Newton seed bailed at the first call and zero step
//! pulses fired for the whole accel segment. These tests pin the behaviour
//! of the degree-aware Newton seed (velocity → accel → forward-scan probe)
//! from spec §3.6.
//!
//! Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md §3.6

use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

#[test]
fn accel_from_rest_first_step_under_quadratic_position() {
    // x(u) = (a/2)·u² with a=200, step_distance=1. First step at:
    //   1 = (200/2) u² → u = sqrt(2/200) = sqrt(0.01) = 0.1
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = 0.5 * 200.0 * u64 * u64;
        let vel = 200.0 * u64;
        let acc = 200.0_f64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            assert!((t - 0.1).abs() < 1e-3, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => {
            panic!("Newton must not bail on v(0)=0 when accel is non-zero");
        }
    }
}

#[test]
fn jerk_only_start_first_step_under_cubic_position() {
    // x(u) = (j/6)·u³ with j=6000, step_distance=1.
    //   1 = (6000/6)·u³ → u = (6/6000)^(1/3) = 0.1
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = (6000.0 / 6.0) * u64 * u64 * u64;
        let vel = (6000.0 / 2.0) * u64 * u64;
        let acc = 6000.0 * u64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            assert!((t - 0.1).abs() < 1e-2, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => panic!("Newton must not bail when jerk is non-zero"),
    }
}

#[test]
fn reverse_accel_from_rest_negative_direction() {
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = -0.5 * 200.0 * u64 * u64;
        let vel = -200.0 * u64;
        let acc = -200.0_f64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { dir, .. } => assert_eq!(dir, -1),
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn truly_motionless_curve_exhausts() {
    let eval = |_u: f32| (0.0_f64, 0.0_f64, 0.0_f64);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    assert!(matches!(compute_next_step_time(&q), StepTimeResult::SegmentExhausted));
}

#[test]
fn decel_to_rest_fires_all_but_last_step() {
    // x(u) = 200u - 100u². v(u) = 200 - 200u. a(u) = -200. Total = 100.
    // step_distance = 0.5 → ~200 steps.
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = 200.0 * u64 - 100.0 * u64 * u64;
        let vel = 200.0 - 200.0 * u64;
        let acc = -200.0_f64;
        (pos, vel, acc)
    };
    let mut t_curr = 0.0_f64;
    let mut count = 0_i32;
    loop {
        let q = StepTimeQuery {
            eval: &eval,
            step_distance: 0.5,
            current_step: count,
            t_curr,
            t_segment_end: 1.0,
        };
        match compute_next_step_time(&q) {
            StepTimeResult::NextAt { t, dir } => {
                assert_eq!(dir, 1);
                assert!(t > t_curr);
                t_curr = t;
                count += 1;
            }
            StepTimeResult::SegmentExhausted => break,
        }
    }
    assert!(count >= 199 && count <= 200, "fired {} steps", count);
}
