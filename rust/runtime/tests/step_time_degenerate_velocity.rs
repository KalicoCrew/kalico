//! Degenerate-velocity regression tests under the Bernstein root-finder.
//!
//! Previously these tests pinned the Newton degree-aware seed (velocity →
//! accel → forward-scan probe) so that accel-from-rest segments (v(0) = 0)
//! still produced steps instead of falsely reporting `SegmentExhausted`.
//! The interim Cardano implementation handled v(0) = 0 by closed form;
//! the current bezier_root solver handles it via the midpoint-probe
//! direction fallback wired into `compute_next_step_time`. The behavioural
//! contract is unchanged.
//!
//! Spec: docs/superpowers/specs/2026-05-14-bernstein-step-root-design.md

use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Build the four Bezier CPs of the cubic `a·u³ + b·u² + c·u + d`.
/// Inverse of the standard Bernstein → monomial expansion.
fn cps_from_monomial(a: f64, b: f64, c: f64, d: f64) -> [f64; 4] {
    let p0 = d;
    let p1 = c / 3.0 + p0;
    let p2 = b / 3.0 + 2.0 * p1 - p0;
    let p3 = a + 3.0 * p2 - 3.0 * p1 + p0;
    [p0, p1, p2, p3]
}

#[test]
fn accel_from_rest_first_step_under_quadratic_position() {
    // x(u) = (a/2)·u² with a=200, step_distance=1. First step at:
    //   1 = (200/2) u² → u = sqrt(2/200) = sqrt(0.01) = 0.1
    // Monomial: 0·u³ + 100·u² + 0·u + 0.
    let cps = cps_from_monomial(0.0, 100.0, 0.0, 0.0);
    let q = StepTimeQuery {
        cps,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            // P'(0.1) = 20, EPS_CONVERGENCE = 1e-5 ⇒ t-tolerance ≈ 5e-7.
            assert!((t - 0.1).abs() < 1e-5, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => {
            panic!("solver must not bail on v(0)=0 when accel is non-zero");
        }
    }
}

#[test]
fn jerk_only_start_first_step_under_cubic_position() {
    // x(u) = (j/6)·u³ with j=6000, step_distance=1.
    //   1 = (6000/6)·u³ = 1000·u³ → u³ = 0.001 → u = 0.1.
    // Monomial: 1000·u³ + 0·u² + 0·u + 0.
    let cps = cps_from_monomial(1000.0, 0.0, 0.0, 0.0);
    let q = StepTimeQuery {
        cps,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            // P'(0.1) = 30, EPS_CONVERGENCE = 1e-5 ⇒ t-tolerance ≈ 3.3e-7.
            assert!((t - 0.1).abs() < 1e-5, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => {
            panic!("solver must not bail when jerk is non-zero")
        }
    }
}

#[test]
fn reverse_accel_from_rest_negative_direction() {
    // x(u) = -100·u². v(0) = 0, but the midpoint probe sees x(0.5) - x(0) = -25,
    // so direction is -1. Target = -1·1 = -1. Solve -100·u² = -1 → u = 0.1.
    let cps = cps_from_monomial(0.0, -100.0, 0.0, 0.0);
    let q = StepTimeQuery {
        cps,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, -1);
            assert!((t - 0.1).abs() < 1e-5, "expected t≈0.1, got {}", t);
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn truly_motionless_curve_exhausts() {
    // Constant curve x(u) = 0 everywhere.
    let cps: [f64; 4] = [0.0, 0.0, 0.0, 0.0];
    let q = StepTimeQuery {
        cps,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    assert!(matches!(
        compute_next_step_time(&q),
        StepTimeResult::SegmentExhausted
    ));
}

#[test]
fn decel_to_rest_fires_all_but_last_step() {
    // x(u) = 200u - 100u². v(u) = 200 - 200u. a(u) = -200. Total at u=1: 100.
    // step_distance = 0.5 → ~200 steps.
    // Monomial: 0·u³ + (-100)·u² + 200·u + 0.
    let cps = cps_from_monomial(0.0, -100.0, 200.0, 0.0);
    let mut t_curr = 0.0_f64;
    let mut count = 0_i32;
    loop {
        let q = StepTimeQuery {
            cps,
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
    assert!(
        count >= 199 && count <= 200,
        "fired {} steps (expected 199 or 200)",
        count
    );
}
