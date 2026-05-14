//! Step-time computation tests (Cardano closed-form solver).
//!
//! Rewritten from the prior Newton-iteration-pinning tests. Cardano takes
//! `&CubicCoeffs` directly, so each test synthesises a known cubic via
//! `CubicCoeffs::from_bezier(...)` and asks `compute_next_step_time` for
//! the next step's time. Assertions check the returned `t` against the
//! analytic root.
//!
//! Plan: docs/superpowers/plans/2026-05-14-cardano-cubic-solver.md

use runtime::cardano::CubicCoeffs;
use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Build a `CubicCoeffs` representing the monomial `a·u³ + b·u² + c·u + d`.
/// Goes via the Bezier control-point form so we exercise the same
/// construction path the engine uses.
fn coeffs_from_monomial(a: f64, b: f64, c: f64, d: f64) -> CubicCoeffs {
    // Bezier conversion (the inverse of `CubicCoeffs::from_bezier`):
    //   p0 = d
    //   p1 = (c + 3·p0) / 3
    //   p2 = (b + 6·p1 - 3·p0) / 3
    //   p3 = a + 3·p2 - 3·p1 + p0
    let p0 = d;
    let p1 = (c + 3.0 * p0) / 3.0;
    let p2 = (b + 6.0 * p1 - 3.0 * p0) / 3.0;
    let p3 = a + 3.0 * p2 - 3.0 * p1 + p0;
    CubicCoeffs::from_bezier(p0, p1, p2, p3)
}

/// Linear curve: position(u) = velocity·u.
fn linear_coeffs(velocity: f64) -> CubicCoeffs {
    coeffs_from_monomial(0.0, 0.0, velocity, 0.0)
}

/// Cubic curve: position(u) = a·u³ + b·u² + c·u.
fn cubic_coeffs(a: f64, b: f64, c: f64) -> CubicCoeffs {
    coeffs_from_monomial(a, b, c, 0.0)
}

#[test]
fn linear_curve_returns_analytic_root() {
    // velocity = 1.0 mm/(u-unit); step_distance = 0.0025 mm.
    // Expected next step at u = 0.0025 (forward direction).
    let coeffs = linear_coeffs(1.0);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt { t, dir } => {
            assert!((t - 0.0025).abs() < 1e-9, "expected t≈0.0025, got {}", t);
            assert_eq!(dir, 1, "positive velocity should yield dir=+1");
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn linear_curve_reverse_direction() {
    // Negative velocity → direction = -1, root at u = 0.0025 against the
    // shifted target (current_step + dir) · step_distance = -0.0025, i.e.
    // x(u) = -u; solving -u = -0.0025 gives u = 0.0025.
    let coeffs = linear_coeffs(-1.0);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt { t, dir } => {
            assert!((t - 0.0025).abs() < 1e-9);
            assert_eq!(dir, -1, "negative velocity should yield dir=-1");
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn cubic_curve_returns_root_at_step_boundary() {
    // position(u) = 0.1·u³ + 0.5·u² + 1.0·u  (mm)
    // At u=0: position=0, velocity=1.0. Look for first step at 0.0025 mm.
    let coeffs = cubic_coeffs(0.1, 0.5, 1.0);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    let t = match result {
        StepTimeResult::NextAt { t, .. } => t,
        other => panic!("expected NextAt, got {:?}", other),
    };
    // Verify the returned u actually puts position at the step boundary.
    let pos = coeffs.eval(t);
    assert!(
        (pos - 0.0025).abs() < 0.0025 * 1e-9,
        "position at returned t={} is {}, expected 0.0025",
        t,
        pos,
    );
}

#[test]
fn segment_exhaustion_returns_segment_exhausted() {
    // velocity 1.0 mm/(u-unit), segment ends at u=0.001. One step = 0.0025 mm
    // can't fit before segment end (root would be at u=0.0025 > 0.001).
    let coeffs = linear_coeffs(1.0);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 0.001,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted, got {:?}",
        result,
    );
}

/// Ill-conditioned cubic (near-zero linear term) that previously drove
/// Newton into the fallback path. Cardano's closed-form solver handles it
/// directly — the test now simply verifies the returned root sits at the
/// step boundary regardless of conditioning.
#[test]
fn ill_conditioned_cubic_returns_root_at_step_boundary() {
    // position(u) = 1.0·u³ + 0.0·u² + (-0.0001)·u
    // Starting at u=0.1: position = 0.001 - 0.00001 = 0.00099.
    // step_distance = 0.001, current_step = 0, dir = +1 → target = 0.001.
    // Solve u³ - 0.0001·u - 0.001 = 0 for u > 0.1 — Cardano finds a root.
    let coeffs = cubic_coeffs(1.0, 0.0, -0.0001);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.001,
        current_step: 0,
        t_curr: 0.1,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    let t = match result {
        StepTimeResult::NextAt { t, .. } => t,
        StepTimeResult::SegmentExhausted => panic!(
            "expected NextAt; Cardano should yield a step on this well-formed cubic"
        ),
    };
    assert!(t > 0.1 && t <= 1.0, "t={} not in (0.1, 1.0]", t);
    let pos = coeffs.eval(t);
    let target = 0.001;
    assert!(
        (pos - target).abs() < 1e-9,
        "position at t={} is {}, target={}, err={}",
        t,
        pos,
        target,
        (pos - target).abs(),
    );
}

/// Same cubic as above with a t_segment_end that lies before any root.
/// Cardano must return SegmentExhausted because no root exists in
/// `(t_curr, t_segment_end]`.
#[test]
fn no_root_in_short_segment_returns_segment_exhausted() {
    let coeffs = cubic_coeffs(1.0, 0.0, -0.0001);
    // At u=0.1: x ≈ 0.00099. Target = 0.001. Root is at roughly
    // u ≈ 0.10033; pick a `t_segment_end` strictly before that.
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.001,
        current_step: 0,
        t_curr: 0.1,
        t_segment_end: 0.1001,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted when no root in short segment, got {:?}",
        result,
    );
}

/// Truly motionless curve: constant position, zero velocity everywhere.
/// Cardano correctly reports no step (vs. Newton's velocity-threshold
/// bail; the result is the same).
#[test]
fn motionless_curve_returns_segment_exhausted() {
    // Constant curve x(u) = 5.0 everywhere — control points (5, 5, 5, 5).
    let coeffs = CubicCoeffs::from_bezier(5.0, 5.0, 5.0, 5.0);
    let q = StepTimeQuery {
        coeffs: &coeffs,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted on motionless curve, got {:?}",
        result,
    );
}
