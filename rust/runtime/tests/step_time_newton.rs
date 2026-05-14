//! Step-time computation tests (Bernstein root-finder solver).
//!
//! Each test synthesises a known cubic via Bezier control points and asks
//! `compute_next_step_time` for the next step's time. Assertions check the
//! returned `t` against the analytic root.
//!
//! Spec: docs/superpowers/specs/2026-05-14-bernstein-step-root-design.md

use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Build the four Bezier CPs of the cubic `a·u³ + b·u² + c·u + d`.
/// The inverse of the standard Bernstein → monomial expansion:
///   p0 = d
///   p1 = c/3 + p0
///   p2 = b/3 + 2·p1 − p0
///   p3 = a + 3·p2 − 3·p1 + p0
fn cps_from_monomial(a: f64, b: f64, c: f64, d: f64) -> [f64; 4] {
    let p0 = d;
    let p1 = c / 3.0 + p0;
    let p2 = b / 3.0 + 2.0 * p1 - p0;
    let p3 = a + 3.0 * p2 - 3.0 * p1 + p0;
    [p0, p1, p2, p3]
}

/// Linear curve: position(u) = velocity·u.
fn linear_cps(velocity: f64) -> [f64; 4] {
    cps_from_monomial(0.0, 0.0, velocity, 0.0)
}

/// Cubic curve: position(u) = a·u³ + b·u² + c·u.
fn cubic_cps(a: f64, b: f64, c: f64) -> [f64; 4] {
    cps_from_monomial(a, b, c, 0.0)
}

/// De Casteljau evaluation at `t` for the Bezier curve with CPs `cps`.
/// Used by tests that need to verify the position at a returned root.
fn eval_at(cps: [f64; 4], t: f64) -> f64 {
    let one_minus_t = 1.0 - t;
    let b00 = one_minus_t * cps[0] + t * cps[1];
    let b01 = one_minus_t * cps[1] + t * cps[2];
    let b02 = one_minus_t * cps[2] + t * cps[3];
    let b10 = one_minus_t * b00 + t * b01;
    let b11 = one_minus_t * b01 + t * b02;
    one_minus_t * b10 + t * b11
}

#[test]
fn linear_curve_returns_analytic_root() {
    // velocity = 1.0 mm/(u-unit); step_distance = 0.0025 mm.
    // Expected next step at u = 0.0025 (forward direction).
    let cps = linear_cps(1.0);
    let q = StepTimeQuery {
        cps,
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
    let cps = linear_cps(-1.0);
    let q = StepTimeQuery {
        cps,
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
    let cps = cubic_cps(0.1, 0.5, 1.0);
    let q = StepTimeQuery {
        cps,
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
    let pos = eval_at(cps, t);
    assert!(
        (pos - 0.0025).abs() < 1e-5,
        "position at returned t={} is {}, expected 0.0025",
        t,
        pos,
    );
}

#[test]
fn segment_exhaustion_returns_segment_exhausted() {
    // velocity 1.0 mm/(u-unit), segment ends at u=0.001. One step = 0.0025 mm
    // can't fit before segment end (root would be at u=0.0025 > 0.001).
    let cps = linear_cps(1.0);
    let q = StepTimeQuery {
        cps,
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
/// Newton into the fallback path. The bezier_root solver handles it
/// directly — the test verifies the returned root sits at the step
/// boundary regardless of conditioning.
#[test]
fn ill_conditioned_cubic_returns_root_at_step_boundary() {
    // position(u) = 1.0·u³ + 0.0·u² + (-0.0001)·u
    // Starting at u=0.1: position = 0.001 - 0.00001 = 0.00099.
    // step_distance = 0.001, current_step = 0, dir = +1 → target = 0.001.
    // Solve u³ - 0.0001·u - 0.001 = 0 for u > 0.1 — root is at roughly
    // u ≈ 0.10033.
    let cps = cubic_cps(1.0, 0.0, -0.0001);
    let q = StepTimeQuery {
        cps,
        step_distance: 0.001,
        current_step: 0,
        t_curr: 0.1,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    let t = match result {
        StepTimeResult::NextAt { t, .. } => t,
        StepTimeResult::SegmentExhausted => panic!(
            "expected NextAt; bezier_root should yield a step on this well-formed cubic"
        ),
    };
    assert!(t > 0.1 && t <= 1.0, "t={} not in (0.1, 1.0]", t);
    let pos = eval_at(cps, t);
    let target = 0.001;
    assert!(
        (pos - target).abs() < 1e-5,
        "position at t={} is {}, target={}, err={}",
        t,
        pos,
        target,
        (pos - target).abs(),
    );
}

/// A monotonic cubic with a t_segment_end that lies well before any root.
/// The solver must return SegmentExhausted because no root exists in
/// `(t_curr, t_segment_end]`. Note: `bezier_root` treats targets within
/// `EPS_OUT_OF_RANGE ≈ 1e-5` of the curve's value range as "essentially
/// in range" (f32 noise floor accommodation), so the test interval needs
/// the position at `t_segment_end` to fall strictly further below
/// `target` than that tolerance.
#[test]
fn no_root_in_short_segment_returns_segment_exhausted() {
    // x(u) = u³ + u — strictly monotone-increasing (x'(u) = 3u²+1 > 0).
    // dir=+1 (v0 = 1 > 0), target = (0 + 1)·1.0 = 1.0.
    // At u=0.5: x = 0.625. Gap to target = 0.375 ≫ EPS_OUT_OF_RANGE.
    let cps = cubic_cps(1.0, 0.0, 1.0);
    let q = StepTimeQuery {
        cps,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 0.5,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted when no root in short segment, got {:?}",
        result,
    );
}

/// Truly motionless curve: constant position, zero velocity everywhere.
/// The midpoint-probe fallback also sees zero motion and reports
/// SegmentExhausted.
#[test]
fn motionless_curve_returns_segment_exhausted() {
    // Constant curve x(u) = 5.0 everywhere — control points (5, 5, 5, 5).
    let cps: [f64; 4] = [5.0, 5.0, 5.0, 5.0];
    let q = StepTimeQuery {
        cps,
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
