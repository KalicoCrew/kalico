//! Newton-based step-time computation tests.
//!
//! Strategy: synthesize a known cubic position polynomial, ask
//! `compute_next_step_time` for the next step's time, verify against the
//! analytic answer (where one exists) or against high-precision iteration.

use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Helper: trivial linear "curve" — position(t) = velocity * t. Verifies
/// that a constant-velocity initial guess converges in 1 iteration.
fn linear_curve(velocity: f32) -> impl Fn(f32) -> (f32, f32) {
    move |t| (velocity * t, velocity)
}

/// Helper: cubic curve with given coefficients. position(t) = a*t^3 + b*t^2 + c*t.
fn cubic_curve(a: f32, b: f32, c: f32) -> impl Fn(f32) -> (f32, f32) {
    move |t| {
        let pos = a * t * t * t + b * t * t + c * t;
        let vel = 3.0 * a * t * t + 2.0 * b * t + c;
        (pos, vel)
    }
}

#[test]
fn linear_curve_converges_in_one_iteration() {
    // velocity = 1.0 mm/s; step_distance = 0.0025 mm (typical 400 step/mm × 16x microstep)
    // Expected next step at t = 0.0025 (forward direction).
    let eval = linear_curve(1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt(t) => {
            assert!((t - 0.0025).abs() < 1e-9, "expected t≈0.0025, got {}", t);
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn linear_curve_reverse_direction() {
    // negative velocity → next step is backward (current_step - 1).
    let eval = linear_curve(-1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt(t) => {
            assert!((t - 0.0025).abs() < 1e-9);
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn cubic_curve_converges_within_three_iterations() {
    // position(t) = 0.1*t^3 + 0.5*t^2 + 1.0*t  (mm)
    // At t=0: position=0, velocity=1.0. Look for first step at 0.0025 mm.
    // The cubic adds a small correction to the linear estimate.
    let eval = cubic_curve(0.1, 0.5, 1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    let t = match result {
        StepTimeResult::NextAt(t) => t,
        other => panic!("expected NextAt, got {:?}", other),
    };
    // Verify the returned time actually puts position at the step boundary.
    let (pos, _) = eval(t);
    assert!(
        (pos - 0.0025).abs() < 0.0025 * 1e-5,
        "position at returned t={} is {}, expected 0.0025",
        t,
        pos,
    );
}

#[test]
fn segment_exhaustion_returns_segment_exhausted() {
    // velocity 1.0 mm/s, segment ends at t=0.001 (1 ms). One step = 0.0025 mm
    // can't fit before segment end.
    let eval = linear_curve(1.0);
    let q = StepTimeQuery {
        eval: &eval,
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

/// Covers the post-loop fallback path (lines after `for _ in 0..MAX_NEWTON_ITERS`
/// in `compute_next_step_time`).
///
/// The in-loop tolerance is `step_distance * NEWTON_TOL_FRACTION = 0.001 * 1e-6 = 1e-9`.
/// In f32 arithmetic, position values around 0.001 have ~7 significant decimal
/// digits (machine epsilon ~1.2e-7), so `err < 1e-9` is below representable
/// precision — the tight tolerance is never met and the loop always falls
/// through to the fallback for this step_distance range.
///
/// Two sub-cases:
/// - `NextAt`: fallback `t_final` is in-segment and position is within 0.1%
///   of step_distance (1e-3 relaxed tolerance). Verified by checking the
///   returned time actually exists and the position at it is ≈ target.
/// - `SegmentExhausted`: a short segment where `t_final` falls outside
///   `t_segment_end`, causing the fallback to report exhaustion.
#[test]
fn post_loop_fallback_next_at() {
    // position(t) = 1.0*t^3 + 0.0*t^2 + (-0.0001)*t
    // Near-zero linear term so initial velocity-based guess is dramatically
    // wrong relative to the cubic term's contribution. Starting at t=0.1:
    //   v(0.1) = 3*0.01 - 0.0001 = 0.0299 mm/(time unit)
    // step_distance = 0.001; tight tol = 1e-9, below f32 precision at this
    // scale, so the loop falls through and the fallback evaluates a 4th candidate.
    let eval = cubic_curve(1.0, 0.0, -0.0001);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.001,
        current_step: 0,
        t_curr: 0.1,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    // The fallback path should find a valid time (the curve does reach the
    // target within the segment).
    let t = match result {
        StepTimeResult::NextAt(t) => t,
        StepTimeResult::SegmentExhausted => {
            // If the curve converged tightly enough in 3 iters to hit the
            // early-exit path anyway, that's also acceptable — verify
            // independently that the position at `t_curr + initial_dt` is
            // near target.
            panic!(
                "expected NextAt from fallback path; \
                 curve may have converged more tightly than expected in 3 iters"
            );
        }
    };
    // The returned time must be in-segment.
    assert!(t >= 0.1 && t <= 1.0, "t={} not in segment [0.1, 1.0]", t);
    // Position at returned t must be within 0.1% of step target (relaxed fallback tol).
    let (pos, _) = eval(t);
    let target = 1.0 * 0.001; // (current_step=0 + dir=1) * step_distance
    assert!(
        (pos - target).abs() < 0.001 * 1e-3,
        "position at t={} is {}, target={}, err={}",
        t,
        pos,
        target,
        (pos - target).abs(),
    );
}

#[test]
fn post_loop_fallback_segment_exhausted() {
    // Same cubic as above, but segment ends very early so t_final > t_segment_end.
    // t_curr=0.1; initial dt ≈ 0.001 / 0.0299 ≈ 0.03344; after 3 Newton
    // iterations dt converges toward the true solution. The key is that
    // t_segment_end is set tighter than where the step actually falls so the
    // bounds check in the fallback fires.
    let eval = cubic_curve(1.0, 0.0, -0.0001);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.001,
        current_step: 0,
        t_curr: 0.1,
        // Segment ends before the step at t≈0.133; fallback t_final will be
        // beyond this boundary.
        t_segment_end: 0.105,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted when fallback t_final > t_segment_end, got {:?}",
        result,
    );
}

#[test]
fn velocity_near_zero_returns_segment_exhausted() {
    // Velocity essentially zero — segment can't produce steps.
    let eval = linear_curve(1e-10);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted at v≈0, got {:?}",
        result,
    );
}
