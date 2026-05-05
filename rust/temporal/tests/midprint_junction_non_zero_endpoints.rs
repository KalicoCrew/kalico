//! Non-zero-endpoint mid-print junction fixture.
//!
//! Option B's boundary stencils (i=0 and i=n-1) carry O(h)·b''' truncation
//! that is mass-zero on rest-to-rest moves like homing (√b_endpoint = 0).
//! This fixture probes a v_start=30 / v_end=50 single-segment scenario where
//! the boundary truncation is non-zero, ensuring the verifier accepts within
//! `EPS_FEAS=2e-3`.
//!
//! Spec section 6.5.

use temporal::{
    schedule_segment_with_tolerance, GridConfig, GridScheme, Limits, SolveStatus,
    ToleranceMode,
};
use nurbs::VectorNurbs;

fn pure_x_50mm_collinear_cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [50.0 / 3.0, 0.0, 0.0],
            [100.0 / 3.0, 0.0, 0.0],
            [50.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap()
}

fn standard_limits() -> Limits {
    Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 6000.0],
        25.0_f64 / 1500.0, // a_centripetal at typical sqv=5
    )
}

#[test]
fn midprint_junction_non_zero_endpoints_converge() {
    let curve = pure_x_50mm_collinear_cubic();
    let limits = standard_limits();
    let cfg = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 100,
    };

    // v_start=30, v_end=50: non-zero at both endpoints, exercises the
    // boundary-stencil O(h)*b''' truncation.
    let v_start = 30.0;
    let v_end = 50.0;

    let profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &cfg,
        v_start,
        v_end,
        ToleranceMode::Auto,
    )
    .expect("schedule_segment_with_tolerance");

    assert!(
        matches!(
            profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "expected Solved/SolvedInexact/SolvedSlp, got {:?}",
        profile.status,
    );

    // Boundary samples should reflect the requested endpoint velocities
    // within the trapezoidal-integration tolerance.
    let first = profile.samples.first().expect("at least one sample");
    let last = profile.samples.last().expect("at least one sample");
    assert!(
        (first.v - v_start).abs() < 0.5,
        "first sample v={} vs v_start={}",
        first.v,
        v_start
    );
    assert!(
        (last.v - v_end).abs() < 0.5,
        "last sample v={} vs v_end={}",
        last.v,
        v_end
    );

    // Profile total time should be finite and reasonable for a 50mm
    // segment with v_start=30, v_end=50 under v_max=300, a_max=3000.
    // Lower bound from average velocity 50/v_max = 0.167s; upper bound
    // generous to allow for jerk/accel-limited shaping.
    assert!(profile.total_time.is_finite());
    assert!(
        profile.total_time > 0.15 && profile.total_time < 5.0,
        "total_time={} outside reasonable range [0.15, 5.0] s",
        profile.total_time
    );
}
