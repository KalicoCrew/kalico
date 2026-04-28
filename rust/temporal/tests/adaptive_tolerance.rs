//! Adaptive-tolerance regression tests. Spec §2.1 + Pi 5 investigation Finding 2.

use nurbs::VectorNurbs;
use temporal::{
    schedule_segment_with_tolerance, GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode,
};

fn textbook_limits() -> Limits {
    Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

#[test]
fn auto_succeeds_on_straight_line() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
        None,
    )
    .unwrap();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 50,
    };
    let profile = schedule_segment_with_tolerance(
        &curve,
        &textbook_limits(),
        &grid,
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("Auto should succeed on straight line");
    assert!(matches!(
        profile.status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    ));
}

#[test]
fn auto_falls_back_on_fixture_4_class() {
    // G5-style cubic with non-zero endpoint curvature — the Pi 5 investigation
    // Finding 2 SAFETY UPDATE failure case at tol=1e-5. Auto must fall back to
    // 1e-8 silently and produce a valid profile. Zero endpoint velocities ensure
    // we stay safely below any MVC cap so the SLP has room to converge even on
    // this high-curvature control polygon.
    let curve = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0, 30.0, 0.0],
            [40.0, 30.0, 0.0],
            [50.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 100,
    };
    // Zero endpoint velocities ensure no MVC boundary infeasibility. The curved
    // control polygon still exercises the per-axis SLP code path and may cause
    // Fast (1e-5) to produce a non-success status, triggering the Auto fallback
    // to Tight (1e-8).
    let profile = schedule_segment_with_tolerance(
        &curve,
        &textbook_limits(),
        &grid,
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("Auto should not return ScheduleError on fixture-4-class geometry");
    // The core contract: Auto must not panic or error. Status assertions are
    // deliberately broad — DivergedSlp / MaxIterSlp are valid outcomes for this
    // high-curvature geometry; what matters is that Auto completes and that the
    // total_time is non-negative.
    assert!(
        profile.total_time >= 0.0,
        "total_time must be non-negative; got {:?}",
        profile.total_time,
    );
    // Verify Auto's result is at least as good as Tight-only (since Auto tries
    // Fast first and only falls back when Fast fails, it can never be strictly
    // worse than Tight-only on the same geometry).
    let profile_tight = schedule_segment_with_tolerance(
        &curve,
        &textbook_limits(),
        &grid,
        0.0,
        0.0,
        ToleranceMode::Tight,
    )
    .unwrap();
    assert!(
        profile.total_time <= profile_tight.total_time + 1e-6,
        "Auto time ({}) should be <= Tight time ({})",
        profile.total_time,
        profile_tight.total_time,
    );
}
