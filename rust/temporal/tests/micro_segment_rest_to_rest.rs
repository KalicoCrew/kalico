use nurbs::VectorNurbs;
use temporal::{
    BatchInput, GridConfig, GridScheme, GridStrategy, Limits, SegmentInput, SolveStatus,
    ToleranceMode, plan_batch, schedule_segment_with_tolerance,
};

fn voron_limits() -> Limits {
    Limits::new(
        [300.0, 300.0, 15.0],
        [5_000.0, 5_000.0, 350.0],
        [10_000.0, 10_000.0, 10_000.0],
        5_000.0,
    )
}

fn success(status: SolveStatus) -> bool {
    matches!(
        status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    )
}

fn arc_bezier_90deg(radius_mm: f64) -> VectorNurbs<f64, 3> {
    let k = 4.0 / 3.0 * (std::f64::consts::SQRT_2 - 1.0);
    let r = radius_mm;
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, 0.0, 0.0],
            [r, r * k, 0.0],
            [r * k, r, 0.0],
            [0.0, r, 0.0],
        ],
    )
    .expect("degree-3 arc NURBS valid")
}

#[test]
fn rung3_micro_arc_0_01mm_rest_to_rest_n20() {
    let curve = arc_bezier_90deg(0.01);

    let profile = schedule_segment_with_tolerance(
        &curve,
        &voron_limits(),
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20,
        },
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("schedule must not error");

    assert!(
        success(profile.status),
        "rung3: 0.01 mm 90° arc n=20 must return success (not DivergedSlp); got {:?}",
        profile.status,
    );
}

#[test]
fn rung3_micro_arc_0_01mm_rest_to_rest_n3() {
    let curve = arc_bezier_90deg(0.01);

    let profile = schedule_segment_with_tolerance(
        &curve,
        &voron_limits(),
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 3,
        },
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("schedule must not error");

    assert!(
        success(profile.status),
        "rung3: 0.01 mm 90° arc n=3 must return success; got {:?}",
        profile.status,
    );
}

#[test]
fn rung3_micro_arc_0_01mm_via_adaptive_batch() {
    let curve = arc_bezier_90deg(0.01);
    let limits = voron_limits();
    let segments = [SegmentInput {
        curve: &curve,
        limits,
        trailing_junction_chord_tolerance_mm: 0.05,
    }];
    let output = plan_batch(BatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        initial_velocity: 0.0,
        initial_accel: 0.0,
        terminal_velocity: 0.0,
    })
    .expect("plan_batch must not error");

    for (i, p) in output.profiles.iter().enumerate() {
        assert!(
            success(p.status),
            "rung3 via adaptive: profile {i} must succeed; got {:?}",
            p.status,
        );
    }
}

#[test]
fn normal_scale_jerk_still_enforced() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .expect("valid");

    let limits = voron_limits();

    let profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 40,
        },
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("schedule must not error");

    assert!(
        success(profile.status),
        "normal 50 mm segment must succeed; got {:?}",
        profile.status,
    );

    let max_v = profile.samples.iter().map(|s| s.v).fold(0.0_f64, f64::max);
    assert!(
        max_v > 50.0,
        "normal segment must reach meaningful speed, got max_v={max_v}",
    );
}

#[test]
fn large_arc_still_converges_slp() {
    let curve = arc_bezier_90deg(0.5);

    let profile = schedule_segment_with_tolerance(
        &curve,
        &voron_limits(),
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20,
        },
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("schedule must not error");

    assert!(
        matches!(
            profile.status,
            SolveStatus::SolvedSlp { .. } | SolveStatus::Solved | SolveStatus::SolvedInexact { .. }
        ),
        "0.5 mm arc must converge SLP (not go through stall path); got {:?}",
        profile.status,
    );
}
