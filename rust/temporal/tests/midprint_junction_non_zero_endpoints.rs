use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode, schedule_segment_with_tolerance,
};

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
    )
    .unwrap()
}

fn standard_limits() -> Limits {
    Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 6000.0],
        25.0_f64 / 1500.0,
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

    let v_start = 30.0;
    let v_end = 50.0;

    let profile =
        schedule_segment_with_tolerance(&curve, &limits, &cfg, v_start, v_end, ToleranceMode::Auto)
            .expect("schedule_segment_with_tolerance");

    assert!(
        matches!(
            profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "expected Solved/SolvedInexact/SolvedSlp, got {:?}",
        profile.status,
    );

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

    assert!(profile.total_time.is_finite());
    assert!(
        profile.total_time > 0.15 && profile.total_time < 5.0,
        "total_time={} outside reasonable range [0.15, 5.0] s",
        profile.total_time
    );
}
