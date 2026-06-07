use super::*;

#[test]
fn schedule_segment_straight_line_returns_profile() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
    )
    .unwrap();
    let limits = Limits {
        v_max: [500.0, 500.0, 500.0],
        a_max: [5_000.0, 5_000.0, 5_000.0],
        j_max: [100_000.0, 100_000.0, 100_000.0],
        a_centripetal_max: 2_500.0,
    };
    let cfg = GridConfig {
        scheme: crate::GridScheme::UniformArclength,
        n: 50,
    };
    let profile =
        schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule_segment should succeed");
    assert_eq!(profile.samples.len(), 50);
    assert!(matches!(
        profile.status,
        crate::SolveStatus::Solved | crate::SolveStatus::SolvedInexact { .. }
    ));
    assert!(profile.samples[0].v < 1e-3);
    assert!(profile.samples[49].v < 1e-3);
    assert!(profile.samples[25].v > 100.0);
    assert!(profile.total_time.is_finite() && profile.total_time > 0.0);
}
