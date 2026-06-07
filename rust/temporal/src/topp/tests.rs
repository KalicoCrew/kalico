use super::*;

/// Straight 600 mm collinear cubic at machine-limit speed: v_max = 1000 mm/s,
/// a_max = 50 km/s². Limits mirror the bridge's `to_temporal_limits` output for
/// `max_velocity=1000, max_accel=50000, scv=5` exactly (including the tiny
/// a_centripetal_max) — the solver must produce a usable profile, not MaxIter.
#[test]
fn schedule_segment_straight_line_at_1000mms_solves() {
    let cps = vec![
        [0.0, 0.0, 0.0],
        [200.0, 0.0, 0.0],
        [400.0, 0.0, 0.0],
        [600.0, 0.0, 0.0],
    ];
    let curve =
        VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps)
            .unwrap();
    let limits = Limits {
        v_max: [1000.0, 1000.0, 15.0],
        a_max: [50_000.0, 50_000.0, 100.0],
        j_max: [100_000.0, 100_000.0, 100_000.0],
        a_centripetal_max: 0.001,
    };
    let cfg = GridConfig {
        scheme: crate::GridScheme::UniformArclength,
        n: 200,
    };
    let profile =
        schedule_segment_with_tolerance(&curve, &limits, &cfg, 0.0, 0.0, ToleranceMode::Auto)
            .expect("schedule should not hit a setup error");
    assert!(
        matches!(
            profile.status,
            crate::SolveStatus::Solved
                | crate::SolveStatus::SolvedInexact { .. }
                | crate::SolveStatus::SolvedSlp { .. }
        ),
        "limit-speed straight line must solve, got {:?}",
        profile.status,
    );
    let peak_v = profile
        .samples
        .iter()
        .map(|s| s.v)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (peak_v - 1000.0).abs() < 15.0,
        "peak velocity {peak_v:.1} mm/s, expected cruise ≈ 1000",
    );
}

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
    // Endpoints zero-velocity, midpoint nontrivial.
    assert!(profile.samples[0].v < 1e-3);
    assert!(profile.samples[49].v < 1e-3);
    assert!(profile.samples[25].v > 100.0); // ≥ 100 mm/s
    // Total time should be finite and positive.
    assert!(profile.total_time.is_finite() && profile.total_time > 0.0);
}
