use super::*;
use crate::multi::SegmentInput;
use crate::{GridConfig, GridScheme, Limits};
use nurbs::VectorNurbs;

fn straight() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap()
}

fn limits() -> Limits {
    Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

#[test]
fn fan_out_processes_all_dirty() {
    let curves: Vec<_> = (0..4).map(|_| straight()).collect();
    let inputs: Vec<SegmentInput> = curves
        .iter()
        .map(|c| SegmentInput {
            curve: c,
            limits: limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        })
        .collect();
    let grids = vec![
        GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20
        };
        4
    ];
    let mut states: Vec<_> = (0..4)
        .map(|_| SegmentState {
            v_start: 0.0,
            v_end: 0.0,
            profile: None,
            dirty: true,
        })
        .collect();
    fan_out_solves(&inputs, &mut states, &grids, 3).unwrap();
    for s in &states {
        assert!(s.profile.is_some());
        assert!(!s.dirty);
    }
}

/// A deliberately infeasible solve with both endpoints pinned must return the
/// failed status unmodified. The fallback must not bisect down to a different
/// velocity, as that would create a commanded-velocity discontinuity.
///
/// Infeasibility is forced via the centripetal MVC pre-check: a high-curvature
/// cubic arc with `a_centripetal_max = 2500` and `kappa ≈ 1 mm⁻¹` caps the
/// start velocity at ~50 mm/s. Requesting `v_start = 100` triggers
/// `BuildOutcome::Boundary` before the SOCP is built, which returns an
/// `Infeasible` profile — a guaranteed non-success on the initial solve.
#[test]
fn pinned_both_endpoints_returns_failed_status_unmodified() {
    // Cubic Bézier approximation of a 90° arc with radius ≈ 1.0 mm.
    // Standard formula: k = (4/3)(√2 − 1) ≈ 0.5523.
    let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
    let r = 1.0_f64;
    let curved = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [r * k, 0.0, 0.0],
            [r, r * (1.0 - k), 0.0],
            [r, r, 0.0],
        ],
    )
    .unwrap();
    // kappa ≈ 1/r = 1.0 mm⁻¹ → b_mvc ≈ a_cent / kappa = 2500 → v_mvc ≈ 50 mm/s.
    // v_start = 100 >> 50 → Boundary infeasibility → non-success status.
    let curved_limits = limits();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 20,
    };
    // pin_start=true, pin_end=true: the fallback must NOT bisect or alter any
    // velocity — failing loudly preserves the physical boundary condition.
    let profile =
        solve_with_boundary_fallback(&curved, &curved_limits, grid, 100.0, 0.0, true, true)
            .expect("must not return ScheduleError");
    assert!(
        !is_success(profile.status),
        "with both endpoints pinned and an infeasible problem the fallback \
         must return a non-success status, got {:?}",
        profile.status,
    );
}
