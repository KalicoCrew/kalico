use super::*;
use crate::multi::joining::ChainState;
use crate::topp::chain::ChainGrid;
use crate::topp::path::sample_arclength_grid;
use crate::{Limits};
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

fn straight_chain() -> ChainGrid {
    let grid = sample_arclength_grid(&straight(), 20).unwrap();
    ChainGrid::from_segment_grids(vec![grid], vec![limits()])
}

#[test]
fn fan_out_processes_all_dirty() {
    let chains: Vec<ChainGrid> = (0..4).map(|_| straight_chain()).collect();
    let mut states: Vec<ChainState> = (0..4)
        .map(|_| ChainState {
            v_start: 0.0,
            v_end: 0.0,
            a_start: None,
            profile: None,
            dirty: true,
        })
        .collect();
    fan_out_solves(&chains, &mut states, 3).unwrap();
    for s in &states {
        assert!(s.profile.is_some());
        assert!(!s.dirty);
    }
}

/// Both endpoints pinned + infeasible initial solve (v_start above the
/// centripetal MVC cap) → the failed status returns unmodified, no bisection.
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
    let grid = sample_arclength_grid(&curved, 20).unwrap();
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![curved_limits]);

    let profile =
        solve_with_boundary_fallback(&chain, 100.0, 0.0, None, true, true)
            .expect("must not return ScheduleError");
    assert!(
        !is_success(profile.status),
        "with both endpoints pinned and an infeasible problem the fallback \
         must return a non-success status, got {:?}",
        profile.status,
    );
}
