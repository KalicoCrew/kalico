use super::*;
use crate::Limits;
use crate::topp::path::ArclengthGrid;

fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let u = s.clone();
    let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
    let kappa = vec![0.0; n];
    ArclengthGrid {
        s,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        kappa,
        total_length: length,
    }
}

fn textbook_limits() -> Limits {
    Limits {
        v_max: [500.0, 500.0, 500.0],
        a_max: [5_000.0, 5_000.0, 5_000.0],
        j_max: [100_000.0, 100_000.0, 100_000.0],
        a_centripetal_max: 2_500.0,
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn straight_line_zero_endpoints_builds_ok() {
    let grid = dummy_straight_grid(10, 100.0);
    let limits = textbook_limits();
    match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 0.0,
            v_end: 0.0,
        },
    ) {
        BuildOutcome::Ok(b) => {
            assert_eq!(b.n_grid, 10);
            assert!(b.n_vars >= 10);
            assert_eq!(b.b_max_cent.len(), 10);
            for &cap in &b.b_max_cent {
                assert_eq!(cap, B_MAX_CENT_CAP);
            }
        }
        BuildOutcome::Boundary(_) => panic!("zero endpoints should not be infeasible"),
    }
}

#[test]
fn boundary_above_mvc_returns_boundary_outcome() {
    let mut grid = dummy_straight_grid(5, 10.0);
    grid.kappa = vec![0.05; 5];
    let limits = textbook_limits();
    match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 60_000.0,
            v_end: 0.0,
        },
    ) {
        BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
            assert!((mvc_b - 50_000.0).abs() < 1e-3);
        }
        other => panic!("expected StartAboveMvc, got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp, clippy::manual_range_contains)]
fn straight_line_n_vars_and_cone_count_match_design() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let bundle = match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 0.0,
            v_end: 0.0,
        },
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("zero endpoints should be feasible"),
    };

    assert_eq!(bundle.n_grid, 5);
    assert_eq!(bundle.n_vars, 5 * 5 - 6);

    let nonneg_rows: usize = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Nonneg))
        .map(|(_, n)| *n)
        .sum();
    assert_eq!(nonneg_rows, 32, "structural drift");

    let soc_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::SecondOrder))
        .count();
    assert_eq!(soc_block_count, 3 * (5 - 2));

    let zero_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Zero))
        .map(|(_, n)| *n)
        .sum::<usize>();
    assert_eq!(zero_block_count, 7);

    let total_cone_dim: usize = bundle.cones.iter().map(|(_, d)| d).sum();
    assert_eq!(bundle.a_rows.len(), total_cone_dim);
    assert_eq!(bundle.b_rhs.len(), total_cone_dim);
    for row in &bundle.a_rows {
        assert_eq!(row.len(), bundle.n_vars, "row width mismatch");
    }

    for (idx, &coeff) in bundle.objective.iter().enumerate() {
        if idx >= 10 && idx < 13 {
            assert_eq!(coeff, 1.0, "t var at idx {idx} should have obj coeff 1.0");
        } else {
            assert_eq!(coeff, 0.0, "var at idx {idx} should have obj coeff 0.0");
        }
    }
}

#[test]
fn n_eq_2_minimum_grid_no_interior_points() {
    let grid = dummy_straight_grid(2, 50.0);
    let limits = textbook_limits();
    let bundle = match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 0.0,
            v_end: 0.0,
        },
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("zero endpoints should be feasible"),
    };
    assert_eq!(bundle.n_grid, 2);
    assert_eq!(bundle.n_vars, 5 * 2 - 6);
    let soc_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::SecondOrder))
        .count();
    assert_eq!(soc_block_count, 0);
    assert!(bundle.objective.iter().all(|c| c.abs() < 1e-12));
}
