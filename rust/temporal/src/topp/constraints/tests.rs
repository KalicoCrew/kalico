use super::*;
use crate::Limits;
use crate::topp::path::ArclengthGrid;

// -------------------------------------------------------------------------
// Test fixtures
// -------------------------------------------------------------------------

fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
    // Synthetic grid: straight X-aligned line, zero curvature, unit X tangent.
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

// -------------------------------------------------------------------------
// Test 1 (plan's test): straight line, zero endpoints → BuildOutcome::Ok
// -------------------------------------------------------------------------

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
            assert!(b.n_vars >= 10); // at least the b_i variables
            assert_eq!(b.b_max_cent.len(), 10);
            // Zero curvature ⇒ no centripetal limit ⇒ b_max_cent at cap.
            for &cap in &b.b_max_cent {
                assert_eq!(cap, B_MAX_CENT_CAP);
            }
        }
        BuildOutcome::Boundary(_) => panic!("zero endpoints should not be infeasible"),
    }
}

// -------------------------------------------------------------------------
// Test 2 (plan's test): boundary-above-MVC returns Boundary outcome
// -------------------------------------------------------------------------

#[test]
fn boundary_above_mvc_returns_boundary_outcome() {
    // Curved grid: κ = 0.05 mm⁻¹ ⇒ b_max_cent = 2500 / 0.05 = 50_000.
    // v_start² = 60_000² = 3.6e9 > 50_000 ⇒ infeasible at start.
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

// -------------------------------------------------------------------------
// Test 3 (structural): N=5 straight line — pin variable layout and cone counts
// -------------------------------------------------------------------------

#[test]
#[allow(clippy::float_cmp, clippy::manual_range_contains)]
fn straight_line_n_vars_and_cone_count_match_design() {
    // N = 5, straight X line, zero endpoints.
    // Expect: n_vars = 5N - 6 = 5*5 - 6 = 19.
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
    assert_eq!(bundle.n_vars, 5 * 5 - 6); // = 19

    // ---- Nonneg-cone row counts ----
    //
    // For N=5, straight X-line (c'=[1,0,0], c''=[0,0,0], κ=0):
    //
    // (c) velocity UB:
    //   For each of the 5 grid points, only X-axis has |c'| = 1 ≥ COMP_FLOOR.
    //   Y and Z have |c'| = 0 → skipped. → 5 rows.
    //
    // (d) acceleration two-sided:
    //   gp = c'_ax, gpp = c''_ax. For X: gp=1.0, gpp=0.0 → 2 rows per point.
    //   For Y, Z: gp=0, gpp=0 → skipped.
    //   → 5 × 2 = 10 rows.
    //
    // (e) centripetal: 5 rows (always N rows).
    //
    // (f) jerk envelope: 2 × (5-2) = 6 rows.
    //
    // (g) x1, x2 nonneg: 2 × (5-2) = 6 rows.
    //
    // Total nonneg = 5 + 10 + 5 + 6 + 6 = 32.

    let nonneg_rows: usize = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Nonneg))
        .map(|(_, n)| *n)
        .sum();
    assert_eq!(nonneg_rows, 32, "structural drift");

    // ---- SOC block counts ----
    //
    // Block (h) emits 3 SOC-3 blocks per interior point = 3 × 3 = 9 blocks.
    let soc_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::SecondOrder))
        .count();
    assert_eq!(soc_block_count, 3 * (5 - 2));

    // ---- Zero-cone row counts ----
    //
    // (a) boundary equalities: 2 rows.
    // (b) acceleration linkage: 5 rows.
    // Total: 7 rows.
    let zero_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Zero))
        .map(|(_, n)| *n)
        .sum::<usize>();
    assert_eq!(zero_block_count, 7);

    // ---- Dimension sanity ----
    let total_cone_dim: usize = bundle.cones.iter().map(|(_, d)| d).sum();
    assert_eq!(bundle.a_rows.len(), total_cone_dim);
    assert_eq!(bundle.b_rhs.len(), total_cone_dim);
    for row in &bundle.a_rows {
        assert_eq!(row.len(), bundle.n_vars, "row width mismatch");
    }

    // ---- Objective pins ----
    // t variables at indices 2N..3N-2 = 10..13 should have objective = 1.0.
    // All others 0.
    for (idx, &coeff) in bundle.objective.iter().enumerate() {
        if idx >= 10 && idx < 13 {
            assert_eq!(coeff, 1.0, "t var at idx {idx} should have obj coeff 1.0");
        } else {
            assert_eq!(coeff, 0.0, "var at idx {idx} should have obj coeff 0.0");
        }
    }
}

// -------------------------------------------------------------------------
// Test 4 (edge case): N=2 minimum grid, no interior points
// -------------------------------------------------------------------------

#[test]
fn n_eq_2_minimum_grid_no_interior_points() {
    // N = 2: only the two boundary points, no interior.
    // The (N-2)-sized blocks (jerk envelope, x1/x2 nonneg, SOC chain) all become
    // zero-sized — the implementation guards with `if count > 0` skips so no
    // zero-dim cones leak into the output. Verifies that contract.
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
    assert_eq!(bundle.n_vars, 5 * 2 - 6); // = 4: only b_0, b_1, a_0, a_1
    // No SOC blocks should be emitted.
    let soc_block_count = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::SecondOrder))
        .count();
    assert_eq!(soc_block_count, 0);
    // Objective should be zero everywhere (no interior t_i to minimize).
    assert!(bundle.objective.iter().all(|c| c.abs() < 1e-12));
}
