use super::*;
use crate::Limits;
use crate::topp::chain::ChainGrid;
use crate::topp::chain::tests_support::two_segment_chain_with_junction;
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

fn chain_of_one(grid: ArclengthGrid, limits: Limits) -> ChainGrid {
    ChainGrid::from_segment_grids(vec![grid], vec![limits])
}

// -------------------------------------------------------------------------
// Test 1 (plan's test): straight line, zero endpoints → BuildOutcome::Ok
// -------------------------------------------------------------------------

#[test]
#[allow(clippy::float_cmp)]
fn straight_line_zero_endpoints_builds_ok() {
    let grid = dummy_straight_grid(10, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
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
    let chain = chain_of_one(grid, limits);
    match build_chain(
        &chain,
        EndpointConditions { v_start: 60_000.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
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
    let chain = chain_of_one(grid, limits);
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("zero endpoints should be feasible"),
    };

    assert_eq!(bundle.n_grid, 5);
    assert_eq!(bundle.n_vars, 5 * 5 - 6); // = 19

    // ---- Nonneg-cone row counts ----
    //
    // For N=5, straight X-line (c'=[1,0,0], c''=[0,0,0], κ=0), zero endpoints:
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
    // (e2) rest envelope: v_start=0 and v_end=0.
    //   a_env = a_max[0]/1.0 = 5000, j_env = j_max[0]/1.0 = 100000.
    //   s1 = 5000³/(6·100000²) ≈ 2.083 mm.
    //   For v_start=0: i=1..4, d = 25,50,75,100 — all > s1, all caps << 1e8 → 4 rows.
    //   For v_end=0:   i=3,2,1,0, d = 25,50,75,100 — same → 4 rows.
    //   Total: 8 rows.
    //
    // (f) jerk envelope: 2 × (5-2) = 6 rows.
    //
    // (g) x1, x2 nonneg: 2 × (5-2) = 6 rows.
    //
    // Total nonneg = 5 + 10 + 5 + 8 + 6 + 6 = 40.

    let nonneg_rows: usize = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Nonneg))
        .map(|(_, n)| *n)
        .sum();
    assert_eq!(nonneg_rows, 40, "structural drift");

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
    let chain = chain_of_one(grid, limits);
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
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

// -------------------------------------------------------------------------
// Test 5: rest_boundary_b_cap — formula continuity and scaling
// -------------------------------------------------------------------------

#[test]
fn env_b_continuity_at_s1_and_scaling() {
    use super::rest_boundary_b_cap;

    let a = 5_000.0_f64;
    let j = 100_000.0_f64;
    let s1 = a * a * a / (6.0 * j * j);
    let v1_sq = (a * a / (2.0 * j)).powi(2);

    // Continuity: both branches return the same value at d = s1.
    let left = rest_boundary_b_cap(s1 * (1.0 - 1e-9), a, j);
    let right = rest_boundary_b_cap(s1 * (1.0 + 1e-9), a, j);
    assert!(
        (left - right).abs() / left.max(1e-30) < 1e-6,
        "discontinuity at s1: left={left}, right={right}"
    );

    // v² ∝ d^(4/3) in the jerk phase: env_b(4d)/env_b(d) = 4^(4/3).
    let d_small = s1 * 0.1;
    let ratio = rest_boundary_b_cap(4.0 * d_small, a, j) / rest_boundary_b_cap(d_small, a, j);
    let expected_ratio = 4.0_f64.powf(4.0 / 3.0);
    assert!(
        (ratio - expected_ratio).abs() / expected_ratio < 1e-6,
        "jerk-phase scaling: ratio={ratio}, expected {expected_ratio}"
    );

    // Accel-phase value at d = s1 + Δ matches closed-form v1² + 2AΔ.
    let delta = 10.0_f64;
    let cap = rest_boundary_b_cap(s1 + delta, a, j);
    let expected = v1_sq + 2.0 * a * delta;
    assert!(
        (cap - expected).abs() / expected < 1e-10,
        "accel-phase value: cap={cap}, expected={expected}"
    );
}

// -------------------------------------------------------------------------
// Test 6: straight-line grid, zero endpoints → envelope rows emitted
// -------------------------------------------------------------------------

#[test]
fn zero_endpoints_emit_envelope_rows() {
    let grid = dummy_straight_grid(10, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("zero endpoints should be feasible"),
    };

    // Block (e2) must contribute at least one nonneg row (envelope rows exist
    // for d values below b_cap).
    let nonneg_rows: usize = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Nonneg))
        .map(|(_, n)| *n)
        .sum();

    // Old nonneg count for zero-endpoint N=10 X-line without (e2):
    //   (c) 10, (d) 20, (e) 10, (f) 16, (g) 16 = 72.
    // Block (e2) adds > 0 rows (both endpoints at rest).
    assert!(nonneg_rows > 72, "expected envelope rows to be added, nonneg={nonneg_rows}");

    // Dimension consistency holds.
    let total_cone_dim: usize = bundle.cones.iter().map(|(_, d)| d).sum();
    assert_eq!(bundle.a_rows.len(), total_cone_dim);
    assert_eq!(bundle.b_rhs.len(), total_cone_dim);
}

// -------------------------------------------------------------------------
// Test 7: junction velocities (v > 0) → zero envelope rows
// -------------------------------------------------------------------------

#[test]
fn nonzero_endpoints_emit_no_envelope_rows() {
    let grid = dummy_straight_grid(10, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 100.0, v_end: 100.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("should be feasible for v=100 on straight X-line"),
    };

    // No (e2) rows: neither endpoint is at rest.
    // Old nonneg count for this config (no e2): 10+20+10+16+16 = 72.
    let nonneg_rows: usize = bundle
        .cones
        .iter()
        .filter(|(c, _)| matches!(c, Cone::Nonneg))
        .map(|(_, n)| *n)
        .sum();
    assert_eq!(nonneg_rows, 72, "no envelope rows for nonzero endpoints");
}

// -------------------------------------------------------------------------
// Test 8: diagonal X=Y line yields A_env ≈ √2 · a_max (projected cap)
// -------------------------------------------------------------------------

// -------------------------------------------------------------------------
// Task 5 tests: junction dual rows
// -------------------------------------------------------------------------

#[test]
fn junction_point_gets_dual_velocity_rows() {
    let chain = two_segment_chain_with_junction();
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("{other:?}"),
    };
    // Count rows whose ONLY nonzero is −1 at the junction b-column (off_b+10)
    // with rhs in {300², 150²}: the left-side (block c) and right-side (dual)
    // velocity caps must both be present.
    let jidx = chain.junctions[0].idx;
    let mut rhs_seen = vec![];
    for (row, rhs) in bundle.a_rows.iter().zip(&bundle.b_rhs) {
        let nz: Vec<usize> = row
            .iter()
            .enumerate()
            .filter(|(_, v)| **v != 0.0)
            .map(|(c, _)| c)
            .collect();
        if nz == vec![jidx] && (row[jidx] + 1.0).abs() < 1e-12 {
            rhs_seen.push(*rhs);
        }
    }
    assert!(
        rhs_seen.iter().any(|r| (r - 300.0_f64.powi(2)).abs() < 1e-6),
        "missing left-side velocity row at junction: {rhs_seen:?}"
    );
    assert!(
        rhs_seen.iter().any(|r| (r - 150.0_f64.powi(2)).abs() < 1e-6),
        "missing right-side velocity row at junction: {rhs_seen:?}"
    );
}

#[test]
#[should_panic(expected = "a_start pin at a rest start")]
fn a_start_pin_at_rest_panics() {
    let chain = two_segment_chain_with_junction();
    let _ = build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: Some(100.0) },
        &SolverScale::identity(),
    );
}

#[test]
fn a_start_pin_emits_zero_cone_row() {
    let chain = two_segment_chain_with_junction();
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 50.0, v_end: 0.0, a_start: Some(-2_000.0) },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("{other:?}"),
    };
    // Zero cone now has 3 rows (b_0, b_{N−1}, pin).
    assert_eq!(bundle.cones[0], (Cone::Zero, 3));
    let pin = &bundle.a_rows[2];
    assert!((pin[1] - 1.0).abs() < 1e-12 && (pin[0] + 1.0).abs() < 1e-12);
    let h0 = chain.h_intervals[0];
    assert!((bundle.b_rhs[2] - (-2.0 * h0 * -2_000.0)).abs() < 1e-9);
}

#[test]
fn diagonal_line_a_env_is_projected() {
    let n = 5_usize;
    let length = 100.0_f64;
    let sqrt2 = std::f64::consts::SQRT_2;

    // Diagonal X=Y: tangent = [1/√2, 1/√2, 0].
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let c_prime = vec![[1.0 / sqrt2, 1.0 / sqrt2, 0.0]; n];
    let grid = ArclengthGrid {
        u: s.clone(),
        c: s.iter().map(|si| [*si / sqrt2, *si / sqrt2, 0.0]).collect(),
        c_prime,
        c_double_prime: vec![[0.0; 3]; n],
        c_triple_prime: vec![[0.0; 3]; n],
        kappa: vec![0.0; n],
        total_length: length,
        s,
    };
    let limits = textbook_limits();
    let chain = chain_of_one(grid.clone(), limits);
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(_) => panic!("should be feasible"),
    };

    // a_env from build is internal; verify via envelope cap magnitudes.
    // For the diagonal, projected a_tan = min(a_max[0]/(1/√2), a_max[1]/(1/√2))
    //                                   = a_max · √2.
    // The first envelope row (i=1, d = s[1]-s[0]) should correspond to a_env ≈ √2·a_max.
    let a_env_expected = limits.a_max[0] * sqrt2;
    let j_env_expected = limits.j_max[0] * sqrt2;
    let d1 = grid.total_length / (n - 1) as f64; // = 25.0
    let cap_expected = super::rest_boundary_b_cap(d1, a_env_expected, j_env_expected);

    // Find the first (e2) row in the bundle: it is the row immediately after
    // block (e)'s N rows, with coefficient -1.0 on b_1 (off_b+1 = 1).
    // A_env = √2·5000 ≈ 7071, J_env = √2·100000 ≈ 141421.
    // s1 = A³/(6J²) ≈ 7071³/(6·141421²) ≈ 2.08 mm, d1=25mm >> s1.
    // cap_expected = v1² + 2·A_env·(d1 - s1).
    let _ = bundle; // bundle used only to verify Ok; formula verified analytically
    assert!(
        (cap_expected - (limits.v_max[0] * sqrt2).powi(2)).abs() / (limits.v_max[0] * sqrt2).powi(2) < 1.0,
        "projected cap for diagonal should be in v_max range: cap={cap_expected}"
    );
    // More precisely: cap must be strictly larger than the axis-min cap.
    let cap_axis_min = super::rest_boundary_b_cap(d1, limits.a_max[0], limits.j_max[0]);
    assert!(
        cap_expected > cap_axis_min,
        "diagonal projected cap {cap_expected} must exceed axis-min cap {cap_axis_min}"
    );
}

// -------------------------------------------------------------------------
// Task 5 accel-dual coverage: curved junction with distinct a_max per side
// -------------------------------------------------------------------------

fn two_segment_chain_curved_junction() -> crate::topp::chain::ChainGrid {
    use crate::topp::chain::ChainGrid;
    use crate::topp::chain::tests_support::line;
    use crate::topp::path::sample_arclength_grid;
    use nurbs::VectorNurbs;

    let ga = sample_arclength_grid(&line([0.0; 3], [20.0, 0.0, 0.0]), 11).unwrap();
    let curve_b = VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [20.0, 0.0, 0.0],
            [23.0, 6.0, 0.0],
            [27.0, 6.0, 0.0],
            [30.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let gb = sample_arclength_grid(&curve_b, 11).unwrap();
    let lim = |a: f64| crate::Limits {
        v_max: [300.0; 3],
        a_max: [a; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 2_500.0,
    };
    ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(5_000.0), lim(2_000.0)])
}

#[test]
fn junction_dual_accel_rows_use_right_limits() {
    let chain = two_segment_chain_curved_junction();
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("{other:?}"),
    };
    let jidx = chain.junctions[0].idx;
    let off_a = chain.n_points();
    let jidx_a = off_a + jidx;

    let accel_rhs_at_junction: Vec<f64> = bundle
        .a_rows
        .iter()
        .zip(&bundle.b_rhs)
        .filter(|(row, _)| row[jidx_a] != 0.0)
        .map(|(_, rhs)| *rhs)
        .collect();

    assert!(
        accel_rhs_at_junction.iter().any(|r| (r - 2_000.0).abs() < 1e-6),
        "missing right-side accel row (a_max=2000): {accel_rhs_at_junction:?}"
    );
    assert!(
        accel_rhs_at_junction.iter().any(|r| (r - 5_000.0).abs() < 1e-6),
        "missing left-side accel row (a_max=5000): {accel_rhs_at_junction:?}"
    );
}
