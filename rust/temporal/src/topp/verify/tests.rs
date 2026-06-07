use super::*;
use crate::Limits;
use crate::topp::chain::ChainGrid;
use crate::topp::path::ArclengthGrid;
use crate::topp::solver::{SolverResult, SolverStatus};

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

fn chain_of_one(grid: ArclengthGrid, limits: Limits) -> ChainGrid {
    ChainGrid::from_segment_grids(vec![grid], vec![limits])
}

#[test]
fn zero_profile_is_feasible() {
    let grid = dummy_straight_grid(5, 10.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let result = SolverResult {
        b: vec![0.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(report.feasible);
    assert!(report.worst_violation < EPS_FEAS);
    assert!(
        report
            .binding_per_grid
            .iter()
            .all(|b| matches!(b, BindingConstraint::Boundary | BindingConstraint::None))
    );
}

#[test]
fn over_velocity_profile_flagged() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    // b = v² far above v_max² = 250_000 ⇒ infeasible on velocity.
    let result = SolverResult {
        b: vec![1_000_000.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(!report.feasible);
}

// ---- Additional coverage tests -----------------------------------------

/// A profile right at `v_max` should be feasible (ratio == 1.0, violation ==
/// 0.0 which is <= `EPS_FEAS`).
#[test]
fn at_limit_velocity_is_feasible() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    // v = 500.0 mm/s  →  b = v² = 250_000
    let result = SolverResult {
        b: vec![250_000.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    // worst_violation should be ~0.0 (right at limit), not positive.
    assert!(
        report.feasible,
        "at-limit profile must be feasible; worst_violation = {}",
        report.worst_violation
    );
    assert!(
        report.worst_violation.abs() < 1e-9,
        "expected worst_violation ≈ 0.0, got {}",
        report.worst_violation
    );
}

/// A profile with a non-zero acceleration on a straight segment should
/// bind on `AxisAccel`, not Velocity (when velocity is well below `v_max`).
#[test]
fn over_accel_profile_flagged_as_accel() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    // v = 100 mm/s (well below limit), but a = 10_000 mm/s² > a_max = 5_000.
    let result = SolverResult {
        b: vec![10_000.0; 5], // v = 100 mm/s
        a: vec![10_000.0; 5], // s̈ = 10_000 mm/s² (2× a_max)
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(!report.feasible, "over-accel profile should be infeasible");
    // The binding constraint at interior points must be AxisAccel{X} since
    // the tangent is purely in X on a straight grid.
    let interior = &report.binding_per_grid[1];
    assert!(
        matches!(interior, BindingConstraint::AxisAccel { axis: Axis::X }),
        "expected AxisAccel{{X}} at interior point, got {interior:?}",
    );
}

/// Boundary points with b ≈ 0 should always be tagged Boundary,
/// regardless of what interior ratios are.
#[test]
fn boundary_points_tagged_correctly() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    // Endpoints pinned to zero; interior at moderate velocity.
    let result = SolverResult {
        b: vec![0.0, 50_000.0, 100_000.0, 50_000.0, 0.0],
        a: vec![0.0, 1_000.0, 0.0, -1_000.0, 0.0],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(
        matches!(report.binding_per_grid[0], BindingConstraint::Boundary),
        "start should be Boundary, got {:?}",
        report.binding_per_grid[0]
    );
    assert!(
        matches!(report.binding_per_grid[4], BindingConstraint::Boundary),
        "end should be Boundary, got {:?}",
        report.binding_per_grid[4]
    );
}

/// Jerk violation at ratio 1.03 (within EPS_FEAS_JERK = 5%) must be feasible.
#[test]
fn jerk_ratio_1_03_is_feasible() {
    let n = 5;
    let length = 1.0_f64;
    let h = length / (n - 1) as f64;
    // Straight segment: jerk = s‴ = v·delta/h² for b = [v², v²+δ, v², v²+δ, v²],
    // so delta = ratio·j_max·h²/v hits the target ratio exactly.
    let v = 500.0_f64;
    let j_max = 100_000.0_f64;
    let target_ratio = 1.03_f64;
    let delta = target_ratio * j_max * h * h / v;
    let b_v2 = v * v;

    let grid = dummy_straight_grid(n, length);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let b = vec![b_v2, b_v2 + delta, b_v2, b_v2 + delta, b_v2];
    let result = SolverResult {
        b,
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(
        report.feasible,
        "jerk ratio 1.03 must be within EPS_FEAS_JERK (5%) and thus feasible; \
         worst_violation = {:.4}",
        report.worst_violation,
    );
}

/// Jerk violation at ratio 1.06 (> EPS_FEAS_JERK = 5%) must be infeasible.
#[test]
fn jerk_ratio_1_06_is_infeasible() {
    let n = 5;
    let length = 1.0_f64;
    let h = length / (n - 1) as f64;
    let v = 500.0_f64;
    let j_max = 100_000.0_f64;
    let target_ratio = 1.06_f64;
    let delta = target_ratio * j_max * h * h / v;
    let b_v2 = v * v;

    let grid = dummy_straight_grid(n, length);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let b = vec![b_v2, b_v2 + delta, b_v2, b_v2 + delta, b_v2];
    let result = SolverResult {
        b,
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(
        !report.feasible,
        "jerk ratio 1.06 must exceed EPS_FEAS_JERK (5%) and thus be infeasible; \
         worst_violation = {:.4}",
        report.worst_violation,
    );
}

/// Accel violation at ratio 1.03 must stay infeasible (tight band held).
#[test]
fn accel_ratio_1_03_is_infeasible() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let result = SolverResult {
        b: vec![10_000.0; 5], // v = 100 mm/s
        a: vec![5_150.0; 5],  // 3% over a_max
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(
        !report.feasible,
        "accel ratio 1.03 must exceed the tight EPS_FEAS (0.2%) band and be \
         infeasible; worst_violation = {:.4}",
        report.worst_violation,
    );
}

/// Centripetal constraint violation is detected.
///
/// Build a grid with non-zero curvature and inject `b_i` large enough to
/// violate `b·κ ≤ a_centripetal_max`.
#[test]
fn over_centripetal_profile_flagged() {
    let n = 5;
    let length = 10.0_f64;
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let u = s.clone();
    let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
    // Inject κ = 1.0 mm⁻¹ at every point.
    let kappa = vec![1.0; n];

    let grid = ArclengthGrid {
        s,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        kappa,
        total_length: length,
    };

    let limits = textbook_limits(); // a_centripetal_max = 2_500
    let chain = chain_of_one(grid, limits);
    // b = 5_000 → centripetal accel = b·κ = 5_000 > 2_500.
    let result = SolverResult {
        b: vec![5_000.0; n],
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let report = check_chain(&chain, &result);
    assert!(
        !report.feasible,
        "over-centripetal profile should be infeasible"
    );
    // At least one interior point should be tagged Centripetal.
    let has_centripetal = report
        .binding_per_grid
        .iter()
        .any(|b| matches!(b, BindingConstraint::Centripetal));
    assert!(
        has_centripetal,
        "expected at least one Centripetal tag, got {:?}",
        report.binding_per_grid
    );
}

use crate::topp::chain::tests_support::two_segment_chain_with_junction;

#[test]
fn junction_dual_limits_are_verified() {
    // Smooth b: baseline v≈148 (under the right 150 cap) with a gentle bump to
    // v≈173 at the junction. The junction point carries the LEFT (300) limit in
    // the primary scan — fine — so its right (150) cap is enforced ONLY by the
    // dual pass. The bump is gentle enough that the primary stencil path-jerk
    // ratio stays <1, so the dual velocity violation is the SOLE infeasibility.
    let chain = two_segment_chain_with_junction();
    let n = chain.n_points();
    let mut b = vec![22_000.0; n];
    b[10] = 30_000.0;
    let a = vec![0.0; n];
    let result = SolverResult { b, a, status: SolverStatus::Solved };
    let report = check_chain(&chain, &result);
    assert!(!report.feasible, "right-side junction velocity cap not enforced");
    assert_eq!(
        report.worst_violation_grid, 10,
        "the sole violation must be the junction's right-side velocity cap"
    );
}

/// An in-band jerk ratio (1.04) that wins every grid point must not mask a
/// co-located out-of-band accel violation (1.01 > 0.2%).
#[test]
fn jerk_riding_does_not_mask_accel_violation() {
    let n = 5;
    let length = 1.0_f64;
    let h = length / (n - 1) as f64; // 0.25 mm

    let v = 500.0_f64; // ride v_max so velocity ratio == 1.0
    let j_max = 100_000.0_f64;
    let a_max = 5_000.0_f64;

    // Jerk ratio ≈ 1.04 (within EPS_FEAS_JERK = 5%) — jerk is per-point worst.
    let jerk_ratio = 1.04_f64;
    let delta = jerk_ratio * j_max * h * h / v;
    let b_v2 = v * v;
    let b = vec![b_v2, b_v2 + delta, b_v2, b_v2 + delta, b_v2];

    // Accel ratio = 1.01 > EPS_FEAS (0.2%).  On a straight grid accel_x = s̈,
    // so set a_i = 1.01 * a_max.
    let a_val = 1.01 * a_max;
    let a = vec![a_val; n];

    let grid = dummy_straight_grid(n, length);
    let limits = textbook_limits();
    let chain = chain_of_one(grid, limits);
    let result = SolverResult {
        b,
        a,
        status: SolverStatus::Solved,
    };

    let report = check_chain(&chain, &result);
    assert!(
        !report.feasible,
        "accel ratio 1.01 must be caught even when jerk (ratio ~1.04) is the \
         per-point worst; worst_violation = {:.4}",
        report.worst_violation,
    );
}
