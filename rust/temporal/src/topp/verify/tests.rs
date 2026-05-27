use super::*;
use crate::Limits;
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

#[test]
fn zero_profile_is_feasible() {
    let grid = dummy_straight_grid(5, 10.0);
    let limits = textbook_limits();
    let result = SolverResult {
        b: vec![0.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
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
    // b = v² far above v_max² = 250_000 ⇒ infeasible on velocity.
    let result = SolverResult {
        b: vec![1_000_000.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
    assert!(!report.feasible);
}

// ---- Additional coverage tests -----------------------------------------

/// A profile right at `v_max` should be feasible (ratio == 1.0, violation ==
/// 0.0 which is <= `EPS_FEAS`).
#[test]
fn at_limit_velocity_is_feasible() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits();
    // v = 500.0 mm/s  →  b = v² = 250_000
    let result = SolverResult {
        b: vec![250_000.0; 5],
        a: vec![0.0; 5],
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
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
    // v = 100 mm/s (well below limit), but a = 10_000 mm/s² > a_max = 5_000.
    let result = SolverResult {
        b: vec![10_000.0; 5], // v = 100 mm/s
        a: vec![10_000.0; 5], // s̈ = 10_000 mm/s² (2× a_max)
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
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
    // Endpoints pinned to zero; interior at moderate velocity.
    let result = SolverResult {
        b: vec![0.0, 50_000.0, 100_000.0, 50_000.0, 0.0],
        a: vec![0.0, 1_000.0, 0.0, -1_000.0, 0.0],
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
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
    // b = 5_000 → centripetal accel = b·κ = 5_000 > 2_500.
    let result = SolverResult {
        b: vec![5_000.0; n],
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let h = grid.s[1] - grid.s[0];
    let report = check(&grid, &result, &limits, h);
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
