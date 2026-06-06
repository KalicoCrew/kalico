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

/// Jerk violation at ratio 1.03 (within EPS_FEAS_JERK = 5%) must be feasible.
///
/// On a straight segment c' = [1,0,0], c'' = c''' = 0, so jerk = c' · s‴.
/// With b = [b0, b1, b2, b3, b4] (uniform), s‴ = √b · (b'' / 2).
/// Choosing b_uniform and h so that jerk/j_max = 1.03 exercises the jerk band.
#[test]
fn jerk_ratio_1_03_is_feasible() {
    let n = 5;
    let length = 1.0_f64;
    let h = length / (n - 1) as f64; // 0.25 mm
    // j_max[0] = 100_000 mm/s³.
    // We want |c'·s‴| / j_max = 1.03, so s‴ = 103_000 mm/s³.
    // s‴ = √b · (b''/2) where b'' = (b_{i-1} - 2b_i + b_{i+1}) / h².
    // Pick b constant = v²: s‴ = √b · (0 / h²) / 2 = 0 — that's zero.
    // Instead craft a non-flat b so the FD second-difference is nonzero.
    // b = [v², v², v², v², v²] ← still zero FD. Need asymmetric values.
    // Interior FD at i=2: (b[1] - 2*b[2] + b[3]) / h².
    // Set b[2] = v², b[1] = v² + delta, b[3] = v² + delta.
    // b'' = 2*delta / h² at i=2.
    // s‴ = √v² * (2*delta/h²) / 2 = v * delta / h².
    // For ratio = 1.03: v * delta / h² = 1.03 * j_max
    //   → delta = 1.03 * j_max * h² / v.
    // Use v = 500 mm/s (v_max), h = 0.25 mm, j_max = 100_000.
    let v = 500.0_f64;
    let j_max = 100_000.0_f64;
    let target_ratio = 1.03_f64;
    let delta = target_ratio * j_max * h * h / v;
    let b_v2 = v * v;

    let grid = dummy_straight_grid(n, length);
    let limits = textbook_limits(); // j_max = 100_000
    let b = vec![b_v2, b_v2 + delta, b_v2, b_v2 + delta, b_v2];
    let result = SolverResult {
        b,
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let report = check(&grid, &result, &limits, h);
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
    let b = vec![b_v2, b_v2 + delta, b_v2, b_v2 + delta, b_v2];
    let result = SolverResult {
        b,
        a: vec![0.0; n],
        status: SolverStatus::Solved,
    };
    let report = check(&grid, &result, &limits, h);
    assert!(
        !report.feasible,
        "jerk ratio 1.06 must exceed EPS_FEAS_JERK (5%) and thus be infeasible; \
         worst_violation = {:.4}",
        report.worst_violation,
    );
}

/// Acceleration violation at ratio 1.03 must be infeasible (tight band held).
///
/// Non-jerk constraints use EPS_FEAS = 0.2%, so a 3% accel overshoot fails.
#[test]
fn accel_ratio_1_03_is_infeasible() {
    let grid = dummy_straight_grid(5, 100.0);
    let limits = textbook_limits(); // a_max = 5_000
    // a_i = 1.03 * a_max = 5_150 mm/s²; b modest so velocity is within limits.
    let result = SolverResult {
        b: vec![10_000.0; 5], // v = 100 mm/s
        a: vec![5_150.0; 5],  // 3% over a_max
        status: SolverStatus::Solved,
    };
    let h = 100.0 / 4.0;
    let report = check(&grid, &result, &limits, h);
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

/// Regression: jerk riding the limit does NOT mask a co-located accel violation.
///
/// The defect this catches: before the fix, `check()` routed only the per-point
/// worst `(ratio, tag)` into the per-class trackers.  On a jerk-riding profile
/// jerk is the per-point winner everywhere, so the accel ratio never reached
/// `worst_non_jerk_ratio` — a 1.01 accel overshoot silently passed.
///
/// Setup: straight X-only grid (c' = [1,0,0], c'' = c''' = 0).
///   - b is non-flat so the FD stencil produces a jerk ratio of ~1.04 (within
///     `EPS_FEAS_JERK = 5%`), making jerk the per-point winner.
///   - a_i is set so `|c'·s̈| / a_max = 1.01`, which exceeds `EPS_FEAS = 0.2%`.
///
/// Expected: `feasible == false` because the accel class violates its band,
/// even though jerk is the dominant (and technically in-band) constraint.
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
    let result = SolverResult {
        b,
        a,
        status: SolverStatus::Solved,
    };

    let report = check(&grid, &result, &limits, h);
    assert!(
        !report.feasible,
        "accel ratio 1.01 must be caught even when jerk (ratio ~1.04) is the \
         per-point worst; worst_violation = {:.4}",
        report.worst_violation,
    );
}
