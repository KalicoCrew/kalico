//! Post-solve feasibility check.
//!
//! Spec §6.2. `ε_feas` = 1e-3 (0.1%). Records the binding constraint per grid
//! point for downstream tagging.
//!
//! # Algorithm overview
//!
//! For each grid point i the check reconstructs the time-domain derivatives
//! of the Cartesian path:
//!
//! ```text
//! dx/dt[axis]      = c_prime[i][axis]        · ṡ
//! d²x/dt²[axis]   = c_double_prime[i][axis]  · ṡ²  +  c_prime[i][axis]  · s̈
//! d³x/dt³[axis]   = c_triple_prime[i][axis]  · ṡ³  +  3·c_double_prime[i][axis]·ṡ·s̈  +  c_prime[i][axis]·s⃛
//! ```
//!
//! where `ṡ = v_i = √(b_i.max(0))`, `s̈ = a_i`, and
//! `s⃛ = da/ds · ṡ` (path-jerk in time from the a-profile via chain rule).
//! `da/ds` is estimated by central finite-difference on `a` vs `s`.
//!
//! Each derivative is normalised by the corresponding per-axis limit.  The
//! maximum normalised ratio across all axes and constraint types gives the
//! per-grid-point violation.  The `BindingConstraint` tag is the constraint
//! that produced that maximum.
//!
//! # Tie-breaking / determinism
//!
//! When two ratios are equal at the same grid point, precedence is:
//! Velocity > `AxisAccel` > `AxisJerk` > Centripetal.
//! Within each type, X > Y > Z.
//! This is purely a deterministic label choice; feasibility depends only on
//! the worst normalised ratio.

#![allow(dead_code)] // wired in Task 8 via schedule_segment

use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;
use crate::{Axis, BindingConstraint, Limits};

/// 0.1% feasibility margin per spec §6.2.
pub(crate) const EPS_FEAS: f64 = 1e-3;

/// Threshold below which a normalised ratio is treated as "not binding" at
/// all (fully slack).  Kept very small so we only emit `None` when every
/// constraint is negligibly loaded.
const SLACK_THRESHOLD: f64 = 1e-6;

/// `b_i` magnitude below which we consider the endpoint "pinned to zero" and tag
/// `BindingConstraint::Boundary` regardless of which ratio would otherwise win.
const BOUNDARY_B_TOL: f64 = 1e-9;

#[derive(Debug, Clone)]
pub(crate) struct VerifyReport {
    pub binding_per_grid: Vec<BindingConstraint>,
    /// `max_i(worst_ratio_at_i) − 1.0`.  Positive means infeasible; negative
    /// means every constraint is slack; 0.0 means right at the limit.
    pub worst_violation: f64,
    /// Index into the grid where the worst violation occurred.
    pub worst_violation_grid: usize,
    /// True iff every constraint at every grid point is within `ε_feas`, i.e.
    /// `worst_violation ≤ EPS_FEAS`.
    pub feasible: bool,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute `da/ds` at grid index `i` using finite differences.
fn da_ds_at(result: &SolverResult, s: &[f64], i: usize) -> f64 {
    let n = s.len();
    if n <= 1 {
        return 0.0;
    }
    let a = &result.a;
    if i == 0 {
        let ds = s[1] - s[0];
        if ds.abs() > 1e-15 { (a[1] - a[0]) / ds } else { 0.0 }
    } else if i == n - 1 {
        let ds = s[n - 1] - s[n - 2];
        if ds.abs() > 1e-15 { (a[n - 1] - a[n - 2]) / ds } else { 0.0 }
    } else {
        let ds = s[i + 1] - s[i - 1];
        if ds.abs() > 1e-15 { (a[i + 1] - a[i - 1]) / ds } else { 0.0 }
    }
}

/// All quantities needed to evaluate one grid point's constraint ratios.
struct PointInputs<'a> {
    cp: [f64; 3],
    cpp: [f64; 3],
    cppp: [f64; 3],
    kappa: f64,
    b_i: f64,
    s_dot: f64,
    s_ddot: f64,
    s_dddot: f64,
    limits: &'a Limits,
}

/// Worst normalised ratio and its `BindingConstraint` tag at a single grid
/// point.
///
/// Tie-breaking: Velocity > `AxisAccel` > `AxisJerk` > Centripetal; within
/// each type X > Y > Z.  First maximum wins (`>` not `>=`).
fn worst_ratio_at(p: &PointInputs<'_>) -> (f64, BindingConstraint) {
    let s_dot2 = p.s_dot * p.s_dot;
    let s_dot3 = s_dot2 * p.s_dot;

    // Time-domain Cartesian derivatives.
    let vel = [p.cp[0] * p.s_dot, p.cp[1] * p.s_dot, p.cp[2] * p.s_dot];
    let accel = [
        p.cpp[0] * s_dot2 + p.cp[0] * p.s_ddot,
        p.cpp[1] * s_dot2 + p.cp[1] * p.s_ddot,
        p.cpp[2] * s_dot2 + p.cp[2] * p.s_ddot,
    ];
    let jerk = [
        p.cppp[0] * s_dot3 + 3.0 * p.cpp[0] * p.s_dot * p.s_ddot + p.cp[0] * p.s_dddot,
        p.cppp[1] * s_dot3 + 3.0 * p.cpp[1] * p.s_dot * p.s_ddot + p.cp[1] * p.s_dddot,
        p.cppp[2] * s_dot3 + 3.0 * p.cpp[2] * p.s_dot * p.s_ddot + p.cp[2] * p.s_dddot,
    ];

    // Build (ratio, tag) pairs in precedence order.
    let lim = p.limits;
    let entries: [(f64, BindingConstraint); 10] = [
        (vel[0].abs() / lim.v_max[0],   BindingConstraint::Velocity  { axis: Axis::X }),
        (vel[1].abs() / lim.v_max[1],   BindingConstraint::Velocity  { axis: Axis::Y }),
        (vel[2].abs() / lim.v_max[2],   BindingConstraint::Velocity  { axis: Axis::Z }),
        (accel[0].abs() / lim.a_max[0], BindingConstraint::AxisAccel { axis: Axis::X }),
        (accel[1].abs() / lim.a_max[1], BindingConstraint::AxisAccel { axis: Axis::Y }),
        (accel[2].abs() / lim.a_max[2], BindingConstraint::AxisAccel { axis: Axis::Z }),
        (jerk[0].abs() / lim.j_max[0],  BindingConstraint::AxisJerk  { axis: Axis::X }),
        (jerk[1].abs() / lim.j_max[1],  BindingConstraint::AxisJerk  { axis: Axis::Y }),
        (jerk[2].abs() / lim.j_max[2],  BindingConstraint::AxisJerk  { axis: Axis::Z }),
        (p.b_i.max(0.0) * p.kappa / lim.a_centripetal_max, BindingConstraint::Centripetal),
    ];

    let mut worst_ratio = 0.0_f64;
    let mut worst_tag = BindingConstraint::None;
    for (ratio, tag) in entries {
        // Strict `>`: first maximum wins → respects precedence order.
        if ratio > worst_ratio {
            worst_ratio = ratio;
            worst_tag = tag;
        }
    }

    if worst_ratio < SLACK_THRESHOLD {
        (worst_ratio, BindingConstraint::None)
    } else {
        (worst_ratio, worst_tag)
    }
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

pub(crate) fn check(
    grid: &ArclengthGrid,
    result: &SolverResult,
    limits: &Limits,
) -> VerifyReport {
    let n = grid.s.len();
    debug_assert_eq!(result.b.len(), n);
    debug_assert_eq!(result.a.len(), n);

    let mut binding_per_grid: Vec<BindingConstraint> = Vec::with_capacity(n);
    let mut global_worst_ratio: f64 = f64::NEG_INFINITY;
    let mut global_worst_idx: usize = 0;

    for i in 0..n {
        let b_i = result.b[i];
        let a_i = result.a[i];

        // Path-domain quantities.
        let s_dot = b_i.max(0.0).sqrt(); // ṡ; guard tiny-negative b
        let s_ddot = a_i;
        let s_dddot = da_ds_at(result, &grid.s, i) * s_dot; // chain rule

        let (worst_ratio, tag) = worst_ratio_at(&PointInputs {
            cp: grid.c_prime[i],
            cpp: grid.c_double_prime[i],
            cppp: grid.c_triple_prime[i],
            kappa: grid.kappa[i],
            b_i,
            s_dot,
            s_ddot,
            s_dddot,
            limits,
        });

        // Boundary override: endpoints pinned to zero are always tagged
        // Boundary regardless of computed ratios.
        let final_tag = if (i == 0 || i == n - 1) && b_i.abs() < BOUNDARY_B_TOL {
            BindingConstraint::Boundary
        } else {
            tag
        };

        // Track global worst using the raw physics ratio (not the tag).
        if worst_ratio > global_worst_ratio {
            global_worst_ratio = worst_ratio;
            global_worst_idx = i;
        }

        binding_per_grid.push(final_tag);
    }

    // Edge-case: empty grid (shouldn't happen in practice).
    if n == 0 {
        return VerifyReport {
            binding_per_grid,
            worst_violation: f64::NEG_INFINITY,
            worst_violation_grid: 0,
            feasible: true,
        };
    }

    let worst_violation = global_worst_ratio - 1.0;
    VerifyReport {
        binding_per_grid,
        worst_violation,
        worst_violation_grid: global_worst_idx,
        feasible: worst_violation <= EPS_FEAS,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topp::solver::{SolverResult, SolverStatus};
    use crate::Limits;

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
        let report = check(&grid, &result, &limits);
        assert!(report.feasible);
        assert!(report.worst_violation < EPS_FEAS);
        assert!(report.binding_per_grid.iter().all(|b| matches!(
            b,
            BindingConstraint::Boundary | BindingConstraint::None
        )));
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
        let report = check(&grid, &result, &limits);
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
        let report = check(&grid, &result, &limits);
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
        let report = check(&grid, &result, &limits);
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
        let report = check(&grid, &result, &limits);
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
        let report = check(&grid, &result, &limits);
        assert!(!report.feasible, "over-centripetal profile should be infeasible");
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
}
