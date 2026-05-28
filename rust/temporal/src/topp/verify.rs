//! Per-axis Cartesian jerk + per-axis acceleration / velocity / centripetal
//! verifier for solver outputs. Spec §6.2.
//!
//! Computes the binding-constraint tag at every grid point and the worst-
//! case ratio across all binding constraints. Used by the public solver
//! entry point to convert `SolverStatus::Solved` into the public
//! `SolveStatus::Solved` only when the post-solve trajectory is feasible
//! (handles Consolini-Locatelli relaxation gaps where Clarabel reports
//! success but the relaxation didn't fully bind on a non-convex constraint).
//!
//! # Stencil
//!
//! Path-third-derivative `s‴` is computed via the shared
//! `topp::stencil::s_dddot_at` helper (width-1 b-FD: forward at i=0,
//! central at i ∈ [1, n-2], backward at i=n-1). Same stencil as the
//! path-jerk SOC chain in `constraints::block_(h)` and the per-axis SLP
//! cut linearization in `solver::append_axis_jerk_cut_to_clarabel` —
//! single source of truth across SOCP/SLP/verifier per
//! `docs/superpowers/specs/2026-05-05-stencil-unification-design.md`.
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
//! where `ṡ = v_i = √(b_i.max(0))`, `s̈ = a_i`, and `s⃛` is sourced from
//! `topp::stencil::s_dddot_at(&result.b, i, h)`.
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

use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;
use crate::{Axis, BindingConstraint, Limits};

/// 0.2% feasibility margin per spec §6.2. Uniform width-1 b-FD across
/// SOCP/SLP/verifier per
/// `docs/superpowers/specs/2026-05-05-stencil-unification-design.md`.
pub(crate) const EPS_FEAS: f64 = 2e-3;

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
    #[allow(dead_code)]
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
        (
            vel[0].abs() / lim.v_max[0],
            BindingConstraint::Velocity { axis: Axis::X },
        ),
        (
            vel[1].abs() / lim.v_max[1],
            BindingConstraint::Velocity { axis: Axis::Y },
        ),
        (
            vel[2].abs() / lim.v_max[2],
            BindingConstraint::Velocity { axis: Axis::Z },
        ),
        (
            accel[0].abs() / lim.a_max[0],
            BindingConstraint::AxisAccel { axis: Axis::X },
        ),
        (
            accel[1].abs() / lim.a_max[1],
            BindingConstraint::AxisAccel { axis: Axis::Y },
        ),
        (
            accel[2].abs() / lim.a_max[2],
            BindingConstraint::AxisAccel { axis: Axis::Z },
        ),
        (
            jerk[0].abs() / lim.j_max[0],
            BindingConstraint::AxisJerk { axis: Axis::X },
        ),
        (
            jerk[1].abs() / lim.j_max[1],
            BindingConstraint::AxisJerk { axis: Axis::Y },
        ),
        (
            jerk[2].abs() / lim.j_max[2],
            BindingConstraint::AxisJerk { axis: Axis::Z },
        ),
        (
            p.b_i.max(0.0) * p.kappa / lim.a_centripetal_max,
            BindingConstraint::Centripetal,
        ),
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
    h: f64,
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
        // Width-1 b-FD via shared stencil helper (already includes √b factor).
        let s_dddot = crate::topp::stencil::s_dddot_at(&result.b, i, h);

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
mod tests;
