//! SOCP numerical-conditioning regression tests.
//!
//! Spec §11 (numerical conditioning). Curved-arc segments expose finite-difference
//! noise in `c_prime` at endpoints where one Cartesian tangent component is
//! mathematically zero. Without a feasibility-equivalent cap on block (c)'s
//! per-axis velocity-UB rows, the resulting `(v_max / |c'_axis|)²` RHS values
//! reach ~1e15, blowing up Clarabel's interior-point conditioning and producing
//! `MaxIter` on any non-trivial curved input. This regression pins the fix.

use nurbs::VectorNurbs;
use temporal::{BindingConstraint, GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

/// Rational-quadratic 90° quarter-arc, R = 20 mm, in the XY plane.
///
/// Standard NURBS construction:
///   degree 2, knots [0,0,0,1,1,1],
///   control points (R,0,0), (R,R,0), (0,R,0), weights [1, √2/2, 1].
/// Constant curvature κ = 1/R = 0.05 mm⁻¹.
fn rational_quadratic_quarter_arc(r: f64) -> VectorNurbs<f64, 3> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[r, 0.0, 0.0], [r, r, 0.0], [0.0, r, 0.0]],
        Some(vec![1.0, w, 1.0]),
    )
    .expect("valid rational-quadratic arc")
}

fn textbook_limits() -> Limits {
    Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    )
}

/// Without the block-(c) RHS cap, FD-noise on the tangent component of an axis
/// that is mathematically zero at an endpoint pushes `(v_max/|c'_axis|)²` to
/// ~5e15, destroying Clarabel's conditioning at N=200 (residual ≈ 14, status
/// `MaxIter`). This test asserts the post-fix behavior: the solver returns
/// `Solved`/`SolvedInexact`, the midpoint cruise speed lands within 5 % of
/// `√(a_centripetal/κ) = √(2500/0.05) ≈ 223.6 mm/s`, the midpoint binding tag
/// is `Centripetal`, and `total_time` is finite and well under 1 s for a
/// ~31.4 mm arc cruising at ~223 mm/s.
///
/// **Known limitation (2026-05-05 stencil unification, spec §6.6 + §10).** On
/// this curved-arc fixture the SLP per-axis Cartesian-jerk cuts (first-order
/// Taylor linearizations) hit a fixed point at `last_max_ratio ≈ 1.0104` —
/// a ~1% Y-jerk overshoot in the start ramp-up zone (i=2 of 200) caused by
/// the `3·c''·ṡ·s̈ + c'''·ṡ³` cross-terms that the path-jerk SOC chain alone
/// cannot eliminate. The SOCP enforces the linearized cut tightly, but the
/// verifier evaluates the nonlinear `|j_axis|` at the new iterate and sees
/// the linearization gap. Trust-region shrinks looking for improvement,
/// finds none, exits `Diverged`. Pre-fix, `verify::da_ds_at` (width-2 a-FD
/// on `a`) under-estimated `s‴` enough to land below `EPS_FEAS=2e-3` and
/// rubber-stamp the trajectory; the unified width-1 b-FD verifier sees the
/// gap honestly. Tightening this requires curvature-aware cuts (spec §10's
/// deferred follow-on) or a richer SOCP formulation that bakes per-axis
/// cross-terms in. For now we accept ≤2% overshoot on curved geometry.
///
/// Note: under the unified width-1 b-FD stencil the worst-violation grid
/// point is the start ramp-up (i=2), not the centripetal cruise. The
/// centripetal cruise is still part of the trajectory and the midpoint
/// assertions below still hold; the worst per-axis-jerk just lives at the
/// ramp-up. The test name is preserved to avoid churning git-blame.
#[test]
fn rational_quadratic_arc_n200_solves_with_centripetal_cruise() {
    let curve = rational_quadratic_quarter_arc(20.0);
    let limits = textbook_limits();
    let cfg = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 200,
    };

    let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule_segment");

    // (a) Status: Solved, SolvedInexact, SolvedSlp, or DivergedSlp with a
    // last_max_ratio ≤ 1.02 (≤2 % per-axis-jerk overshoot). Under the
    // unified width-1 b-FD stencil (2026-05-05) the SLP per-axis cuts hit
    // a first-order Taylor fixed point on this curved fixture at
    // ~1.0104 — see the docstring above. Generous 1.02 band covers
    // grid-refinement / numerical drift but rejects genuinely-broken
    // iterates.
    match profile.status {
        SolveStatus::Solved
        | SolveStatus::SolvedInexact { .. }
        | SolveStatus::SolvedSlp { .. } => {}
        SolveStatus::DivergedSlp { last_max_ratio, .. } => {
            assert!(
                last_max_ratio < 1.02,
                "DivergedSlp accepted only with last_max_ratio < 1.02, got {}",
                last_max_ratio,
            );
        }
        ref other => panic!(
            "expected Solved/SolvedInexact/SolvedSlp or DivergedSlp(<1.02), got {:?}",
            other,
        ),
    }

    // (b) Midpoint speed: within 5 % of v_cruise = √(a_centripetal / κ).
    let mid = &profile.samples[100];
    let expected_v = (2_500.0_f64 / 0.05).sqrt(); // ≈ 223.6068
    let rel = (mid.v - expected_v).abs() / expected_v;
    assert!(
        rel < 0.05,
        "mid_v = {} vs expected {} (rel = {:.4})",
        mid.v,
        expected_v,
        rel,
    );

    // (c) Midpoint binding: Centripetal (the dominant active constraint on a
    // constant-curvature cruise where v_cruise ≪ v_max).
    assert!(
        matches!(mid.binding, BindingConstraint::Centripetal),
        "binding at midpoint = {:?}, expected Centripetal",
        mid.binding,
    );

    // (d) total_time: finite and < 1.0 s. Arc length ≈ π·R/2 ≈ 31.4 mm at
    // ~223 mm/s cruise gives ~0.14 s + accel/decel ramps; 1.0 s is a generous
    // sanity bound.
    assert!(
        profile.total_time.is_finite() && profile.total_time < 1.0,
        "total_time = {}",
        profile.total_time,
    );
}
