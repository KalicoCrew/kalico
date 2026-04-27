//! SOCP numerical-conditioning regression tests.
//!
//! Spec §11 (numerical conditioning). Curved-arc segments expose finite-difference
//! noise in `c_prime` at endpoints where one Cartesian tangent component is
//! mathematically zero. Without a feasibility-equivalent cap on block (c)'s
//! per-axis velocity-UB rows, the resulting `(v_max / |c'_axis|)²` RHS values
//! reach ~1e15, blowing up Clarabel's interior-point conditioning and producing
//! `MaxIter` on any non-trivial curved input. This regression pins the fix.

use nurbs::VectorNurbs;
use temporal::{
    schedule_segment, BindingConstraint, GridConfig, GridScheme, Limits, SolveStatus,
};

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
#[test]
fn rational_quadratic_arc_n200_solves_with_centripetal_cruise() {
    let curve = rational_quadratic_quarter_arc(20.0);
    let limits = textbook_limits();
    let cfg = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 200,
    };

    let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0).expect("schedule_segment");

    // (a) Status: Solved, SolvedInexact, or SolvedSlp (Lee 2024 SLP outer
    // iteration, spec §11). Not MaxIter, not Infeasible. The CL-2024
    // SOCP relaxation is empirically loose on this fixture (Conjecture 4.1
    // counterexample at grid 184, 2.43× ratio); the SLP outer loop tightens
    // it via Taylor cuts on `1/√b`.
    assert!(
        matches!(
            profile.status,
            SolveStatus::Solved
                | SolveStatus::SolvedInexact { .. }
                | SolveStatus::SolvedSlp { .. }
        ),
        "expected Solved/SolvedInexact/SolvedSlp, got {:?}",
        profile.status,
    );

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
