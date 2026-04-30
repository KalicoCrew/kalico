//! Regression tests for cubic Béziers with interior speed minima (cusps
//! and near-cusps). These previously failed via the per-midpoint speed
//! floor in `build_table_via_integrand`; the whole-curve degeneracy
//! check accepts them.
//!
//! See verifier report from Codex pass-4 review for the failing-case
//! corpus.

#![allow(clippy::cast_lossless)]

use nurbs::{
    VectorNurbs,
    arc_length::{build_arc_length_table_vector, param_from_arc_length},
};

fn cubic_clamped_knots() -> Vec<f64> {
    vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
}

#[test]
fn true_cusp_at_u_half_succeeds() {
    // P0=(0,0,0), P1=(2,1,0), P2=(0,1,0), P3=(2,0,0): true cusp at u=0.5,
    // analytic min |dr/du| = 0. Pre-fix: ArcLengthError::DegenerateCurve.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64)
        .expect("true cusp should not block table construction");
    assert!(
        table.s_max() > 0.5,
        "expected non-trivial arc length, got {}",
        table.s_max()
    );
}

#[test]
fn modest_perturbation_min_speed_3e_minus_3_succeeds() {
    // Pre-fix this also tripped DegenerateCurve via central-difference
    // noise even though analytic min |dr/du| is ~3.4e-3 — 6.5 orders of
    // magnitude above MIN_PARAMETRIC_SPEED.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.1, 0.0],
            [2.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64)
        .expect("non-cusp cubic with modest speed minimum should succeed");
    assert!(table.s_max() > 0.5);
}

#[test]
fn out_and_back_collinear_succeeds() {
    // Out-and-back collinear cubic: P0=(0,0,0), P1=(3,0,0), P2=(-3,0,0),
    // P3=(0,0,0). Has interior speed minimum where the curve reverses;
    // analytic min |dr/du| ~ 7.7e-4.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [-3.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64)
        .expect("out-and-back collinear should succeed");
    assert!(table.s_max() > 0.0, "out-and-back has nonzero arc length");
}

#[test]
fn whole_curve_zero_length_still_rejected() {
    // Genuine degeneracy: all four CPs at the origin. Total arc length = 0.
    // Approach A's whole-curve check must preserve this rejection.
    let xyz =
        VectorNurbs::<f64, 3>::try_new(3, cubic_clamped_knots(), vec![[0.0; 3]; 4], None).unwrap();
    let result = build_arc_length_table_vector(&xyz, 1e-9, 64);
    assert!(
        matches!(result, Err(nurbs::ArcLengthError::DegenerateCurve)),
        "expected DegenerateCurve, got {result:?}"
    );
}

#[test]
fn param_from_arc_length_handles_plateau_at_cusp() {
    // True cusp creates a plateau in s(u) near u=0.5. Verify lookup
    // works without divide-by-zero.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64).unwrap();
    let table_ref = table.as_view();
    // Sample at multiple s values across the (potentially-plateaued) range.
    let s_max = table.s_max();
    for i in 0..=20 {
        let s_query = s_max * (i as f64) / 20.0;
        let u = param_from_arc_length(&table_ref, s_query);
        assert!(
            u.is_finite(),
            "u must be finite for s_query={s_query}, got {u}"
        );
        assert!(
            (0.0..=1.0).contains(&u),
            "u must be in [0,1] for s_query={s_query}, got {u}"
        );
    }
}
