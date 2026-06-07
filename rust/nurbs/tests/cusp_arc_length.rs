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
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
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
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.1, 0.0],
            [2.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64)
        .expect("non-cusp cubic with modest speed minimum should succeed");
    assert!(table.s_max() > 0.5);
}

#[test]
fn out_and_back_collinear_succeeds() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [-3.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64)
        .expect("out-and-back collinear should succeed");
    assert!(table.s_max() > 0.0, "out-and-back has nonzero arc length");
}

#[test]
fn whole_curve_zero_length_still_rejected() {
    let xyz = VectorNurbs::<f64, 3>::try_new(3, cubic_clamped_knots(), vec![[0.0; 3]; 4]).unwrap();
    let result = build_arc_length_table_vector(&xyz, 1e-9, 64);
    assert!(
        matches!(result, Err(nurbs::ArcLengthError::DegenerateCurve)),
        "expected DegenerateCurve, got {result:?}"
    );
}

#[test]
fn param_from_arc_length_handles_plateau_at_cusp() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        cubic_clamped_knots(),
        vec![
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let table = build_arc_length_table_vector(&xyz, 1e-3, 64).unwrap();
    let table_ref = table.as_view();
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
