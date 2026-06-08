use nurbs::{VectorNurbs, arc_length::xy_arc_length};

#[test]
fn pure_xy_straight_line_collinear_cubic() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();

    let l = xy_arc_length(&xyz);
    assert!((l - 3.0).abs() < 1e-9, "expected ~3.0, got {l}");
}

#[test]
fn pure_z_motion_xy_length_zero() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 3.0],
        ],
    )
    .unwrap();

    let l = xy_arc_length(&xyz);
    assert!(l.abs() < 1e-9, "expected ~0.0, got {l}");
}

#[test]
fn diagonal_xy_straight_line() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [2.0, 2.0, 0.0],
            [3.0, 3.0, 0.0],
        ],
    )
    .unwrap();

    let l = xy_arc_length(&xyz);
    let expected = 3.0 * std::f64::consts::SQRT_2;
    assert!((l - expected).abs() < 1e-9, "expected ~{expected}, got {l}");
}

#[test]
fn pure_xy_curve_matches_3d_length() {
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 8.0).tan();
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [1.0, 0.0, 0.0],
            [1.0, k, 0.0],
            [k, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
    )
    .unwrap();

    let xy_l = xy_arc_length(&xyz);
    let table_3d = nurbs::arc_length::build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let l_3d = table_3d.s_max();

    assert!(
        (xy_l - l_3d).abs() < 1e-9,
        "pure-XY: xy_arc_length should match 3D arc length, got xy={xy_l} vs 3d={l_3d}"
    );
}

#[test]
fn xy_loop_chord_zero_arc_length_nonzero() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    )
    .unwrap();

    let l = xy_arc_length(&xyz);
    assert!(l > 0.5, "loop should have nonzero XY arc length, got {l}");
}
