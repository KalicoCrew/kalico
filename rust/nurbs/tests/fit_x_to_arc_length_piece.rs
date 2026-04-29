#![allow(clippy::cast_lossless)]
//
// Test corpus for `fit_x_to_arc_length_piece` per plan §Phase 2 / Task 2.3.

use nurbs::algebra::{FitError, fit_x_to_arc_length_piece};
use nurbs::{VectorNurbs, arc_length::build_arc_length_table_vector};

fn cubic_straight_line() -> VectorNurbs<f64, 3> {
    // Line from (0,0,0) to (10,0,0).
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0 / 3.0, 0.0, 0.0],
            [20.0 / 3.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap()
}

#[test]
fn straight_line_fits_at_low_degree() {
    let xyz = cubic_straight_line();
    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();
    // Fit a 0.5 mm piece in the middle of the line.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        4.0,
        4.5,
        /*target_degree=*/ 3,
        /*max_degree=*/ 10,
        /*tolerance_mm=*/ 1e-3,
    );
    assert!(result.is_ok(), "expected Ok, got {result:?}");
    let pieces = result.unwrap();
    // Verify each piece's u_start and u_end match the s-domain.
    for axis in 0..3 {
        assert!((pieces[axis].u_start - 4.0).abs() < 1e-9);
        assert!((pieces[axis].u_end - 4.5).abs() < 1e-9);
    }
}

#[test]
fn quarter_arc_fits_at_low_degree() {
    // Cubic Bézier approximation of a quarter circle, R = 10.
    let r = 10.0;
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 8.0).tan();
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, 0.0, 0.0],
            [r, r * k, 0.0],
            [r * k, r, 0.0],
            [0.0, r, 0.0],
        ],
        None,
    )
    .unwrap();

    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();
    let s_max = table.s_max();

    // Fit a 0.5 mm piece in the middle of the quarter-arc.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        s_max * 0.4,
        s_max * 0.4 + 0.5,
        /*target_degree=*/ 3,
        /*max_degree=*/ 10,
        /*tolerance_mm=*/ 1e-3,
    );
    assert!(result.is_ok(), "quarter arc fit failed: {result:?}");
}

/// Round-1-review fix: residual verification at points NOT used during the fit.
#[test]
fn tight_arc_r1mm_residual_within_tolerance() {
    use nurbs::arc_length::param_from_arc_length;
    use nurbs::eval::vector_eval;

    // R = 1 mm quarter arc.
    let r = 1.0;
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 8.0).tan();
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, 0.0, 0.0],
            [r, r * k, 0.0],
            [r * k, r, 0.0],
            [0.0, r, 0.0],
        ],
        None,
    )
    .unwrap();

    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();
    let s_max = table.s_max();

    let s_lo = s_max * 0.3;
    let s_hi = s_max * 0.3 + 0.5;
    let tolerance_mm = 1e-3;

    let pieces = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        s_lo,
        s_hi,
        /*target_degree=*/ 4,
        /*max_degree=*/ 10,
        tolerance_mm,
    )
    .expect("R=1mm fit must converge with adaptive degree");

    // Verify residual at 100 points (not the 4·(d+1) the primitive itself uses).
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let s = s_lo + (s_hi - s_lo) * t;
        let u = param_from_arc_length(&table_ref, s);
        let truth = vector_eval(&xyz, u);
        for axis in 0..3 {
            let p_val = pieces[axis].evaluate(s);
            let err = (truth[axis] - p_val).abs();
            assert!(
                err <= tolerance_mm * 1.5,
                "axis {axis} residual at s={s} was {err}, tolerance {tolerance_mm}"
            );
        }
    }
}

/// Round-1-review fix: endpoint integrity. Chebyshev-of-2nd-kind nodes include endpoints.
#[test]
fn endpoint_integrity() {
    use nurbs::arc_length::param_from_arc_length;
    use nurbs::eval::vector_eval;

    let xyz = cubic_straight_line();
    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();

    let s_lo = 1.0;
    let s_hi = 4.0;
    let pieces = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        s_lo,
        s_hi,
        /*target_degree=*/ 4,
        /*max_degree=*/ 10,
        /*tolerance_mm=*/ 1e-9,
    )
    .unwrap();

    let u_lo = param_from_arc_length(&table_ref, s_lo);
    let u_hi = param_from_arc_length(&table_ref, s_hi);
    let truth_lo = vector_eval(&xyz, u_lo);
    let truth_hi = vector_eval(&xyz, u_hi);

    for axis in 0..3 {
        let p_lo = pieces[axis].evaluate(s_lo);
        let p_hi = pieces[axis].evaluate(s_hi);
        assert!(
            (p_lo - truth_lo[axis]).abs() < 1e-9,
            "endpoint s_lo axis {axis} mismatch: p={p_lo} truth={}",
            truth_lo[axis]
        );
        assert!(
            (p_hi - truth_hi[axis]).abs() < 1e-9,
            "endpoint s_hi axis {axis} mismatch: p={p_hi} truth={}",
            truth_hi[axis]
        );
    }
}

/// Round-1-review fix: degenerate-input rejection.
#[test]
fn degenerate_input_returns_err() {
    let xyz = cubic_straight_line();
    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();

    // s_hi <= s_lo: degenerate range.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        5.0,
        4.0,
        /*target_degree=*/ 4,
        /*max_degree=*/ 10,
        /*tolerance_mm=*/ 1e-3,
    );
    assert!(matches!(result, Err(FitError::DegenerateInput { .. })));

    // NaN s_lo.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz,
        &table_ref,
        f64::NAN,
        4.0,
        /*target_degree=*/ 4,
        /*max_degree=*/ 10,
        /*tolerance_mm=*/ 1e-3,
    );
    assert!(matches!(result, Err(FitError::DegenerateInput { .. })));
}
