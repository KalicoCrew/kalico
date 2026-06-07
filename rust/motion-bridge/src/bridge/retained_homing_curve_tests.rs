use super::{
    PIECE_BOUNDARY_TOLERANCE, RetainedHomingCurve, RetainedHomingPiece, eval_retained_curve,
};

fn linear_axis(p0: f64, p1: f64, t_start: f64, t_end: f64) -> nurbs::ScalarNurbs<f64> {
    let d = p1 - p0;
    let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
    let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, t_start, t_end);
    nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
}

fn bernstein_eval(p0: f64, p1: f64, p2: f64, p3: f64, u: f64) -> f64 {
    let v = 1.0 - u;
    v * v * v * p0 + 3.0 * v * v * u * p1 + 3.0 * v * u * u * p2 + u * u * u * p3
}

fn make_two_piece_curve() -> RetainedHomingCurve {
    let t0 = 1000.0_f64;
    RetainedHomingCurve {
        pieces: vec![
            RetainedHomingPiece {
                axes: [
                    linear_axis(0.0, 10.0, 0.0, 0.5),
                    linear_axis(0.0, 0.0, 0.0, 0.5),
                    linear_axis(0.0, 0.0, 0.0, 0.5),
                ],
                t_abs_start: t0 + 0.0,
                t_abs_end: t0 + 0.5,
                t0,
            },
            RetainedHomingPiece {
                axes: [
                    linear_axis(10.0, 20.0, 0.5, 1.0),
                    linear_axis(0.0, 5.0, 0.5, 1.0),
                    linear_axis(0.0, 0.0, 0.5, 1.0),
                ],
                t_abs_start: t0 + 0.5,
                t_abs_end: t0 + 1.0,
                t0,
            },
        ],
    }
}

#[test]
fn mid_piece_eval_matches_bernstein() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_abs = t0 + 0.25;
    let pos = eval_retained_curve(&curve, t_abs, 1, 0).unwrap();
    assert_eq!(pos.len(), 3);
    let u = 0.25 / 0.5;
    let expected_x = bernstein_eval(0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0, u);
    assert!(
        (pos[0] - expected_x).abs() < 1e-9,
        "x mismatch: got={} expected={}",
        pos[0],
        expected_x
    );
    assert!((pos[1]).abs() < 1e-9, "y should be 0 for first piece");
}

#[test]
fn boundary_at_piece_junction_evaluates_without_error() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_junction = t0 + 0.5;
    let pos = eval_retained_curve(&curve, t_junction, 1, 0).unwrap();
    assert_eq!(pos.len(), 3);
    assert!(
        (pos[0] - 10.0).abs() < 1e-9,
        "x at junction should be 10.0, got={}",
        pos[0]
    );
}

#[test]
fn t_before_first_piece_errors() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_early = t0 - 0.1 - PIECE_BOUNDARY_TOLERANCE;
    let result = eval_retained_curve(&curve, t_early, 1, 12345);
    assert!(result.is_err(), "expected Err for t before first piece");
    let msg = result.unwrap_err().to_string();
    assert!(
        msg.contains("before the first retained piece"),
        "error should mention 'before the first retained piece'; got: {msg}"
    );
    assert!(
        msg.contains("12345"),
        "error should contain trip_clock; got: {msg}"
    );
}

#[test]
fn t_after_last_piece_by_more_than_one_piece_duration_errors() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_late = t0 + 1.0 + 0.51;
    let result = eval_retained_curve(&curve, t_late, 1, 99999);
    assert!(result.is_err(), "expected Err for t > last piece by >1 piece duration");
    let msg = result.unwrap_err().to_string();
    assert!(
        msg.contains("overshoots last"),
        "error should mention 'overshoots last'; got: {msg}"
    );
}

#[test]
fn t_after_last_piece_within_one_piece_duration_clamps_to_last() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_slightly_after = t0 + 1.0 + 0.1;
    let pos = eval_retained_curve(&curve, t_slightly_after, 1, 0).unwrap();
    assert_eq!(pos.len(), 3);
    assert!(
        (pos[0] - 20.0).abs() < 1e-9,
        "x should clamp to end of last piece (20.0); got={}",
        pos[0]
    );
}

#[test]
fn second_piece_mid_point_evaluates_correctly() {
    let curve = make_two_piece_curve();
    let t0 = 1000.0_f64;
    let t_abs = t0 + 0.75;
    let pos = eval_retained_curve(&curve, t_abs, 1, 0).unwrap();
    assert_eq!(pos.len(), 3);
    let u = (0.75 - 0.5) / 0.5;
    let expected_x = bernstein_eval(10.0, 10.0 + 10.0 / 3.0, 10.0 + 20.0 / 3.0, 20.0, u);
    let expected_y = bernstein_eval(0.0, 5.0 / 3.0, 10.0 / 3.0, 5.0, u);
    assert!(
        (pos[0] - expected_x).abs() < 1e-9,
        "x mismatch at 0.75 s: got={} expected={}",
        pos[0],
        expected_x
    );
    assert!(
        (pos[1] - expected_y).abs() < 1e-9,
        "y mismatch at 0.75 s: got={} expected={}",
        pos[1],
        expected_y
    );
}

#[test]
fn empty_curve_errors() {
    let curve = RetainedHomingCurve { pieces: vec![] };
    let result = eval_retained_curve(&curve, 1000.0, 1, 0);
    assert!(result.is_err());
    let msg = result.unwrap_err().to_string();
    assert!(msg.contains("no pieces"), "got: {msg}");
}
