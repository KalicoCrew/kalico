use super::*;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};

fn motor_curve(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    let d = (p1 - p0) / 3.0;
    let bern = [p0, p0 + d, p0 + 2.0 * d, p1];
    bezier_pieces_to_nurbs(&[BezierPiece::from_bernstein(&bern, t0, t1)])
}

#[test]
fn retain_then_eval_midpoint_and_clamp() {
    let mut ret = RetainedMotorCurve::default();
    ret.push_piece(0, 0, motor_curve(0.0, 10.0, 100.0, 102.0), 100.0, 102.0);
    assert!((ret.eval(0, 0, 101.0).unwrap() - 5.0).abs() < 1e-9); // uniform cubic midpoint
    assert!((ret.eval(0, 0, 99.0).unwrap() - 0.0).abs() < 1e-9); // clamp before start
    assert!((ret.eval(0, 0, 200.0).unwrap() - 10.0).abs() < 1e-9); // clamp after end
}

#[test]
fn truncate_at_trip_sets_endpoint_to_trip_position() {
    let mut ret = RetainedMotorCurve::default();
    ret.push_piece(0, 0, motor_curve(0.0, 10.0, 100.0, 102.0), 100.0, 102.0);
    ret.truncate_at(0, 101.0); // trip at t_abs=101 -> endpoint must be the position at 101 (=5.0)
    assert!((ret.endpoint(0, 0).unwrap() - 5.0).abs() < 1e-9);
}
