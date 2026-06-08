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

/// Exercises the live path: relative-domain curve (knots in [seg.t_start, seg.t_end])
/// goes through `rebase_to_absolute_domain(t0)` and then push_piece before eval.
///
/// Segment runs from relative t=0.2 to t=0.8 (duration 0.6 s), position 0->12 mm.
/// t0 = 1000.0 (host anchor), so absolute window is [1000.2, 1000.8].
/// The midpoint of [0.2,0.8] is 0.5 (relative) = 1000.5 (absolute), expected position = 6.0 mm.
#[test]
fn rebase_then_eval_uses_absolute_domain() {
    let t0: f64 = 1000.0;
    let rel_t_start = 0.2_f64;
    let rel_t_end = 0.8_f64;

    let rel_curve = motor_curve(0.0, 12.0, rel_t_start, rel_t_end);

    let abs_curve = rebase_to_absolute_domain(&rel_curve, t0);
    let t_abs_start = t0 + rel_t_start;
    let t_abs_end = t0 + rel_t_end;

    let mut ret = RetainedMotorCurve::default();
    ret.push_piece(0, 0, abs_curve, t_abs_start, t_abs_end);

    let t_abs_mid = t0 + (rel_t_start + rel_t_end) / 2.0; // 1000.5
    assert!(
        (ret.eval(0, 0, t_abs_mid).unwrap() - 6.0).abs() < 1e-9,
        "midpoint eval after rebase expected 6.0, got {:?}",
        ret.eval(0, 0, t_abs_mid)
    );

    assert!((ret.eval(0, 0, t0 - 1.0).unwrap() - 0.0).abs() < 1e-9, "clamp below");
    assert!((ret.eval(0, 0, t0 + 999.0).unwrap() - 12.0).abs() < 1e-9, "clamp above");
    assert!((ret.endpoint(0, 0).unwrap() - 12.0).abs() < 1e-9, "endpoint");
}
