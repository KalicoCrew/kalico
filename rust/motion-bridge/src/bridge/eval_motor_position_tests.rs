use super::*;
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

fn motor_curve(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    let d = (p1 - p0) / 3.0;
    let bern = [p0, p0 + d, p0 + 2.0 * d, p1];
    bezier_pieces_to_nurbs(&[BezierPiece::from_bernstein(&bern, t0, t1)])
}

#[test]
fn eval_core_resolves_oid_and_evaluates() {
    let b = PyMotionBridge::new_for_test();
    b.insert_stepper_slot(5 /*mcu*/, 9 /*oid*/, 1 /*slot*/);
    {
        let mut g = b.retained_motor_curve.lock().unwrap_or_else(|p| p.into_inner());
        g.push_piece(5, 1, motor_curve(0.0, 10.0, 100.0, 102.0), 100.0, 102.0);
    }
    assert!((b.eval_motor_mm_at_t_abs(5, 9, 101.0).unwrap() - 5.0).abs() < 1e-9);
    assert!((b.eval_motor_mm_now(5, 9).unwrap() - 10.0).abs() < 1e-9); // endpoint
    assert!(b.eval_motor_mm_at_t_abs(5, 7 /*unknown oid*/, 101.0).is_none());
}
