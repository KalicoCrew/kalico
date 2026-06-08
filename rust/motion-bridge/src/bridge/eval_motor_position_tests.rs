use super::*;
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

fn motor_curve(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    let d = (p1 - p0) / 3.0;
    let bern = [p0, p0 + d, p0 + 2.0 * d, p1];
    bezier_pieces_to_nurbs(&[BezierPiece::from_bernstein(&bern, t0, t1)])
}

#[test]
fn eval_core_resolves_oid_and_evaluates() {
    let mcu = 5;
    let oid = 9;
    let slot = 1;
    let b = PyMotionBridge::new_for_test();
    b.insert_stepper_slot(mcu, oid, slot);
    {
        let mut g = b.retained_motor_curve.lock().unwrap_or_else(|p| p.into_inner());
        g.push_piece(mcu, slot, motor_curve(0.0, 10.0, 100.0, 102.0), 100.0, 102.0);
    }
    assert!((b.eval_motor_mm_at_t_abs(mcu, oid, 101.0).unwrap() - 5.0).abs() < 1e-9);
    assert!((b.eval_motor_mm_now(mcu, oid).unwrap() - 10.0).abs() < 1e-9);
    let unknown_oid = 7;
    assert!(b.eval_motor_mm_at_t_abs(mcu, unknown_oid, 101.0).is_none());
    let known_oid_no_curve = 8;
    b.insert_stepper_slot(mcu, known_oid_no_curve, 2);
    assert!(b.eval_motor_mm_at_t_abs(mcu, known_oid_no_curve, 101.0).is_none());
}
