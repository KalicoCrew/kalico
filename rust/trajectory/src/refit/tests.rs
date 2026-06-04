use super::*;
use nurbs::eval::eval;

fn linear_curve(v_start: f64, v_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v_start, v_end])
        .expect("linear NURBS construction")
}

#[test]
fn refits_linear_passthrough_within_tolerance() {
    let input = linear_curve(0.0, 5.0);
    let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");
    // Sample 33 points and compare against analytic v(u) = 5u.
    for i in 0..=32 {
        let u = (i as f64) / 32.0;
        let truth = 5.0 * u;
        let v = eval(&output.as_view(), u);
        assert!(
            (truth - v).abs() <= REFIT_TOLERANCE_MM,
            "linear residual at u={u}: truth={truth}, refit={v}"
        );
    }
}

#[test]
fn refits_high_degree_polynomial_within_tolerance() {
    let p = 9_usize;
    let cps: Vec<f64> = (0..=p)
        .map(|i| {
            let u = (i as f64) / (p as f64);
            100.0 + 5.0 * (2.0 * std::f64::consts::PI * u).sin()
        })
        .collect();
    let piece = nurbs::bezier::BezierPiece::from_bernstein(&cps, 0.0, 1.0);
    let input = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);

    let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");

    for i in 0..=200 {
        let u = (i as f64) / 200.0;
        let truth = eval(&input.as_view(), u);
        let refit = eval(&output.as_view(), u);
        let diff = (truth - refit).abs();
        assert!(
            diff <= REFIT_TOLERANCE_MM * 1.5,
            "residual at u={u}: input={truth}, refit={refit}, diff={diff}"
        );
    }

    assert_eq!(output.degree(), 3, "refit output should be cubic");
}

#[test]
fn refit_is_idempotent_on_cubic_input() {
    let cps = vec![0.0, 1.5, 2.5, 4.0];
    let piece = nurbs::bezier::BezierPiece::from_bernstein(&cps, 0.0, 1.0);
    let input = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);
    let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");
    for i in 0..=64 {
        let u = (i as f64) / 64.0;
        let truth = eval(&input.as_view(), u);
        let refit = eval(&output.as_view(), u);
        let diff = (truth - refit).abs();
        assert!(
            diff <= 1e-9,
            "cubic should be reproduced exactly: u={u}, diff={diff}"
        );
    }
    assert_eq!(output.degree(), 3);
}
