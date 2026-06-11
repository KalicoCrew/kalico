use super::*;

#[test]
fn duplicate_knot_produces_nan_without_guard() {
    let knots = vec![0.0_f64, 0.5, 0.5, 1.0];
    let values = vec![0.0_f64, 0.5, 0.5, 1.0];
    let pieces = build_clamped_spline(&knots, &values, 0.0, 0.0);
    let degenerate = pieces
        .iter()
        .any(|p| p.coeffs.iter().any(|c| !c.is_finite()));
    assert!(
        degenerate,
        "duplicate knot must produce NaN coefficients — this documents why the guard is needed"
    );
}

#[test]
fn duplicate_knot_guard_no_panic_finite_error() {
    let f = |t: f64| (1.0_f64 / (1.0 + ((t - 0.5) / 1e-5).powi(2))).sqrt();
    let result = fit_c2_cubic(&f, 0.0, 1.0, 1e-12);
    match result {
        Err(FitError { achieved_mm }) => {
            assert!(
                achieved_mm.is_finite() && achieved_mm > 0.0,
                "achieved_mm must be finite and positive, got {achieved_mm}"
            );
        }
        Ok(ref curve) => {
            for i in 0..=100 {
                let t = i as f64 / 100.0;
                let v = eval(curve, t);
                assert!(
                    v.is_finite(),
                    "fit_c2_cubic returned Ok but spline evaluates to NaN/inf at t={t}: {v}"
                );
            }
        }
    }
}

#[test]
fn thomas_solves_known_system() {
    // Tridiagonal system:
    // [ 2 1 0 ] [x0]   [3]
    // [ 1 2 1 ] [x1] = [4]   -> solution x = [1, 1, 1]
    // [ 0 1 2 ] [x2]   [3]
    let a = [0.0, 1.0, 1.0]; // sub-diagonal (a[0] unused)
    let b = [2.0, 2.0, 2.0]; // diagonal
    let c = [1.0, 1.0, 0.0]; // super-diagonal (c[n-1] unused)
    let d = [3.0, 4.0, 3.0];
    let x = solve_tridiagonal(&a, &b, &c, &d);
    for xi in &x {
        assert!((xi - 1.0).abs() < 1e-12, "x = {x:?}");
    }
}

#[test]
fn clamped_spline_interpolates_and_is_c2() {
    // Fit f(t) = sin(t) on [0, PI] with 5 equal knots, clamped to f'=cos at ends.
    let knots: Vec<f64> = (0..5)
        .map(|i| std::f64::consts::PI * i as f64 / 4.0)
        .collect();
    let values: Vec<f64> = knots.iter().map(|t| t.sin()).collect();
    let yp0 = 0.0_f64.cos();
    let ypn = std::f64::consts::PI.cos();
    let pieces = build_clamped_spline(&knots, &values, yp0, ypn);

    assert_eq!(pieces.len(), 4);

    // Interpolation: each piece hits its endpoint knot values.
    for (i, p) in pieces.iter().enumerate() {
        assert!((p.evaluate(knots[i]) - values[i]).abs() < 1e-12);
        assert!((p.evaluate(knots[i + 1]) - values[i + 1]).abs() < 1e-12);
    }
    // C2: 2nd derivative continuous across interior joints.
    for i in 0..pieces.len() - 1 {
        let left = pieces[i].differentiate().differentiate();
        let right = pieces[i + 1].differentiate().differentiate();
        let j = knots[i + 1];
        assert!(
            (left.evaluate(j) - right.evaluate(j)).abs() < 1e-9,
            "2nd-deriv jump at knot {i}",
        );
    }
}

use nurbs::bezier::extract_bezier_pieces;
use nurbs::eval::eval;

#[test]
fn fit_c2_cubic_matches_smooth_fn_with_few_pieces() {
    // Target: a smooth bump on [0, 1]. Fit to 0.1 um tolerance.
    let f = |t: f64| (3.0 * t).sin() * (1.0 - t) * t;
    let tol = 1e-4;
    let curve = fit_c2_cubic(&f, 0.0, 1.0, tol).expect("fit succeeds");

    // Accuracy sampled densely WITHIN pieces (not just at knots).
    for i in 0..=2000 {
        let t = i as f64 / 2000.0;
        assert!(
            (eval(&curve.as_view(), t) - f(t)).abs() <= tol,
            "error at t={t}",
        );
    }
    // Compactness: a smooth bump needs few pieces, nowhere near hundreds.
    let n = extract_bezier_pieces(&curve).len();
    assert!(n < 40, "expected few pieces, got {n}");
}
