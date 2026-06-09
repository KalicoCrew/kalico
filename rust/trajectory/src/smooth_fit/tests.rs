use super::*;

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
    let knots: Vec<f64> = (0..5).map(|i| std::f64::consts::PI * i as f64 / 4.0).collect();
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
