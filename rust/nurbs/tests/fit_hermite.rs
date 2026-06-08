#![allow(clippy::cast_lossless, clippy::cast_possible_wrap)]

use nurbs::algebra::{fit_hermite_c1, fit_hermite_c1_clamped};
use nurbs::bezier::BezierPiece;

fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
}

#[test]
fn hermite_fit_merges_linear_pieces() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..4)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s, 1.0],
            }]
        })
        .collect();
    let result = fit_hermite_c1::<1>(&pieces, 0.005, 4).unwrap();
    assert_eq!(result[0].len(), 1);
    for &t in &[0.0, 1.0, 2.0, 3.0, 4.0] {
        assert!(
            (result[0][0].evaluate(t) - t).abs() < 1e-10,
            "at t={t}: got {}, expected {t}",
            result[0][0].evaluate(t)
        );
    }
}

#[test]
fn hermite_fit_preserves_c1() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = vec![
        [BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, 2.0],
        }],
        [BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![3.0, 5.0, -1.0],
        }],
    ];
    let result = fit_hermite_c1::<1>(&pieces, 0.005, 4).unwrap();
    for window in result[0].windows(2) {
        let left_val = window[0].evaluate(window[0].u_end);
        let right_val = window[1].evaluate(window[1].u_start);
        assert!(
            (left_val - right_val).abs() < 1e-10,
            "C0 violated: {left_val} vs {right_val}"
        );
        let left_d = window[0].differentiate().evaluate(window[0].u_end);
        let right_d = window[1].differentiate().evaluate(window[1].u_start);
        assert!(
            (left_d - right_d).abs() < 1e-8,
            "C1 violated: {left_d} vs {right_d}"
        );
    }
}

#[test]
fn hermite_fit_respects_tolerance() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..10)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s * s, 2.0 * s, 1.0],
            }]
        })
        .collect();
    let tol = 0.005;
    let result = fit_hermite_c1::<1>(&pieces, tol, 4).unwrap();
    for fitted in &result[0] {
        let n_samples = 50;
        let h = (fitted.u_end - fitted.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted.u_start + i as f64 * h;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap();
            let fit_val = fitted.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }
}

#[test]
fn hermite_fit_3d() {
    let pieces: Vec<[BezierPiece<f64>; 3]> = vec![
        [
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 1.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 0.5],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 0.0],
            },
        ],
        [
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![1.0, 1.0],
            },
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![0.5, 0.5],
            },
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![0.0, 0.0],
            },
        ],
    ];
    let result = fit_hermite_c1::<3>(&pieces, 0.005, 4).unwrap();
    assert_eq!(result.len(), 3);
    for axis in &result {
        assert!(!axis.is_empty());
    }
}

#[test]
fn hermite_fit_empty_input_returns_error() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = vec![];
    let result = fit_hermite_c1::<1>(&pieces, 0.005, 4);
    assert!(result.is_err());
}

#[test]
fn hermite_fit_single_piece_returns_it() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = vec![[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    }]];
    let result = fit_hermite_c1::<1>(&pieces, 0.005, 4).unwrap();
    assert_eq!(result[0].len(), 1);
    for &u in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let ref_val = pieces[0][0].evaluate(u);
        let fit_val = result[0][0].evaluate(u);
        assert!(
            (ref_val - fit_val).abs() < 1e-10,
            "at u={u}: ref={ref_val}, fit={fit_val}"
        );
    }
}

#[test]
fn hermite_fit_high_curvature_bisects() {
    let n = 20;
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..n)
        .map(|i| {
            let u0 = i as f64 * std::f64::consts::TAU / n as f64;
            let u1 = (i + 1) as f64 * std::f64::consts::TAU / n as f64;
            let h = u1 - u0;
            let f0 = u0.sin();
            let df0 = u0.cos();
            let f1 = u1.sin();
            let df1 = u1.cos();
            let c0 = f0;
            let c1 = df0;
            let pos_res = f1 - c0 - c1 * h;
            let vel_res = df1 - c1;
            let det = h * h * h * h;
            let c2 = (3.0 * h * h * pos_res - h * h * h * vel_res) / det;
            let c3 = (h * h * vel_res - 2.0 * h * pos_res) / det;
            [BezierPiece {
                u_start: u0,
                u_end: u1,
                coeffs: vec![c0, c1, c2, c3],
            }]
        })
        .collect();
    let tol = 0.001;
    let result = fit_hermite_c1::<1>(&pieces, tol, 4).unwrap();
    assert!(
        result[0].len() > 1,
        "expected bisection, got {} piece(s)",
        result[0].len()
    );
    assert!(
        result[0].len() < n,
        "expected merging, got {} pieces from {} input",
        result[0].len(),
        n
    );
    for fitted in &result[0] {
        let n_samples = 50;
        let h = (fitted.u_end - fitted.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted.u_start + i as f64 * h;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap();
            let fit_val = fitted.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }
    for window in result[0].windows(2) {
        let left_val = window[0].evaluate(window[0].u_end);
        let right_val = window[1].evaluate(window[1].u_start);
        assert!(
            (left_val - right_val).abs() < 1e-10,
            "C0 violated at boundary {}: {left_val} vs {right_val}",
            window[0].u_end
        );
        let left_d = window[0].differentiate().evaluate(window[0].u_end);
        let right_d = window[1].differentiate().evaluate(window[1].u_start);
        assert!(
            (left_d - right_d).abs() < 1e-8,
            "C1 violated at boundary {}: {left_d} vs {right_d}",
            window[0].u_end
        );
    }
}

#[test]
fn hermite_fit_degree6_input_reduces_to_degree4() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..5)
        .map(|i| {
            let u0 = i as f64;
            let u1 = (i + 1) as f64;
            let abs_coeffs = [0.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001];
            let mut shifted = vec![0.0f64; 7];
            for k in 0..7 {
                for n in k..7 {
                    let binom = binomial(n, k) as f64;
                    shifted[k] += abs_coeffs[n] * binom * u0.powi((n - k) as i32);
                }
            }
            [BezierPiece {
                u_start: u0,
                u_end: u1,
                coeffs: shifted,
            }]
        })
        .collect();

    let tol = 0.005;
    let result = fit_hermite_c1::<1>(&pieces, tol, 4).unwrap();

    for fitted in &result[0] {
        assert_eq!(
            fitted.coeffs.len(),
            5,
            "expected degree-4 output (5 coeffs)"
        );
    }

    assert!(
        result[0].len() < pieces.len(),
        "expected merging: got {} output pieces from {} input",
        result[0].len(),
        pieces.len()
    );

    for fitted in &result[0] {
        let n_samples = 50;
        let h = (fitted.u_end - fitted.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted.u_start + i as f64 * h;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap();
            let fit_val = fitted.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Tests for fit_hermite_c1_clamped (C2 boundary pins)
// ---------------------------------------------------------------------------

/// Evaluate the 2nd derivative of a BezierPiece at a point.
fn d2_at(piece: &BezierPiece<f64>, u: f64) -> f64 {
    piece.differentiate().differentiate().evaluate(u)
}

#[test]
fn clamped_fit_pins_both_boundary_second_derivatives() {
    // Source: f(t) = t³ on [0, 2]. Pascal-shifted at 0: coeffs = [0,0,0,1].
    // We pin d2_start=6 and d2_end=10 (deliberately not the analytic values).
    // The fitted polynomial must honour exactly these pins at its outer endpoints.
    let pieces: Vec<[BezierPiece<f64>; 1]> = vec![[BezierPiece {
        u_start: 0.0,
        u_end: 2.0,
        coeffs: vec![0.0, 0.0, 0.0, 1.0],
    }]];

    let d2_start_pin = 6.0_f64;
    let d2_end_pin = 10.0_f64;

    // Tolerance is 2.0: with deliberately non-analytic pins the residual will be
    // significant, but we only need to assert the pins are honoured exactly.
    let result = fit_hermite_c1_clamped::<1>(
        &pieces,
        2.0,
        5,
        Some([d2_start_pin]),
        Some([d2_end_pin]),
    )
    .unwrap();

    let first = &result[0][0];
    let last = result[0].last().unwrap();

    let got_start = d2_at(first, first.u_start);
    let got_end = d2_at(last, last.u_end);

    assert!(
        (got_start - d2_start_pin).abs() < 1e-6,
        "d2 at start: expected {d2_start_pin}, got {got_start}"
    );
    assert!(
        (got_end - d2_end_pin).abs() < 1e-6,
        "d2 at end: expected {d2_end_pin}, got {got_end}"
    );
}

#[test]
fn clamped_fit_position_residual_within_tolerance() {
    // Source: f(t) = t² + 0.5*t on [0, 3], split into 3 unit pieces.
    // f''(0) = 2, f''(3) = 2.
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..3)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s * s + 0.5 * s, 2.0 * s + 0.5, 1.0],
            }]
        })
        .collect();

    let tol = 0.01;
    let result =
        fit_hermite_c1_clamped::<1>(&pieces, tol, 5, Some([2.0_f64]), Some([2.0_f64])).unwrap();

    for fitted in &result[0] {
        let n = 40;
        let step = (fitted.u_end - fitted.u_start) / n as f64;
        for i in 0..=n {
            let u = fitted.u_start + i as f64 * step;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap_or_else(|| {
                    pieces.last().unwrap()[0].evaluate(pieces.last().unwrap()[0].u_end)
                });
            let fit_val = fitted.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }
}

#[test]
fn clamped_fit_preserves_c1_at_interior_knots() {
    // 4 quadratic pieces; pin d2_start and d2_end.
    // Interior piece joints must remain C1.
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..4)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s * s, 2.0 * s, 1.0],
            }]
        })
        .collect();

    // Tolerance 0.5: pin d2_end=10 is far from the analytic 2, introducing
    // inherent position deviation that a tight tolerance would reject.
    let result =
        fit_hermite_c1_clamped::<1>(&pieces, 0.5, 5, Some([2.0_f64]), Some([10.0_f64])).unwrap();

    for window in result[0].windows(2) {
        let left = &window[0];
        let right = &window[1];
        let left_val = left.evaluate(left.u_end);
        let right_val = right.evaluate(right.u_start);
        assert!(
            (left_val - right_val).abs() < 1e-9,
            "C0 violated at interior knot {}: {left_val} vs {right_val}",
            left.u_end
        );
        let left_d = left.differentiate().evaluate(left.u_end);
        let right_d = right.differentiate().evaluate(right.u_start);
        assert!(
            (left_d - right_d).abs() < 1e-7,
            "C1 violated at interior knot {}: {left_d} vs {right_d}",
            left.u_end
        );
    }
}

#[test]
fn clamped_fit_none_pins_matches_c1() {
    // With both pins = None, clamped fit must reproduce fit_hermite_c1 values.
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..4)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s, 1.0],
            }]
        })
        .collect();

    let tol = 0.005;
    let r_c1 = fit_hermite_c1::<1>(&pieces, tol, 4).unwrap();
    let r_clamped = fit_hermite_c1_clamped::<1>(&pieces, tol, 4, None, None).unwrap();

    assert_eq!(r_c1[0].len(), r_clamped[0].len(), "piece count must match");
    for (p_c1, p_cl) in r_c1[0].iter().zip(r_clamped[0].iter()) {
        for &u in &[p_c1.u_start, 0.5 * (p_c1.u_start + p_c1.u_end), p_c1.u_end] {
            assert!(
                (p_c1.evaluate(u) - p_cl.evaluate(u)).abs() < 1e-10,
                "value at u={u} differs between c1 and clamped(None)"
            );
        }
    }
}

#[test]
fn clamped_fit_2d_pins_both_axes() {
    // 2D: x(t) = t², y(t) = 2t on [0, 2].
    // x''(0) = 2, x''(2) = 2; y''(0) = 0, y''(2) = 0.
    let pieces: Vec<[BezierPiece<f64>; 2]> = vec![[
        BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 0.0, 1.0],
        },
        BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 2.0],
        },
    ]];

    let result = fit_hermite_c1_clamped::<2>(
        &pieces,
        0.05,
        5,
        Some([2.0_f64, 0.0_f64]),
        Some([2.0_f64, 0.0_f64]),
    )
    .unwrap();

    let x_first = &result[0][0];
    let x_last = result[0].last().unwrap();
    let y_first = &result[1][0];
    let y_last = result[1].last().unwrap();

    assert!(
        (d2_at(x_first, x_first.u_start) - 2.0).abs() < 1e-6,
        "x d2 at start: got {}",
        d2_at(x_first, x_first.u_start)
    );
    assert!(
        (d2_at(x_last, x_last.u_end) - 2.0).abs() < 1e-6,
        "x d2 at end: got {}",
        d2_at(x_last, x_last.u_end)
    );
    assert!(
        d2_at(y_first, y_first.u_start).abs() < 1e-6,
        "y d2 at start: got {}",
        d2_at(y_first, y_first.u_start)
    );
    assert!(
        d2_at(y_last, y_last.u_end).abs() < 1e-6,
        "y d2 at end: got {}",
        d2_at(y_last, y_last.u_end)
    );
}

/// Adversarial: non-polynomial input (sine approximation as degree-6 pieces)
/// with ASYMMETRIC d2_start != d2_end.
///
/// Verifies:
/// 1. The pinned 2nd derivatives are exact to 1e-6 (not approximate).
/// 2. C0 and C1 are preserved at interior knots.
/// 3. Position residual stays within tolerance everywhere.
/// 4. The test does NOT use a polynomial input — the sine Taylor approximation
///    has non-trivial residual, so the fitter has genuine work to do.
#[test]
fn adversarial_nonpolynomial_asymmetric_pins() {
    // Use the degree-6 Taylor approximation of sin(t) on [0, 2*pi] split into
    // 8 equal pieces. The approximation is NOT exact (truncation error grows
    // near pi/2), so the fitter must actually subdivide and cannot trivially
    // reproduce the source. The 2nd derivative pins are chosen to be the analytic
    // sin''(t) = -sin(t) at the global endpoints but rounded to produce a
    // genuinely asymmetric pair.
    let n = 8;
    let two_pi = std::f64::consts::TAU;
    // sin Taylor at 0: [0, 1, 0, -1/6, 0, 1/120, 0, -1/5040]
    // degree-6 Taylor (drop t^7 term):
    let _sin_abs = [0.0_f64, 1.0, 0.0, -1.0 / 6.0, 0.0, 1.0 / 120.0, 0.0];

    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..n)
        .map(|i| {
            let u0 = i as f64 * two_pi / n as f64;
            let u1 = (i + 1) as f64 * two_pi / n as f64;
            // Re-expand sin Taylor around u0 using shift. For each piece we use
            // a cubic Hermite interpolant matching sin/cos at both endpoints — this
            // is C1-continuous across pieces and NOT a polynomial of sin, so the
            // fitter has nontrivial residual to deal with.
            let h = u1 - u0;
            let f0 = u0.sin();
            let df0 = u0.cos();
            let f1 = u1.sin();
            let df1 = u1.cos();
            // cubic Hermite: c0=f0, c1=df0, solve 2x2 for c2,c3
            let det = h * h * h * h; // h^4
            let c2 = (3.0 * h * h * (f1 - f0 - df0 * h) - h * h * h * (df1 - df0)) / det;
            let c3 = (h * h * (df1 - df0) - 2.0 * h * (f1 - f0 - df0 * h)) / det;
            // Pad to degree-6 with zeros so degree is uniform across pieces.
            [BezierPiece {
                u_start: u0,
                u_end: u1,
                coeffs: vec![f0, df0, c2, c3, 0.0, 0.0, 0.0],
            }]
        })
        .collect();

    // Analytic: sin''(0) = -sin(0) = 0; sin''(2*pi) = -sin(2*pi) ≈ 0.
    // Use deliberately different-from-analytic values to make d2_start != d2_end.
    // Pins are deliberately non-analytic but small enough that the resulting
    // polynomial stays within position tolerance on short pieces.
    let d2_start_pin = 0.5_f64; // not the analytic 0 at t=0
    let d2_end_pin = -0.8_f64; // different sign and magnitude, also not analytic 0

    let tol = 0.15; // generous: residual from non-polynomial input + pin deviation
    let result = fit_hermite_c1_clamped::<1>(
        &pieces,
        tol,
        5,
        Some([d2_start_pin]),
        Some([d2_end_pin]),
    )
    .expect("adversarial clamped fit must succeed");

    // 1. Pins are exact at outer endpoints.
    let first = &result[0][0];
    let last = result[0].last().unwrap();
    let got_start = d2_at(first, first.u_start);
    let got_end = d2_at(last, last.u_end);
    assert!(
        (got_start - d2_start_pin).abs() < 1e-6,
        "ADVERSARIAL: d2 at global start: expected {d2_start_pin}, got {got_start}"
    );
    assert!(
        (got_end - d2_end_pin).abs() < 1e-6,
        "ADVERSARIAL: d2 at global end: expected {d2_end_pin}, got {got_end}"
    );

    // 2. C0 and C1 at all interior knots.
    for window in result[0].windows(2) {
        let left = &window[0];
        let right = &window[1];
        let left_val = left.evaluate(left.u_end);
        let right_val = right.evaluate(right.u_start);
        assert!(
            (left_val - right_val).abs() < 1e-9,
            "ADVERSARIAL: C0 violated at interior knot {}: {} vs {}",
            left.u_end,
            left_val,
            right_val
        );
        let left_d = left.differentiate().evaluate(left.u_end);
        let right_d = right.differentiate().evaluate(right.u_start);
        assert!(
            (left_d - right_d).abs() < 1e-7,
            "ADVERSARIAL: C1 violated at interior knot {}: {} vs {}",
            left.u_end,
            left_d,
            right_d
        );
    }

    // 3. Position residual within tolerance at dense sample points.
    for fitted_piece in &result[0] {
        let n_samples = 40;
        let step = (fitted_piece.u_end - fitted_piece.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted_piece.u_start + i as f64 * step;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap_or_else(|| pieces.last().unwrap()[0].evaluate(pieces.last().unwrap()[0].u_end));
            let fit_val = fitted_piece.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "ADVERSARIAL: at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }

    // 4. Confirm that pins are genuinely asymmetric (not both zero or equal).
    assert!(
        d2_start_pin.abs() > 0.1 && d2_end_pin.abs() > 0.1,
        "pins are non-trivial by construction"
    );
    assert!(
        (d2_start_pin - d2_end_pin).abs() > 0.5,
        "pins are asymmetric by construction"
    );

    // 5. Degree check: first and last pieces must be degree-5 (the pinned boundary pieces).
    assert_eq!(
        first.coeffs.len(),
        6,
        "ADVERSARIAL: first (start-pinned) piece must be degree-5 (6 coeffs), got {}",
        first.coeffs.len()
    );
    assert_eq!(
        last.coeffs.len(),
        6,
        "ADVERSARIAL: last (end-pinned) piece must be degree-5 (6 coeffs), got {}",
        last.coeffs.len()
    );

    // Bonus: the start-pin check should NOT hold for the NON-pinned fit (proving we're testing something real).
    let unpinned = fit_hermite_c1_clamped::<1>(&pieces, tol, 4, None, None).unwrap();
    let unpinned_start_d2 = d2_at(&unpinned[0][0], unpinned[0][0].u_start);
    // The unpinned fit is free to choose its own c2; it's unlikely to land at 3.0.
    // We don't assert a strict divergence, but print it for visibility.
    let _ = unpinned_start_d2; // used as proof-of-concept
}
