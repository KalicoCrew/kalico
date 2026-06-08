//! Tests for `fit_hermite_c1` and `fit_hermite_c1_clamped`.

#![allow(clippy::cast_lossless, clippy::cast_possible_wrap)]

use nurbs::algebra::{fit_hermite_c1, fit_hermite_c1_clamped};
use nurbs::bezier::BezierPiece;

/// Binomial coefficient C(n, k) — local helper since the crate's binomial is pub(crate).
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
    // 4 linear pieces forming x(t) = t on [0, 4]
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
    assert_eq!(result[0].len(), 1); // merges into 1 piece
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
    // Two quadratic pieces
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
    // Check C1 at boundaries
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
    // Many quadratic pieces representing f(u) = u² on [0, 10].
    // In Pascal-shifted basis at u_start = s: c₀ = s², c₁ = 2s, c₂ = 1.
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
    // Check residual at dense sample points
    for fitted in &result[0] {
        let n_samples = 50;
        let h = (fitted.u_end - fitted.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted.u_start + i as f64 * h;
            // Find reference value from input pieces
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
    // 3D pieces (2 pieces, 3 axes each)
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
    assert_eq!(result.len(), 3); // 3 axes
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
    // Should reproduce the original polynomial exactly
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
    // Create C¹-continuous pieces with high-frequency content that can't be merged into
    // one degree-4 piece. Use cubic Hermite interpolation of sin(x) at piece boundaries,
    // matching both value and derivative — guaranteeing C¹ continuity between adjacent pieces.
    let n = 20;
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..n)
        .map(|i| {
            let u0 = i as f64 * std::f64::consts::TAU / n as f64;
            let u1 = (i + 1) as f64 * std::f64::consts::TAU / n as f64;
            let h = u1 - u0;
            // Cubic Hermite interpolation matching sin/cos at both endpoints.
            // p(u) = c0 + c1*(u-u0) + c2*(u-u0)^2 + c3*(u-u0)^3
            let f0 = u0.sin();
            let df0 = u0.cos();
            let f1 = u1.sin();
            let df1 = u1.cos();
            let c0 = f0;
            let c1 = df0;
            // Solve for c2, c3 from endpoint conditions:
            //   c0 + c1*h + c2*h^2 + c3*h^3 = f1
            //   c1 + 2*c2*h + 3*c3*h^2 = df1
            let pos_res = f1 - c0 - c1 * h;
            let vel_res = df1 - c1;
            // det = 3*h^4 - 2*h^4 = h^4
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
    // Should have more than 1 piece (can't merge all of sin into one degree-4)
    assert!(
        result[0].len() > 1,
        "expected bisection, got {} piece(s)",
        result[0].len()
    );
    // But fewer than input
    assert!(
        result[0].len() < n,
        "expected merging, got {} pieces from {} input",
        result[0].len(),
        n
    );
    // Check tolerance
    for fitted in &result[0] {
        let n_samples = 50;
        let h = (fitted.u_end - fitted.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted.u_start + i as f64 * h;
            // Find reference
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
    // Check C1 continuity
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
    // Simulate the real use case: degree-6 pieces from composition
    // x(t) = t + 0.1*t^2 + 0.01*t^3 + 0.001*t^4 + 0.0001*t^5 + 0.00001*t^6
    // Split into 5 pieces on [0, 5]
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..5)
        .map(|i| {
            let u0 = i as f64;
            let u1 = (i + 1) as f64;
            // Degree 6 polynomial in Pascal-shifted basis at u0
            // f(u) = u + 0.1*u^2 + 0.01*u^3 + 0.001*u^4 + 0.0001*u^5 + 0.00001*u^6
            // Need to shift to basis at u0. Use Taylor expansion approach.
            // f(u) = f(u0) + f'(u0)*(u-u0) + f''(u0)/2!*(u-u0)^2 + ...
            let x = u0;
            let c0 = x
                + 0.1 * x.powi(2)
                + 0.01 * x.powi(3)
                + 0.001 * x.powi(4)
                + 0.0001 * x.powi(5)
                + 0.00001 * x.powi(6);
            let c1 = 1.0
                + 0.2 * x
                + 0.03 * x.powi(2)
                + 0.004 * x.powi(3)
                + 0.0005 * x.powi(4)
                + 0.00006 * x.powi(5);
            let c2 =
                (0.2 + 0.06 * x + 0.012 * x.powi(2) + 0.002 * x.powi(3) + 0.0003 * x.powi(4)) / 1.0; // f''(u0)/2!
            let c3 = (0.06 + 0.024 * x + 0.006 * x.powi(2) + 0.0012 * x.powi(3)) / 1.0; // f'''(u0)/3!... wait
            // Actually let's just use the absolute monomial and convert.
            // Absolute: 0 + 1*u + 0.1*u^2 + 0.01*u^3 + 0.001*u^4 + 0.0001*u^5 + 0.00001*u^6
            // We need Pascal-shifted at u0.
            let abs_coeffs = [0.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001];
            let mut shifted = vec![0.0f64; 7];
            // c'_k = sum_{n=k}^{6} abs[n] * C(n,k) * u0^{n-k}
            for k in 0..7 {
                for n in k..7 {
                    let binom = binomial(n, k) as f64;
                    shifted[k] += abs_coeffs[n] * binom * u0.powi((n - k) as i32);
                }
            }
            let _ = (c0, c1, c2, c3); // suppress unused warnings from the manual computation above
            [BezierPiece {
                u_start: u0,
                u_end: u1,
                coeffs: shifted,
            }]
        })
        .collect();

    let tol = 0.005;
    let result = fit_hermite_c1::<1>(&pieces, tol, 4).unwrap();

    // Output pieces should be degree 4
    for fitted in &result[0] {
        assert_eq!(
            fitted.coeffs.len(),
            5,
            "expected degree-4 output (5 coeffs)"
        );
    }

    // Should merge at least some pieces (the polynomial is gentle)
    assert!(
        result[0].len() < pieces.len(),
        "expected merging: got {} output pieces from {} input",
        result[0].len(),
        pieces.len()
    );

    // Check tolerance
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
