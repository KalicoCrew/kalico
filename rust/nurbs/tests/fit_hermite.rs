#![allow(clippy::cast_lossless, clippy::cast_possible_wrap)]

use nurbs::algebra::fit_hermite_c1;
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
