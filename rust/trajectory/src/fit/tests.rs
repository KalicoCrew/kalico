use super::*;
use nurbs::bezier::BezierPiece;

#[test]
fn fit_and_split_linear_pieces() {
    // 4 linear pieces: x=t, y=0.5t, z=0 on [0, 4].
    // BezierPiece coeffs are Pascal-shifted-monomial: [a0, a1] => a0 + a1*(u - u_start).
    let composed: Vec<[BezierPiece<f64>; 3]> = (0..4)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![s, 1.0],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![s * 0.5, 0.5],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0, 0.0],
                },
            ]
        })
        .collect();

    let result = fit_and_split(&composed, 0.005).unwrap();
    assert!((result.t_start - 0.0).abs() < 1e-12);
    assert!((result.t_end - 4.0).abs() < 1e-12);

    // Each axis should be a valid ScalarNurbs with at least one control point.
    for axis in &result.axes {
        assert!(!axis.control_points().is_empty());
    }

    // X(0) = 0, X(4) = 4
    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    let x_start = x_pieces[0].evaluate(0.0);
    let x_end = x_pieces.last().unwrap().evaluate(4.0);
    assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
    assert!((x_end - 4.0).abs() < 1e-8, "X(4) = {x_end}, expected 4");
}

#[test]
fn fit_and_split_empty_returns_error() {
    let result = fit_and_split(&[], 0.005);
    assert!(matches!(result, Err(crate::ShapeError::EmptySegments)));
}

#[test]
fn fit_and_split_drops_zero_duration_input_piece() {
    let composed: Vec<[BezierPiece<f64>; 3]> = vec![
        [
            BezierPiece {
                u_start: 0.0,
                u_end: 0.0,
                coeffs: vec![0.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 0.0,
                coeffs: vec![0.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 0.0,
                coeffs: vec![0.0],
            },
        ],
        [
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 1.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0],
            },
        ],
    ];

    let result = fit_and_split(&composed, 0.005).unwrap();
    assert_eq!(result.t_start, 0.0);
    assert_eq!(result.t_end, 1.0);

    for axis in &result.axes {
        for piece in nurbs::bezier::extract_bezier_pieces(axis) {
            assert!(piece.u_start.is_finite());
            assert!(piece.u_end.is_finite());
            assert!(piece.u_end > piece.u_start);
            assert!(piece.coeffs.iter().all(|c| c.is_finite()));
        }
    }
}

#[test]
fn fit_and_split_preserves_endpoints() {
    // Two quadratic pieces: x = 0.5t^2 on [0,1] and x = 0.5 + t - 0.5 on [1,2].
    // Split into two pieces so the fitter has bisection room.
    // Piece 0: x = 0.5*(u-0)^2 on [0,1], so x(0)=0, x(1)=0.5.
    // Piece 1: x = 0.5 + 1.0*(u-1) on [1,2], so x(1)=0.5, x(2)=1.5.
    let composed: Vec<[BezierPiece<f64>; 3]> = vec![
        [
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 0.0, 0.5],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0, 1.0],
            },
            BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![0.0],
            },
        ],
        [
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![0.5, 1.0],
            },
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![1.0, 1.0],
            },
            BezierPiece {
                u_start: 1.0,
                u_end: 2.0,
                coeffs: vec![0.0],
            },
        ],
    ];
    let result = fit_and_split(&composed, 0.005).unwrap();

    // X at t=0 should be 0, at t=2 should be 1.5.
    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    let x_start = x_pieces[0].evaluate(0.0);
    let x_end = x_pieces.last().unwrap().evaluate(2.0);
    assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
    assert!((x_end - 1.5).abs() < 1e-8, "X(2) = {x_end}, expected 1.5");

    // Y at t=0 should be 0, at t=2 should be 2.
    let y_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[1]);
    let y_start = y_pieces[0].evaluate(0.0);
    let y_end = y_pieces.last().unwrap().evaluate(2.0);
    assert!((y_start - 0.0).abs() < 1e-8, "Y(0) = {y_start}, expected 0");
    assert!((y_end - 2.0).abs() < 1e-8, "Y(2) = {y_end}, expected 2");
}

#[test]
fn fit_and_split_reduces_piece_count() {
    // 8 linear pieces that are all part of the same line: x = t.
    // A C1 Hermite fitter should merge these into fewer pieces.
    let composed: Vec<[BezierPiece<f64>; 3]> = (0..8)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![s, 1.0],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
            ]
        })
        .collect();

    let result = fit_and_split(&composed, 0.005).unwrap();

    // The fitter should produce fewer pieces than the 8 input pieces
    // (linear motion can be represented exactly by a single degree-4 piece).
    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    assert!(
        x_pieces.len() < 8,
        "expected piece count reduction, got {} pieces",
        x_pieces.len()
    );
}
