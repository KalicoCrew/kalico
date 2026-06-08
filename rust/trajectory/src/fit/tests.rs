use super::*;
use nurbs::bezier::BezierPiece;

#[test]
fn fit_and_split_linear_pieces() {
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

    let result = fit_and_split(&composed, 0.005, None).unwrap();
    assert!((result.t_start - 0.0).abs() < 1e-12);
    assert!((result.t_end - 4.0).abs() < 1e-12);

    for axis in &result.axes {
        assert!(!axis.control_points().is_empty());
    }

    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    let x_start = x_pieces[0].evaluate(0.0);
    let x_end = x_pieces.last().unwrap().evaluate(4.0);
    assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
    assert!((x_end - 4.0).abs() < 1e-8, "X(4) = {x_end}, expected 4");
}

#[test]
fn fit_and_split_empty_returns_error() {
    let result = fit_and_split(&[], 0.005, None);
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

    let result = fit_and_split(&composed, 0.005, None).unwrap();
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
    let result = fit_and_split(&composed, 0.005, None).unwrap();

    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    let x_start = x_pieces[0].evaluate(0.0);
    let x_end = x_pieces.last().unwrap().evaluate(2.0);
    assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
    assert!((x_end - 1.5).abs() < 1e-8, "X(2) = {x_end}, expected 1.5");

    let y_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[1]);
    let y_start = y_pieces[0].evaluate(0.0);
    let y_end = y_pieces.last().unwrap().evaluate(2.0);
    assert!((y_start - 0.0).abs() < 1e-8, "Y(0) = {y_start}, expected 0");
    assert!((y_end - 2.0).abs() < 1e-8, "Y(2) = {y_end}, expected 2");
}

#[test]
fn fit_and_split_reduces_piece_count() {
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

    let result = fit_and_split(&composed, 0.005, None).unwrap();

    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    assert!(
        x_pieces.len() < 8,
        "expected piece count reduction, got {} pieces",
        x_pieces.len()
    );
}

#[test]
fn fit_and_split_start_d2_matches_composed_input() {
    // Verify that fit_and_split pins the output's 2nd derivative at the global
    // start to match the composed input's start 2nd derivative.
    // Source: x(t) = 0.5*t², y(t) = t, z(t) = 0 on [0, 2].
    // x''(0) = 1 (constant curvature); y''(0) = 0.
    let composed: Vec<[BezierPiece<f64>; 3]> = (0..2)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.5 * s * s, s, 0.5],
                },
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
            ]
        })
        .collect();

    let d2_start_x = composed[0][0]
        .differentiate()
        .differentiate()
        .evaluate(composed[0][0].u_start);

    let result = fit_and_split(&composed, 0.005, None).unwrap();
    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);

    let fitted_d2_start = x_pieces[0]
        .differentiate()
        .differentiate()
        .evaluate(x_pieces[0].u_start);

    assert!(
        (fitted_d2_start - d2_start_x).abs() < 1e-4,
        "X d2 at start: composed={d2_start_x}, fitted={fitted_d2_start}"
    );
}

#[test]
fn fit_and_split_end_d2_matches_composed_input() {
    // Symmetric to the start-pin test: fit_and_split must also pin the output's
    // 2nd derivative at the global end to match the composed input's end d2.
    // Source: x(t) = 50*t² on [0, 1] followed by [1, 2].
    // Pascal-shifted at s: coeffs = [50*s², 100*s, 50].
    // x''(t) = 100 everywhere; composed end d2 = 100.
    let a = 50.0_f64;
    let composed: Vec<[BezierPiece<f64>; 3]> = (0..2)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![a * s * s, 2.0 * a * s, a],
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

    let last = composed.last().unwrap();
    let d2_end_x = last[0]
        .differentiate()
        .differentiate()
        .evaluate(last[0].u_end);

    let result = fit_and_split(&composed, 0.005, None).unwrap();
    let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
    let last_piece = x_pieces.last().unwrap();

    let fitted_d2_end = last_piece
        .differentiate()
        .differentiate()
        .evaluate(last_piece.u_end);

    assert!(
        (fitted_d2_end - d2_end_x).abs() < 1e-4,
        "X d2 at end: composed={d2_end_x}, fitted={fitted_d2_end}"
    );
}

#[test]
fn fit_and_split_junction_c2_continuity() {
    // Fused 2-segment accelerating chain.  Both segments represent uniform
    // acceleration (x'' = 100 everywhere) so the SOCP boundary condition makes
    // the composed d2 equal on both sides of the junction.  After fitting each
    // segment independently through fit_and_split, the fitted end d2 of seg0
    // must equal the fitted start d2 of seg1.
    //
    // Seg0: x(t) = 50*t² on [0, 1].  Pascal-shifted: [0, 0, 50].
    // Seg1: x(t) = 50 + 100*(t-1) + 50*(t-1)² on [1, 2].
    //        Pascal-shifted at 1: [50, 100, 50].
    let a = 50.0_f64;
    let seg0: Vec<[BezierPiece<f64>; 3]> = vec![[
        BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 0.0, a],
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
    ]];

    let seg1: Vec<[BezierPiece<f64>; 3]> = vec![[
        BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![50.0, 100.0, a],
        },
        BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.0],
        },
        BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.0],
        },
    ]];

    let tol = 0.005;
    let fit0 = fit_and_split(&seg0, tol, None).unwrap();
    let fit1 = fit_and_split(&seg1, tol, None).unwrap();

    let x0_pieces = nurbs::bezier::extract_bezier_pieces(&fit0.axes[0]);
    let x1_pieces = nurbs::bezier::extract_bezier_pieces(&fit1.axes[0]);

    let seg0_end_d2 = {
        let p = x0_pieces.last().unwrap();
        p.differentiate().differentiate().evaluate(p.u_end)
    };
    let seg1_start_d2 = {
        let p = &x1_pieces[0];
        p.differentiate().differentiate().evaluate(p.u_start)
    };

    assert!(
        (seg0_end_d2 - seg1_start_d2).abs() < 1e-4,
        "C2 junction accel step: seg0_end_d2={seg0_end_d2:.6}, seg1_start_d2={seg1_start_d2:.6}, gap={:.6e}",
        (seg0_end_d2 - seg1_start_d2).abs()
    );
}

#[test]
fn junction_accel_step_before_vs_after_both_pin() {
    // Honest measurement: how much does the junction acceleration gap shrink
    // when the end-accel pin is added?
    //
    // Geometry: a degree-6 x-axis composed piece (sin approximation) whose
    // 2nd derivative at the end is NOT zero.  A degree-4 start-only fit cannot
    // reproduce the end curvature exactly; the end d2 will drift.  The
    // both-pin fit (fit_and_split) must nail the end d2 to the composed value.
    //
    // Source curve: f(t) = sin(t) on [0, π/3].  f''(π/3) = -sin(π/3) ≈ −0.866.
    // We represent it with a degree-6 Pascal-shifted polynomial computed from
    // the Taylor expansion at t=0.  The position error of this truncated series
    // is small over [0, π/3] but nonzero, giving the fitter genuine work to do.
    use nurbs::algebra::fit_hermite_c1_clamped;

    let t_end = std::f64::consts::PI / 3.0;
    // Taylor coefficients of sin(t) at 0: t - t^3/6 + t^5/120 - t^7/5040
    // Pascal-shifted at 0 these are the raw coefficients:
    let sin_coeffs = vec![
        0.0_f64,           // c0 = sin(0)
        1.0,               // c1 = cos(0)
        0.0,               // c2 = -sin(0)/2! = 0
        -1.0 / 6.0,        // c3 = -cos(0)/3! = -1/6
        0.0,               // c4 = sin(0)/4! = 0
        1.0 / 120.0,       // c5 = cos(0)/5! = 1/120
        0.0,               // c6 = -sin(0)/6! = 0  (actually 0 due to sin series)
    ];

    let composed_seg0: Vec<[BezierPiece<f64>; 3]> = vec![[
        BezierPiece {
            u_start: 0.0,
            u_end: t_end,
            coeffs: sin_coeffs,
        },
        BezierPiece {
            u_start: 0.0,
            u_end: t_end,
            coeffs: vec![0.0],
        },
        BezierPiece {
            u_start: 0.0,
            u_end: t_end,
            coeffs: vec![0.0],
        },
    ]];

    let composed_d2_start: [f64; 3] = std::array::from_fn(|axis| {
        let p = &composed_seg0[0][axis];
        p.differentiate().differentiate().evaluate(p.u_start)
    });
    let composed_d2_end: f64 = {
        let p = &composed_seg0[0][0];
        p.differentiate().differentiate().evaluate(p.u_end)
    };

    // "Before": start-pin-only fit at degree-4.  The fitter is free to choose
    // any end d2 it likes in order to minimise position residual.
    let before_fit = fit_hermite_c1_clamped::<3>(
        &composed_seg0,
        0.005,
        4,
        Some(composed_d2_start),
        None,
    )
    .unwrap();
    let before_end_d2 = {
        let p = before_fit[0].last().unwrap();
        p.differentiate().differentiate().evaluate(p.u_end)
    };

    // "After": both pins via fit_and_split.
    let after_fit = fit_and_split(&composed_seg0, 0.005, None).unwrap();
    let after_pieces = nurbs::bezier::extract_bezier_pieces(&after_fit.axes[0]);
    let after_end_d2 = {
        let p = after_pieces.last().unwrap();
        p.differentiate().differentiate().evaluate(p.u_end)
    };

    let gap_before = (before_end_d2 - composed_d2_end).abs();
    let gap_after = (after_end_d2 - composed_d2_end).abs();

    // After must nail the end accel to the composed value.
    assert!(
        gap_after < 1e-4,
        "After both-pin fit: end d2 gap={gap_after:.6e} (expected < 1e-4)"
    );
    // Before must show a larger gap (demonstrating the improvement).
    assert!(
        gap_before > gap_after,
        "Before gap ({gap_before:.6}) should exceed after gap ({gap_after:.6e}); \
         before_end_d2={before_end_d2:.6}, after_end_d2={after_end_d2:.6}, \
         composed_d2_end={composed_d2_end:.6}"
    );
}
