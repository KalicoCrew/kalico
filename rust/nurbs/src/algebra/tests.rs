use super::*;
use crate::eval::eval;

#[test]
fn convolve_linear_input_with_constant_kernel_yields_correct_integral() {
    // x(s) = s on [0, 1], w(t) = 1 on [-0.25, 0.25].
    // y(u) = ∫_{u-0.25}^{u+0.25} s ds = (1/2) * ((u+0.25)^2 - (u-0.25)^2) = u/2
    // for u in [0.25, 0.75] (kernel window fully inside x's support).
    // y(0.5) = 0.5/2 = 0.25.  Equivalently: width * average = 0.5 * 0.5 = 0.25.
    let x =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
            .unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.25, 0.25));

    let y = convolve(&x, &kernel).unwrap();

    let val = eval(&y.as_view(), 0.5);
    assert!((val - 0.25).abs() < 1e-10, "y(0.5) = {val}, expected 0.25");
}

#[test]
fn breakpoint_sort_handles_nan_without_panicking() {
    let mut out_breaks = vec![0.0_f64, f64::NAN, 1.0];
    out_breaks.sort_by(|a, b| <f64 as crate::Float>::total_cmp(*a, *b));
    assert_eq!(out_breaks.len(), 3);
}

#[test]
fn convolve_constant_input_with_constant_kernel_gives_triangle() {
    // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
    // Convolution support: [0 + (-0.5), 1 + 0.5] = [-0.5, 1.5].
    // Output: triangle peaking in [0.5, 0.5] at value 6, sloping linearly to 0 at boundaries.
    let x =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 2.0], None)
            .unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![3.0_f64], (-0.5, 0.5));

    let y = convolve(&x, &kernel).unwrap();

    // Spot-check: at u = 0.5, the kernel window [0.0, 1.0] is fully inside x's support,
    // so y(0.5) = ∫_{0}^{1} 2 * 3 ds = 6.
    let val = eval(&y.as_view(), 0.5);
    assert!((val - 6.0).abs() < 1e-10, "y(0.5) = {val}, expected 6");

    // At u = -0.5 (left boundary of output), y = 0.
    let val_lo = eval(&y.as_view(), -0.5);
    assert!(val_lo.abs() < 1e-10, "y(-0.5) = {val_lo}, expected 0");

    // At u = 1.5 (right boundary), y = 0.
    let val_hi = eval(&y.as_view(), 1.5);
    assert!(val_hi.abs() < 1e-10, "y(1.5) = {val_hi}, expected 0");
}

#[test]
fn integrate_product_constant_input_constant_kernel_yields_linear_result() {
    // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
    // y(u) = ∫ x(s) w(u - s) ds, integration range = intersection of
    // [u - 0.5, u + 0.5] with [0, 1].
    // For u ∈ [0.5, 0.5] (single point), y = 2*3*1 = 6.
    // Generally y(u) = 6 * (length of overlap window).

    let x = crate::bezier::BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![2.0], // constant 2
    };
    let w = crate::bezier::BezierPiece::<f64> {
        u_start: -0.5,
        u_end: 0.5,
        coeffs: vec![3.0], // constant 3
    };

    // Integrate over output sub-interval [0.5, 1.0] where the kernel window
    // shrinks linearly (from full overlap to half overlap at u=1.0).
    let contribution = integrate_product_piece(&x, &w, 0.5, 1.0);

    // Expected: y(u) = 6 * (1.0 - (u - 0.5)) for u ∈ [0.5, 1.0]
    //                = 6 * (1.5 - u)
    //                = 9 - 6u
    // In Pascal-shifted basis at α = 0.5: y(u) = 9 - 6u = 9 - 6*(0.5 + (u - 0.5))
    //                                          = 6 - 6 * (u - 0.5)
    // So coeffs at u_start = 0.5 should be [6.0, -6.0].
    assert!((contribution.coeffs[0] - 6.0).abs() < 1e-10);
    assert!((contribution.coeffs[1] - (-6.0)).abs() < 1e-10);
}

#[test]
fn convolve_rejects_rational_input() {
    let curve = crate::ScalarNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0_f64, 1.0],
        Some(vec![1.0, 1.0]),
    )
    .unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));
    let result = convolve(&curve, &kernel);
    assert!(matches!(
        result,
        Err(AlgebraError::RationalNotSupported {
            operation: "convolve",
            ..
        })
    ));
}

#[test]
fn single_poly_kernel_constructs_one_piece() {
    let k = PiecewisePolynomialKernel::single_poly(vec![1.0, 0.5_f64], (-1.0, 1.0));
    assert_eq!(k.pieces.len(), 1);
    assert_eq!(k.pieces[0].u_start, -1.0);
    assert_eq!(k.pieces[0].u_end, 1.0);
    assert_eq!(k.pieces[0].coeffs, vec![1.0, 0.5]);
}

#[test]
fn kernel_support_returns_endpoints() {
    let k = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.5, 0.5));
    assert_eq!(k.support(), (-0.5, 0.5));
}

#[test]
fn knot_remove_redundant_simplifies_overproduct() {
    let a =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
            .unwrap();
    let b = a.clone();
    let mut c = multiply(&a, &b).unwrap();
    let initial_knot_count = c.knots().len();

    knot_remove_redundant(&mut c, 1e-10);

    // For a single-piece input, no interior knots to remove; result unchanged.
    assert_eq!(c.knots().len(), initial_knot_count);
    // Eval still correct.
    for u in [0.0, 0.5, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-10);
    }
}

#[test]
fn multiply_curves_with_different_interior_knots() {
    // a has interior knot at 0.4, b has interior knot at 0.7.
    let a = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
        None,
    )
    .unwrap();
    let b = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0],
        vec![1.0, 2.0, 0.0, 1.0],
        None,
    )
    .unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 4);
    for u in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn multiply_two_linear_curves_gives_quadratic() {
    // a(u) = u, b(u) = 2u + 1, expected c(u) = u(2u + 1) = 2u^2 + u.
    let a =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
            .unwrap();
    let b =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 3.0], None)
            .unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 2);
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn scalar_multiply_doubles_evaluation() {
    let curve =
        crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
    let doubled = scalar_multiply(&curve, 2.0_f64);
    assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
}

#[test]
fn scalar_multiply_preserves_weights() {
    let curve = crate::ScalarNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![1.0, 2.0],
        Some(vec![1.0, 1.0]),
    )
    .unwrap();
    let result = scalar_multiply(&curve, 3.0_f64);
    assert_eq!(result.weights().unwrap(), &[1.0, 1.0]);
    assert_eq!(result.control_points(), &[3.0, 6.0]);
}

#[test]
fn add_two_compatible_curves() {
    let a =
        crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
    let b =
        crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 3.0], None).unwrap();
    let sum = add(&a, &b).unwrap();
    // At u=0.5: 0.5 + 2.5 = 3.0
    assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12);
}

#[test]
fn add_rejects_mismatched_degree() {
    let a =
        crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
    let b = crate::ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.5, 1.0],
        None,
    )
    .unwrap();
    let result = add(&a, &b);
    assert!(matches!(result, Err(crate::AlgebraError::KnotMismatch)));
}

#[test]
fn multiply_rejects_rational_input() {
    let a = crate::ScalarNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 1.0],
        Some(vec![1.0, 1.0]),
    )
    .unwrap();
    let b = a.clone();
    let result = multiply(&a, &b);
    assert!(matches!(
        result,
        Err(crate::AlgebraError::RationalNotSupported {
            operation: "multiply",
            ..
        })
    ));
}

#[test]
fn from_pieces_accepts_contiguous_kernel() {
    let pieces = vec![
        crate::bezier::BezierPiece {
            u_start: -0.5,
            u_end: 0.0,
            coeffs: vec![1.0_f64],
        },
        crate::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![2.0_f64],
        },
    ];
    let k = PiecewisePolynomialKernel::from_pieces(pieces).unwrap();
    assert_eq!(k.pieces.len(), 2);
    assert_eq!(k.support(), (-0.5, 0.5));
}

#[test]
fn from_pieces_rejects_non_contiguous() {
    let pieces = vec![
        crate::bezier::BezierPiece {
            u_start: -0.5_f64,
            u_end: 0.0,
            coeffs: vec![1.0],
        },
        crate::bezier::BezierPiece {
            u_start: 0.1,
            u_end: 0.5,
            coeffs: vec![2.0],
        }, // gap
    ];
    let result = PiecewisePolynomialKernel::from_pieces(pieces);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
}

#[test]
fn from_pieces_rejects_empty() {
    let result = PiecewisePolynomialKernel::<f64>::from_pieces(vec![]);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
}

#[test]
fn pascal_shift_round_trip_preserves_polynomial() {
    // Cubic with non-zero shift — exercises both directions of basis conversion.
    let coeffs = vec![1.0, 2.0, 3.0, -1.5_f64];
    let shift = 0.7;
    let absolute = pascal_shift_to_absolute(&coeffs, shift);
    let back = absolute_to_pascal_shift(&absolute, shift);
    for i in 0..coeffs.len() {
        assert!(
            (back[i] - coeffs[i]).abs() < 1e-12,
            "coeff[{i}]: original {} != round-tripped {}",
            coeffs[i],
            back[i],
        );
    }
}

#[test]
fn single_poly_from_absolute_constructs_kernel_with_correct_polynomial() {
    // Absolute form: w(t) = 1 + 2t on [0.5, 1.5].
    // Pascal-shifted at u_start = 0.5: w(u) = 1 + 2*(u - 0.5 + 0.5) = 2 + 2*(u - 0.5).
    let k = PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![1.0_f64, 2.0], // 1 + 2t in absolute t
        (0.5, 1.5),
    );
    assert_eq!(k.pieces.len(), 1);
    assert_eq!(k.pieces[0].u_start, 0.5);
    assert_eq!(k.pieces[0].u_end, 1.5);
    // Pascal-shifted coeffs at u_start=0.5: c_0 = 1 + 2*0.5 = 2.0, c_1 = 2.0.
    assert!((k.pieces[0].coeffs[0] - 2.0).abs() < 1e-12);
    assert!((k.pieces[0].coeffs[1] - 2.0).abs() < 1e-12);
    // Polynomial value at t=0.5 (u=1.0) should be 1 + 2*0.5 = 2.0 in original basis.
    // In Pascal-shifted basis: c_0 + c_1*(1 - 0.5) = 2 + 2*0.5 = 3.0... wait.
    // Let me recheck: w_absolute(t) = 1 + 2t evaluated at t=1.0 = 3.0.
    // In Pascal-shifted basis with u_start=0.5: shifted polynomial is the same function,
    // i.e., w(u) = 1 + 2u (the absolute t IS the absolute u for a kernel),
    // expressed as Σ c_k * (u - 0.5)^k.
    // At u=1.0 (which is t=1.0 since t=u for a kernel): expected 1 + 2*1.0 = 3.0.
    // Pascal: c_0 + c_1*(1 - 0.5) = 2 + 2*0.5 = 3.0 ✓
    let val_at_one = k.pieces[0].evaluate(1.0);
    assert!((val_at_one - 3.0).abs() < 1e-12);
}

#[test]
fn single_poly_from_absolute_round_trips_via_evaluate() {
    // Quadratic kernel: w(t) = 1 - 2t + 3t^2, on [-0.5, 0.5].
    let k = PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![1.0_f64, -2.0, 3.0],
        (-0.5, 0.5),
    );
    // Sample at three points and confirm Pascal-shifted eval == absolute eval.
    for t in [-0.5_f64, 0.0, 0.25, 0.5] {
        let absolute_val = 1.0 - 2.0 * t + 3.0 * t * t;
        let pascal_val = k.pieces[0].evaluate(t);
        assert!(
            (absolute_val - pascal_val).abs() < 1e-12,
            "t={t}: absolute={absolute_val}, pascal={pascal_val}"
        );
    }
}

#[test]
fn multiply_regression_proptest_shrunk_failing_input() {
    // Captured from algebra_proptest::multiply_multi_piece_eval_matches_pointwise_product
    // pre-Fix-1 (Mørken-bounded knot removal). At u=0.1, b has C⁰ kink (m_b=1, d_b=1)
    // and a has interior multiplicity-1 knot (m_a=1, d_a=3). Per Mørken Eq. (1):
    // μ_target(0.1) = max(3+1, 1+1) = 4. Pre-Fix-1 the unbounded knot_remove_redundant
    // peeled below 4, smearing the C⁰ kink and producing wrong eval at u=0.1.
    let a = crate::ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.1, 0.55, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.181_828_016_839_598_23, 0.0, 0.0],
        None,
    )
    .unwrap();
    let b = crate::ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 0.1, 1.0, 1.0],
        vec![0.0, 4.267_190_258_636_853, 0.0],
        None,
    )
    .unwrap();
    let c = multiply(&a, &b).unwrap();
    // Pointwise product at u=0.1 should be ≈ 0.014107177131003477.
    // Pre-fix `multiply` returned ≈ 0.007758947422051913.
    let exp = eval(&a.as_view(), 0.1) * eval(&b.as_view(), 0.1);
    let got = eval(&c.as_view(), 0.1);
    assert!(
        (exp - got).abs() < 1e-10,
        "u=0.1: pointwise={exp}, multiply={got} (regression)"
    );
}

#[test]
fn multiply_quadratic_x_linear_gives_cubic() {
    // a(u) = u^2 (Bernstein cps [0, 0, 1] for monomial u^2 on [0, 1]).
    let a = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0],
        None,
    )
    .unwrap();
    // b(u) = u, same as before.
    let b =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
            .unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 3);
    // Expected: c(u) = u^3.
    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
        let exp = u * u * u;
        let got = eval(&c.as_view(), u);
        assert!(
            (exp - got).abs() < 1e-12,
            "u={u}: u^3={exp}, multiply={got}"
        );
    }
}

/// Diagnostic: convolve preserves the natural multiplicity at cross-sum
/// kink images, and produces correct eval, on a multi-piece input with a
/// C0 kink. This is the convolve analog of the multi-piece multiply
/// regression test from Fix 1 (Mørken-bounded knot removal).
///
/// Mathematical setup (per the convolution-continuity rule):
/// - `x` is degree 2 with interior knot at `u_x = 0.3` of multiplicity
///   `m_x = 2` (a C0 kink — slope jumps across `u = 0.3`).
/// - `w` is the constant kernel `1` on `[-0.1, 0.1]` (single-piece, no
///   interior knots, kernel degree `d_w = 0`).
/// - Output `y` has degree `p_y = d_x + d_w + 1 = 3`.
/// - The kink images in `y` are at `u = u_x ± t_w_endpoint = {0.2, 0.4}`.
///   Per the convolution-continuity rule, `μ_y` at each kink image equals
///   `m_x = 2` (input C0 → output `C^{0 + d_w + 1}` = C1 → `μ_y = p_y − 1 = 2`).
///
/// This test asserts BOTH:
///   (A) `μ_y(0.2) = μ_y(0.4) = 2` — the kink images are preserved at the
///       natural multiplicity, not over-peeled like multiply pre-Fix 1.
///   (B) `y(0.2)` and `y(0.4)` match closed-form integrals of `x` against
///       the constant kernel — the load-bearing eval check.
///
/// Note: this test deliberately does NOT assert `μ_y = 0` at the boundary
/// cross-sums `u ∈ {0.1, 0.9}`. The spec's convolution-continuity rule
/// predicts `μ_y = 0` there (no real continuity break), but in practice the
/// post-pass `knot_remove_redundant` (Tiller A5.8 with chord-error tol)
/// can only peel a knot when both polynomial pieces match as polynomial
/// expressions, not just as functions agreeing at the join. At a boundary
/// cross-sum the left and right pieces of `y` differ by `(u − u_break)^k`
/// terms that vanish at the join but not elsewhere, so Tiller refuses
/// removal even though geometrically the curve is C-infinity there. This
/// leaves extra multiplicity at boundary cross-sums; harmless (eval is
/// correct, downstream ops don't care) and not the bug class tested here.
#[test]
fn convolve_multi_piece_input_with_c0_kink_preserves_natural_multiplicity() {
    // x: degree 2, knots [0,0,0, 0.3,0.3, 1,1,1], asymmetric CPs.
    // Two pieces, sharing x(0.3) = 4 with a slope jump (C⁰ kink at 0.3).
    let x = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.3, 0.3, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 4.0, 0.5, 0.2],
        None,
    )
    .unwrap();
    // w(t) = 1 on [-0.1, 0.1] — single piece, no interior knots.
    let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));

    let y = convolve(&x, &kernel).unwrap();

    // Output degree = d_x + d_w + 1 = 2 + 0 + 1 = 3.
    assert_eq!(y.degree(), 3, "output degree");

    // (A) Multiplicity at kink images: μ_y(0.2) = μ_y(0.4) = m_x = 2.
    let p = y.degree() as usize;
    let interior = &y.knots()[p + 1..y.knots().len() - p - 1];
    let mult_at_02 = interior.iter().filter(|k| (**k - 0.2).abs() < 1e-9).count();
    let mult_at_04 = interior.iter().filter(|k| (**k - 0.4).abs() < 1e-9).count();
    assert_eq!(
        mult_at_02, 2,
        "expected μ_y(0.2) = m_x = 2 (kink image), got {mult_at_02}; full interior = {interior:?}",
    );
    assert_eq!(
        mult_at_04, 2,
        "expected μ_y(0.4) = m_x = 2 (kink image), got {mult_at_04}; full interior = {interior:?}",
    );

    // (B) Pointwise eval check vs hand-computed integrals.
    // x_1(s) = (20/3) s + (200/9) s² on [0, 0.3];
    // x_2(s) = 4 − 10 (s − 0.3) + (320/49) (s − 0.3)² on [0.3, 1.0].
    //   y(0.2) = ∫_{0.1}^{0.3} x_1(s) ds
    //          = (10/3)(0.09 − 0.01) + (200/27)(0.027 − 0.001)
    //          = 0.8/3 + 5.2/27.
    //   y(0.4) = ∫_{0.3}^{0.5} x_2(s) ds
    //          = 4·0.2 − 5·(0.2)² + (320/147)·(0.2)³
    //          = 0.6 + 2.56/147.
    let exp_02 = 0.8 / 3.0 + 5.2 / 27.0;
    let exp_04 = 0.6 + 2.56 / 147.0;
    let got_02 = eval(&y.as_view(), 0.2);
    let got_04 = eval(&y.as_view(), 0.4);
    assert!(
        (got_02 - exp_02).abs() < 1e-10,
        "y(0.2): expected {exp_02}, got {got_02}, diff {}",
        (got_02 - exp_02).abs(),
    );
    assert!(
        (got_04 - exp_04).abs() < 1e-10,
        "y(0.4): expected {exp_04}, got {got_04}, diff {}",
        (got_04 - exp_04).abs(),
    );

    // Also spot-check eval at a non-kink interior point — confirms the
    // entire shape (not just kink-image samples) matches the convolution
    // integral. At u = 0.5 the kernel window [0.4, 0.6] is fully inside
    // x's piece-2 support [0.3, 1.0], so:
    //   y(0.5) = ∫_{0.4}^{0.6} x_2(s) ds
    //          = ∫_{0.1}^{0.3} (4 − 10·v + (320/49)·v²) dv  (v = s − 0.3)
    //          = 4·0.2 − 5·(0.09 − 0.01) + (320/147)·(0.027 − 0.001)
    //          = 0.8 − 0.4 + (320·0.026)/147
    //          = 0.4 + 8.32/147.
    let exp_05 = 0.4 + 8.32 / 147.0;
    let got_05 = eval(&y.as_view(), 0.5);
    assert!(
        (got_05 - exp_05).abs() < 1e-10,
        "y(0.5): expected {exp_05}, got {got_05}, diff {}",
        (got_05 - exp_05).abs(),
    );
}

// ── add_with_knot_union tests ────────────────────────────────────────────

/// Identical knot vectors → fast path: delegates to `add`, no refine.
#[test]
fn add_with_knot_union_identical_knots_fast_path() {
    let a = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![0.0, 1.0],
        None,
    )
    .unwrap();
    let b = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![2.0, 3.0],
        None,
    )
    .unwrap();
    let sum = add_with_knot_union(&a, &b).unwrap();
    // At u=0: 0+2=2. At u=1: 1+3=4. At u=0.5 (midpoint): 0.5+2.5=3.
    assert!((eval(&sum.as_view(), 0.0_f64) - 2.0).abs() < 1e-12, "fast-path u=0");
    assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12, "fast-path u=0.5");
    assert!((eval(&sum.as_view(), 1.0_f64) - 4.0).abs() < 1e-12, "fast-path u=1");
}

/// Mismatched knot vectors → knot-union path: piece counts and eval values correct.
///
/// `a` has two pieces (linear 0→5 on [0,0.5] then 5→10 on [0.5,1]).
/// `b` has one piece (constant 20). After union, both are 2 pieces, and
/// the sum evaluates to 20→25→30.
#[test]
fn add_with_knot_union_mismatched_knots_union_path() {
    use crate::bezier::{BezierPiece, bezier_pieces_to_nurbs};

    // Two-piece linear curve on [0,1]: 0→5 then 5→10.
    // Pascal-shifted coefficients at u_start.
    let a = bezier_pieces_to_nurbs(&[
        BezierPiece::<f64> { u_start: 0.0, u_end: 0.5, coeffs: vec![0.0, 10.0] },
        BezierPiece::<f64> { u_start: 0.5, u_end: 1.0, coeffs: vec![5.0, 10.0] },
    ]);
    // Single-piece constant 20.
    let b = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![20.0, 20.0],
        None,
    )
    .unwrap();

    let sum = add_with_knot_union(&a, &b).unwrap();
    // Check at domain boundary and midpoint of each piece.
    let cases = [(0.0_f64, 20.0), (0.25, 22.5), (0.5, 25.0), (0.75, 27.5), (1.0, 30.0)];
    for (u, expected) in cases {
        let got = eval(&sum.as_view(), u);
        assert!(
            (got - expected).abs() < 1e-10,
            "union-path u={u}: expected {expected}, got {got}",
        );
    }
}

/// Degree mismatch → `KnotMismatch` error (same as `add`'s contract).
#[test]
fn add_with_knot_union_rejects_degree_mismatch() {
    let a = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![0.0, 1.0],
        None,
    )
    .unwrap();
    let b = crate::ScalarNurbs::try_new(
        2,
        vec![0.0_f64, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.5, 1.0],
        None,
    )
    .unwrap();
    let result = add_with_knot_union(&a, &b);
    assert!(
        matches!(result, Err(crate::AlgebraError::KnotMismatch)),
        "expected KnotMismatch, got {result:?}",
    );
}

/// Weighted curve → `NotImplemented` error (homogeneous lift deferred).
#[test]
fn add_with_knot_union_rejects_weighted_curves() {
    let a = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![0.0, 1.0],
        Some(vec![1.0, 2.0]),
    )
    .unwrap();
    let b = crate::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![0.0, 1.0],
        None,
    )
    .unwrap();
    let result = add_with_knot_union(&a, &b);
    assert!(
        matches!(result, Err(crate::AlgebraError::NotImplemented(_))),
        "expected NotImplemented for weighted operand, got {result:?}",
    );
    // Also: b weighted, a not.
    let result2 = add_with_knot_union(&b, &a);
    assert!(
        matches!(result2, Err(crate::AlgebraError::NotImplemented(_))),
        "expected NotImplemented for weighted operand (swapped), got {result2:?}",
    );
}
