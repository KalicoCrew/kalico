#![cfg(feature = "host")]

use proptest::prelude::*;

fn arb_degree() -> impl Strategy<Value = u8> {
    1u8..=4
}

fn arb_simple_polynomial_curve() -> impl Strategy<Value = nurbs::ScalarNurbs<f64>> {
    arb_degree().prop_flat_map(|p| {
        let n = p as usize + 1;
        let cps = prop::collection::vec(-5.0..5.0_f64, n);
        cps.prop_map(move |cps_vec| {
            let pad = p as usize + 1;
            let mut knots = vec![0.0; pad];
            knots.extend(vec![1.0; pad]);
            nurbs::ScalarNurbs::try_new(p, knots, cps_vec).unwrap()
        })
    })
}

fn arb_multi_piece_curve() -> impl Strategy<Value = nurbs::ScalarNurbs<f64>> {
    // Generates curves with 1-2 interior knots, well-separated (>0.1 apart) to
    // avoid coincident-knot collisions during proptest shrinking.
    (1u8..=4, 1usize..=2).prop_flat_map(|(p, num_interior)| {
        let pad = p as usize + 1;
        let n = p as usize + 1 + num_interior;
        let cps = prop::collection::vec(-5.0..5.0_f64, n);
        let interiors: proptest::strategy::BoxedStrategy<Vec<f64>> = match num_interior {
            1 => (0.1..0.9_f64).prop_map(|k| vec![k]).boxed(),
            2 => (0.1..0.45_f64, 0.55..0.9_f64)
                .prop_map(|(a, b)| vec![a, b])
                .boxed(),
            _ => unreachable!(),
        };
        (cps, interiors).prop_map(move |(cps_vec, ints)| {
            let mut knots = vec![0.0; pad];
            knots.extend(ints);
            knots.extend(vec![1.0; pad]);
            nurbs::ScalarNurbs::try_new(p, knots, cps_vec).unwrap()
        })
    })
}

fn arb_single_poly_kernel() -> impl Strategy<Value = nurbs::algebra::PiecewisePolynomialKernel<f64>>
{
    (1usize..=4, 0.05..0.4_f64).prop_map(|(d, half)| {
        let coeffs: Vec<f64> = (0..=d).map(|i| (i as f64 + 1.0) * 0.5).collect();
        nurbs::algebra::PiecewisePolynomialKernel::single_poly(coeffs, (-half, half))
    })
}

fn arb_curve_with_existing_interior_multiplicity()
-> impl Strategy<Value = (nurbs::ScalarNurbs<f64>, f64, usize, usize)> {
    // Generates (curve, u_knot, p, existing_mult_at_u) with TWO interior knots:
    // the target knot at `existing_mult_at_u >= 1` plus one other. Without the
    // second knot the local CP polygon is constrained by uniform surrounding knots
    // and the A5.3 bug produces the right eval by accident. Constraint
    // `existing <= p - 2` ensures r_max >= 2, the regime that triggered A5.3.
    (3u8..=4, 0.1..0.45_f64, 0.55..0.9_f64, prop::bool::ANY).prop_flat_map(|(p, ka, kb, swap)| {
        let (u_knot, other_knot) = if swap { (ka, kb) } else { (kb, ka) };
        let existing_strategy: BoxedStrategy<usize> = (1usize..=(p as usize - 2)).boxed();
        existing_strategy.prop_flat_map(move |existing| {
            let n = p as usize + 2 + existing;
            let pad = p as usize + 1;
            prop::collection::vec(-3.0..3.0_f64, n).prop_map(move |cps| {
                let mut knots = vec![0.0; pad];
                let (lo_val, lo_mult, hi_val, hi_mult) = if u_knot < other_knot {
                    (u_knot, existing, other_knot, 1)
                } else {
                    (other_knot, 1, u_knot, existing)
                };
                for _ in 0..lo_mult {
                    knots.push(lo_val);
                }
                for _ in 0..hi_mult {
                    knots.push(hi_val);
                }
                knots.extend(vec![1.0; pad]);
                let curve = nurbs::ScalarNurbs::try_new(p, knots, cps).unwrap();
                (curve, u_knot, p as usize, existing)
            })
        })
    })
}

fn arb_curve_with_c0_kink() -> impl Strategy<Value = (nurbs::ScalarNurbs<f64>, f64)> {
    (1u8..=3, 0.1..0.9_f64).prop_flat_map(|(p, kink)| {
        let pad = p as usize + 1;
        let n = 2 * (p as usize) + 1;
        let cps = prop::collection::vec(-3.0..3.0_f64, n);
        cps.prop_map(move |cps_vec| {
            let mut knots = vec![0.0; pad];
            for _ in 0..p {
                knots.push(kink);
            }
            knots.extend(vec![1.0; pad]);
            let curve = nurbs::ScalarNurbs::try_new(p, knots, cps_vec).unwrap();
            (curve, kink)
        })
    })
}

proptest! {
    #[test]
    fn insert_knot_preserves_evaluation(
        curve in arb_simple_polynomial_curve(),
        u in 0.01..0.99_f64,
    ) {
        let inserted = nurbs::knot::insert_knot(&curve, u, 1).unwrap();
        for sample_u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let before = nurbs::eval::eval(&curve.as_view(), sample_u);
            let after = nurbs::eval::eval(&inserted.as_view(), sample_u);
            prop_assert!(
                (before - after).abs() < 1e-9,
                "u={sample_u}: before={before}, after={after}"
            );
        }
    }

    #[test]
    fn insert_knot_multifold_preserves_evaluation(
        (curve, u, p, existing) in arb_curve_with_existing_interior_multiplicity(),
    ) {
        let r_max = p - existing;
        debug_assert!(r_max >= 2);
        for r in 1..=r_max {
            let inserted = nurbs::knot::insert_knot(&curve, u, r).unwrap();
            let mut samples = vec![0.0_f64, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, u];
            samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
            for sample_u in samples {
                let before = nurbs::eval::eval(&curve.as_view(), sample_u);
                let after = nurbs::eval::eval(&inserted.as_view(), sample_u);
                prop_assert!(
                    (before - after).abs() < 1e-9,
                    "p={p}, existing={existing}, r={r}, u={sample_u}: before={before}, after={after}"
                );
            }
        }
    }

    #[test]
    fn multiply_degree_equals_sum(
        a in arb_simple_polynomial_curve(),
        b in arb_simple_polynomial_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        prop_assert_eq!(c.degree(), a.degree() + b.degree());
    }

    #[test]
    fn multiply_eval_matches_pointwise_product(
        a in arb_simple_polynomial_curve(),
        b in arb_simple_polynomial_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        for u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let exp = nurbs::eval::eval(&a.as_view(), u) * nurbs::eval::eval(&b.as_view(), u);
            let got = nurbs::eval::eval(&c.as_view(), u);
            prop_assert!(
                (exp - got).abs() < 1e-9,
                "u={u}: a*b={exp}, multiply={got}"
            );
        }
    }

    #[test]
    fn multiply_multi_piece_degree_equals_sum(
        a in arb_multi_piece_curve(),
        b in arb_multi_piece_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        prop_assert_eq!(c.degree(), a.degree() + b.degree());
    }

    #[test]
    fn multiply_multi_piece_eval_matches_pointwise_product(
        a in arb_multi_piece_curve(),
        b in arb_multi_piece_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        for u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let exp = nurbs::eval::eval(&a.as_view(), u) * nurbs::eval::eval(&b.as_view(), u);
            let got = nurbs::eval::eval(&c.as_view(), u);
            prop_assert!(
                (exp - got).abs() < 1e-9,
                "u={u}: a*b={exp}, multiply={got}"
            );
        }
    }
}

proptest! {
    #[test]
    fn convolve_degree_equals_input_plus_kernel_plus_one(
        curve in arb_simple_polynomial_curve(),
        kernel in arb_single_poly_kernel(),
    ) {
        let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();
        let expected = curve.degree() as usize + kernel.pieces[0].degree() + 1;
        prop_assert_eq!(y.degree() as usize, expected);
    }

    #[test]
    fn convolve_support_is_minkowski_sum(
        curve in arb_simple_polynomial_curve(),
        kernel in arb_single_poly_kernel(),
    ) {
        let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();
        let (k_lo, k_hi) = kernel.support();
        let expected_lo = curve.knots()[0] + k_lo;
        let expected_hi = curve.knots()[curve.knots().len() - 1] + k_hi;
        prop_assert!((y.knots()[0] - expected_lo).abs() < 1e-12);
        prop_assert!((y.knots()[y.knots().len() - 1] - expected_hi).abs() < 1e-12);
    }

    #[test]
    fn convolve_multi_piece_input_support_is_minkowski_sum(
        curve in arb_multi_piece_curve(),
        kernel in arb_single_poly_kernel(),
    ) {
        let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();
        let (k_lo, k_hi) = kernel.support();
        let expected_lo = curve.knots()[0] + k_lo;
        let expected_hi = curve.knots()[curve.knots().len() - 1] + k_hi;
        prop_assert!((y.knots()[0] - expected_lo).abs() < 1e-12);
        prop_assert!((y.knots()[y.knots().len() - 1] - expected_hi).abs() < 1e-12);
    }
}

proptest! {
    // Knot values propagate bit-exact through insertion / Bezier extraction in
    // our pipeline; the exact equality check is intentional structural verification.
    #[test]
    #[allow(clippy::float_cmp)]
    fn multiply_product_has_morken_multiplicities_at_kinks(
        (a, a_kink) in arb_curve_with_c0_kink(),
        (b, b_kink) in arb_curve_with_c0_kink(),
    ) {
        prop_assume!((a_kink - b_kink).abs() > 1e-6);

        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        let p = c.degree() as usize;

        let mult_at_a_kink = c.knots().iter().filter(|k| **k == a_kink).count();
        let mult_at_b_kink = c.knots().iter().filter(|k| **k == b_kink).count();
        prop_assert_eq!(mult_at_a_kink, p, "a_kink={}: expected μ={}, got {}", a_kink, p, mult_at_a_kink);
        prop_assert_eq!(mult_at_b_kink, p, "b_kink={}: expected μ={}, got {}", b_kink, p, mult_at_b_kink);

        let interior = &c.knots()[p + 1 .. c.knots().len() - p - 1];
        for &k in interior {
            prop_assert!(
                (k - a_kink).abs() < 1e-12 || (k - b_kink).abs() < 1e-12,
                "spurious interior knot at u={} (expected only kinks at {}, {})", k, a_kink, b_kink
            );
        }

        for u in [a_kink, b_kink] {
            let exp = nurbs::eval::eval(&a.as_view(), u) * nurbs::eval::eval(&b.as_view(), u);
            let got = nurbs::eval::eval(&c.as_view(), u);
            prop_assert!(
                (exp - got).abs() < 1e-9,
                "u={}: pointwise={}, multiply={}", u, exp, got
            );
        }
    }
}
