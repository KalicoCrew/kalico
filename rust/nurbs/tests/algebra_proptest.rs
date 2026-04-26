//! Property-based tests for NURBS algebra primitives.
//! These exercise random inputs and check structural invariants.

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
            nurbs::ScalarNurbs::try_new(p, knots, cps_vec, None).unwrap()
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
}
