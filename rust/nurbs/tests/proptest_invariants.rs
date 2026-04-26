//! Property-based tests for NURBS evaluation invariants.
//!
//! These hold by construction — eval at first knot returns first cp,
//! derivative of constant is zero, etc. Catches regressions after refactors
//! that the fixed corpus oracle wouldn't.

#![cfg(feature = "host")]

use proptest::prelude::*;

fn arb_degree() -> impl Strategy<Value = u8> {
    1u8..=5
}

fn arb_cp_count(degree: u8) -> impl Strategy<Value = usize> {
    let min = (degree as usize) + 1;
    min..=10
}

fn arb_curve() -> impl Strategy<Value = nurbs::ScalarNurbs<f64>> {
    arb_degree().prop_flat_map(|p| {
        arb_cp_count(p).prop_flat_map(move |n| {
            let cps = prop::collection::vec(-10.0..10.0_f64, n);
            cps.prop_map(move |cps_vec| {
                // Build a clamped uniform knot vector.
                let pad = p as usize + 1;
                let interior = n.saturating_sub(p as usize + 1);
                let mut knots = Vec::with_capacity(2 * pad + interior);
                knots.resize(pad, 0.0);
                for i in 1..=interior {
                    knots.push(i as f64 / (interior + 1) as f64);
                }
                knots.resize(knots.len() + pad, 1.0);
                nurbs::ScalarNurbs::try_new(p, knots, cps_vec, None).unwrap()
            })
        })
    })
}

proptest! {
    #[test]
    fn eval_at_first_knot_returns_first_cp(curve in arb_curve()) {
        let view = curve.as_view();
        let u_start = view.knots()[0];
        let result = nurbs::eval::eval(&view, u_start);
        let expected = view.control_points()[0];
        prop_assert!((result - expected).abs() < 1e-9, "got {result}, expected {expected}");
    }

    #[test]
    fn eval_at_last_knot_returns_last_cp(curve in arb_curve()) {
        let view = curve.as_view();
        let u_end = view.knots()[view.knots().len() - 1];
        let result = nurbs::eval::eval(&view, u_end);
        let expected = view.control_points()[view.control_points().len() - 1];
        prop_assert!((result - expected).abs() < 1e-9, "got {result}, expected {expected}");
    }

    #[test]
    fn derivative_of_constant_curve_is_zero(p in 1u8..=5) {
        let n = (p as usize) + 1;
        let cps = vec![2.5_f64; n];
        let pad = p as usize + 1;
        let interior = n.saturating_sub(p as usize + 1);
        let mut knots = Vec::with_capacity(2 * pad + interior);
        knots.resize(pad, 0.0);
        for i in 1..=interior {
            knots.push(i as f64 / (interior + 1) as f64);
        }
        knots.resize(knots.len() + pad, 1.0);
        let curve = nurbs::ScalarNurbs::try_new(p, knots, cps, None).unwrap();
        let d = nurbs::eval::derivative(&curve);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let val = nurbs::eval::eval(&d.as_view(), u);
            prop_assert!(val.abs() < 1e-9, "constant curve derivative at {u} = {val}");
        }
    }
}
