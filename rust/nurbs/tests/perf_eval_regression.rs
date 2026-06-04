//! Performance-regression tests for the MCU-targeted hot-path NURBS
//! evaluators (`eval_polynomial`, `eval_derivative`).
//!
//! These run on the host (cargo's standard `cargo test`) — they cannot
//! check absolute MCU cycle counts because the host's `Duration::elapsed`
//! is wall-clock-granular and the host CPU is enormously faster than the
//! H7. What they CAN catch is *relative* algorithmic regressions: if
//! someone reintroduces the 14.7-KB stack zero-init in
//! `scalar_derivative_eval`, the windowed `eval_derivative` will collapse
//! to near-baseline speed and these ratios will drop below threshold.
//!
//! The thresholds are deliberately conservative (windowed must be at
//! least 3× faster than the materialized form) to avoid spurious failures
//! on CI runners with a noisy thermal envelope. The on-MCU advantage is
//! ~10-30× per the H7 measurements; if this test ever drops below 3× on
//! the host, the on-MCU regression is much worse.
//!
//! Per `CLAUDE.md`'s "we cannot ship a measurably slower trajectory"
//! constraint, these tests act as a circuit-breaker on the per-tick eval
//! path — the planner's trajectory optimality is meaningless if the MCU
//! evaluator can't keep up at modulation rate.

use nurbs::{ScalarNurbs, eval};
use std::time::Instant;

/// Build a synthetic post-shape-like curve: degree 5, ~30 control points,
/// non-uniform knot vector. Mirrors the shape of curves that smooth-ZV
/// pre-bake produces from a single G5 cubic Bézier piece.
fn synthetic_postshape_curve() -> ScalarNurbs<f64> {
    let degree = 5_u8;
    let n_cps = 30;
    let p = degree as usize;

    // Clamped open knots: degree+1 zeros, interior, degree+1 ones.
    let mut knots = Vec::with_capacity(n_cps + p + 1);
    knots.resize(p + 1, 0.0_f64);
    let n_interior = n_cps - p - 1;
    for i in 1..=n_interior {
        knots.push(i as f64 / (n_interior + 1) as f64);
    }
    knots.resize(knots.len() + p + 1, 1.0_f64);

    // Smoothly varying cps — anything non-degenerate works for timing.
    let cps: Vec<f64> = (0..n_cps)
        .map(|i| {
            let t = i as f64 / (n_cps - 1) as f64;
            10.0 * t + 5.0 * (t * std::f64::consts::PI).sin()
        })
        .collect();

    ScalarNurbs::try_new(degree, knots, cps, None).unwrap()
}

/// Single-piece cubic Bézier (the live G5/G1 input shape pre-shape).
fn cubic_bezier_curve() -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
        None,
    )
    .unwrap()
}

const ITERATIONS: usize = 200_000;

#[test]
fn eval_derivative_windowed_at_least_3x_faster_than_materialized() {
    let curve = synthetic_postshape_curve();

    // Pre-build the materialized degree-lowered curve once for the
    // "reference" path (`derivative` is host-only, we'd never call it
    // per-tick on the MCU). The per-tick cost we're protecting against
    // is the BUILD cost on the MCU side, which the windowed form skips
    // entirely.
    //
    // To make the comparison fair against the MCU's situation, we
    // simulate "rebuild on every call" by including `derivative()` in
    // the reference loop body. That's exactly what the prior
    // `scalar_derivative_eval` did per tick.
    let mut sink = 0.0_f64;
    let baseline = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            let lowered = eval::derivative(&curve);
            sink += eval::eval(&lowered.as_view(), u);
        }
        start.elapsed()
    };

    let windowed = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            sink += eval::eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
        }
        start.elapsed()
    };

    // Touch sink to keep the optimizer honest.
    assert!(sink.is_finite(), "sink={sink}");

    let ratio = baseline.as_nanos() as f64 / windowed.as_nanos().max(1) as f64;
    eprintln!(
        "eval_derivative perf: baseline={baseline:?}, windowed={windowed:?}, ratio={ratio:.2}x"
    );
    assert!(
        ratio >= 3.0,
        "windowed eval_derivative regressed: only {ratio:.2}x faster than \
         materialized derivative+eval (expected ≥3×). Did someone reintroduce \
         the [0.0; MAX_CONTROL_POINTS] stack zero-init in scalar_derivative_eval?"
    );
}

#[test]
fn eval_polynomial_at_least_as_fast_as_eval_for_validated_curves() {
    // For pre-validated curves the polynomial fast path should never be
    // slower than going through `ScalarNurbsRef::try_new` + `eval`. It
    // skips the O(n) knot-monotonicity sweep that `validate()` does on
    // every `try_new` call.
    let curve = synthetic_postshape_curve();

    let mut sink = 0.0_f64;
    let via_eval = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            let view = nurbs::ScalarNurbsRef::<f64>::try_new(
                curve.degree(),
                curve.knots(),
                curve.control_points(),
                None,
            )
            .unwrap();
            sink += eval::eval(&view, u);
        }
        start.elapsed()
    };

    let via_polynomial = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            sink += eval::eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
        }
        start.elapsed()
    };

    assert!(sink.is_finite(), "sink={sink}");

    let ratio = via_eval.as_nanos() as f64 / via_polynomial.as_nanos().max(1) as f64;
    eprintln!(
        "eval_polynomial perf: via_eval(+try_new)={via_eval:?}, via_polynomial={via_polynomial:?}, ratio={ratio:.2}x"
    );
    assert!(
        ratio >= 1.3,
        "eval_polynomial regressed: only {ratio:.2}x faster than \
         try_new+eval (expected ≥1.3×). Did `validate()` get short-circuited \
         on the via_eval side, or did eval_polynomial pick up a hidden cost?"
    );
}

#[test]
fn eval_polynomial_with_derivative_at_least_1_3x_faster_than_separate_calls() {
    // Combined `eval_polynomial_with_derivative` shares find_knot_span +
    // d-array init + the full de Boor pyramid between the value and
    // derivative recurrences. On the H7 (degree 9, 82 cps, 92 knots) this
    // collapses 2 + 3 per-tick eval/derivative calls into 2 combined
    // calls, dropping motion-tick avg from ~28 us to ~20 us.
    //
    // On host the combined recurrence has more arithmetic ops than the
    // separate-pass form (it carries a parallel `dd` recurrence on top of
    // `d`), but better ILP + halved find_knot_span/init overhead +
    // halved function-call overhead net out faster. Threshold is
    // conservative for noisy CI hosts.
    let curve = synthetic_postshape_curve();

    let mut sink = 0.0_f64;
    let separate = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            let v = eval::eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
            let d = eval::eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
            sink += v + d;
        }
        start.elapsed()
    };

    let combined = {
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let u = (i as f64) / (ITERATIONS as f64);
            let (v, d) = eval::eval_polynomial_with_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            sink += v + d;
        }
        start.elapsed()
    };

    assert!(sink.is_finite(), "sink={sink}");

    let ratio = separate.as_nanos() as f64 / combined.as_nanos().max(1) as f64;
    eprintln!(
        "combined-eval perf: separate={separate:?}, combined={combined:?}, ratio={ratio:.2}x"
    );
    assert!(
        ratio >= 1.3,
        "combined eval+derivative regressed: only {ratio:.2}x faster than \
         separate eval_polynomial + eval_derivative (expected ≥1.3×). \
         Did the d/dd parallel recurrence get split into separate passes?"
    );
}

#[test]
fn eval_polynomial_with_derivative_matches_separate_calls_bitwise() {
    // Bit-exact agreement (within 1e-12) between combined and separate
    // forms. Guards against subtle algebraic divergence.
    for curve in [synthetic_postshape_curve(), cubic_bezier_curve()] {
        for i in 0..=200 {
            let u = f64::from(i) / 200.0;
            let (v_combined, d_combined) = eval::eval_polynomial_with_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            let v_sep =
                eval::eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
            let d_sep =
                eval::eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
            assert!(
                (v_combined - v_sep).abs() < 1e-12,
                "u={u}: combined value {v_combined} vs separate {v_sep}"
            );
            assert!(
                (d_combined - d_sep).abs() < 1e-12,
                "u={u}: combined deriv {d_combined} vs separate {d_sep}"
            );
        }
    }
}

#[test]
fn eval_polynomial_matches_eval_bitwise_for_polynomial_curves() {
    // Bit-exact agreement between fast-path and validated path — guards
    // against any future refactor that diverges the two evaluators.
    for curve in [synthetic_postshape_curve(), cubic_bezier_curve()] {
        let view = curve.as_view();
        for i in 0..=200 {
            let u = f64::from(i) / 200.0;
            let v_eval = eval::eval(&view, u);
            let v_poly =
                eval::eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
            assert_eq!(
                v_eval.to_bits(),
                v_poly.to_bits(),
                "u={u}: eval={v_eval} vs eval_polynomial={v_poly}"
            );
        }
    }
}
