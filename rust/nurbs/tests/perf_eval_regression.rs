use nurbs::{ScalarNurbs, eval};
use std::time::Instant;

fn synthetic_postshape_curve() -> ScalarNurbs<f64> {
    let degree = 5_u8;
    let n_cps = 30;
    let p = degree as usize;

    let mut knots = Vec::with_capacity(n_cps + p + 1);
    knots.resize(p + 1, 0.0_f64);
    let n_interior = n_cps - p - 1;
    for i in 1..=n_interior {
        knots.push(i as f64 / (n_interior + 1) as f64);
    }
    knots.resize(knots.len() + p + 1, 1.0_f64);

    let cps: Vec<f64> = (0..n_cps)
        .map(|i| {
            let t = i as f64 / (n_cps - 1) as f64;
            10.0 * t + 5.0 * (t * std::f64::consts::PI).sin()
        })
        .collect();

    ScalarNurbs::try_new(degree, knots, cps).unwrap()
}

fn cubic_bezier_curve() -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap()
}

const ITERATIONS: usize = 200_000;

#[test]
fn eval_derivative_windowed_at_least_3x_faster_than_materialized() {
    let curve = synthetic_postshape_curve();

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
