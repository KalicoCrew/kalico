use super::*;

fn linear_e_nurbs(e_start: f64, e_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![e_start, e_end]).unwrap()
}

fn default_limits() -> ELimits {
    ELimits {
        v_max: 100.0,
        a_max: 5000.0,
    }
}

#[test]
fn e_duration_simple() {
    let e_nurbs = linear_e_nurbs(10.0, 5.0);
    let limits = default_limits();
    let duration = schedule_e_duration(&e_nurbs, 50.0, &limits);
    assert!(
        (duration - 0.11).abs() < 1e-10,
        "expected 0.11, got {duration}"
    );
}

#[test]
fn e_duration_triangular() {
    let e_nurbs = linear_e_nurbs(10.0, 9.9);
    let limits = default_limits();
    let duration = schedule_e_duration(&e_nurbs, 100.0, &limits);
    let expected = 2.0 * (0.1_f64 / 5000.0).sqrt();
    assert!(
        (duration - expected).abs() < 1e-10,
        "expected {expected}, got {duration}"
    );
}

#[test]
fn e_duration_zero_length() {
    let e_nurbs = linear_e_nurbs(5.0, 5.0);
    let limits = default_limits();
    let duration = schedule_e_duration(&e_nurbs, 50.0, &limits);
    assert!((duration - 0.0).abs() < 1e-15, "got {duration}");
}

#[test]
fn e_duration_capped_by_v_max() {
    let e_nurbs = linear_e_nurbs(0.0, 10.0);
    let limits = default_limits();
    let duration = schedule_e_duration(&e_nurbs, 200.0, &limits);
    assert!(
        (duration - 0.12).abs() < 1e-10,
        "expected 0.12, got {duration}"
    );
}

#[test]
fn e_full_trapezoidal_endpoints() {
    let e_nurbs = linear_e_nurbs(10.0, 5.0);
    let limits = default_limits();
    let t_start = 1.0;
    let result = schedule_e_full(&e_nurbs, 50.0, &limits, t_start).unwrap();

    let knots = result.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let val_start = nurbs_eval(&result.as_view(), u_start);
    let val_end = nurbs_eval(&result.as_view(), u_end);

    assert!(
        (val_start - 10.0).abs() < 1e-10,
        "start: expected 10.0, got {val_start}"
    );
    assert!(
        (val_end - 5.0).abs() < 1e-10,
        "end: expected 5.0, got {val_end}"
    );
}

#[test]
fn e_full_triangular_endpoints() {
    let e_nurbs = linear_e_nurbs(10.0, 9.9);
    let limits = default_limits();
    let t_start = 0.0;
    let result = schedule_e_full(&e_nurbs, 100.0, &limits, t_start).unwrap();

    let knots = result.knots();
    let val_start = nurbs_eval(&result.as_view(), knots[0]);
    let val_end = nurbs_eval(&result.as_view(), knots[knots.len() - 1]);

    assert!(
        (val_start - 10.0).abs() < 1e-10,
        "start: expected 10.0, got {val_start}"
    );
    assert!(
        (val_end - 9.9).abs() < 1e-10,
        "end: expected 9.9, got {val_end}"
    );
}

#[test]
fn e_full_monotone_retraction() {
    let e_nurbs = linear_e_nurbs(10.0, 5.0);
    let limits = default_limits();
    let result = schedule_e_full(&e_nurbs, 50.0, &limits, 0.0).unwrap();
    let knots = result.knots();
    let t0 = knots[0];
    let t_end = knots[knots.len() - 1];
    let n = 50;
    let mut prev = nurbs_eval(&result.as_view(), t0);
    for i in 1..=n {
        let t = t0 + (t_end - t0) * f64::from(i) / f64::from(n);
        let val = nurbs_eval(&result.as_view(), t);
        assert!(
            val <= prev + 1e-12,
            "non-monotone at sample {i}: prev={prev}, val={val}"
        );
        prev = val;
    }
}

#[test]
fn e_full_monotone_prime() {
    let e_nurbs = linear_e_nurbs(5.0, 10.0);
    let limits = default_limits();
    let result = schedule_e_full(&e_nurbs, 50.0, &limits, 0.0).unwrap();
    let knots = result.knots();
    let t0 = knots[0];
    let t_end = knots[knots.len() - 1];
    let n = 50;
    let mut prev = nurbs_eval(&result.as_view(), t0);
    for i in 1..=n {
        let t = t0 + (t_end - t0) * f64::from(i) / f64::from(n);
        let val = nurbs_eval(&result.as_view(), t);
        assert!(
            val >= prev - 1e-12,
            "non-monotone at sample {i}: prev={prev}, val={val}"
        );
        prev = val;
    }
}
