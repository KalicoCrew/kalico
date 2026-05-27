use super::*;

/// Build a simple linear E NURBS from `e_start` to `e_end` in `[0, 1]`.
fn linear_e_nurbs(e_start: f64, e_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![e_start, e_end], None).unwrap()
}

fn default_limits() -> ELimits {
    ELimits {
        v_max: 100.0,
        a_max: 5000.0,
    }
}

#[test]
fn e_duration_simple() {
    // 5mm retraction at 50mm/s with a_max=5000 mm/s^2.
    // v_cruise = 50 mm/s (feedrate < v_max).
    // t_ramp = 50/5000 = 0.01 s.
    // s_ramp = 0.5 * 5000 * 0.01^2 = 0.25 mm.
    // 2 * s_ramp = 0.5 mm < 5 mm -> trapezoidal.
    // s_cruise = 5 - 0.5 = 4.5 mm.
    // t_cruise = 4.5 / 50 = 0.09 s.
    // Total = 2*0.01 + 0.09 = 0.11 s.
    let e_nurbs = linear_e_nurbs(10.0, 5.0); // 5mm retraction (negative direction)
    let limits = default_limits();
    let duration = schedule_e_duration(&e_nurbs, 50.0, &limits);
    assert!(
        (duration - 0.11).abs() < 1e-10,
        "expected 0.11, got {duration}"
    );
}

#[test]
fn e_duration_triangular() {
    // Very short retraction that can't reach cruise speed.
    // 0.1mm at 100mm/s with a_max=5000 mm/s^2.
    // t_ramp_full = 100/5000 = 0.02 s.
    // s_ramp_full = 0.5 * 5000 * 0.02^2 = 1.0 mm.
    // 2 * s_ramp_full = 2.0 mm > 0.1 mm -> triangular.
    // t_ramp_tri = sqrt(0.1 / 5000) = sqrt(2e-5) ≈ 0.004472 s.
    // Total = 2 * t_ramp_tri ≈ 0.008944 s.
    let e_nurbs = linear_e_nurbs(10.0, 9.9); // 0.1mm retraction
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
    // Feedrate 200mm/s but v_max is 100mm/s — should use 100.
    // 10mm at 100mm/s with a_max=5000.
    // t_ramp = 100/5000 = 0.02 s.
    // s_ramp = 0.5 * 5000 * 0.0004 = 1.0 mm.
    // s_cruise = 10 - 2 = 8 mm.
    // t_cruise = 8/100 = 0.08 s.
    // Total = 0.04 + 0.08 = 0.12 s.
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
    // Build full E NURBS and verify it hits the right start/end positions.
    let e_nurbs = linear_e_nurbs(10.0, 5.0); // 5mm retraction
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
    // Triangular profile: 0.1mm retraction.
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
    // A 5mm retraction should be monotonically decreasing.
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
    // A 5mm prime (positive direction) should be monotonically increasing.
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
