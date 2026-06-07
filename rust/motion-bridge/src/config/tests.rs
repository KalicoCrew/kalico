use super::*;

#[test]
fn default_config_has_sensible_values() {
    let c = PlannerConfig::default();
    assert_eq!(c.window_capacity, 32);
    assert_eq!(c.beta_max_iters, 10);
}

#[test]
fn temporal_limits_converts() {
    let l = PlannerLimits {
        max_velocity: 300.0,
        max_accel: 3000.0,
        max_z_velocity: 15.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
    };
    let tl = l.to_temporal_limits();
    assert_eq!(tl.v_max[0], 300.0);
    assert_eq!(tl.v_max[2], 15.0);
    assert_eq!(tl.a_max[0], 3000.0);
    assert_eq!(tl.a_centripetal_max, 3000.0);
}

#[test]
fn junction_deviation_mm_matches_klipper_formula() {
    let l = PlannerLimits {
        max_velocity: 300.0,
        max_accel: 3000.0,
        max_z_velocity: 15.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
    };
    let jd = l.junction_deviation_mm();
    let expected = 5.0_f64.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / 3000.0;
    assert!(
        (jd - expected).abs() < 1e-12,
        "junction_deviation_mm = {jd:.6e}, expected {expected:.6e}",
    );
    assert!(
        (jd - 3.4518e-3).abs() < 1e-6,
        "junction_deviation_mm = {jd:.6e}, expected ≈ 3.4518e-3 mm",
    );
}

#[test]
fn a_centripetal_max_is_max_accel_not_length() {
    let l = PlannerLimits {
        max_velocity: 300.0,
        max_accel: 3000.0,
        max_z_velocity: 15.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
    };
    let tl = l.to_temporal_limits();
    assert_eq!(
        tl.a_centripetal_max, 3000.0,
        "a_centripetal_max must equal max_accel (mm/s²), not the old scv²/accel formula (mm)",
    );
}

#[test]
fn parse_shaper_types() {
    assert!(matches!(
        parse_required_shaper("smooth_mzv", 50.0),
        Ok(RequiredShaper::SmoothMzv { frequency_hz }) if (frequency_hz - 50.0).abs() < 1e-9
    ));
    assert!(parse_required_shaper("ei", 50.0).is_err());

    let err = parse_required_shaper("smooth_zv", 0.0)
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("shaper_freq"),
        "error must name the field, got: {err}"
    );

    assert!(parse_required_shaper("smooth_mzv", -1.0).is_err());

    assert!(parse_required_shaper("smooth_zv", f64::NAN).is_err());
    assert!(parse_required_shaper("smooth_zv", f64::INFINITY).is_err());
}
