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
