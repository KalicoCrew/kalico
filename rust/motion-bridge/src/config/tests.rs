use super::*;

#[test]
fn default_config_has_sensible_values() {
    let c = PlannerConfig::default();
    assert_eq!(c.window_capacity, 32);
    assert_eq!(c.beta_max_iters, 10);
}

#[test]
fn default_config_shaper_is_passthrough() {
    let c = PlannerConfig::default();
    assert!(matches!(c.shaper.x, AxisShaper::Passthrough));
    assert!(matches!(c.shaper.y, AxisShaper::Passthrough));
    assert!(matches!(c.shaper.z, AxisShaper::Passthrough));
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
        parse_axis_shaper("smooth_mzv", 50.0),
        Ok(AxisShaper::SmoothMzv { frequency_hz }) if (frequency_hz - 50.0).abs() < 1e-9
    ));
    assert!(parse_axis_shaper("smooth_zv", 50.0).is_ok());
    assert!(parse_axis_shaper("ei", 50.0).is_err());

    // freq ≤ 0 or non-finite → Passthrough, not an error
    assert!(matches!(
        parse_axis_shaper("smooth_zv", 0.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_mzv", -1.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_zv", f64::NAN),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_zv", f64::INFINITY),
        Ok(AxisShaper::Passthrough)
    ));
}

#[test]
fn parse_explicit_passthrough_names() {
    assert!(matches!(
        parse_axis_shaper("", 0.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("none", 50.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("passthrough", 50.0),
        Ok(AxisShaper::Passthrough)
    ));
}
