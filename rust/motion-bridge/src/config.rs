//! Planner configuration types. Parsed from klippy's `printer.cfg` values
//! passed through PyO3 at bridge init and runtime updates.

use temporal::Limits;
use trajectory::{AxisShaper, ELimits, RequiredShaper, ShaperConfig};

/// Full planner configuration snapshot.
#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub limits: PlannerLimits,
    pub shaper: ShaperConfig,
    pub e_limits: ELimits,
    pub window_capacity: usize,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub fit_tolerance_mm: f64,
    pub worker_threads: usize,
}

/// Dynamic velocity/acceleration limits (updateable at runtime).
#[derive(Debug, Clone, Copy)]
pub struct PlannerLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
}

impl PlannerLimits {
    /// Convert to temporal's `Limits` struct.
    ///
    /// Jerk is set to 2× accel as a reasonable default; the β-medium loop
    /// further constrains accel based on post-shape peak.
    pub fn to_temporal_limits(&self) -> Limits {
        Limits::new(
            [self.max_velocity, self.max_velocity, self.max_z_velocity],
            [self.max_accel, self.max_accel, self.max_z_accel],
            [
                self.max_accel * 2.0,
                self.max_accel * 2.0,
                self.max_z_accel * 2.0,
            ],
            self.square_corner_velocity.powi(2) / (self.max_accel * 0.5),
        )
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            limits: PlannerLimits {
                max_velocity: 300.0,
                max_accel: 3000.0,
                max_z_velocity: 15.0,
                max_z_accel: 100.0,
                square_corner_velocity: 5.0,
            },
            shaper: ShaperConfig {
                x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                z: AxisShaper::Passthrough,
            },
            e_limits: ELimits {
                v_max: 50.0,
                a_max: 5000.0,
            },
            window_capacity: 32,
            beta_max_iters: 10,
            beta_convergence_ratio: 0.05,
            fit_tolerance_mm: 0.005,
            worker_threads: 3,
        }
    }
}

/// Parse a shaper type string into a `RequiredShaper`.
pub fn parse_required_shaper(name: &str, freq: f64) -> Result<RequiredShaper, String> {
    if !freq.is_finite() || freq <= 0.0 {
        return Err(format!(
            "shaper frequency must be finite and > 0 Hz, got {freq}; \
             check shaper_freq_x / shaper_freq_y in printer.cfg \
             (sim configs commonly set 0 to disable shaping — there is no passthrough \
             for X/Y today; use a real frequency, e.g. 50)"
        ));
    }
    match name {
        "smooth_zv" | "smooth-zv" => Ok(RequiredShaper::SmoothZv { frequency_hz: freq }),
        "smooth_mzv" | "smooth-mzv" => Ok(RequiredShaper::SmoothMzv { frequency_hz: freq }),
        other => Err(format!(
            "unsupported shaper type for MVP: '{other}'. Use smooth_zv or smooth_mzv"
        )),
    }
}

#[cfg(test)]
mod tests {
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

        // freq=0 must be rejected with an error mentioning the field name
        let err = parse_required_shaper("smooth_zv", 0.0).unwrap_err();
        assert!(err.contains("shaper_freq"), "error must name the field, got: {err}");

        // negative freq rejected
        assert!(parse_required_shaper("smooth_mzv", -1.0).is_err());

        // NaN/Inf rejected
        assert!(parse_required_shaper("smooth_zv", f64::NAN).is_err());
        assert!(parse_required_shaper("smooth_zv", f64::INFINITY).is_err());
    }
}
