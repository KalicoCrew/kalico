//! Planner configuration types. Parsed from klippy's `printer.cfg` values
//! passed through PyO3 at bridge init and runtime updates.

use temporal::Limits;
use thiserror::Error;
use trajectory::{AxisShaper, ELimits, RequiredShaper, ShaperConfig};

/// Errors returned by `parse_required_shaper` and `build_shaper_config`.
#[derive(Debug, Error)]
pub enum ShaperConfigError {
    #[error(
        "shaper frequency must be finite and > 0 Hz, got {value}; \
         check shaper_freq_x / shaper_freq_y in printer.cfg \
         (sim configs commonly set 0 to disable shaping — there is no passthrough \
         for X/Y today; use a real frequency, e.g. 50)"
    )]
    InvalidFrequency { value: f64 },

    #[error("unsupported shaper type for MVP: '{kind}'. Use smooth_zv or smooth_mzv")]
    UnsupportedKind { kind: String },
}

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
pub fn parse_required_shaper(name: &str, freq: f64) -> Result<RequiredShaper, ShaperConfigError> {
    if !freq.is_finite() || freq <= 0.0 {
        return Err(ShaperConfigError::InvalidFrequency { value: freq });
    }
    match name {
        "smooth_zv" | "smooth-zv" => Ok(RequiredShaper::SmoothZv { frequency_hz: freq }),
        "smooth_mzv" | "smooth-mzv" => Ok(RequiredShaper::SmoothMzv { frequency_hz: freq }),
        other => Err(ShaperConfigError::UnsupportedKind {
            kind: other.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests;
