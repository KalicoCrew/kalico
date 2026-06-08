use temporal::Limits;
use thiserror::Error;
use trajectory::{AxisShaper, ELimits, ShaperConfig};

#[derive(Debug, Error)]
pub enum ShaperConfigError {
    #[error("unsupported shaper type: '{kind}'. Use smooth_zv or smooth_mzv")]
    UnsupportedKind { kind: String },
}

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

#[derive(Debug, Clone, Copy)]
pub struct PlannerLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
}

impl PlannerLimits {
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
                x: AxisShaper::Passthrough,
                y: AxisShaper::Passthrough,
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

/// Parses an axis shaper from a type name and frequency.
///
/// - `smooth_zv` / `smooth-zv` with finite `freq > 0` → `SmoothZv`
/// - `smooth_mzv` / `smooth-mzv` with finite `freq > 0` → `SmoothMzv`
/// - `""` / `"none"` / `"passthrough"` (any freq), or any type with `freq ≤ 0` / non-finite → `Passthrough`
/// - Any other non-empty type string with finite `freq > 0` → `Err(UnsupportedKind)`
pub fn parse_axis_shaper(name: &str, freq: f64) -> Result<AxisShaper, ShaperConfigError> {
    match name {
        "" | "none" | "passthrough" => return Ok(AxisShaper::Passthrough),
        _ => {}
    }

    if !freq.is_finite() || freq <= 0.0 {
        return Ok(AxisShaper::Passthrough);
    }

    match name {
        "smooth_zv" | "smooth-zv" => Ok(AxisShaper::SmoothZv { frequency_hz: freq }),
        "smooth_mzv" | "smooth-mzv" => Ok(AxisShaper::SmoothMzv { frequency_hz: freq }),
        other => Err(ShaperConfigError::UnsupportedKind {
            kind: other.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests;
