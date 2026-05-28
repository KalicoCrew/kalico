// Shaper kernel construction: smooth_zv and smooth_mzv polynomial kernels.
//
// Both shapers use the same bell-shaped degree-4 polynomial kernel
//     w(t) = c · (h² − t²)²
// on compact support [−h, h], where h = T_sm / 2.  The normalization constant
// c = 15 / (16 h⁵) ensures unit integral.  Only T_sm differs between the two
// shaper families:
//     smooth_zv:  T_sm = 0.8025  / f
//     smooth_mzv: T_sm = 0.95625 / f

use nurbs::algebra::PiecewisePolynomialKernel;

/// Build the unit-integral bell kernel `w(t) = c·(h²−t²)²` on `[−h, h]`
/// where `h = t_sm / 2` and `c = 15/(16·h⁵)`.
fn build_bell_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    // w(t) = c·(h² − t²)² = c·h⁴ − 2c·h²·t² + c·t⁴
    // Absolute monomial basis: coeffs[k] is the coefficient of t^k.
    let coeffs = vec![
        c * h.powi(4),    // t^0
        0.0,              // t^1
        -2.0 * c * h * h, // t^2
        0.0,              // t^3
        c,                // t^4
    ];
    PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h, h))
}

/// Build a smooth-ZV kernel with total support width `t_sm`.
pub fn build_smooth_zv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

/// Build a smooth-MZV kernel with total support width `t_sm`.
pub fn build_smooth_mzv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

// ---------------------------------------------------------------------------
// Convenience methods on the config enums
// ---------------------------------------------------------------------------

impl crate::RequiredShaper {
    /// Produce the `PiecewisePolynomialKernel` for this shaper configuration.
    pub fn to_kernel(&self) -> PiecewisePolynomialKernel<f64> {
        match self {
            Self::SmoothZv { frequency_hz } => build_smooth_zv_kernel(0.8025 / frequency_hz),
            Self::SmoothMzv { frequency_hz } => build_smooth_mzv_kernel(0.95625 / frequency_hz),
        }
    }
}

impl crate::AxisShaper {
    /// Produce the `PiecewisePolynomialKernel` for this shaper configuration,
    /// or `None` for `Passthrough`.
    pub fn to_kernel(&self) -> Option<PiecewisePolynomialKernel<f64>> {
        match self {
            Self::SmoothZv { frequency_hz } => Some(build_smooth_zv_kernel(0.8025 / frequency_hz)),
            Self::SmoothMzv { frequency_hz } => {
                Some(build_smooth_mzv_kernel(0.95625 / frequency_hz))
            }
            Self::Passthrough => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;
