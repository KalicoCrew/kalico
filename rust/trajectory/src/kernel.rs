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
        c * h.powi(4), // t^0
        0.0,           // t^1
        -2.0 * c * h * h, // t^2
        0.0,           // t^3
        c,             // t^4
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
            Self::SmoothZv { frequency_hz } => {
                build_smooth_zv_kernel(0.8025 / frequency_hz)
            }
            Self::SmoothMzv { frequency_hz } => {
                build_smooth_mzv_kernel(0.95625 / frequency_hz)
            }
        }
    }
}

impl crate::AxisShaper {
    /// Produce the `PiecewisePolynomialKernel` for this shaper configuration,
    /// or `None` for `Passthrough`.
    pub fn to_kernel(&self) -> Option<PiecewisePolynomialKernel<f64>> {
        match self {
            Self::SmoothZv { frequency_hz } => {
                Some(build_smooth_zv_kernel(0.8025 / frequency_hz))
            }
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
mod tests {
    use super::*;

    #[test]
    fn smooth_zv_kernel_is_normalized() {
        let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
        let (lo, hi) = kernel.support();
        // Simpson's rule integration
        let n = 1000;
        let step = (hi - lo) / n as f64;
        let mut integral = 0.0;
        for i in 0..=n {
            let t = lo + i as f64 * step;
            let w = if i == 0 || i == n {
                1.0
            } else if i % 2 == 0 {
                2.0
            } else {
                4.0
            };
            integral += w * kernel.pieces[0].evaluate(t);
        }
        integral *= step / 3.0;
        assert!(
            (integral - 1.0).abs() < 1e-6,
            "integral = {integral}"
        );
    }

    #[test]
    fn smooth_mzv_kernel_is_normalized() {
        let kernel = build_smooth_mzv_kernel(0.95625 / 120.0);
        let (lo, hi) = kernel.support();
        let n = 1000;
        let step = (hi - lo) / n as f64;
        let mut integral = 0.0;
        for i in 0..=n {
            let t = lo + i as f64 * step;
            let w = if i == 0 || i == n {
                1.0
            } else if i % 2 == 0 {
                2.0
            } else {
                4.0
            };
            integral += w * kernel.pieces[0].evaluate(t);
        }
        integral *= step / 3.0;
        assert!(
            (integral - 1.0).abs() < 1e-6,
            "integral = {integral}"
        );
    }

    #[test]
    fn kernel_vanishes_at_boundaries() {
        let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
        let (lo, hi) = kernel.support();
        assert!(kernel.pieces[0].evaluate(lo).abs() < 1e-12);
        assert!(kernel.pieces[0].evaluate(hi).abs() < 1e-12);
    }

    #[test]
    fn kernel_derivative_vanishes_at_boundaries() {
        let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
        let (lo, hi) = kernel.support();
        let dk = kernel.pieces[0].differentiate();
        // The lo boundary evaluates at the shifted-basis origin (exact zero).
        // The hi boundary evaluates at s = 2h where large-magnitude terms
        // cancel, so floating-point error is O(eps * |max_term|) ≈ 1e-8.
        assert!(dk.evaluate(lo).abs() < 1e-10, "lo = {}", dk.evaluate(lo));
        assert!(dk.evaluate(hi).abs() < 1e-8, "hi = {}", dk.evaluate(hi));
    }

    #[test]
    fn kernel_is_positive_inside() {
        let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
        let (lo, hi) = kernel.support();
        let n = 100;
        for i in 1..n {
            let t = lo + (hi - lo) * i as f64 / n as f64;
            assert!(
                kernel.pieces[0].evaluate(t) > 0.0,
                "negative at t={t}"
            );
        }
    }

    #[test]
    fn kernel_peak_at_center() {
        let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
        let center_val = kernel.pieces[0].evaluate(0.0);
        let off_center = kernel.pieces[0].evaluate(0.001);
        assert!(center_val > off_center);
    }

    #[test]
    fn smooth_zv_support_width() {
        let f = 150.0;
        let kernel = crate::RequiredShaper::SmoothZv { frequency_hz: f }.to_kernel();
        let (lo, hi) = kernel.support();
        let expected_t_sm = 0.8025 / f;
        assert!((hi - lo - expected_t_sm).abs() < 1e-12);
    }

    #[test]
    fn smooth_mzv_support_width() {
        let f = 120.0;
        let kernel = crate::RequiredShaper::SmoothMzv { frequency_hz: f }.to_kernel();
        let (lo, hi) = kernel.support();
        let expected_t_sm = 0.95625 / f;
        assert!((hi - lo - expected_t_sm).abs() < 1e-12);
    }
}
