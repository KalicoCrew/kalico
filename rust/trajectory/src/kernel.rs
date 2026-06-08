use nurbs::algebra::PiecewisePolynomialKernel;

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

pub fn build_smooth_zv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

pub fn build_smooth_mzv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

impl crate::AxisShaper {
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

#[cfg(test)]
mod tests;
