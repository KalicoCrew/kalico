use super::*;

#[test]
fn smooth_zv_kernel_is_normalized() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let (lo, hi) = kernel.support();
    // Simpson's rule integration
    let n = 1000;
    let step = (hi - lo) / f64::from(n);
    let mut integral = 0.0;
    for i in 0..=n {
        let t = lo + f64::from(i) * step;
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
    assert!((integral - 1.0).abs() < 1e-6, "integral = {integral}");
}

#[test]
fn smooth_mzv_kernel_is_normalized() {
    let kernel = build_smooth_mzv_kernel(0.95625 / 120.0);
    let (lo, hi) = kernel.support();
    let n = 1000;
    let step = (hi - lo) / f64::from(n);
    let mut integral = 0.0;
    for i in 0..=n {
        let t = lo + f64::from(i) * step;
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
    assert!((integral - 1.0).abs() < 1e-6, "integral = {integral}");
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
        let t = lo + (hi - lo) * f64::from(i) / f64::from(n);
        assert!(kernel.pieces[0].evaluate(t) > 0.0, "negative at t={t}");
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
