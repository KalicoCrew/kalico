use super::*;

#[test]
fn b_dd_weights_exact_on_quadratic_nonuniform() {
    // b(s) = 3s² − 2s + 1 → b″ = 6 everywhere, any spacing.
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0;
    let (hl, hr) = (0.3, 0.7);
    let s_i = 1.0;
    let w = b_dd_weights(hl, hr);
    let approx = w[0] * b(s_i - hl) + w[1] * b(s_i) + w[2] * b(s_i + hr);
    assert!((approx - 6.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn b_d_weights_exact_on_quadratic_nonuniform() {
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0; // b′(1) = 4
    let (hl, hr) = (0.3, 0.7);
    let w = b_d_weights(hl, hr);
    let approx = w[0] * b(1.0 - hl) + w[1] * b(1.0) + w[2] * b(1.0 + hr);
    assert!((approx - 4.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn weights_reduce_to_uniform() {
    let h = 0.5;
    let wd = b_d_weights(h, h);
    assert!((wd[0] - (-1.0 / (2.0 * h))).abs() < 1e-12);
    assert!(wd[1].abs() < 1e-12);
    assert!((wd[2] - 1.0 / (2.0 * h)).abs() < 1e-12);
    let wdd = b_dd_weights(h, h);
    assert!((wdd[0] - 1.0 / (h * h)).abs() < 1e-12);
    assert!((wdd[1] - (-2.0 / (h * h))).abs() < 1e-12);
    assert!((wdd[2] - 1.0 / (h * h)).abs() < 1e-12);
}
