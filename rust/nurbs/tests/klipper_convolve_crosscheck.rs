//! Cross-check our convolve against scipy/Klipper-derived numerical reference.
//! Reference file: `tests/data/klipper_smooth_zv_reference.json`.
//!
//! Note on file location: the plan called for `rust/tests/`, but the workspace
//! `Cargo.toml` is workspace-only (no `[lib]`/`[[bin]]`), so there is no
//! workspace-level test crate. This integration test lives in `rust/nurbs/tests/`
//! instead.

#![cfg(feature = "host")]

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-4; // numerical-quadrature reference, not exact

/// Convert absolute-monomial coefficients (Σ `a_n` * u^n) to Pascal-shifted
/// coefficients (Σ `c_k` * (u - shift)^k). Mirrors the algebra crate's internal
/// helper; needed because the JSON stores the kernel in absolute form
/// (the natural Klipper / `init_smoother` convention) while
/// `PiecewisePolynomialKernel::single_poly` expects Pascal-shifted-at-`u_start`.
fn absolute_to_pascal_shift(absolute: &[f64], shift: f64) -> Vec<f64> {
    let d = absolute.len() - 1;
    let mut out = vec![0.0; d + 1];
    let mut shift_pow = vec![1.0; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = binomial(n, k) as f64;
            out[k] += absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut acc: u64 = 1;
    for i in 0..k {
        acc = acc * (n - i) as u64 / (i + 1) as u64;
    }
    acc
}

#[test]
fn convolve_matches_scipy_reference_for_smooth_zv_kernel() {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "tests",
        "data",
        "klipper_smooth_zv_reference.json",
    ]
    .iter()
    .collect();
    let raw = fs::read_to_string(&path).expect("reference file must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    let kernel_coeffs_abs: Vec<f64> = v["kernel_coeffs"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| c.as_f64().unwrap())
        .collect();
    let t_sm = v["kernel_t_sm"].as_f64().unwrap();
    let accel = v["input_accel"].as_f64().unwrap();
    let t_end = v["input_t_end"].as_f64().unwrap();

    // Build the input as a quadratic NURBS: x(t) = 0.5 * a * t^2 on [0, t_end].
    // In Pascal-shifted-at-u_start=0 monomial basis, this is simply [0, 0, 0.5*a].
    let mono = nurbs::BezierPiece::<f64> {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0, 0.0, 0.5 * accel],
    };
    let bernstein = mono.to_bernstein();
    let curve =
        nurbs::ScalarNurbs::try_new(2, vec![0.0, 0.0, 0.0, t_end, t_end, t_end], bernstein, None)
            .unwrap();

    // Kernel coeffs are absolute monomial around t=0; convert to the
    // Pascal-shifted basis used by PiecewisePolynomialKernel (shift = u_start = -t_sm/2).
    let half = t_sm / 2.0;
    let kernel_coeffs_shifted = absolute_to_pascal_shift(&kernel_coeffs_abs, -half);
    let kernel = nurbs::algebra::PiecewisePolynomialKernel::single_poly(
        kernel_coeffs_shifted,
        (-half, half),
    );
    let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();

    for sample in v["samples"].as_array().unwrap() {
        let t = sample["T"].as_f64().unwrap();
        let expected = sample["value"].as_f64().unwrap();
        let got = nurbs::eval::eval(&y.as_view(), t);
        let diff = (got - expected).abs();
        assert!(
            diff < TOLERANCE,
            "T={t}: got {got}, expected {expected} (diff {diff})"
        );
    }
}
