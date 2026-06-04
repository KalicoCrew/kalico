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

    let kernel_coeffs: Vec<f64> = v["kernel_coeffs"]
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
    let curve = nurbs::ScalarNurbs::try_new(2, vec![0.0, 0.0, 0.0, t_end, t_end, t_end], bernstein)
        .unwrap();

    let kernel = nurbs::algebra::PiecewisePolynomialKernel::single_poly_from_absolute(
        kernel_coeffs,
        (-t_sm / 2.0, t_sm / 2.0),
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
