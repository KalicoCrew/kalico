//! Cross-precision regression harness. Runs the same curves at f32 and f64,
//! asserts the f32 result is within a documented bound of f64 on a representative
//! corpus. Catches numerical regressions in the f32 codegen path that the
//! geomdl oracle (f64 only) would miss.
//!
//! The bound is empirical — measure on the corpus, assert. If the bound creeps
//! up after a refactor, you've introduced a precision regression.

#![cfg(feature = "host")]

const F32_VS_F64_TOLERANCE: f32 = 1e-5;

#[test]
fn vector_eval_f32_matches_f64_within_tolerance() {
    // Cubic 3D curve.
    let degree = 3u8;
    let knots_f64 = vec![0.0_f64, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let cps_f64: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 0.5],
        [3.0, 2.0, 1.0],
        [4.0, 0.0, 0.0],
    ];
    let curve_f64 =
        nurbs::VectorNurbs::<f64, 3>::try_new(degree, knots_f64.clone(), cps_f64.clone()).unwrap();

    let knots_f32: Vec<f32> = knots_f64.iter().map(|&x| x as f32).collect();
    let cps_f32: Vec<[f32; 3]> = cps_f64
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    let curve_f32 = nurbs::VectorNurbs::<f32, 3>::try_new(degree, knots_f32, cps_f32).unwrap();

    for u in [0.0_f64, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
        let p64 = nurbs::eval::vector_eval(&curve_f64.as_view(), u);
        let p32 = nurbs::eval::vector_eval(&curve_f32.as_view(), u as f32);
        for axis in 0..3 {
            let diff = (p32[axis] - p64[axis] as f32).abs();
            assert!(
                diff < F32_VS_F64_TOLERANCE,
                "u={u} axis={axis}: f32={} f64={} diff={diff}",
                p32[axis],
                p64[axis],
            );
        }
    }
}
