//! Per-MCU curve-size validation.
//!
//! Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §5.3.

use crate::dispatch::McuCaps;
use kalico_host_rt::producer::CurveLoadParams;
use nurbs::ScalarNurbs;

/// True if the curve fits within the caps reported by the destination MCU.
pub fn fits(caps: &McuCaps, curve: &ScalarNurbs<f64>) -> bool {
    curve.control_points().len() as u32 <= caps.max_control_points
        && curve.knots().len() as u32 <= caps.max_knot_vector_len
        && curve.degree() <= caps.max_degree
}

/// True if a `CurveLoadParams` payload fits the destination MCU's caps.
/// Mirrors the dispatch-time check in `bridge.rs`; both must agree on what
/// "fits" means.
pub fn fits_curve_load(caps: &McuCaps, curve: &CurveLoadParams) -> bool {
    curve.cps_f32.len() as u32 <= caps.max_control_points
        && curve.knots_f32.len() as u32 <= caps.max_knot_vector_len
        && curve.degree <= caps.max_degree
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::McuCaps;

    fn small_caps() -> McuCaps {
        McuCaps {
            max_control_points: 64,
            max_knot_vector_len: 76,
            max_degree: 10,
            curve_pool_n: 4,
        }
    }

    fn clamped_uniform_knots(n_cps: usize, degree: u8) -> Vec<f64> {
        // Clamped knot vector with len = n_cps + degree + 1.
        let p = degree as usize;
        let n_knots = n_cps + p + 1;
        let n_interior = n_knots - 2 * (p + 1);
        let mut knots = Vec::with_capacity(n_knots);
        for _ in 0..=p {
            knots.push(0.0);
        }
        for i in 1..=n_interior {
            knots.push(i as f64 / (n_interior as f64 + 1.0));
        }
        for _ in 0..=p {
            knots.push(1.0);
        }
        knots
    }

    #[test]
    fn small_curve_fits_small_caps() {
        // 8-piece cubic-equivalent: 25 cps, 29-knot clamped vector.
        let n_cps = 25;
        let cps = vec![0.0_f64; n_cps];
        let knots = clamped_uniform_knots(n_cps, 3);
        let curve = ScalarNurbs::try_new(3, knots, cps, None).unwrap();
        assert!(fits(&small_caps(), &curve));
    }

    #[test]
    fn curve_load_params_over_cap_does_not_fit() {
        let caps = small_caps();
        let too_big = CurveLoadParams {
            degree: 3,
            knots_f32: vec![0.0_f32; 104], // > 76
            cps_f32: vec![0.0_f32; 100],   // > 64
        };
        assert!(!fits_curve_load(&caps, &too_big));

        let just_right = CurveLoadParams {
            degree: 3,
            knots_f32: vec![0.0_f32; 29],
            cps_f32: vec![0.0_f32; 25],
        };
        assert!(fits_curve_load(&caps, &just_right));
    }

    #[test]
    fn oversize_curve_does_not_fit() {
        // 301 cps cubic clamped vector — exceeds 64-cps cap.
        let n_cps = 301;
        let cps = vec![0.0_f64; n_cps];
        let knots = clamped_uniform_knots(n_cps, 3);
        let curve = ScalarNurbs::try_new(3, knots, cps, None).unwrap();
        assert!(!fits(&small_caps(), &curve));
    }
}
