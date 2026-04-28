//! Junction velocity from curvature continuity. Per spec §2.2.

use crate::Limits;
use crate::multi::JunctionBindingCap;
use nurbs::VectorNurbs;
use nurbs::eval::{curvature_from_derivs, vector_derivative, vector_eval};

/// Numerical floor on κ — below this the centripetal cap is treated as ∞ and
/// we fall back to the JD sharp-corner sub-case. Per spec §2.2.
const KAPPA_FLOOR: f64 = 1e-12;

/// Numerical ceiling on `b = ṡ²` for "no centripetal cap" cases. Per spec §2.2
/// + matching `constraints.rs::B_MAX_CENT_CAP`. ~10⁴ mm/s.
const B_MAX_CENT_CAP: f64 = 1e8;

/// Threshold below which the JD branch returns ∞ (no corner cap). Per spec §2.2.
const ALPHA_COLLINEAR_THRESHOLD: f64 = 1e-3;

/// Threshold above which the JD branch caps `v_jd` at a small positive floor
/// (avoid exact-zero boundary conditions confusing downstream solver).
const ALPHA_REVERSAL_THRESHOLD: f64 = std::f64::consts::PI * 0.99;

/// Floor `v_jd` at this value at near-reversal junctions. Per spec §2.2.
const V_JD_REVERSAL_FLOOR_MM_S: f64 = 1.0;

pub(crate) struct JunctionResult {
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    pub kappa_left: f64,
    pub kappa_right: f64,
}

pub(crate) fn compute_junction_velocity(
    left: &VectorNurbs<f64, 3>,
    right: &VectorNurbs<f64, 3>,
    left_limits: &Limits,
    right_limits: &Limits,
    chord_tolerance_mm: f64,
) -> JunctionResult {
    let t_left = forward_unit_tangent_at_end(left);
    let t_right = forward_unit_tangent_at_start(right);

    let kappa_left = curvature_at_end(left);
    let kappa_right = curvature_at_start(right);

    // Cap 1: per-axis MVC from tangent direction at junction.
    // Since |t| = 1, |dx_axis/dt| = |t_axis| · |ṡ|, so |ṡ| ≤ v_max,axis / |t_axis|.
    // Apply on both sides; take the more-restrictive limits.
    let cap_per_axis = per_axis_velocity_cap(&t_left, left_limits)
        .min(per_axis_velocity_cap(&t_right, right_limits));

    // Cap 2: centripetal cap.
    let cap_centripetal =
        centripetal_cap(kappa_left, left_limits).min(centripetal_cap(kappa_right, right_limits));

    // Cap 3: sharp-corner JD when both sides are below the κ floor.
    let cap_sharp = if kappa_left.abs() <= KAPPA_FLOOR && kappa_right.abs() <= KAPPA_FLOOR {
        sharp_corner_jd_cap(&t_left, &t_right, left_limits, chord_tolerance_mm)
    } else {
        f64::INFINITY
    };

    // Cap 4: global per-axis v_max (each axis independently).
    let cap_v_max = left_limits
        .v_max
        .iter()
        .chain(right_limits.v_max.iter())
        .copied()
        .fold(f64::INFINITY, f64::min);

    // Take the minimum and tag which cap was binding.
    let (v, binding) = min_with_tag([
        (cap_per_axis, JunctionBindingCap::PerAxisVelocity),
        (cap_centripetal, JunctionBindingCap::Centripetal),
        (cap_sharp, JunctionBindingCap::SharpCornerChord),
        (cap_v_max, JunctionBindingCap::GlobalVMax),
    ]);

    JunctionResult {
        v_junction: v,
        binding_cap: binding,
        kappa_left,
        kappa_right,
    }
}

fn per_axis_velocity_cap(t: &[f64; 3], limits: &Limits) -> f64 {
    let mut cap = f64::INFINITY;
    for axis in 0..3 {
        let t_abs = t[axis].abs();
        if t_abs > 1e-12 {
            cap = cap.min(limits.v_max[axis] / t_abs);
        }
    }
    cap
}

fn centripetal_cap(kappa: f64, limits: &Limits) -> f64 {
    let k = kappa.abs();
    if k <= KAPPA_FLOOR {
        B_MAX_CENT_CAP.sqrt()
    } else {
        (limits.a_centripetal_max / k).sqrt()
    }
}

/// Sharp-corner JD cap. Per spec §2.2 sharp-corner sub-case.
///
/// Uses the deviation-angle convention (α = 0 collinear, α = π reversal) and
/// computes `cos(α/2)` directly via the half-angle identity to avoid the
/// `arccos(dot)`-then-`cos(α/2)` NaN trap in f64 (see spec §2.2 numerical-safety
/// note + `docs/research/junction-deviation-cornering-formula.md`).
fn sharp_corner_jd_cap(
    t_left: &[f64; 3],
    t_right: &[f64; 3],
    limits: &Limits,
    chord_tolerance_mm: f64,
) -> f64 {
    let dot =
        (t_left[0] * t_right[0] + t_left[1] * t_right[1] + t_left[2] * t_right[2]).clamp(-1.0, 1.0);

    // Half-angle identity: cos(α/2) = sqrt((1 + dot)/2). Always non-negative.
    let cos_half_alpha = ((1.0 + dot) * 0.5).max(0.0).sqrt();

    // Compute α only for the threshold checks (stable form via `asin` half-angle).
    let sin_half_alpha = ((1.0 - dot) * 0.5).max(0.0).sqrt();
    let alpha = 2.0 * sin_half_alpha.asin();

    if alpha <= ALPHA_COLLINEAR_THRESHOLD {
        return B_MAX_CENT_CAP.sqrt();
    }
    if alpha >= ALPHA_REVERSAL_THRESHOLD {
        return V_JD_REVERSAL_FLOOR_MM_S;
    }

    // v_jd² = a · δ · cos(α/2) / (1 − cos(α/2))
    let denom = 1.0 - cos_half_alpha;
    if denom <= 1e-15 {
        return B_MAX_CENT_CAP.sqrt();
    }
    (limits.a_centripetal_max * chord_tolerance_mm * cos_half_alpha / denom).sqrt()
}

fn min_with_tag(caps: [(f64, JunctionBindingCap); 4]) -> (f64, JunctionBindingCap) {
    let mut best = caps[0];
    for &(v, tag) in &caps[1..] {
        if v < best.0 {
            best = (v, tag);
        }
    }
    best
}

fn forward_unit_tangent_at_end(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_end = *curve.knots().last().expect("knots non-empty");
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_end);
    normalize_3(t)
}

fn forward_unit_tangent_at_start(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_start = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_start);
    normalize_3(t)
}

fn curvature_at_end(curve: &VectorNurbs<f64, 3>) -> f64 {
    if curve.degree() < 2 {
        // degree-1 NURBS (`G1` segment) has zero curvature everywhere.
        return 0.0;
    }
    let u_end = *curve.knots().last().expect("knots non-empty");
    let d1 = vector_derivative(curve);
    let d2 = vector_derivative(&d1);
    curvature_from_derivs(&d1, &d2, u_end)
}

fn curvature_at_start(curve: &VectorNurbs<f64, 3>) -> f64 {
    if curve.degree() < 2 {
        return 0.0;
    }
    let u_start = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let d2 = vector_derivative(&d1);
    curvature_from_derivs(&d1, &d2, u_start)
}

#[inline]
fn normalize_3(v: [f64; 3]) -> [f64; 3] {
    let m = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if m < 1e-12 {
        [0.0; 3]
    } else {
        [v[0] / m, v[1] / m, v[2] / m]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn textbook_limits() -> Limits {
        Limits {
            v_max: [500.0, 500.0, 500.0],
            a_max: [5_000.0, 5_000.0, 5_000.0],
            j_max: [100_000.0, 100_000.0, 100_000.0],
            a_centripetal_max: 2_500.0,
        }
    }

    #[test]
    fn jd_collinear_no_cap() {
        let t_x = [1.0, 0.0, 0.0];
        let cap = sharp_corner_jd_cap(&t_x, &t_x, &textbook_limits(), 0.05);
        // Collinear should give ∞ (or B_MAX_CENT_CAP.sqrt() = 10000 mm/s).
        assert!(
            cap >= 9999.9,
            "collinear should give ~10000 mm/s cap, got {cap}"
        );
    }

    #[test]
    fn jd_90_degree_corner_matches_klipper() {
        let t_x = [1.0, 0.0, 0.0];
        let t_y = [0.0, 1.0, 0.0];
        let limits = textbook_limits();
        // a · δ = 2500 · 0.05 = 125. v² = 125 · 2.414 = 301.75. v = 17.37 mm/s.
        let cap = sharp_corner_jd_cap(&t_x, &t_y, &limits, 0.05);
        let expected = (limits.a_centripetal_max * 0.05 * 2.414_213_562).sqrt();
        assert!(
            (cap - expected).abs() < 0.05,
            "90° JD: got {cap}, expected ~{expected}",
        );
    }

    #[test]
    fn compute_junction_velocity_g1_to_g1_90deg() {
        let left = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        let right = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]],
            None,
        )
        .unwrap();
        let limits = textbook_limits();
        let result = compute_junction_velocity(&left, &right, &limits, &limits, 0.05);
        let expected = (limits.a_centripetal_max * 0.05 * 2.414_213_562).sqrt();
        assert!(
            (result.v_junction - expected).abs() < 0.05,
            "got {}, expected ~{}",
            result.v_junction,
            expected
        );
        assert!(matches!(
            result.binding_cap,
            JunctionBindingCap::SharpCornerChord
        ));
    }
}
