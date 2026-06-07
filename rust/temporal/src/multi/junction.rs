use crate::Limits;
use crate::multi::JunctionBindingCap;
use nurbs::VectorNurbs;
use nurbs::eval::{curvature_from_derivs, vector_derivative, vector_eval};

const KAPPA_FLOOR: f64 = 1e-12;
const B_MAX_CENT_CAP: f64 = 1e8;
const ALPHA_COLLINEAR_THRESHOLD: f64 = 1e-3;
const ALPHA_REVERSAL_THRESHOLD: f64 = std::f64::consts::PI * 0.99;
const V_JD_REVERSAL_FLOOR_MM_S: f64 = 1.0;

/// Fuse threshold: junctions with tangent disagreement at or below this are
/// chain-fused (treated G1-continuous). At 1000 mm/s a 1e-3 rad kink is a
/// ~1 mm/s lateral step — far inside the scv impulse budget.
const THETA_FUSE_RAD: f64 = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JunctionKind {
    Smooth,
    Corner,
}

pub(crate) struct JunctionResult {
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    pub kappa_left: f64,
    pub kappa_right: f64,
    pub kind: JunctionKind,
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

    let kind = classify_junction(&t_left, &t_right);

    let kappa_left = curvature_at_end(left);
    let kappa_right = curvature_at_start(right);

    let cap_per_axis = per_axis_velocity_cap(&t_left, left_limits)
        .min(per_axis_velocity_cap(&t_right, right_limits));

    let cap_centripetal =
        centripetal_cap(kappa_left, left_limits).min(centripetal_cap(kappa_right, right_limits));

    let cap_sharp = if kappa_left.abs() <= KAPPA_FLOOR && kappa_right.abs() <= KAPPA_FLOOR {
        sharp_corner_jd_cap(&t_left, &t_right, left_limits, chord_tolerance_mm)
    } else {
        f64::INFINITY
    };

    let cap_v_max = left_limits
        .v_max
        .iter()
        .chain(right_limits.v_max.iter())
        .copied()
        .fold(f64::INFINITY, f64::min);

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
        kind,
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

fn classify_junction(t_left: &[f64; 3], t_right: &[f64; 3]) -> JunctionKind {
    let left_degenerate = t_left[0].abs() + t_left[1].abs() + t_left[2].abs() < 1e-12;
    let right_degenerate = t_right[0].abs() + t_right[1].abs() + t_right[2].abs() < 1e-12;
    if left_degenerate || right_degenerate {
        return JunctionKind::Corner;
    }
    if turn_angle(t_left, t_right) <= THETA_FUSE_RAD {
        JunctionKind::Smooth
    } else {
        JunctionKind::Corner
    }
}

fn turn_angle(t_left: &[f64; 3], t_right: &[f64; 3]) -> f64 {
    let dot = (t_left[0] * t_right[0] + t_left[1] * t_right[1] + t_left[2] * t_right[2])
        .clamp(-1.0, 1.0);
    let sin_half = ((1.0 - dot) * 0.5).max(0.0).sqrt();
    2.0 * sin_half.asin()
}

fn centripetal_cap(kappa: f64, limits: &Limits) -> f64 {
    let k = kappa.abs();
    if k <= KAPPA_FLOOR {
        B_MAX_CENT_CAP.sqrt()
    } else {
        (limits.a_centripetal_max / k).sqrt()
    }
}

/// Sharp-corner JD cap.
///
/// Uses `cos(α/2) = sqrt((1 + dot)/2)` (half-angle identity) to avoid the
/// `arccos(dot)`-then-`cos(α/2)` NaN trap in f64.
fn sharp_corner_jd_cap(
    t_left: &[f64; 3],
    t_right: &[f64; 3],
    limits: &Limits,
    chord_tolerance_mm: f64,
) -> f64 {
    let dot =
        (t_left[0] * t_right[0] + t_left[1] * t_right[1] + t_left[2] * t_right[2]).clamp(-1.0, 1.0);

    let cos_half_alpha = ((1.0 + dot) * 0.5).max(0.0).sqrt();
    let alpha = turn_angle(t_left, t_right);

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
        // degree-1 NURBS has zero curvature everywhere.
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
mod tests;
