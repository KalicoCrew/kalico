//! Cubic Bezier in monomial form for fast per-sample evaluation.
//!
//! Bernstein form (stored in pieces from the host) is convenient for
//! geometric reasoning but slow to evaluate. Monomial form (Horner) is
//! ~3x faster for position+velocity. We convert once per piece-load
//! and cache the result in `BezierPieceMonomial`.

/// Cubic Bezier piece in monomial form: P(t) = c0 + c1·t + c2·t² + c3·t³.
/// Velocity coefficients pre-baked: V(t) = vc0 + vc1·t + vc2·t².
#[derive(Clone, Copy, Debug)]
pub struct BezierPieceMonomial {
    pub coeffs: [f32; 4],      // c0, c1, c2, c3 for position
    pub vel_coeffs: [f32; 3],  // vc0=c1, vc1=2·c2, vc2=3·c3
    pub duration: f32,          // seconds in this piece
}

/// Convert Bernstein control points [b0, b1, b2, b3] to monomial form.
///
/// Identities for cubic Bezier:
///   c0 = b0
///   c1 = 3·(b1 - b0)
///   c2 = 3·(b2 - 2·b1 + b0)
///   c3 = b3 - 3·b2 + 3·b1 - b0
#[inline]
pub fn bernstein_to_monomial(bp: [f32; 4]) -> BezierPieceMonomial {
    let c0 = bp[0];
    let c1 = 3.0 * (bp[1] - bp[0]);
    let c2 = 3.0 * (bp[2] - 2.0 * bp[1] + bp[0]);
    let c3 = bp[3] - 3.0 * bp[2] + 3.0 * bp[1] - bp[0];
    BezierPieceMonomial {
        coeffs: [c0, c1, c2, c3],
        vel_coeffs: [c1, 2.0 * c2, 3.0 * c3],
        duration: 1.0,
    }
}

/// Cubic Bezier Bernstein control points → seconds-domain monomial form.
///
/// Wraps [`bernstein_to_monomial`] (which produces unit-interval coefficients)
/// with a duration rescale: `c_k' = c_k / d^k`. After rescale, evaluating
/// the monomial at `t_sec ∈ [0, duration]` produces the same physical mm value
/// the unit-interval evaluation would at `τ = t_sec / duration`.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §3.2.
#[inline]
pub fn bernstein_to_monomial_with_duration(
    bp: [f32; 4],
    duration_sec: f32,
) -> BezierPieceMonomial {
    let m = bernstein_to_monomial(bp);
    let c0 = m.coeffs[0];
    let c1 = m.coeffs[1] / duration_sec;
    let c2 = m.coeffs[2] / (duration_sec * duration_sec);
    let c3 = m.coeffs[3] / (duration_sec * duration_sec * duration_sec);
    BezierPieceMonomial {
        coeffs: [c0, c1, c2, c3],
        vel_coeffs: [c1, 2.0 * c2, 3.0 * c3],
        duration: duration_sec,
    }
}

/// Evaluate P(t) = c0 + c1·t + c2·t² + c3·t³ via Horner's method:
/// P(t) = c0 + t·(c1 + t·(c2 + t·c3)).
#[inline]
pub fn eval_position(m: &BezierPieceMonomial, t: f32) -> f32 {
    let c = &m.coeffs;
    c[0] + t * (c[1] + t * (c[2] + t * c[3]))
}

/// Evaluate V(t) = vc0 + vc1·t + vc2·t² via Horner's method:
/// V(t) = vc0 + t·(vc1 + t·vc2).
#[inline]
pub fn eval_velocity(m: &BezierPieceMonomial, t: f32) -> f32 {
    let v = &m.vel_coeffs;
    v[0] + t * (v[1] + t * v[2])
}

/// Evaluate position and velocity together, sharing the intermediate
/// `t·c3` / `t·vc2` work where possible.
#[inline]
pub fn eval_position_velocity(m: &BezierPieceMonomial, t: f32) -> (f32, f32) {
    let c = &m.coeffs;
    let v = &m.vel_coeffs;
    let p = c[0] + t * (c[1] + t * (c[2] + t * c[3]));
    let vel = v[0] + t * (v[1] + t * v[2]);
    (p, vel)
}
