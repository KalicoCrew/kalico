//! Closed-form cubic root solver for uniform cubic Bezier curves.
//!
//! Replaces Newton's method in the step-time computation path. For our
//! pipeline (uniform cubic Beziers per CLAUDE.md), we can always extract
//! monomial coefficients from the 4 control points and solve directly via
//! Cardano's method, with deterministic compute time and no seed /
//! convergence concerns.
//!
//! Spec: docs/superpowers/plans/2026-05-14-cardano-cubic-solver.md

#![cfg_attr(not(feature = "host"), no_std)]

/// Monomial-form coefficients of a cubic polynomial `a*u^3 + b*u^2 + c*u + d`.
/// f64 to keep precision comfortable in shifted-target arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicCoeffs {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl CubicCoeffs {
    /// Construct from uniform cubic Bezier control points
    /// (knots [0,0,0,0,1,1,1,1]).
    #[must_use]
    pub fn from_bezier(p0: f64, p1: f64, p2: f64, p3: f64) -> Self {
        Self {
            a: -p0 + 3.0 * p1 - 3.0 * p2 + p3,
            b: 3.0 * p0 - 6.0 * p1 + 3.0 * p2,
            c: -3.0 * p0 + 3.0 * p1,
            d: p0,
        }
    }

    /// Evaluate `x(u) = a*u^3 + b*u^2 + c*u + d` via Horner.
    #[must_use]
    pub fn eval(&self, u: f64) -> f64 {
        ((self.a * u + self.b) * u + self.c) * u + self.d
    }

    /// Evaluate the derivative `x'(u) = 3a*u^2 + 2b*u + c`.
    #[must_use]
    pub fn eval_d1(&self, u: f64) -> f64 {
        (3.0 * self.a * u + 2.0 * self.b) * u + self.c
    }

    /// Add another set of coefficients (`CoreXY` motor `A = X + Y`).
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            a: self.a + other.a,
            b: self.b + other.b,
            c: self.c + other.c,
            d: self.d + other.d,
        }
    }

    /// Subtract another set of coefficients (`CoreXY` motor `B = X - Y`).
    #[must_use]
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            a: self.a - other.a,
            b: self.b - other.b,
            c: self.c - other.c,
            d: self.d - other.d,
        }
    }
}

/// Relative degeneracy threshold for leading-coefficient classification.
/// Each call site scales this against the maximum magnitude of the
/// polynomial's coefficients (floored at 1.0) so the test scales with the
/// cubic's working amplitude. Bezier control points span sub-mm precision
/// moves to 100s of mm of toolhead coordinates; a fixed absolute floor
/// would misclassify the small-scale regime as degenerate.
const REL_EPS: f64 = 1e-12;

/// Find the smallest real root of `a*u^3 + b*u^2 + c*u + (d - target) = 0`
/// strictly greater than `t_low` and less-than-or-equal to `t_high`. Returns
/// `None` if no such root exists.
///
/// Uses Cardano's method with the depression substitution `u = v - B/3`,
/// branching on the discriminant for one-real vs three-real-roots cases.
/// Falls back to a numerically-stable quadratic solver when the leading
/// coefficient is near zero.
#[must_use]
pub fn solve_smallest_root_in(
    coeffs: &CubicCoeffs,
    target: f64,
    t_low: f64,
    t_high: f64,
) -> Option<f64> {
    // Defensive: reject non-finite inputs and degenerate intervals.
    if !coeffs.a.is_finite()
        || !coeffs.b.is_finite()
        || !coeffs.c.is_finite()
        || !coeffs.d.is_finite()
        || !target.is_finite()
        || !t_low.is_finite()
        || !t_high.is_finite()
    {
        return None;
    }
    if t_high <= t_low {
        return None;
    }

    // Shift-by-target into a' = a, b' = b, c' = c, d' = d - target.
    let a = coeffs.a;
    let b = coeffs.b;
    let c = coeffs.c;
    let d_shifted = coeffs.d - target;

    // Degenerate cubic: |a| relatively small -> quadratic fallback.
    // Scale the threshold to the largest coefficient magnitude so we judge
    // `a` against the polynomial's own working scale, not an absolute floor.
    let scale = libm::fmax(
        libm::fmax(libm::fabs(a), libm::fabs(b)),
        libm::fmax(libm::fabs(c), libm::fabs(d_shifted)),
    );
    let eps_a = REL_EPS * libm::fmax(scale, 1.0);
    if libm::fabs(a) < eps_a {
        return solve_quadratic_smallest_root_in(b, c, d_shifted, t_low, t_high);
    }

    // Normalize to depressed cubic. Working coefficients:
    //   B = b/a, C = c/a, D = d_shifted/a
    let big_b = b / a;
    let big_c = c / a;
    let big_d = d_shifted / a;

    // Depress: u = v - B/3 -> v^3 + p*v + q = 0
    let p = big_c - big_b * big_b / 3.0;
    let q = 2.0 * big_b * big_b * big_b / 27.0 - big_b * big_c / 3.0 + big_d;

    let half_q = q / 2.0;
    let third_p = p / 3.0;
    let disc = half_q * half_q + third_p * third_p * third_p;

    // Relative repeated-root threshold. The discriminant is the sum of
    // (q/2)^2 and (p/3)^3; size-it-up relative to the larger of those
    // magnitudes so the test scales with the cubic's amplitude.
    let half_q_sq = half_q * half_q;
    let third_p_cu = third_p * third_p * third_p;
    let disc_scale = libm::fmax(libm::fmax(half_q_sq, libm::fabs(third_p_cu)), 1.0);
    let disc_eps = 1e-12 * disc_scale;

    // Collect at most 3 candidate roots, then pick the smallest in
    // (t_low, t_high]. Using an Option array keeps the code branch-free of
    // a Vec / allocator in no_std.
    let candidates: [Option<f64>; 3] = if libm::fabs(disc) < disc_eps {
        // Repeated root regime. v_1 = 2 * cbrt(-q/2) (single), v_2 = -cbrt(-q/2) (double).
        let cbrt_hq = libm::cbrt(-half_q);
        [
            Some(2.0 * cbrt_hq - big_b / 3.0),
            Some(-cbrt_hq - big_b / 3.0),
            None,
        ]
    } else if disc > 0.0 {
        // One real root.
        let sq = libm::sqrt(disc);
        let v_real = libm::cbrt(-half_q + sq) + libm::cbrt(-half_q - sq);
        [Some(v_real - big_b / 3.0), None, None]
    } else {
        // disc < 0 -> three real roots, trigonometric form.
        // p must be negative here: disc < 0 means (q/2)^2 < -(p/3)^3,
        // which requires (p/3)^3 < 0, hence p < 0.
        let two_sqrt_neg_third_p = 2.0 * libm::sqrt(-third_p);
        // Argument to acos: (3q / (2p)) * sqrt(-3/p). Clamp for safety
        // against rounding pushing it slightly outside [-1, 1].
        let acos_arg_raw = (3.0 * q / (2.0 * p)) * libm::sqrt(-3.0 / p);
        let acos_arg = acos_arg_raw.clamp(-1.0, 1.0);
        let theta = libm::acos(acos_arg);
        let two_pi_3 = 2.0 * core::f64::consts::PI / 3.0;
        [
            Some(two_sqrt_neg_third_p * libm::cos(theta / 3.0) - big_b / 3.0),
            Some(two_sqrt_neg_third_p * libm::cos(theta / 3.0 - two_pi_3) - big_b / 3.0),
            Some(two_sqrt_neg_third_p * libm::cos(theta / 3.0 + two_pi_3) - big_b / 3.0),
        ]
    };

    pick_smallest_in_interval(&candidates, t_low, t_high)
}

/// Stable quadratic solver for `a*u^2 + b*u + c = 0` (signature reuses the
/// same letter names as the surrounding cubic context for readability —
/// these are the *shifted-cubic*'s `b, c, d - target` when this is called).
///
/// Avoids cancellation via the form `q = -(b + sign(b) * sqrt(disc)) / 2`,
/// with roots `q/a` and `c/q`.
fn solve_quadratic_smallest_root_in(
    a: f64,
    b: f64,
    c: f64,
    t_low: f64,
    t_high: f64,
) -> Option<f64> {
    // Relative degeneracy threshold for the quadratic leading coefficient.
    let scale = libm::fmax(
        libm::fmax(libm::fabs(a), libm::fabs(b)),
        libm::fabs(c),
    );
    let eps_a = REL_EPS * libm::fmax(scale, 1.0);
    if libm::fabs(a) < eps_a {
        // Linear: b*u + c = 0 -> u = -c/b.
        let scale_l = libm::fmax(libm::fabs(b), libm::fabs(c));
        let eps_b = REL_EPS * libm::fmax(scale_l, 1.0);
        if libm::fabs(b) < eps_b {
            // Constant. No finite root; if c == 0 every u is a root but
            // "any u" isn't actionable for step timing, so return None.
            return None;
        }
        let u = -c / b;
        return if u > t_low && u <= t_high && u.is_finite() {
            Some(u)
        } else {
            None
        };
    }

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }
    let sq = libm::sqrt(disc);

    // Numerically-stable form: choose the sign of sqrt that matches sign(b)
    // to avoid catastrophic cancellation in the numerator.
    let sign_b = if b >= 0.0 { 1.0 } else { -1.0 };
    let q = -0.5 * (b + sign_b * sq);

    // Two roots: q/a and c/q. Guard against q == 0 (means b == 0 and sq == 0,
    // i.e. disc == 0, double root at u = 0). Use a relative threshold scaled
    // to the working magnitudes of b and sqrt(disc).
    let q_scale = libm::fmax(libm::fabs(b), sq);
    let eps_q = REL_EPS * libm::fmax(q_scale, 1.0);
    let candidates: [Option<f64>; 3] = if libm::fabs(q) < eps_q {
        // Both formulas degenerate; with b ~ 0 and disc ~ 0 the double root
        // is u = 0 (from -b/(2a) with b ~ 0).
        [Some(0.0), None, None]
    } else {
        [Some(q / a), Some(c / q), None]
    };

    pick_smallest_in_interval(&candidates, t_low, t_high)
}

/// Pick the smallest candidate strictly greater than `t_low` and
/// less-than-or-equal-to `t_high`. Returns `None` if no candidate qualifies.
fn pick_smallest_in_interval(
    candidates: &[Option<f64>; 3],
    t_low: f64,
    t_high: f64,
) -> Option<f64> {
    let mut best: Option<f64> = None;
    for slot in candidates {
        if let Some(r) = *slot {
            if r.is_finite() && r > t_low && r <= t_high {
                best = match best {
                    None => Some(r),
                    Some(prev) if r < prev => Some(r),
                    Some(_) => best,
                };
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_bezier_matches_eval_on_linear_curve() {
        // Linear: p0=0, p1=1/3, p2=2/3, p3=1 -> x(u) = u
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        assert!((c.eval(0.0) - 0.0).abs() < 1e-12);
        assert!((c.eval(0.5) - 0.5).abs() < 1e-12);
        assert!((c.eval(1.0) - 1.0).abs() < 1e-12);
        assert!((c.eval_d1(0.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn from_bezier_matches_eval_on_jerk_only_cubic() {
        // x(u) = u^3 -> p0=0, p1=0, p2=0, p3=1
        let c = CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 1.0);
        for &u in &[0.1_f64, 0.3, 0.5, 0.7, 0.9] {
            let expected = u * u * u;
            assert!(
                (c.eval(u) - expected).abs() < 1e-12,
                "u={} eval={} expected={}",
                u,
                c.eval(u),
                expected
            );
        }
    }

    #[test]
    fn solve_smallest_root_on_linear_curve() {
        // x(u) = u, target=0.5 -> root at u=0.5
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.5, 0.0, 1.0);
        assert!(r.is_some(), "expected a root");
        assert!((r.unwrap() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn solve_smallest_root_on_jerk_only_cubic() {
        // x(u) = u^3, target = 0.001 -> u = 0.1
        let c = CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.001, 0.0, 1.0);
        assert!(r.is_some(), "expected a root");
        assert!(
            (r.unwrap() - 0.1).abs() < 1e-6,
            "expected u~=0.1, got {}",
            r.unwrap()
        );
    }

    #[test]
    fn solve_target_out_of_range_returns_none() {
        // x(u) ∈ [0, 1]; target=2.0 has no root in [0, 1].
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 2.0, 0.0, 1.0);
        assert!(r.is_none());
    }

    #[test]
    fn solve_constrains_low_bound() {
        // x(u) = u; target = 0.3. Root at 0.3. With t_low=0.5, no valid root.
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.3, 0.5, 1.0);
        assert!(r.is_none());
    }

    #[test]
    fn solve_accel_from_rest_quadratic() {
        // x(u) = u^2 -> Bezier (0, 0, 1/3, 1).
        // Verify: 0 + 0 + 3*(1/3)*(1-u)*u^2 + u^3 = (1-u)*u^2 + u^3 = u^2.
        // This is the v(0)=0 case Newton struggled with; Cardano should
        // handle it analytically.
        let c = CubicCoeffs::from_bezier(0.0, 0.0, 1.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.01, 0.0, 1.0);
        assert!(r.is_some(), "Cardano must find root in accel-from-rest curve");
        assert!(
            (r.unwrap() - 0.1).abs() < 1e-6,
            "expected u~=0.1, got {:?}",
            r
        );
    }

    #[test]
    fn solve_decel_to_rest() {
        // x(u) = 2u - u^2; target = 0.99 -> u = 0.9.
        // Bezier cps: p0=0, p1=2/3, p2=1, p3=1.
        let c = CubicCoeffs::from_bezier(0.0, 2.0 / 3.0, 1.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.99, 0.0, 1.0);
        assert!(r.is_some());
        assert!(
            (r.unwrap() - 0.9).abs() < 1e-6,
            "expected u~=0.9, got {:?}",
            r
        );
    }

    #[test]
    fn solve_three_real_roots_picks_smallest_in_range() {
        // (u-0.2)(u-0.5)(u-0.8) -> a=1, b=-1.5, c=0.66, d=-0.08
        // Bezier cps derived in plan: (-0.08, 0.14, -0.14, 0.08).
        let c = CubicCoeffs::from_bezier(-0.08, 0.14, -0.14, 0.08);
        let r = solve_smallest_root_in(&c, 0.0, 0.0, 1.0);
        assert!(r.is_some(), "expected a real root in (0, 1]");
        assert!(
            (r.unwrap() - 0.2).abs() < 1e-6,
            "smallest root should be 0.2; got {:?}",
            r
        );
    }

    #[test]
    fn solve_three_real_roots_constrained_low_skips_first() {
        // Same cubic as the previous test; t_low=0.3 -> should pick 0.5.
        let c = CubicCoeffs::from_bezier(-0.08, 0.14, -0.14, 0.08);
        let r = solve_smallest_root_in(&c, 0.0, 0.3, 1.0);
        assert!(r.is_some());
        assert!(
            (r.unwrap() - 0.5).abs() < 1e-6,
            "smallest root > 0.3 should be 0.5; got {:?}",
            r
        );
    }

    #[test]
    fn solve_reverse_direction() {
        // x(u) = 1 - u; target = 0.5 -> u = 0.5.
        let c = CubicCoeffs::from_bezier(1.0, 2.0 / 3.0, 1.0 / 3.0, 0.0);
        let r = solve_smallest_root_in(&c, 0.5, 0.0, 1.0);
        assert!(r.is_some());
        assert!((r.unwrap() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn solve_root_at_t_high_inclusive() {
        // x(u) = u; target = 1.0 -> root at exactly u = 1.0. Must be included.
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 1.0, 0.0, 1.0);
        assert!(r.is_some(), "root at t_high boundary must be included");
        assert!((r.unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn solve_root_at_t_low_exclusive() {
        // x(u) = u; target = 0.1. With t_low=0.1, root at 0.1 must be excluded.
        let c = CubicCoeffs::from_bezier(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        let r = solve_smallest_root_in(&c, 0.1, 0.1, 1.0);
        assert!(r.is_none(), "root at t_low boundary must be excluded");
    }

    #[test]
    fn coeffs_corexy_composition_motor_a_x_plus_y() {
        // X curve x(u) = 10u, Y curve y(u) = 5u^2.
        let cx = CubicCoeffs::from_bezier(0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0);
        let cy = CubicCoeffs::from_bezier(0.0, 0.0, 5.0 / 3.0, 5.0);
        let a = cx.add(&cy);
        // A(0.5) = 10*0.5 + 5*0.25 = 6.25
        assert!(
            (a.eval(0.5) - 6.25).abs() < 1e-9,
            "X+Y at u=0.5 should be 6.25; got {}",
            a.eval(0.5)
        );
        // A'(0.5) = 10 + 5 = 15
        assert!(
            (a.eval_d1(0.5) - 15.0).abs() < 1e-9,
            "(X+Y)' at u=0.5 should be 15; got {}",
            a.eval_d1(0.5)
        );
    }

    #[test]
    fn coeffs_corexy_composition_motor_b_x_minus_y() {
        let cx = CubicCoeffs::from_bezier(0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0);
        let cy = CubicCoeffs::from_bezier(0.0, 0.0, 5.0 / 3.0, 5.0);
        let b = cx.sub(&cy);
        // B(0.5) = 5 - 1.25 = 3.75
        assert!((b.eval(0.5) - 3.75).abs() < 1e-9);
    }

    #[test]
    fn solve_relative_threshold_keeps_tiny_uniform_cubic_as_cubic() {
        // All four control points at nm scale: cps = (0, 1e-9, 2e-9, 3e-9).
        // Monomial form:
        //   a = -0 + 3·1e-9 - 3·2e-9 + 3e-9 = 0
        //   b = 0 - 6e-9 + 6e-9 = 0
        //   c = -0 + 3e-9 = 3e-9
        //   d = 0
        // x(u) = 3e-9 · u (perfectly linear).
        //
        // Target = 1.5e-9 -> root at u = 0.5.
        //
        // The cubic and quadratic leading coefficients are zero; the linear
        // coefficient c = 3e-9 is below an absolute 1e-12 threshold *would*
        // accept (3e-9 > 1e-12), but this case exercises the relative-
        // threshold fallback chain (cubic -> quadratic -> linear) at small
        // scale and ensures we still recover the solvable linear root.
        let c = CubicCoeffs::from_bezier(0.0, 1e-9, 2e-9, 3e-9);
        let r = solve_smallest_root_in(&c, 1.5e-9, 0.0, 1.0);
        assert!(r.is_some(), "tiny linear-from-Bezier should still be solvable");
        assert!(
            (r.unwrap() - 0.5).abs() < 1e-9,
            "expected u~=0.5 for tiny linear, got {:?}",
            r
        );
    }

    #[test]
    fn solve_double_root_returns_finite_value() {
        // (u-0.3)^2 * (u-0.7) = u^3 - 1.3u^2 + 0.51u - 0.063
        // Bezier cps from monomial-to-Bernstein conversion (per plan):
        //   p0 = d = -0.063
        //   p1 = (c + 3*p0)/3 = (0.51 - 0.189)/3 = 0.107
        //   p2 = (b + 6*p1 - 3*p0)/3 = (-1.3 + 0.642 + 0.189)/3 = -0.156333...
        //   p3 = a + 3*p2 - 3*p1 + p0 = 1 - 0.469 - 0.321 - 0.063 = 0.147
        let p0 = -0.063_f64;
        let p1 = (0.51_f64 + 3.0 * p0) / 3.0;
        let p2 = (-1.3_f64 + 6.0 * p1 - 3.0 * p0) / 3.0;
        let p3 = 1.0_f64 + 3.0 * p2 - 3.0 * p1 + p0;
        let c = CubicCoeffs::from_bezier(p0, p1, p2, p3);
        let r = solve_smallest_root_in(&c, 0.0, 0.0, 1.0);
        assert!(r.is_some(), "must find a real root even with double root");
        let u = r.unwrap();
        let residual = c.eval(u);
        assert!(
            residual.abs() < 1e-6,
            "residual at returned root should be near zero; got {} at u={}",
            residual,
            u
        );
    }
}
