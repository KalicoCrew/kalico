//! Regression for bug #18: `convolve` produced a corrupt final Bézier piece
//! when the input spans multiple neighbour segments and the output domain is
//! offset from the origin (batched dispatch, where successive segments live at
//! larger and larger absolute t-coordinates).
//!
//! Symptom (observed in the live planner via `motion-bridge`'s
//! `batched_two_move_curves_are_sane`): for the second segment in a two-move
//! batch, the curve evaluated correctly at u<99.99% of segment progress, then
//! jumped ~14 mm at literally the final knot. Sub-µm sampling against the
//! polynomial showed the LAST Bézier piece was ill-conditioned: its
//! `coeffs` were alternating-sign with magnitudes ≥ 1e19 (vs ≤ 1e6 in the
//! preceding piece), the natural fingerprint of catastrophic cancellation in
//! `absolute_to_pascal_shift` when shift α is large and the piece is narrow.
//!
//! Root cause: `integrate_product_piece` formed the convolution result in the
//! absolute-u monomial basis and re-shifted to Pascal-at-α at the end. With
//! α ≈ 2 (second segment of a batch starts at t ≈ 0.7s and ends at ≈ 2s) and
//! a degree-9 output, the absolute-u coefficients reached u^9 ≈ 512×, then
//! `absolute_to_pascal_shift` summed binomial products of α^(n-k) against
//! those coefficients — alternating-sign cancellation killed ~10 digits of
//! accuracy on the trailing tiny piece (width ≈ kernel half-support).
//!
//! Fix: do all integrand arithmetic in the (u−α, s−α) frame so every
//! intermediate coefficient is O(width^k) instead of O(α^k). The result
//! lands directly in Pascal-shifted-at-α basis, eliminating the lossy
//! re-shift entirely.

use nurbs::algebra::{convolve, restrict_to_domain, PiecewisePolynomialKernel};
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
use nurbs::eval::eval;

/// Smooth-MZV-shaped two-segment ramp where the second segment lives at large
/// absolute t (off-origin), exercising the same conditioning regime that bug
/// #18 surfaces in the live planner. The endpoint sample must agree with the
/// curve value just below it.
#[test]
fn convolve_then_restrict_preserves_endpoint_on_offset_second_segment() {
    // Smooth-MZV kernel @ 50 Hz (matches motion-bridge default).
    let f_hz = 50.0_f64;
    let t_sm = 0.95625 / f_hz;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    let abs_coeffs = vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c];
    let kernel =
        PiecewisePolynomialKernel::single_poly_from_absolute(abs_coeffs, (-h, h));

    // Input: left-extension constant 50 (starting before t=0.728), linear ramp
    // 50 → 100 over [0.728, 2.069], right-extension constant 100. Mirrors how
    // the trajectory pad path lays out a batched second segment.
    let t_start = 0.7283517369016971_f64;
    let t_end = 2.0692055134039826_f64;
    // Degree-4 pieces match the post-`fit_hermite_c1` shape that
    // `trajectory::shaper::shape_axis` actually feeds into `convolve` in the
    // live planner. The bug manifests at degree-9 output, which requires
    // degree-(d_x + d_w + 1) ≥ ~9 — i.e. degree-4 input × degree-4 kernel.
    // Splitting the linear ramp into many small pieces (matching the fitter's
    // ~5 µm tolerance budget) drives `convolve`'s Minkowski-sum to produce
    // many degree-9 output pieces, of which the right-most one is the narrow
    // [t_end − h, t_end] cell that corrupts pre-fix.
    // Build a fit-hermite-c1-shaped input: many narrow degree-4 pieces with
    // non-trivial higher-order coefficients (the hermite fitter does NOT emit
    // pure linears — it emits degree-4 pieces whose monomial coefficients are
    // O(slope) in c1, but c2..c4 have small non-zero values from the
    // continuity solve). The catastrophic cancellation in
    // `absolute_to_pascal_shift` only amplifies pre-existing higher-order
    // structure, so a pure-linear test is not representative.
    let n_ramp_pieces = 18;
    let mut pieces: Vec<BezierPiece<f64>> = Vec::new();
    pieces.push(BezierPiece::<f64> {
        u_start: t_start - h,
        u_end: t_start,
        coeffs: vec![50.0, 0.0, 0.0, 0.0, 0.0],
    });
    let total = t_end - t_start;
    let slope = 50.0 / total;
    // Shape function that emulates a velocity-profile wiggle: a small
    // sinusoidal modulation atop the linear ramp ensures the degree-4
    // coefficients are non-zero per piece. Amplitude well below mm.
    let modulation_amp = 1.0e-3;
    let modulation_freq = 30.0;
    for i in 0..n_ramp_pieces {
        let u0 = t_start + total * (i as f64) / (n_ramp_pieces as f64);
        let u1 = t_start + total * ((i + 1) as f64) / (n_ramp_pieces as f64);
        // Approximate position(u) ≈ 50 + slope*(u - t_start) + amp*sin(2π·f·u)
        // by its 4th-order Taylor expansion around u0, expressed in shifted
        // basis (u − u0). Coefficients:
        //   c0 = pos(u0); c1 = slope + amp·2πf·cos(2πf·u0);
        //   c2 = -amp·(2πf)^2/2! · sin(2πf·u0);
        //   c3 = -amp·(2πf)^3/3! · cos(2πf·u0);
        //   c4 = +amp·(2πf)^4/4! · sin(2πf·u0).
        let u_lin = u0;
        let omega = 2.0 * std::f64::consts::PI * modulation_freq;
        let s = (omega * u_lin).sin();
        let cs = (omega * u_lin).cos();
        let c0 = 50.0 + slope * (u0 - t_start) + modulation_amp * s;
        let c1 = slope + modulation_amp * omega * cs;
        let c2 = -modulation_amp * omega.powi(2) / 2.0 * s;
        let c3 = -modulation_amp * omega.powi(3) / 6.0 * cs;
        let c4 = modulation_amp * omega.powi(4) / 24.0 * s;
        pieces.push(BezierPiece::<f64> {
            u_start: u0,
            u_end: u1,
            coeffs: vec![c0, c1, c2, c3, c4],
        });
    }
    pieces.push(BezierPiece::<f64> {
        u_start: t_end,
        u_end: t_end + h,
        coeffs: vec![100.0, 0.0, 0.0, 0.0, 0.0],
    });

    let input = bezier_pieces_to_nurbs(&pieces);
    let convolved = convolve(&input, &kernel).unwrap();
    let restricted = restrict_to_domain(&convolved, t_start, t_end).unwrap();

    // The curve must be smooth at the right boundary: value at the final knot
    // must agree with sampling just below it. Pre-fix this differed by ~14 mm.
    let v_end = eval(&restricted.as_view(), t_end);
    let v_just_below = eval(&restricted.as_view(), t_end - 1e-9);
    assert!(
        (v_end - v_just_below).abs() < 1e-6,
        "discontinuity at final knot: eval({t_end})={v_end}, \
         eval({t_end}−1e-9)={v_just_below}, diff={}",
        v_end - v_just_below,
    );

    // Last control point of the stitched NURBS == polynomial value at t_end.
    // Pre-fix the trailing piece's monomial coefficients had alternating signs
    // with magnitudes 1e19, so the recovered Bernstein last cp was ~14 mm off.
    let last_cp = *restricted.control_points().last().unwrap();
    assert!(
        (last_cp - v_end).abs() < 1e-6,
        "last control point ({last_cp}) does not match polynomial value at \
         t_end ({v_end}) — degenerate trailing piece, diff={}",
        last_cp - v_end,
    );

    // The endpoint should land in a physically reasonable neighbourhood of
    // 100 mm — the linear input ends at 100 and the right pad is constant at
    // 100, so the smoothed curve at u = t_end is the kernel-weighted average
    // of values close to 100 on both sides. Pre-fix the trailing piece
    // returned ~114 mm from the corrupted polynomial.
    assert!(
        (v_end - 100.0).abs() < 1.0,
        "endpoint {v_end} more than 1 mm off expected ~100; pre-fix was ~114",
    );
}
