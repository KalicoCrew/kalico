//! Time-reparameterization: Stages 2a and 2b of the trajectory shaping pipeline.
//!
//! **Stage 2a** — `build_s_of_t_pieces`: constructs degree-2 polynomial pieces
//! `s(t)` from the TOPP-RA grid samples, mapping batch-global time to arc length.
//!
//! **Stage 2b** — `compose_segment`: fits `x(s)` per grid piece and composes with
//! `s(t)` to produce `x(t)` — per-axis Bezier pieces in the time domain.

use nurbs::bezier::BezierPiece;

/// Velocity threshold below which a grid interval is treated as near-zero
/// (constant-position). Both endpoints must be below this threshold.
const NEAR_ZERO_V: f64 = 0.01;

/// Pieces of `s(t)` — degree-2 polynomial mapping time to arc length — plus
/// metadata identifying constant-position (near-zero-velocity) pieces.
#[derive(Debug, Clone)]
pub struct SOfTPieces {
    /// One `BezierPiece` per TOPP-RA grid interval, in Pascal-shifted monomial
    /// basis with domain in batch-global time.
    pub pieces: Vec<BezierPiece<f64>>,
    /// `near_zero[k]` is `true` when both endpoint velocities of grid interval
    /// `k` are below `NEAR_ZERO_V`. Stage 2b skips composition for these pieces
    /// and emits constant-position output instead.
    pub near_zero: Vec<bool>,
    /// Batch-global time at the start of the first piece.
    pub t_start: f64,
    /// Batch-global time at the end of the last piece.
    pub t_end: f64,
    /// Sum of all piece durations (used by downstream timeline construction).
    #[allow(dead_code)]
    pub total_duration: f64,
}

/// Build the s(t) piecewise-quadratic mapping from a TOPP-RA velocity profile.
///
/// Each grid interval `[s_k, s_{k+1}]` produces one degree-2 `BezierPiece`:
///
/// ```text
/// s(t) = s_k + v_k * (t - t_k) + (a_k / 2) * (t - t_k)^2
/// ```
///
/// where `v_k = sample[k].v`, `a_k = (b_{k+1} - b_k) / (2 * Ds_k)`, and
/// `Dt_k = 2 * Ds_k / (v_k + v_{k+1})`.
///
/// Grid intervals where both endpoint velocities are below `NEAR_ZERO_V` are
/// flagged as near-zero; their duration is `Ds / NEAR_ZERO_V` and the piece is
/// a constant `s_k`.
pub fn build_s_of_t_pieces(profile: &temporal::TopProfile, t_global_offset: f64) -> SOfTPieces {
    let n = profile.samples.len();
    assert!(n >= 2, "TopProfile must have at least 2 samples");

    let mut pieces = Vec::with_capacity(n - 1);
    let mut near_zero = Vec::with_capacity(n - 1);
    let mut t_cursor = t_global_offset;

    for k in 0..n - 1 {
        let s_k = profile.samples[k].s;
        let s_k1 = profile.samples[k + 1].s;
        let v_k = profile.samples[k].v;
        let v_k1 = profile.samples[k + 1].v;
        let b_k = profile.samples[k].b;
        let b_k1 = profile.samples[k + 1].b;

        let ds = s_k1 - s_k;

        let is_near_zero = v_k < NEAR_ZERO_V && v_k1 < NEAR_ZERO_V;

        if is_near_zero {
            // Constant-position piece: s(t) = s_k (flat).
            let dt = ds / NEAR_ZERO_V;
            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, 0.0, 0.0],
            });
            near_zero.push(true);
            t_cursor = t_end;
        } else {
            // Normal piece: s(t) = s_k + v_k*(t-t_k) + (a_k/2)*(t-t_k)^2.
            let v_sum = v_k + v_k1;
            let dt = if v_sum > 1e-12 {
                2.0 * ds / v_sum
            } else {
                // Defensive: shouldn't happen for feasible TOPP-RA output when
                // not both near-zero, but guard against numerical edge cases.
                ds / NEAR_ZERO_V
            };

            let a_k = if ds.abs() > 1e-15 {
                (b_k1 - b_k) / (2.0 * ds)
            } else {
                0.0
            };

            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, v_k, a_k / 2.0],
            });
            near_zero.push(false);
            t_cursor = t_end;
        }
    }

    let t_start = t_global_offset;
    let t_end = t_cursor;
    SOfTPieces {
        pieces,
        near_zero,
        t_start,
        t_end,
        total_duration: t_end - t_start,
    }
}

/// Compose a segment's geometry `x(s)` with the s(t) mapping to produce `x(t)`
/// pieces in the time domain.
///
/// For each non-near-zero grid piece:
/// 1. Fit `x(s)` on `[s_k, s_{k+1}]` via `fit_x_to_arc_length_piece::<3>`.
/// 2. Compose `x(s) . s(t)` via `compose_vector_piece::<3>`.
///
/// Near-zero pieces produce constant-position output: `x(s_k)` evaluated once.
///
/// # Errors
///
/// Returns `ShapeError::FitFailure` if the polynomial fit of `x(s)` does not
/// converge within `max_degree=5` on any grid piece.
/// Returns `ShapeError::Algebra` if composition fails.
pub fn compose_segment(
    curve: &nurbs::VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_pieces: &SOfTPieces,
    fit_tolerance: f64,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let mut result = Vec::with_capacity(s_pieces.pieces.len());

    for (k, s_piece) in s_pieces.pieces.iter().enumerate() {
        if s_pieces.near_zero[k] {
            // Constant-position piece: evaluate geometry at s_k and emit a
            // constant BezierPiece per axis.
            let s_k = s_piece.coeffs[0]; // s_k is the constant term
            let u_k = nurbs::arc_length::param_from_arc_length(table, s_k);
            let pos = nurbs::eval::vector_eval(curve, u_k);

            let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                u_start: s_piece.u_start,
                u_end: s_piece.u_end,
                coeffs: vec![pos[axis]],
            });
            result.push(axes);
        } else {
            // Fit x(s) on [s_lo, s_hi].
            let s_lo = s_piece.evaluate(s_piece.u_start); // = s_k (constant term)
            let s_hi = s_piece.evaluate(s_piece.u_end); // = s_{k+1}

            // Clamp s_hi to table's total length to avoid floating-point overshoot.
            let s_hi_clamped = s_hi.min(table.s_max());
            // Guard against degenerate zero-length arc pieces.
            let s_lo_safe = s_lo.max(0.0);

            if s_hi_clamped - s_lo_safe < 1e-15 {
                // Degenerate: arc-length span is essentially zero. Emit constant.
                let u_k = nurbs::arc_length::param_from_arc_length(table, s_lo_safe);
                let pos = nurbs::eval::vector_eval(curve, u_k);
                let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                    u_start: s_piece.u_start,
                    u_end: s_piece.u_end,
                    coeffs: vec![pos[axis]],
                });
                result.push(axes);
                continue;
            }

            let x_of_s: [BezierPiece<f64>; 3] = nurbs::algebra::fit_x_to_arc_length_piece::<3>(
                curve,
                table,
                s_lo_safe,
                s_hi_clamped,
                3, // target_degree
                5, // max_degree
                fit_tolerance,
            )
            .map_err(|detail| crate::ShapeError::FitFailure { index: k, detail })?;

            // Compose: x(s(t)). The outer pieces are in s-domain, the inner is
            // s(t) in t-domain. compose_vector_piece checks that
            // inner.evaluate(inner.u_start) == outer.u_start and
            // inner.evaluate(inner.u_end) == outer.u_end.
            //
            // We need to adjust the s(t) piece so its evaluated endpoints match
            // the fit's s-domain exactly (s_lo_safe, s_hi_clamped), since the fit
            // was performed on the clamped range.
            let s_piece_adjusted =
                if (s_lo_safe - s_lo).abs() > 1e-15 || (s_hi_clamped - s_hi).abs() > 1e-15 {
                    // Rebuild with clamped endpoints. The quadratic form is the same
                    // but we adjust the constant and linear terms to match.
                    // s_adj(t) maps [t_start, t_end] to [s_lo_safe, s_hi_clamped].
                    // We keep the same polynomial shape but adjust the constant term.
                    let mut adj = s_piece.clone();
                    adj.coeffs[0] = s_lo_safe;
                    // Recompute the quadratic coefficient so s_adj(t_end) = s_hi_clamped.
                    let dt = adj.u_end - adj.u_start;
                    if dt > 1e-15 && adj.coeffs.len() >= 3 {
                        // s_adj(t_end) = s_lo_safe + v_k*dt + (a_k/2)*dt^2 should = s_hi_clamped
                        // So (a_k/2) = (s_hi_clamped - s_lo_safe - v_k*dt) / dt^2
                        let v_k = adj.coeffs[1];
                        adj.coeffs[2] = (s_hi_clamped - s_lo_safe - v_k * dt) / (dt * dt);
                    }
                    adj
                } else {
                    s_piece.clone()
                };

            let outer_refs: [&BezierPiece<f64>; 3] = [&x_of_s[0], &x_of_s[1], &x_of_s[2]];

            let composed =
                nurbs::algebra::compose_vector_piece::<3>(&outer_refs, &s_piece_adjusted)
                    .map_err(|detail| crate::ShapeError::Algebra { index: k, detail })?;

            result.push(composed);
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests;
