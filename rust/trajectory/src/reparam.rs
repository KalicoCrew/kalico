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
    /// Sum of all piece durations.
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
pub fn build_s_of_t_pieces(
    profile: &temporal::TopProfile,
    t_global_offset: f64,
) -> SOfTPieces {
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
            let s_piece_adjusted = if (s_lo_safe - s_lo).abs() > 1e-15
                || (s_hi_clamped - s_hi).abs() > 1e-15
            {
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

            let composed = nurbs::algebra::compose_vector_piece::<3>(
                &outer_refs,
                &s_piece_adjusted,
            )
            .map_err(|detail| crate::ShapeError::Algebra {
                index: k,
                detail,
            })?;

            result.push(composed);
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use temporal::{BindingConstraint, GridSample, GridScheme, SolveStatus, TopProfile};

    /// Build a synthetic TopProfile with uniform velocity and uniform grid.
    fn uniform_profile(n: usize, total_length: f64, velocity: f64) -> TopProfile {
        let mut samples = Vec::with_capacity(n);
        let b = velocity * velocity;
        for i in 0..n {
            let s = total_length * (i as f64) / ((n - 1) as f64);
            samples.push(GridSample {
                s,
                v: velocity,
                a: 0.0,
                b,
                binding: BindingConstraint::None,
            });
        }
        let total_time = total_length / velocity;
        TopProfile {
            samples,
            status: SolveStatus::Solved,
            grid_scheme: GridScheme::UniformArclength,
            total_time,
        }
    }

    #[test]
    fn s_of_t_uniform_velocity_is_linear() {
        let profile = uniform_profile(11, 50.0, 500.0);
        let s_pieces = build_s_of_t_pieces(&profile, 0.0);

        assert_eq!(s_pieces.pieces.len(), 10);
        assert!(s_pieces.near_zero.iter().all(|nz| !nz));

        // Total duration should be 50 / 500 = 0.1 s.
        assert!(
            (s_pieces.total_duration - 0.1).abs() < 1e-12,
            "total_duration = {}",
            s_pieces.total_duration
        );

        // Each piece should be linear (a_k = 0 since b is constant).
        for piece in &s_pieces.pieces {
            assert_eq!(piece.coeffs.len(), 3);
            assert!(
                piece.coeffs[2].abs() < 1e-12,
                "quadratic coeff should be ~0, got {}",
                piece.coeffs[2]
            );
        }
    }

    #[test]
    fn s_of_t_endpoint_consistency() {
        // Accelerating profile: v linearly from 0 to 100 over 10 grid points.
        let n = 11;
        let total_length = 50.0;
        let mut samples = Vec::with_capacity(n);
        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            let s = total_length * frac;
            let v = 100.0 * frac;
            samples.push(GridSample {
                s,
                v,
                a: 0.0,
                b: v * v,
                binding: BindingConstraint::None,
            });
        }
        // First sample has v=0, so the first interval has one near-zero endpoint.
        // But v_k1 for k=0 is 10.0, which is > NEAR_ZERO_V, so not near-zero.
        let profile = TopProfile {
            samples,
            status: SolveStatus::Solved,
            grid_scheme: GridScheme::UniformArclength,
            total_time: 1.0, // not used in build_s_of_t_pieces
        };

        let s_pieces = build_s_of_t_pieces(&profile, 0.0);
        assert_eq!(s_pieces.pieces.len(), 10);

        // Check that s(t_end) of each piece matches s_{k+1} from the profile.
        for k in 0..s_pieces.pieces.len() {
            let piece = &s_pieces.pieces[k];
            let s_at_end = piece.evaluate(piece.u_end);
            let expected_s = profile.samples[k + 1].s;
            assert!(
                (s_at_end - expected_s).abs() < 1e-9,
                "piece {k}: s_at_end = {s_at_end}, expected = {expected_s}, diff = {}",
                (s_at_end - expected_s).abs()
            );
        }

        // Also check start-of-piece matches s_k.
        for k in 0..s_pieces.pieces.len() {
            let piece = &s_pieces.pieces[k];
            let s_at_start = piece.evaluate(piece.u_start);
            let expected_s = profile.samples[k].s;
            assert!(
                (s_at_start - expected_s).abs() < 1e-9,
                "piece {k}: s_at_start = {s_at_start}, expected = {expected_s}",
            );
        }
    }

    #[test]
    fn s_of_t_near_zero_handling() {
        // All velocities near zero.
        let profile = TopProfile {
            samples: vec![
                GridSample {
                    s: 0.0,
                    v: 0.001,
                    a: 0.0,
                    b: 1e-6,
                    binding: BindingConstraint::None,
                },
                GridSample {
                    s: 0.5,
                    v: 0.005,
                    a: 0.0,
                    b: 2.5e-5,
                    binding: BindingConstraint::None,
                },
                GridSample {
                    s: 1.0,
                    v: 0.002,
                    a: 0.0,
                    b: 4e-6,
                    binding: BindingConstraint::None,
                },
            ],
            status: SolveStatus::Solved,
            grid_scheme: GridScheme::UniformArclength,
            total_time: 100.0,
        };

        let s_pieces = build_s_of_t_pieces(&profile, 0.0);
        assert_eq!(s_pieces.pieces.len(), 2);
        assert!(s_pieces.near_zero[0]);
        assert!(s_pieces.near_zero[1]);

        // Near-zero pieces should have zero velocity and acceleration coefficients.
        for piece in &s_pieces.pieces {
            assert!(
                piece.coeffs[1].abs() < 1e-15,
                "near-zero piece should have v=0"
            );
            assert!(
                piece.coeffs[2].abs() < 1e-15,
                "near-zero piece should have a/2=0"
            );
        }
    }

    #[test]
    fn s_of_t_global_offset() {
        let profile = uniform_profile(3, 10.0, 100.0);
        let offset = 5.0;
        let s_pieces = build_s_of_t_pieces(&profile, offset);

        assert_eq!(s_pieces.t_start, offset);
        assert!(
            (s_pieces.t_end - (offset + 10.0 / 100.0)).abs() < 1e-12,
            "t_end = {}",
            s_pieces.t_end
        );
        assert_eq!(s_pieces.pieces[0].u_start, offset);
    }

    #[test]
    fn s_of_t_pieces_contiguous() {
        let profile = uniform_profile(6, 25.0, 200.0);
        let s_pieces = build_s_of_t_pieces(&profile, 1.0);

        // Adjacent pieces should share endpoints.
        for k in 0..s_pieces.pieces.len() - 1 {
            assert!(
                (s_pieces.pieces[k].u_end - s_pieces.pieces[k + 1].u_start).abs() < 1e-15,
                "pieces {} and {} are not contiguous",
                k,
                k + 1
            );
        }
    }

    #[test]
    fn compose_straight_line_constant_velocity() {
        // Straight line from (0,0,0) to (50,0,0), uniform velocity 500 mm/s.
        let curve = nurbs::VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
            None,
        )
        .unwrap();

        let table = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 1024).unwrap();

        let profile = uniform_profile(11, table.s_max(), 500.0);
        let s_pieces = build_s_of_t_pieces(&profile, 0.0);

        let composed = compose_segment(&curve, &table.as_view(), &s_pieces, 1e-4).unwrap();

        assert_eq!(composed.len(), s_pieces.pieces.len());

        // At t=0, x should be 0; at t=total_duration, x should be ~50.
        let first = &composed[0];
        let x_at_start = first[0].evaluate(first[0].u_start);
        assert!(
            x_at_start.abs() < 1e-6,
            "x(t=0) = {x_at_start}, expected ~0"
        );

        let last = &composed[composed.len() - 1];
        let x_at_end = last[0].evaluate(last[0].u_end);
        assert!(
            (x_at_end - 50.0).abs() < 0.1,
            "x(t_end) = {x_at_end}, expected ~50"
        );

        // Y and Z should remain ~0 throughout.
        for pieces_k in &composed {
            let y_mid = pieces_k[1].evaluate(
                (pieces_k[1].u_start + pieces_k[1].u_end) / 2.0,
            );
            let z_mid = pieces_k[2].evaluate(
                (pieces_k[2].u_start + pieces_k[2].u_end) / 2.0,
            );
            assert!(y_mid.abs() < 1e-6, "y should be ~0, got {y_mid}");
            assert!(z_mid.abs() < 1e-6, "z should be ~0, got {z_mid}");
        }

        // X should be monotonically increasing: check at piece boundaries.
        let mut prev_x = f64::NEG_INFINITY;
        for pieces_k in &composed {
            let x_start = pieces_k[0].evaluate(pieces_k[0].u_start);
            assert!(
                x_start >= prev_x - 1e-9,
                "x not monotone: prev={prev_x}, curr={x_start}"
            );
            prev_x = pieces_k[0].evaluate(pieces_k[0].u_end);
        }
    }

    #[test]
    fn compose_diagonal_line() {
        // Diagonal from (0,0,0) to (30,40,0): arc length = 50.
        let curve = nurbs::VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [30.0, 40.0, 0.0]],
            None,
        )
        .unwrap();

        let table = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 1024).unwrap();
        assert!(
            (table.s_max() - 50.0_f64).abs() < 0.01,
            "arc length = {}, expected 50",
            table.s_max()
        );

        let profile = uniform_profile(6, table.s_max(), 250.0);
        let s_pieces = build_s_of_t_pieces(&profile, 0.0);
        let composed = compose_segment(&curve, &table.as_view(), &s_pieces, 1e-4).unwrap();

        // At the end, position should be ~(30, 40, 0).
        let last = &composed[composed.len() - 1];
        let x_end = last[0].evaluate(last[0].u_end);
        let y_end = last[1].evaluate(last[1].u_end);
        let z_end = last[2].evaluate(last[2].u_end);

        assert!(
            (x_end - 30.0).abs() < 0.5,
            "x_end = {x_end}, expected ~30"
        );
        assert!(
            (y_end - 40.0).abs() < 0.5,
            "y_end = {y_end}, expected ~40"
        );
        assert!(z_end.abs() < 1e-6, "z_end = {z_end}, expected ~0");
    }
}
