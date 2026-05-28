// Stage 3b: post-shape cubic refit.
//
// After Stage 3 (per-axis convolution with the smooth shaper kernel), each
// axis's `ScalarNurbs<f64>` comes out at degree `d_fit + d_kernel + 1` — for
// the smooth-MZV degree-4 kernel and Hermite-refit degree-4 input, that's
// degree 9 (Step-7A spec, "Degree and piece-count budget" table).
//
// f32 De Boor evaluation on degree-9 NURBS suffers from catastrophic
// cancellation when control-point magnitudes grow large relative to the
// per-tick position delta — observed on the H7 as ≥ 0.8 mm position spikes
// that trip `KALICO_FAULT_STEP_BURST_EXCEEDED` (-21).
//
// This stage refits each axis to a chain of cubic Bézier pieces (degree 3)
// with C¹ continuity and bounded L∞ residual. Cubic De Boor in f32 is
// well-conditioned, restoring CLAUDE.md's "uniform cubic Bézier across
// Layer 1/2/3/4" mandate at the post-shape boundary.
//
// Closes the deferred-fix entry from `docs/superpowers/plan-changes-log.md`
// 2026-05-05 ("MCU step-burst cap raised 16 → 64 (deferred-fix workaround)").

use nurbs::algebra::{fit_hermite_c1, FitError};
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at, BezierPiece};
use nurbs::ScalarNurbs;

/// L∞ refit tolerance — 0.1 µm. Matches the Goldapp 1991 cubic-fit tolerance
/// CLAUDE.md uses for the Step-13 compat layer's arc reduction; sub-motor-
/// resolution everywhere (motor resolution ≈ 2.5 µm at 80 steps/mm).
pub const REFIT_TOLERANCE_MM: f64 = 1.0e-4;

/// Maximum bisection depth for the adaptive split-and-retry loop. Each level
/// doubles the input piece count; bound at 8 to keep the worst-case piece
/// count bounded (≤ 256× input) while preserving correctness on pathological
/// curves.
const MAX_SUBDIVISIONS: usize = 8;

/// Skip the floor below which an input piece is treated as already-degenerate.
/// Same constant as `fit_and_split`'s Hermite-merge stage.
const MIN_PIECE_DURATION: f64 = 1e-12;

/// Refit a high-degree post-shape `ScalarNurbs<f64>` to a chain of cubic
/// Bézier pieces with C¹ continuity. Output covers the same parameter domain
/// with L∞ position error ≤ `tolerance_mm`.
///
/// Strategy mirrors `fit::fit_hermite_c1_adaptive` (D=3 vector variant):
/// 1. Extract the input curve's Bézier pieces.
/// 2. Wrap each piece as a 1-axis array and call `fit_hermite_c1::<1>` with
///    `target_degree = 3`. Internally this merges runs of input pieces into
///    single cubic outputs where tolerance permits, and bisects at input
///    boundaries when it doesn't.
/// 3. If `fit_hermite_c1` returns `ToleranceNotReached` (a single input
///    piece can't be cubic-fit), split every input piece at its midpoint
///    and retry. Bounded at `MAX_SUBDIVISIONS` levels.
/// 4. Recompose the per-piece output to a `ScalarNurbs<f64>`.
///
/// Idempotent for already-cubic input: `fit_hermite_c1` accepts each piece
/// as its own cubic at zero residual.
pub fn refit_to_cubic(
    curve: &ScalarNurbs<f64>,
    tolerance_mm: f64,
) -> Result<ScalarNurbs<f64>, FitError> {
    let pieces_in = extract_bezier_pieces(curve);
    if pieces_in.is_empty() {
        return Err(FitError::DegenerateInput {
            reason: "refit_to_cubic: no Bézier pieces in input",
        });
    }

    // Wrap each piece as a 1-axis array for fit_hermite_c1::<1>.
    let mut wrapped: Vec<[BezierPiece<f64>; 1]> = pieces_in.into_iter().map(|p| [p]).collect();

    for depth in 0..=MAX_SUBDIVISIONS {
        match fit_hermite_c1::<1>(&wrapped, tolerance_mm, 3) {
            Ok(fitted) => return Ok(bezier_pieces_to_nurbs(&fitted[0])),
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == MAX_SUBDIVISIONS {
                    return Err(err);
                }
                wrapped = split_at_midpoints(wrapped)?;
            }
            Err(err) => return Err(err),
        }
    }

    unreachable!("bounded refit loop always returns before exhausting range")
}

/// Halve every input piece at its parameter midpoint. Degenerate pieces
/// (non-finite or sub-`MIN_PIECE_DURATION`) pass through unchanged so the
/// next `fit_hermite_c1` call surfaces them as a hard error rather than
/// looping indefinitely on a zero-width input.
fn split_at_midpoints(
    pieces: Vec<[BezierPiece<f64>; 1]>,
) -> Result<Vec<[BezierPiece<f64>; 1]>, FitError> {
    let mut refined: Vec<[BezierPiece<f64>; 1]> = Vec::with_capacity(pieces.len() * 2);

    for piece_arr in pieces {
        let piece = &piece_arr[0];
        let u_start = piece.u_start;
        let u_end = piece.u_end;
        let duration = u_end - u_start;

        if !duration.is_finite() || duration <= MIN_PIECE_DURATION {
            refined.push(piece_arr);
            continue;
        }

        let u_mid = 0.5 * (u_start + u_end);
        if !u_mid.is_finite() || u_mid <= u_start || u_mid >= u_end {
            return Err(FitError::DegenerateInput {
                reason: "refit_to_cubic: midpoint split produced a degenerate piece",
            });
        }

        let (left, right) = split_piece_at(piece, u_mid);
        refined.push([left]);
        refined.push([right]);
    }

    Ok(refined)
}

#[cfg(test)]
#[allow(clippy::doc_markdown, clippy::cast_lossless)]
mod tests {
    use super::*;
    use nurbs::eval::eval;

    /// Build a simple degree-1 ScalarNurbs spanning u ∈ [0, 1] with values
    /// [v_start, v_end]. Used to exercise the already-low-degree passthrough.
    fn linear_curve(v_start: f64, v_end: f64) -> ScalarNurbs<f64> {
        // Degree 1, knots = [0, 0, 1, 1], cps = [v_start, v_end].
        ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v_start, v_end], None)
            .expect("linear NURBS construction")
    }

    #[test]
    fn refits_linear_passthrough_within_tolerance() {
        // Linear input is already cubic-representable (with degenerate higher
        // coeffs). Expect ~0 residual.
        let input = linear_curve(0.0, 5.0);
        let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");
        // Sample 33 points and compare against analytic v(u) = 5u.
        for i in 0..=32 {
            let u = (i as f64) / 32.0;
            let truth = 5.0 * u;
            let v = eval(&output.as_view(), u);
            assert!(
                (truth - v).abs() <= REFIT_TOLERANCE_MM,
                "linear residual at u={u}: truth={truth}, refit={v}"
            );
        }
    }

    #[test]
    fn refits_high_degree_polynomial_within_tolerance() {
        // Build a degree-9 BezierPiece. We don't construct an exact analytic
        // identity — the input's pieced representation IS the truth, and we
        // verify the refit reproduces it within tolerance.
        let p = 9_usize;
        let cps: Vec<f64> = (0..=p)
            .map(|i| {
                let u = (i as f64) / (p as f64);
                100.0 + 5.0 * (2.0 * std::f64::consts::PI * u).sin()
            })
            .collect();
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&cps, 0.0, 1.0);
        let input = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);

        let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");

        for i in 0..=200 {
            let u = (i as f64) / 200.0;
            let truth = eval(&input.as_view(), u);
            let refit = eval(&output.as_view(), u);
            let diff = (truth - refit).abs();
            assert!(
                diff <= REFIT_TOLERANCE_MM * 1.5,
                "residual at u={u}: input={truth}, refit={refit}, diff={diff}"
            );
        }

        assert_eq!(output.degree(), 3, "refit output should be cubic");
    }

    #[test]
    fn refit_is_idempotent_on_cubic_input() {
        let cps = vec![0.0, 1.5, 2.5, 4.0];
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&cps, 0.0, 1.0);
        let input = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);
        let output = refit_to_cubic(&input, REFIT_TOLERANCE_MM).expect("refit succeeds");
        for i in 0..=64 {
            let u = (i as f64) / 64.0;
            let truth = eval(&input.as_view(), u);
            let refit = eval(&output.as_view(), u);
            let diff = (truth - refit).abs();
            assert!(
                diff <= 1e-9,
                "cubic should be reproduced exactly: u={u}, diff={diff}"
            );
        }
        assert_eq!(output.degree(), 3);
    }
}
