// Stage 2c-d: C1-constrained refit of composed pieces and per-axis split.

use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

const HERMITE_REFIT_MAX_SUBDIVISIONS: usize = 8;
const MIN_HERMITE_PIECE_DURATION: f64 = 1e-12;

/// Result of Stage 2: per-axis fitted trajectories in the time domain.
#[derive(Debug, Clone)]
pub struct FittedSegment {
    /// Per-axis shaped NURBS: `[X(t), Y(t), Z(t)]`.
    pub axes: [ScalarNurbs<f64>; 3],
    /// Start time of this segment (seconds).
    pub t_start: f64,
    /// End time of this segment (seconds).
    pub t_end: f64,
}

/// Stage 2c-d: refit composed pieces via C1 Hermite merge and split to per-axis
/// scalar NURBS.
///
/// **Stage 2c** calls `fit_hermite_c1::<3>` to merge adjacent degree-6 composed
/// pieces into fewer degree-4 pieces with C1 continuity and <= `tolerance`
/// position error (L-infinity across all 3 axes).
///
/// **Stage 2d** converts each axis's `Vec<BezierPiece<f64>>` into a
/// `ScalarNurbs<f64>` via `bezier_pieces_to_nurbs`.
pub fn fit_and_split(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
) -> Result<FittedSegment, crate::ShapeError> {
    use nurbs::bezier::bezier_pieces_to_nurbs;

    if composed.is_empty() {
        return Err(crate::ShapeError::EmptySegments);
    }

    let t_start = composed[0][0].u_start;
    let t_end = composed.last().unwrap()[0].u_end;

    let fit_input = nondegenerate_composed_pieces(composed)?;

    // Stage 2c: C1 Hermite refit — merge adjacent pieces into fewer degree-4
    // pieces while maintaining C1 continuity and L-inf error <= tolerance.
    let fitted = fit_hermite_c1_adaptive(&fit_input, tolerance, 4).map_err(|e| {
        crate::ShapeError::FitFailure {
            index: 0,
            detail: e,
        }
    })?;

    // Stage 2d: convert per-axis Vec<BezierPiece> to ScalarNurbs.
    let axes = [
        bezier_pieces_to_nurbs(&fitted[0]),
        bezier_pieces_to_nurbs(&fitted[1]),
        bezier_pieces_to_nurbs(&fitted[2]),
    ];

    Ok(FittedSegment {
        axes,
        t_start,
        t_end,
    })
}

fn nondegenerate_composed_pieces(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let filtered: Vec<[BezierPiece<f64>; 3]> = composed
        .iter()
        .filter(|piece_set| {
            let duration = piece_set[0].u_end - piece_set[0].u_start;
            duration.is_finite() && duration > MIN_HERMITE_PIECE_DURATION
        })
        .cloned()
        .collect();

    if filtered.is_empty() {
        return Err(crate::ShapeError::FitFailure {
            index: 0,
            detail: nurbs::algebra::FitError::DegenerateInput {
                reason: "fit_and_split: no non-degenerate Hermite input pieces",
            },
        });
    }

    Ok(filtered)
}

fn fit_hermite_c1_adaptive(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    target_degree: u8,
) -> Result<[Vec<BezierPiece<f64>>; 3], nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1, FitError};

    let mut refined = composed.to_vec();

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        match fit_hermite_c1::<3>(&refined, tolerance, target_degree) {
            Ok(fitted) => return Ok(fitted),
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == HERMITE_REFIT_MAX_SUBDIVISIONS {
                    return Err(err);
                }
                refined = split_composed_midpoints(&refined)?;
            }
            Err(err) => return Err(err),
        }
    }

    unreachable!("bounded Hermite refit loop always returns before exhausting range")
}

fn split_composed_midpoints(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<Vec<[BezierPiece<f64>; 3]>, nurbs::algebra::FitError> {
    use nurbs::algebra::FitError;
    use nurbs::bezier::split_piece_at;

    let mut refined = Vec::with_capacity(composed.len() * 2);

    for piece_set in composed {
        let u_start = piece_set[0].u_start;
        let u_end = piece_set[0].u_end;
        let duration = u_end - u_start;

        if !duration.is_finite() || duration <= MIN_HERMITE_PIECE_DURATION {
            refined.push(piece_set.clone());
            continue;
        }

        let u_mid = 0.5 * (u_start + u_end);

        if !u_mid.is_finite() || u_mid <= u_start || u_mid >= u_end {
            return Err(FitError::DegenerateInput {
                reason: "fit_and_split: cannot split degenerate Hermite input piece",
            });
        }

        let left: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| {
            let (left, _) = split_piece_at(&piece_set[axis], u_mid);
            left
        });
        let right: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| {
            let (_, right) = split_piece_at(&piece_set[axis], u_mid);
            right
        });

        refined.push(left);
        refined.push(right);
    }

    Ok(refined)
}

/// Convert composed pieces directly to per-axis `ScalarNurbs` WITHOUT the
/// C1 Hermite merge. Preserves exact polynomial accuracy of the composed
/// pieces (degree ≤ 6 for typical paths).
///
/// Use this when the beta-medium loop needs accurate second derivatives for
/// peak-acceleration checking. The Hermite refit (`fit_and_split`) can be
/// applied as a post-processing step on the final converged output if
/// piece-count reduction is desired.
pub fn split_without_refit(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<FittedSegment, crate::ShapeError> {
    use nurbs::bezier::bezier_pieces_to_nurbs;

    if composed.is_empty() {
        return Err(crate::ShapeError::EmptySegments);
    }

    let t_start = composed[0][0].u_start;
    let t_end = composed.last().unwrap()[0].u_end;

    // Collect per-axis pieces and convert directly. Single-pass split:
    // walks `composed` once and pushes per-axis clones into preallocated
    // vecs, avoiding three independent iterator passes.
    let mut x_pieces: Vec<BezierPiece<f64>> = Vec::with_capacity(composed.len());
    let mut y_pieces: Vec<BezierPiece<f64>> = Vec::with_capacity(composed.len());
    let mut z_pieces: Vec<BezierPiece<f64>> = Vec::with_capacity(composed.len());
    for arr in composed {
        x_pieces.push(arr[0].clone());
        y_pieces.push(arr[1].clone());
        z_pieces.push(arr[2].clone());
    }

    let axes = [
        bezier_pieces_to_nurbs(&x_pieces),
        bezier_pieces_to_nurbs(&y_pieces),
        bezier_pieces_to_nurbs(&z_pieces),
    ];

    Ok(FittedSegment {
        axes,
        t_start,
        t_end,
    })
}

#[cfg(test)]
mod tests;
