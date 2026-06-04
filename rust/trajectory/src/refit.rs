use nurbs::algebra::{fit_hermite_c1, FitError};
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at, BezierPiece};
use nurbs::ScalarNurbs;

/// L∞ refit tolerance (0.1 µm — sub-motor-resolution at 80 steps/mm).
pub const REFIT_TOLERANCE_MM: f64 = 1.0e-4;

const MAX_SUBDIVISIONS: usize = 8;
const MIN_PIECE_DURATION: f64 = 1e-12;

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
mod tests;
