use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

const HERMITE_REFIT_MAX_SUBDIVISIONS: usize = 8;
const MIN_HERMITE_PIECE_DURATION: f64 = 1e-12;

#[derive(Debug, Clone)]
pub struct FittedSegment {
    pub axes: [ScalarNurbs<f64>; 3],
    pub t_start: f64,
    pub t_end: f64,
}

pub fn fit_and_split(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    start_d2_override: Option<[f64; 3]>,
) -> Result<FittedSegment, crate::ShapeError> {
    use nurbs::bezier::bezier_pieces_to_nurbs;

    if composed.is_empty() {
        return Err(crate::ShapeError::EmptySegments);
    }

    let t_start = composed[0][0].u_start;
    let t_end = composed.last().unwrap()[0].u_end;

    let fit_input = nondegenerate_composed_pieces(composed)?;

    let d2_start = start_d2_override.unwrap_or_else(|| boundary_second_derivative(&fit_input));

    let fitted = fit_hermite_c2_adaptive(&fit_input, tolerance, d2_start).map_err(|e| {
        crate::ShapeError::FitFailure {
            index: 0,
            detail: e,
        }
    })?;

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

fn boundary_second_derivative(composed: &[[BezierPiece<f64>; 3]]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let piece = &composed[0][axis];
        piece.differentiate().differentiate().evaluate(piece.u_start)
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

fn fit_hermite_c2_adaptive(
    composed: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    d2_start: [f64; 3],
) -> Result<[Vec<BezierPiece<f64>>; 3], nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1_clamped, FitError};

    let mut refined = composed.to_vec();

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        match fit_hermite_c1_clamped::<3>(&refined, tolerance, 4, Some(d2_start), None) {
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

pub fn split_without_refit(
    composed: &[[BezierPiece<f64>; 3]],
) -> Result<FittedSegment, crate::ShapeError> {
    use nurbs::bezier::bezier_pieces_to_nurbs;

    if composed.is_empty() {
        return Err(crate::ShapeError::EmptySegments);
    }

    let t_start = composed[0][0].u_start;
    let t_end = composed.last().unwrap()[0].u_end;

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
