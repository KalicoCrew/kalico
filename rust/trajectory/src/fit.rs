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

    let d2_start = start_d2_override.unwrap_or_else(|| boundary_second_derivative_start(&fit_input));
    let d2_end = boundary_second_derivative_end(&fit_input);

    let mut fitted = fit_hermite_c2_adaptive(&fit_input, tolerance, d2_start, d2_end).map_err(|e| {
        crate::ShapeError::FitFailure {
            index: 0,
            detail: e,
        }
    })?;

    // Normalize all axes to a uniform degree (the max across all pieces on any
    // axis).  Phase-2 re-fitting produces degree-5 pieces for the last output
    // piece while Phase-1 produces degree-4 for the others; `bezier_pieces_to_nurbs`
    // requires a uniform degree throughout.
    let max_degree = fitted
        .iter()
        .flat_map(|axis_pieces| axis_pieces.iter().map(|p| p.coeffs.len().saturating_sub(1)))
        .max()
        .unwrap_or(4);
    for axis_pieces in fitted.iter_mut() {
        for piece in axis_pieces.iter_mut() {
            while piece.coeffs.len() <= max_degree {
                piece.coeffs.push(0.0);
            }
        }
    }

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

fn boundary_second_derivative_start(composed: &[[BezierPiece<f64>; 3]]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let piece = &composed[0][axis];
        piece.differentiate().differentiate().evaluate(piece.u_start)
    })
}

fn boundary_second_derivative_end(composed: &[[BezierPiece<f64>; 3]]) -> [f64; 3] {
    std::array::from_fn(|axis| {
        let piece = composed.last().unwrap()[axis].clone();
        piece.differentiate().differentiate().evaluate(piece.u_end)
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
    d2_end: [f64; 3],
) -> Result<[Vec<BezierPiece<f64>>; 3], nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1_clamped, FitError};

    // Phase 1: run the original start-pin-only fit (degree-4) to get a
    // well-behaved velocity profile.  This is unchanged from the prior
    // single-pin implementation.
    let mut refined = composed.to_vec();
    let mut fitted: [Vec<BezierPiece<f64>>; 3] = std::array::from_fn(|_| Vec::new());

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        match fit_hermite_c1_clamped::<3>(&refined, tolerance, 4, Some(d2_start), None) {
            Ok(f) => {
                fitted = f;
                break;
            }
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == HERMITE_REFIT_MAX_SUBDIVISIONS {
                    return Err(err);
                }
                refined = split_composed_midpoints(&refined)?;
            }
            Err(err) => return Err(err),
        }
    }

    // Phase 2: re-fit the last output piece with the end-accel pin added.
    // We operate on the same `refined` array used by Phase 1 and locate the
    // composed pieces that were merged into the last output piece.
    refit_last_piece_with_end_pin(&mut fitted, &refined, tolerance, d2_end)?;

    Ok(fitted)
}

fn refit_last_piece_with_end_pin(
    fitted: &mut [Vec<BezierPiece<f64>>; 3],
    refined: &[[BezierPiece<f64>; 3]],
    tolerance: f64,
    d2_end: [f64; 3],
) -> Result<(), nurbs::algebra::FitError> {
    use nurbs::algebra::{fit_hermite_c1_clamped, FitError};

    // Find the time range of the last Phase-1 output piece (axis 0 is
    // representative; all axes have pieces over the same time domain).
    let last_out_start = match fitted[0].last() {
        Some(p) => p.u_start,
        None => return Ok(()),
    };

    // Collect the refined composed pieces that were folded into the last output
    // piece: those whose time range falls within [last_out_start, global_end].
    let last_refined: Vec<[BezierPiece<f64>; 3]> = refined
        .iter()
        .filter(|ps| ps[0].u_start >= last_out_start - 1e-12)
        .cloned()
        .collect();

    if last_refined.is_empty() {
        return Ok(());
    }

    // The start accel pin for Phase 2 is the composed accel at last_out_start
    // (read from the first last-range composed piece) so that the new last
    // piece is C2 with the preceding Phase-1 piece on its left side as well.
    let last_d2_start: [f64; 3] = std::array::from_fn(|axis| {
        let piece = &last_refined[0][axis];
        piece.differentiate().differentiate().evaluate(piece.u_start)
    });

    // Re-fit the last piece range with BOTH accel pins at degree-5. degree-5
    // is required because the both-ends pin (start + end accel) imposes 6
    // constraints that exactly determine a degree-5 polynomial; a single
    // both-pinned piece is numerically well-behaved for the short time span
    // covered by the last output piece.
    let mut refined_last = last_refined;
    let mut last_fitted: Option<[Vec<BezierPiece<f64>>; 3]> = None;

    for depth in 0..=HERMITE_REFIT_MAX_SUBDIVISIONS {
        match fit_hermite_c1_clamped::<3>(
            &refined_last,
            tolerance,
            5,
            Some(last_d2_start),
            Some(d2_end),
        ) {
            Ok(f) => {
                last_fitted = Some(f);
                break;
            }
            Err(err @ FitError::ToleranceNotReached { .. }) => {
                if depth == HERMITE_REFIT_MAX_SUBDIVISIONS {
                    return Err(err);
                }
                refined_last = split_composed_midpoints(&refined_last)?;
            }
            Err(err) => return Err(err),
        }
    }

    if let Some(new_last) = last_fitted {
        for axis in 0..3 {
            // Remove the Phase-1 last piece (degree-4, end-free).
            fitted[axis].pop();
            // Append the re-fitted last piece(s) (degree-5, end pinned).
            fitted[axis].extend(new_last[axis].iter().cloned());
        }
    }

    Ok(())
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
