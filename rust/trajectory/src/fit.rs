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
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use nurbs::bezier::BezierPiece;

    #[test]
    fn fit_and_split_linear_pieces() {
        // 4 linear pieces: x=t, y=0.5t, z=0 on [0, 4].
        // BezierPiece coeffs are Pascal-shifted-monomial: [a0, a1] => a0 + a1*(u - u_start).
        let composed: Vec<[BezierPiece<f64>; 3]> = (0..4)
            .map(|i| {
                let s = f64::from(i);
                [
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![s, 1.0],
                    },
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![s * 0.5, 0.5],
                    },
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![0.0, 0.0],
                    },
                ]
            })
            .collect();

        let result = fit_and_split(&composed, 0.005).unwrap();
        assert!((result.t_start - 0.0).abs() < 1e-12);
        assert!((result.t_end - 4.0).abs() < 1e-12);

        // Each axis should be a valid ScalarNurbs with at least one control point.
        for axis in &result.axes {
            assert!(!axis.control_points().is_empty());
        }

        // X(0) = 0, X(4) = 4
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
        let x_start = x_pieces[0].evaluate(0.0);
        let x_end = x_pieces.last().unwrap().evaluate(4.0);
        assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
        assert!((x_end - 4.0).abs() < 1e-8, "X(4) = {x_end}, expected 4");
    }

    #[test]
    fn fit_and_split_empty_returns_error() {
        let result = fit_and_split(&[], 0.005);
        assert!(matches!(result, Err(crate::ShapeError::EmptySegments)));
    }

    #[test]
    fn fit_and_split_drops_zero_duration_input_piece() {
        let composed: Vec<[BezierPiece<f64>; 3]> = vec![
            [
                BezierPiece {
                    u_start: 0.0,
                    u_end: 0.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 0.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 0.0,
                    coeffs: vec![0.0],
                },
            ],
            [
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0, 1.0],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0],
                },
            ],
        ];

        let result = fit_and_split(&composed, 0.005).unwrap();
        assert_eq!(result.t_start, 0.0);
        assert_eq!(result.t_end, 1.0);

        for axis in &result.axes {
            for piece in nurbs::bezier::extract_bezier_pieces(axis) {
                assert!(piece.u_start.is_finite());
                assert!(piece.u_end.is_finite());
                assert!(piece.u_end > piece.u_start);
                assert!(piece.coeffs.iter().all(|c| c.is_finite()));
            }
        }
    }

    #[test]
    fn fit_and_split_preserves_endpoints() {
        // Two quadratic pieces: x = 0.5t^2 on [0,1] and x = 0.5 + t - 0.5 on [1,2].
        // Split into two pieces so the fitter has bisection room.
        // Piece 0: x = 0.5*(u-0)^2 on [0,1], so x(0)=0, x(1)=0.5.
        // Piece 1: x = 0.5 + 1.0*(u-1) on [1,2], so x(1)=0.5, x(2)=1.5.
        let composed: Vec<[BezierPiece<f64>; 3]> = vec![
            [
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0, 0.0, 0.5],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0, 1.0],
                },
                BezierPiece {
                    u_start: 0.0,
                    u_end: 1.0,
                    coeffs: vec![0.0],
                },
            ],
            [
                BezierPiece {
                    u_start: 1.0,
                    u_end: 2.0,
                    coeffs: vec![0.5, 1.0],
                },
                BezierPiece {
                    u_start: 1.0,
                    u_end: 2.0,
                    coeffs: vec![1.0, 1.0],
                },
                BezierPiece {
                    u_start: 1.0,
                    u_end: 2.0,
                    coeffs: vec![0.0],
                },
            ],
        ];
        let result = fit_and_split(&composed, 0.005).unwrap();

        // X at t=0 should be 0, at t=2 should be 1.5.
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
        let x_start = x_pieces[0].evaluate(0.0);
        let x_end = x_pieces.last().unwrap().evaluate(2.0);
        assert!((x_start - 0.0).abs() < 1e-8, "X(0) = {x_start}, expected 0");
        assert!((x_end - 1.5).abs() < 1e-8, "X(2) = {x_end}, expected 1.5");

        // Y at t=0 should be 0, at t=2 should be 2.
        let y_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[1]);
        let y_start = y_pieces[0].evaluate(0.0);
        let y_end = y_pieces.last().unwrap().evaluate(2.0);
        assert!((y_start - 0.0).abs() < 1e-8, "Y(0) = {y_start}, expected 0");
        assert!((y_end - 2.0).abs() < 1e-8, "Y(2) = {y_end}, expected 2");
    }

    #[test]
    fn fit_and_split_reduces_piece_count() {
        // 8 linear pieces that are all part of the same line: x = t.
        // A C1 Hermite fitter should merge these into fewer pieces.
        let composed: Vec<[BezierPiece<f64>; 3]> = (0..8)
            .map(|i| {
                let s = f64::from(i);
                [
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![s, 1.0],
                    },
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![0.0],
                    },
                    BezierPiece {
                        u_start: s,
                        u_end: s + 1.0,
                        coeffs: vec![0.0],
                    },
                ]
            })
            .collect();

        let result = fit_and_split(&composed, 0.005).unwrap();

        // The fitter should produce fewer pieces than the 8 input pieces
        // (linear motion can be represented exactly by a single degree-4 piece).
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&result.axes[0]);
        assert!(
            x_pieces.len() < 8,
            "expected piece count reduction, got {} pieces",
            x_pieces.len()
        );
    }
}
