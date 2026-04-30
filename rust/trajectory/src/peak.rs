// Post-shape peak acceleration check.
//
// Stage 4: compute the peak (maximum absolute) acceleration of a shaped
// `ScalarNurbs<f64>` by differentiating twice, finding critical points of
// |x''(t)| via the roots of x'''(t), and evaluating at those points plus
// piece endpoints.

use nurbs::bezier::extract_bezier_pieces;
use nurbs::ScalarNurbs;

/// Compute the peak absolute acceleration (max |x''(t)|) of a shaped trajectory.
pub fn peak_accel(curve: &ScalarNurbs<f64>) -> f64 {
    let pieces = extract_bezier_pieces(curve);
    let mut global_max: f64 = 0.0;

    for piece in &pieces {
        // x'(t)
        let d1 = piece.differentiate();
        // x''(t) — the acceleration
        let d2 = d1.differentiate();
        // x'''(t) — the jerk; roots are critical points of |x''(t)|
        let d3 = d2.differentiate();

        // Evaluate |x''(t)| at piece endpoints.
        let mut piece_max = d2.evaluate(piece.u_start).abs();
        piece_max = piece_max.max(d2.evaluate(piece.u_end).abs());

        // Evaluate |x''(t)| at each critical point (root of x'''(t)).
        for root in d3.real_roots_in_domain() {
            piece_max = piece_max.max(d2.evaluate(root).abs());
        }

        global_max = global_max.max(piece_max);
    }

    global_max
}

#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

    #[test]
    fn peak_accel_of_constant_is_zero() {
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![5.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        assert!(peak_accel(&curve).abs() < 1e-12);
    }

    #[test]
    fn peak_accel_of_linear_is_zero() {
        // x(t) = 3 + 2t -> x''(t) = 0
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![3.0, 2.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        assert!(peak_accel(&curve).abs() < 1e-12);
    }

    #[test]
    fn peak_accel_of_quadratic() {
        // x(t) = 5t^2 -> x''(t) = 10 (constant)
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 0.0, 5.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        assert!(
            (peak_accel(&curve) - 10.0).abs() < 1e-10,
            "expected 10.0, got {}",
            peak_accel(&curve)
        );
    }

    #[test]
    fn peak_accel_of_cubic() {
        // x(t) = t^3 on [0, 2] -> x''(t) = 6t -> peak at t=2: |x''(2)| = 12
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 0.0, 0.0, 1.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        assert!(
            (peak_accel(&curve) - 12.0).abs() < 1e-8,
            "expected 12.0, got {}",
            peak_accel(&curve)
        );
    }

    #[test]
    fn peak_accel_multi_piece() {
        // Two quadratic pieces: first has low accel (2), second has high accel (100)
        // bezier_pieces_to_nurbs requires consistent degrees across pieces.
        let p1 = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, 1.0],
        }; // x(t) = t + t^2, x''(t) = 2
        let p2 = BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![2.0, 1.0, 50.0],
        }; // quadratic, x''(t) = 100
        let curve = bezier_pieces_to_nurbs(&[p1, p2]);
        assert!(
            (peak_accel(&curve) - 100.0).abs() < 1e-8,
            "expected 100.0, got {}",
            peak_accel(&curve)
        );
    }

    #[test]
    fn peak_accel_interior_extremum() {
        // x(t) = t - t^3 -> x''(t) = -6t -> peak at endpoints: max(|0|, |-6|) = 6
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, 0.0, -1.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        assert!(
            (peak_accel(&curve) - 6.0).abs() < 1e-8,
            "expected 6.0, got {}",
            peak_accel(&curve)
        );
    }
}
