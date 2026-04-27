//! Adaptive-N policy per spec §2.5.

use crate::multi::GridStrategy;
use nurbs::VectorNurbs;

// Wired into plan_batch by Task 9. Until then they appear dead from the
// compiler's perspective. TODO(task-9): drop these attrs when wired up.
#[allow(dead_code)]
pub(crate) fn compute_n(strategy: &GridStrategy, curve: &VectorNurbs<f64, 3>) -> usize {
    match *strategy {
        GridStrategy::Fixed(n) => n,
        GridStrategy::Adaptive {
            min_n,
            max_n,
            target_grid_spacing_mm,
        } => {
            debug_assert!(
                target_grid_spacing_mm > 0.0,
                "target_grid_spacing_mm must be > 0; got {target_grid_spacing_mm}"
            );
            let l = control_polygon_length_mm(curve);
            // `l / target_grid_spacing_mm` is non-negative (both positive by construction)
            // and bounded by `max_n` after the clamp, so truncation is lossless.
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            let n = (l / target_grid_spacing_mm).ceil() as usize;
            n.clamp(min_n, max_n)
        }
    }
}

/// Returns control-polygon length (sum of `‖cp[i+1] − cp[i]‖`).
///
/// For non-rational degree-1 NURBS this equals arclength exactly; for
/// higher-degree or rational curves it is a strict upper bound — `compute_n`
/// only uses it as a heuristic for grid-density and the `max_n` clamp absorbs
/// over-estimates.
#[allow(dead_code)]
fn control_polygon_length_mm(curve: &VectorNurbs<f64, 3>) -> f64 {
    let cps = curve.control_points();
    cps.windows(2)
        .map(|w| {
            let dx = w[1][0] - w[0][0];
            let dy = w[1][1] - w[0][1];
            let dz = w[1][2] - w[0][2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_100mm() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
            None,
        )
        .unwrap()
    }

    #[test]
    fn fixed_strategy_returns_n_unchanged() {
        let curve = straight_100mm();
        assert_eq!(compute_n(&GridStrategy::Fixed(50), &curve), 50);
        assert_eq!(compute_n(&GridStrategy::Fixed(200), &curve), 200);
    }

    #[test]
    fn adaptive_short_segment_floors_to_min_n() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], // 1 mm
            None,
        )
        .unwrap();
        let strategy = GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        };
        // 1mm / 0.5mm = 2; clamped to min_n = 10.
        assert_eq!(compute_n(&strategy, &curve), 10);
    }

    #[test]
    fn adaptive_typical_segment_scales_with_arclength() {
        let curve_50 = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        let strategy = GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        };
        // 50mm / 0.5mm = 100.
        assert_eq!(compute_n(&strategy, &curve_50), 100);
    }

    #[test]
    fn adaptive_long_segment_caps_to_max_n() {
        let curve_200mm = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        let strategy = GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        };
        // 200mm / 0.5mm = 400; clamped to max_n = 200.
        assert_eq!(compute_n(&strategy, &curve_200mm), 200);
    }

    #[test]
    fn adaptive_zero_length_segment_floors_to_min_n() {
        // Degenerate G1 with two identical control points — no path length.
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]], // zero-length
            None,
        )
        .unwrap();
        let strategy = GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        };
        // 0 / 0.5 = 0 → ceil 0 → clamp to min_n = 10.
        assert_eq!(compute_n(&strategy, &curve), 10);
    }

    #[test]
    fn adaptive_curved_segment_uses_polygon_upper_bound() {
        // Rational quadratic quarter-arc, radius 10 mm. True arclength = π·10/2 ≈
        // 15.71 mm, but control-polygon length = 10 + 10 = 20 mm (the polygon
        // overshoots arclength by ~27% on a 90° arc).
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let curve = VectorNurbs::<f64, 3>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[10.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]],
            Some(vec![1.0, w, 1.0]),
        )
        .unwrap();
        let strategy = GridStrategy::Adaptive {
            min_n: 10,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        };
        // control_polygon_length = 20 mm; 20 / 0.5 = 40 → N = 40.
        // (True arclength would have given ceil(15.71/0.5) = 32 — the upper bound
        // over-densifies by ~25% on this geometry, which is the documented behavior.)
        assert_eq!(compute_n(&strategy, &curve), 40);
    }
}
