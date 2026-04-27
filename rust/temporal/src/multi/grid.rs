//! Adaptive-N policy per spec §2.5.

// `compute_n` and `arclength_mm` are wired into `plan_batch` in Task 9;
// they are dead-code from the compiler's perspective until then.
#![allow(dead_code)]

use crate::multi::GridStrategy;
use nurbs::VectorNurbs;

pub(crate) fn compute_n(strategy: &GridStrategy, curve: &VectorNurbs<f64, 3>) -> usize {
    match *strategy {
        GridStrategy::Fixed(n) => n,
        GridStrategy::Adaptive {
            min_n,
            max_n,
            target_grid_spacing_mm,
        } => {
            let l = arclength_mm(curve);
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

/// Approximate arclength via control-polygon length (cheap upper-bound estimate;
/// exact arclength would require Layer 0's quadrature which we don't need at this
/// granularity — the policy clamps anyway).
fn arclength_mm(curve: &VectorNurbs<f64, 3>) -> f64 {
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
}
