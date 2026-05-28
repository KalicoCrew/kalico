//! Adaptive-N policy per spec §2.5.

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
            debug_assert!(
                target_grid_spacing_mm > 0.0,
                "target_grid_spacing_mm must be > 0; got {target_grid_spacing_mm}"
            );
            let l = control_polygon_length_mm(curve);
            // `l / target_grid_spacing_mm` is non-negative (both positive by construction)
            // and bounded by `max_n` after the clamp, so truncation is lossless.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
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
mod tests;
