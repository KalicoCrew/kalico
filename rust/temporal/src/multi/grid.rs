use crate::multi::GridStrategy;
use crate::topp::chain::MAX_JUNCTION_SPACING_RATIO;
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
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let n = (l / target_grid_spacing_mm).ceil() as usize;
            n.clamp(min_n, max_n)
        }
    }
}

/// Adjust per-segment node counts so that at every junction the boundary
/// interval ratio stays within `MAX_JUNCTION_SPACING_RATIO`. Each entry in
/// `ns` corresponds to the matching entry in `curves`; the two slices must be
/// the same length and represent consecutive segments in one chain.
///
/// Strategy (single forward pass, then single backward pass):
///
/// For each junction (i, i+1) the boundary spacing is `h = L/(n-1)` where L
/// is the control-polygon length.  When `h[i]/h[i+1] > MAX`:
/// 1. Increase `n[i]` toward `max_n` to reduce `h[i]`.
/// 2. If `max_n` is insufficient, reduce `n[i+1]` (floor toward 2) to raise
///    `h[i+1]` until the ratio is within bound.
///
/// This never increases n beyond `max_n` and never decreases n below 2, so
/// the resulting grids are always valid inputs to `sample_arclength_grid`.
/// `Fixed` grids are left unchanged.
pub(crate) fn reconcile_junction_n(
    ns: &mut [usize],
    curves: &[&VectorNurbs<f64, 3>],
    max_n: Option<usize>,
) {
    debug_assert_eq!(ns.len(), curves.len());
    if ns.len() < 2 {
        return;
    }

    let lengths: Vec<f64> = curves
        .iter()
        .map(|c| control_polygon_length_mm(c))
        .collect();

    let h = |n: usize, l: f64| -> f64 { if n <= 1 { l } else { l / (n - 1) as f64 } };

    let reconcile_pair =
        |n_left: &mut usize, n_right: &mut usize, l_left: f64, l_right: f64, cap: Option<usize>| {
            let hl = h(*n_left, l_left);
            let hr = h(*n_right, l_right);

            if hl <= 0.0 || hr <= 0.0 {
                return;
            }

            let ratio = hl / hr;
            if ratio > MAX_JUNCTION_SPACING_RATIO {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n_needed = (l_left / (hr * MAX_JUNCTION_SPACING_RATIO)).ceil() as usize + 1;
                let n_new = match cap {
                    Some(c) => n_needed.min(c).max(*n_left),
                    None => n_needed.max(*n_left),
                };
                *n_left = n_new;
                let hl_new = h(*n_left, l_left);
                if hl_new > hr * MAX_JUNCTION_SPACING_RATIO {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let n_right_max =
                        (l_right * MAX_JUNCTION_SPACING_RATIO / hl_new).floor() as usize + 1;
                    *n_right = (*n_right).min(n_right_max).max(2);
                }
            } else if hr / hl > MAX_JUNCTION_SPACING_RATIO {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n_needed = (l_right / (hl * MAX_JUNCTION_SPACING_RATIO)).ceil() as usize + 1;
                let n_new = match cap {
                    Some(c) => n_needed.min(c).max(*n_right),
                    None => n_needed.max(*n_right),
                };
                *n_right = n_new;
                let hr_new = h(*n_right, l_right);
                if hr_new > hl * MAX_JUNCTION_SPACING_RATIO {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let n_left_max =
                        (l_left * MAX_JUNCTION_SPACING_RATIO / hr_new).floor() as usize + 1;
                    *n_left = (*n_left).min(n_left_max).max(2);
                }
            }
        };

    for i in 0..ns.len() - 1 {
        let (left, right) = ns.split_at_mut(i + 1);
        reconcile_pair(
            &mut left[i],
            &mut right[0],
            lengths[i],
            lengths[i + 1],
            max_n,
        );
    }

    for i in (0..ns.len() - 1).rev() {
        let (left, right) = ns.split_at_mut(i + 1);
        reconcile_pair(
            &mut left[i],
            &mut right[0],
            lengths[i],
            lengths[i + 1],
            max_n,
        );
    }
}

/// Control-polygon length (sum of `‖cp[i+1] − cp[i]‖`).
///
/// For non-rational degree-1 NURBS this equals arclength exactly; for
/// higher-degree or rational curves it is a strict upper bound — used only
/// as a heuristic for grid-density.
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
