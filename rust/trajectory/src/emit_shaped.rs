use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use nurbs::eval::eval as nurbs_eval;
use nurbs::ScalarNurbs;

use crate::beta::kernel_half_support;
use crate::fit::FittedSegment;
use crate::pad::{pad_segment_axis_with_history, EHalo};
use crate::smooth_fit::fit_c2_cubic_with_bc;
use crate::{ShapeError, ShapedSegment};

const SMOOTH_FIT_TOLERANCE_MM: f64 = 5.0e-3;

#[derive(Debug, Clone, Copy)]
pub struct PerAxisHistory<'a> {
    pub axes: [&'a [BezierPiece<f64>]; 4],
}

impl PerAxisHistory<'_> {
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            axes: [&[], &[], &[], &[]],
        }
    }
}

impl Default for PerAxisHistory<'_> {
    fn default() -> Self {
        Self::empty()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EmitSegmentMeta {
    pub e_mode: geometry::segment::EMode,
    pub extrusion_per_xy_mm: f64,
}

pub fn emit_shaped(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    kernels: &[Option<PiecewisePolynomialKernel<f64>>; 4],
    e_halos: &[EHalo],
    history: &PerAxisHistory<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
) -> Result<Vec<ShapedSegment>, ShapeError> {
    emit_shaped_with_left_bc(
        planned,
        meta,
        kernels,
        e_halos,
        history,
        batch_t_start,
        batch_t_end,
        [None, None, None],
    )
}

/// Like [`emit_shaped`] but overrides the left-boundary slope for the FIRST
/// segment's per-axis `fit_c2_cubic` call.  Passing the previous emission's
/// right-boundary slope enforces C1 continuity across the dispatch seam.
pub fn emit_shaped_with_left_bc(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    kernels: &[Option<PiecewisePolynomialKernel<f64>>; 4],
    e_halos: &[EHalo],
    history: &PerAxisHistory<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
    first_seg_left_bc: [Option<f64>; 3],
) -> Result<Vec<ShapedSegment>, ShapeError> {
    debug_assert_eq!(
        planned.len(),
        meta.len(),
        "emit_shaped: planned and meta lengths must match",
    );

    let half_supports = [
        kernels[0].as_ref().map_or(0.0, kernel_half_support),
        kernels[1].as_ref().map_or(0.0, kernel_half_support),
        kernels[2].as_ref().map_or(0.0, kernel_half_support),
    ];

    let mut output: Vec<ShapedSegment> = Vec::with_capacity(planned.len());

    let mut prev_right_bc: [Option<f64>; 3] = first_seg_left_bc;

    for (seg_idx, fitted) in planned.iter().enumerate() {
        let t_start = fitted.t_start;
        let t_end = fitted.t_end;

        let mut shaped_axes: [Option<ScalarNurbs<f64>>; 3] = [None, None, None];
        let mut next_left_bc: [Option<f64>; 3] = [None; 3];

        for axis in 0..3 {
            let cps = fitted.axes[axis].control_points();
            let &first = cps.first().unwrap_or_else(|| {
                panic!("emit_shaped: seg {seg_idx} axis {axis} has empty control points — fitter produced a degenerate FittedSegment")
            });
            let axis_is_constant = cps.iter().all(|c| (*c - first).abs() < 1e-12);

            let left_bc = prev_right_bc[axis];

            let axis_shaped = if axis_is_constant {
                crate::beta::constant_cubic_nurbs(first, t_start, t_end)
            } else if let Some(kernel) = kernels[axis].as_ref() {
                let padded = pad_segment_axis_with_history(
                    seg_idx,
                    axis,
                    planned,
                    e_halos,
                    history.axes[axis],
                    half_supports[axis],
                    batch_t_start,
                    batch_t_end,
                );
                let sig = crate::shaper::ShapedSignal::new(&padded, kernel, t_start, t_end);
                fit_c2_cubic_with_bc(
                    &|t| sig.eval(t),
                    t_start,
                    t_end,
                    SMOOTH_FIT_TOLERANCE_MM,
                    left_bc,
                    None,
                )
                .map_err(|e| ShapeError::FitFailure {
                    index: seg_idx,
                    detail: nurbs::algebra::FitError::ToleranceNotReached {
                        achieved_mm: e.achieved_mm,
                        at_degree: 3,
                    },
                })?
            } else {
                let passthrough = fitted.axes[axis].clone();
                fit_c2_cubic_with_bc(
                    &|t| nurbs_eval(&passthrough, t),
                    t_start,
                    t_end,
                    SMOOTH_FIT_TOLERANCE_MM,
                    left_bc,
                    None,
                )
                .map_err(|e| ShapeError::FitFailure {
                    index: seg_idx,
                    detail: nurbs::algebra::FitError::ToleranceNotReached {
                        achieved_mm: e.achieved_mm,
                        at_degree: 3,
                    },
                })?
            };

            let right_vel = {
                let d1 = nurbs::eval::derivative(&axis_shaped);
                nurbs::eval::eval(&d1, t_end)
            };
            next_left_bc[axis] = Some(right_vel);

            shaped_axes[axis] = Some(axis_shaped);
        }

        prev_right_bc = next_left_bc;

        let m = meta[seg_idx];
        output.push(ShapedSegment {
            axes: [
                shaped_axes[0].take().unwrap(),
                shaped_axes[1].take().unwrap(),
                shaped_axes[2].take().unwrap(),
            ],
            e_mode: m.e_mode,
            extrusion_per_xy_mm: m.extrusion_per_xy_mm,
            e_independent: None,
            t_start,
            t_end,
        });
    }

    Ok(output)
}

#[cfg(test)]
mod tests;
