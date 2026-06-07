use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

use crate::beta::kernel_half_support;
use crate::fit::FittedSegment;
use crate::pad::{pad_segment_axis_with_history, EHalo};
use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
use crate::shaper::shape_axis;
use crate::{ShapeError, ShapedSegment};

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

    for (seg_idx, fitted) in planned.iter().enumerate() {
        let t_start = fitted.t_start;
        let t_end = fitted.t_end;

        let mut shaped_axes: [Option<ScalarNurbs<f64>>; 3] = [None, None, None];

        for axis in 0..3 {
            let cps = fitted.axes[axis].control_points();
            let axis_is_constant = if let Some(&first) = cps.first() {
                cps.iter().all(|c| (*c - first).abs() < 1e-12)
            } else {
                true
            };

            let mut axis_shaped = if axis_is_constant {
                fitted.axes[axis].clone()
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
                shape_axis(&padded, kernel, t_start, t_end)
            } else {
                fitted.axes[axis].clone()
            };

            if !axis_is_constant {
                axis_shaped =
                    refit_to_cubic(&axis_shaped, REFIT_TOLERANCE_MM).map_err(|detail| {
                        ShapeError::FitFailure {
                            index: seg_idx,
                            detail,
                        }
                    })?;
            }

            shaped_axes[axis] = Some(axis_shaped);
        }

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
