//! Phase-2 Task-2.2: shaping half of the streaming-shaper split.
//!
//! `emit_shaped` consumes the time-domain β-converged [`FittedSegment`]s
//! produced by [`crate::plan_velocity`] (Task 2.1) and runs the
//! pad → convolve → trim → C¹ refit pipeline that the existing `shape_batch`
//! does inline. It returns one [`ShapedSegment`] per input segment, ready
//! for the wire (E-gap insertion is the caller's job; see
//! [`crate::shape_batch`]).
//!
//! The streaming planner (`ShaperState::emit_committed` in Phase 3) calls
//! this with a non-empty [`PerAxisHistory`] supplying real prior planned
//! pieces as the left-pad source — the convolution sees a continuous time
//! line across `submit_move` boundaries instead of the constant-extension
//! seam that v5 of the spec exists to eliminate.
//!
//! The right-pad still uses constant-extension at `batch_t_end` (current
//! `pad_segment_axis` semantics). Phase 3 swaps the right-pad for held-back
//! planned-tail content; that lives in `ShaperState::emit_committed`'s
//! caller, not here.
//!
//! See `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.3.

use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

use crate::beta::kernel_half_support;
use crate::fit::FittedSegment;
use crate::pad::{pad_segment_axis_with_history, EHalo};
use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
use crate::shaper::shape_axis;
use crate::{ShapeError, ShapedSegment};

/// Per-axis history of prior planned `BezierPiece`s, in absolute time order,
/// immediately preceding `batch_t_start`.
///
/// Slot ordering matches [`crate::plan_velocity::PlanInput::kernels`]:
/// `[X, Y, Z, E]`. The E slot is unused today (the extruder is not a
/// shaped axis); it is kept for forward-compatibility with
/// [`crate::streaming::ShaperState`]'s 4-axis layout.
///
/// Empty per-axis slices fall back to constant-extension at `batch_t_start`,
/// reproducing `pad_segment_axis`'s pre-streaming behaviour byte-for-byte.
#[derive(Debug, Clone, Copy)]
pub struct PerAxisHistory<'a> {
    /// Prior planned pieces per axis, in time order. Each axis is
    /// independent — the history may be present on some axes and empty on
    /// others (e.g., during bring-up where Z is passthrough and the E queue
    /// has no shaping context).
    pub axes: [&'a [BezierPiece<f64>]; 4],
}

impl PerAxisHistory<'_> {
    /// Construct an empty history (all four axes are empty slices).
    /// `emit_shaped` with an empty history reproduces `shape_batch`'s
    /// pre-streaming output byte-for-byte.
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

/// Per-segment metadata that survives [`emit_shaped`]'s shape-and-refit step
/// onto the resulting [`ShapedSegment`].
///
/// `e_independent` and `feedrate_mm_s` are intentionally absent — those
/// belong to E-gap segments which `emit_shaped` does not produce. Callers
/// that need to interleave E gaps (i.e., `shape_batch`) handle that
/// post-`emit_shaped`.
#[derive(Debug, Clone, Copy)]
pub struct EmitSegmentMeta {
    /// E-axis mode classification, forwarded onto the output [`ShapedSegment`].
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc length); forwarded onto output.
    pub extrusion_per_xy_mm: f64,
}

/// Run the shaping half of the shaper pipeline.
///
/// For each segment in `planned`:
///
/// 1. **Pad** each of axes 0/1/2 with neighbour data + history (left) and
///    constant-extension at `batch_t_end` (right).
/// 2. **Convolve** the padded curve with that axis's kernel; or pass the
///    fitted axis through unchanged when the kernel is `None`.
/// 3. **Refit** each post-shape axis to a chain of cubic Bézier pieces with
///    C¹ continuity (CLAUDE.md's "uniform cubic Bézier across Layer 1/2/3/4"
///    invariant at the post-shape boundary).
/// 4. Assemble a [`ShapedSegment`] with `meta[i]`'s e-mode metadata.
///
/// `kernels` mirrors [`crate::plan_velocity::PlanInput::kernels`] in slot
/// ordering: `[X, Y, Z, E]`. Only the first three slots are consulted; the
/// E slot is reserved for forward compatibility. `Some(kernel)` triggers
/// pad+convolve+trim; `None` is passthrough (used for Z when `AxisShaper`
/// is configured `Passthrough`).
///
/// `e_halos` is the same `BatchPartition`-derived halo list `shape_batch`
/// uses to insert constant-position pieces over E-gap intervals during the
/// neighbour scan. Streaming callers supply an empty slice — no E gaps
/// exist in the look-ahead replan window.
///
/// # Errors
///
/// Forwards [`ShapeError::Algebra`] (convolution failure) and
/// [`ShapeError::FitFailure`] (post-shape refit failure). The `index` field
/// in those errors is the index into `planned`.
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
                shape_axis(&padded, kernel, t_start, t_end).map_err(|detail| {
                    ShapeError::Algebra {
                        index: seg_idx,
                        detail,
                    }
                })?
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
            // E-gap segments (which carry the independent-E NURBS) are
            // inserted by `shape_batch` outside this function.
            e_independent: None,
            t_start,
            t_end,
        });
    }

    Ok(output)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;
