use nurbs::bezier::BezierPiece;

use super::{EmitContext, ShaperState};
use crate::emit_shaped::{emit_shaped, PerAxisHistory};
use crate::ShapeError;
use crate::ShapedSegment;

/// Absolute time-domain epsilon for boundary comparisons.
const T_EPSILON: f64 = 1e-12;

impl ShaperState {
    /// Produce shaped output for the dispatch-eligible region `[t_dispatched,
    /// t_decel_start − max_h]`, advance `t_shaped` / `t_dispatched`, and trim
    /// old per-axis history.
    ///
    /// Returns an empty vector when `target ≤ t_dispatched` (nothing newly
    /// eligible, including fresh state before any `append_and_replan`).
    ///
    /// On error the state is left unchanged so the caller can re-attempt.
    ///
    /// # Errors
    ///
    /// Forwards any [`ShapeError`] from [`emit_shaped`].
    pub fn emit_committed(
        &mut self,
        ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);

        let target = self.t_decel_start - max_h;
        if target <= self.t_dispatched + T_EPSILON {
            return Ok(Vec::new());
        }

        if self.planned_fitted.is_empty() {
            return Ok(Vec::new());
        }

        let history_storage: [Vec<BezierPiece<f64>>; 4] = std::array::from_fn(|axis_idx| {
            self.axes[axis_idx]
                .pieces
                .iter()
                .filter(|p| p.u_start < self.t_dispatched + T_EPSILON)
                .cloned()
                .collect()
        });
        let history = PerAxisHistory {
            axes: [
                history_storage[0].as_slice(),
                history_storage[1].as_slice(),
                history_storage[2].as_slice(),
                history_storage[3].as_slice(),
            ],
        };

        let batch_t_start = self.t_dispatched;
        let batch_t_end = self.t_appended;

        let shaped = emit_shaped(
            &self.planned_fitted,
            &self.planned_meta,
            ctx.kernels,
            ctx.e_halos,
            &history,
            batch_t_start,
            batch_t_end,
        )?;

        let mut dispatched: Vec<ShapedSegment> = Vec::with_capacity(shaped.len());
        for seg in shaped {
            if seg.t_start >= target - T_EPSILON {
                break;
            }
            if seg.t_end <= target + T_EPSILON {
                dispatched.push(seg);
            } else {
                let restricted =
                    restrict_segment_to(&seg, target).map_err(|detail| ShapeError::Algebra {
                        index: dispatched.len(),
                        detail,
                    })?;
                dispatched.push(restricted);
                break;
            }
        }

        self.t_shaped = target;
        self.t_dispatched = target;

        let delta_safety = max_h;
        let trim_cutoff = self.t_dispatched - max_h - delta_safety;
        for axis in &mut self.axes {
            while let Some(front) = axis.pieces.front() {
                if front.u_end < trim_cutoff - T_EPSILON {
                    axis.pieces.pop_front();
                } else {
                    break;
                }
            }
        }

        Ok(dispatched)
    }

    /// Commit and dispatch the held-back trailing region `[t_dispatched, t_appended]`
    /// including the terminal decel-to-zero ramp.
    ///
    /// Idempotent: returns `Ok(Vec::new())` if `t_dispatched >= t_appended − ε`.
    ///
    /// On error the state is left unchanged.
    ///
    /// # Errors
    ///
    /// Forwards [`ShapeError`]s from [`emit_shaped`].
    pub fn commit_decel_to_zero(
        &mut self,
        ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        if self.t_dispatched >= self.t_appended - T_EPSILON {
            return Ok(Vec::new());
        }

        if self.planned_fitted.is_empty() {
            return Ok(Vec::new());
        }

        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
        let target = self.t_appended;

        let history_storage = build_history_storage(&self.axes, self.t_dispatched);
        let history = PerAxisHistory {
            axes: [
                history_storage[0].as_slice(),
                history_storage[1].as_slice(),
                history_storage[2].as_slice(),
                history_storage[3].as_slice(),
            ],
        };

        let batch_t_start = self.t_dispatched;
        let batch_t_end = self.t_appended;

        let shaped = emit_shaped(
            &self.planned_fitted,
            &self.planned_meta,
            ctx.kernels,
            ctx.e_halos,
            &history,
            batch_t_start,
            batch_t_end,
        )?;

        let mut dispatched: Vec<ShapedSegment> = Vec::with_capacity(shaped.len());
        for seg in shaped {
            if seg.t_end <= self.t_dispatched + T_EPSILON {
                continue;
            }
            if seg.t_start >= target - T_EPSILON {
                break;
            }
            let lo = self.t_dispatched.max(seg.t_start);
            let hi = target.min(seg.t_end);
            if hi <= lo + T_EPSILON {
                continue;
            }
            if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                dispatched.push(seg);
            } else {
                let restricted =
                    restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| ShapeError::Algebra {
                        index: dispatched.len(),
                        detail,
                    })?;
                dispatched.push(restricted);
            }
        }

        self.t_shaped = target;
        self.t_dispatched = target;

        trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);

        Ok(dispatched)
    }
}

fn build_history_storage(
    axes: &[super::AxisShaperQueue; 4],
    t_dispatched: f64,
) -> [Vec<BezierPiece<f64>>; 4] {
    std::array::from_fn(|axis_idx| {
        axes[axis_idx]
            .pieces
            .iter()
            .filter(|p| p.u_start < t_dispatched + T_EPSILON)
            .cloned()
            .collect()
    })
}

fn trim_per_axis_history(axes: &mut [super::AxisShaperQueue; 4], t_dispatched: f64, max_h: f64) {
    let delta_safety = max_h;
    let trim_cutoff = t_dispatched - max_h - delta_safety;
    for axis in axes {
        while let Some(front) = axis.pieces.front() {
            if front.u_end < trim_cutoff - T_EPSILON {
                axis.pieces.pop_front();
            } else {
                break;
            }
        }
    }
}

fn restrict_segment_to(
    seg: &ShapedSegment,
    t_hi: f64,
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    restrict_segment_lo_hi(seg, seg.t_start, t_hi)
}

fn restrict_segment_lo_hi(
    seg: &ShapedSegment,
    t_lo: f64,
    t_hi: f64,
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use nurbs::algebra::restrict_to_domain;

    let restricted_axes: [nurbs::ScalarNurbs<f64>; 3] = [
        restrict_to_domain(&seg.axes[0], t_lo, t_hi)?,
        restrict_to_domain(&seg.axes[1], t_lo, t_hi)?,
        restrict_to_domain(&seg.axes[2], t_lo, t_hi)?,
    ];
    Ok(ShapedSegment {
        axes: restricted_axes,
        e_mode: seg.e_mode,
        extrusion_per_xy_mm: seg.extrusion_per_xy_mm,
        e_independent: seg.e_independent.clone(),
        t_start: t_lo,
        t_end: t_hi,
    })
}
