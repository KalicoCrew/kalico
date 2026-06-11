use nurbs::bezier::BezierPiece;

use super::{EmitContext, ShaperState};
use crate::emit_shaped::{emit_shaped, emit_shaped_with_left_bc, PerAxisHistory};
use crate::ShapeError;
use crate::ShapedSegment;

const T_EPSILON: f64 = 1e-12;

impl ShaperState {
    /// Produce shaped output for the dispatch-eligible region `[t_dispatched,
    /// t_decel_start − max_h]`, advance `t_dispatched`, and trim old per-axis history.
    ///
    /// The method maintains a `pending_freeze` buffer: after each successful emission,
    /// the shaped output for the kernel-support window immediately after the new
    /// `t_dispatched` is stored.  On the next call (whether or not a replan occurred),
    /// this buffer is dispatched first so the MCU receives a bit-identical signal for
    /// the overlap region — preventing the velocity seam that would otherwise arise from
    /// re-fitting the frozen zone over a shorter domain.
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

        if self.planned_fitted.is_empty() && self.pending_freeze.is_empty() {
            return Ok(Vec::new());
        }

        let t_freeze = self.t_dispatched + max_h;

        let mut dispatched: Vec<ShapedSegment> = Vec::new();

        let mut freeze_end_velocities: [Option<f64>; 3] = [None; 3];

        if !self.pending_freeze.is_empty() {
            let pending = std::mem::take(&mut self.pending_freeze);

            let last_pending = pending.last();
            if let Some(last_seg) = last_pending {
                for axis in 0..3 {
                    freeze_end_velocities[axis] =
                        Some(shaped_axis_velocity_at(&last_seg.axes[axis], t_freeze));
                }
            }

            for seg in pending {
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
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
            }

            if target <= t_freeze + T_EPSILON {
                self.t_shaped = target;
                self.t_dispatched = target;
                trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);
                return Ok(dispatched);
            }
        }

        let emit_start = if !dispatched.is_empty() {
            t_freeze
        } else {
            self.t_dispatched
        };

        if self.planned_fitted.is_empty() {
            self.t_shaped = target;
            self.t_dispatched = target;
            trim_per_axis_history(&mut self.axes, self.t_dispatched, max_h);
            return Ok(dispatched);
        }

        let window_start = self
            .planned_fitted
            .first()
            .map_or(emit_start, |f| f.t_start.max(emit_start));
        let history_storage: [Vec<BezierPiece<f64>>; 4] = std::array::from_fn(|axis_idx| {
            self.axes[axis_idx]
                .pieces
                .iter()
                .filter(|p| p.u_start < window_start + T_EPSILON)
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

        let batch_t_end = self.t_appended;

        let left_bc = if dispatched.is_empty() {
            [None; 3]
        } else {
            freeze_end_velocities
        };

        let shaped = emit_shaped_with_left_bc(
            &self.planned_fitted,
            &self.planned_meta,
            ctx.kernels,
            ctx.e_halos,
            &history,
            emit_start,
            batch_t_end,
            left_bc,
        )?;

        let new_pending_start = target;
        let new_pending_end = (target + max_h).min(self.t_appended);

        let mut new_pending: Vec<ShapedSegment> = Vec::new();

        for seg in shaped {
            if seg.t_end <= emit_start + T_EPSILON {
                continue;
            }
            if seg.t_start >= new_pending_end - T_EPSILON {
                break;
            }

            let lo = emit_start.max(seg.t_start);
            let hi = new_pending_end.min(seg.t_end);
            if hi <= lo + T_EPSILON {
                continue;
            }

            let seg_to_store =
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    seg.clone()
                } else {
                    restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| ShapeError::Algebra {
                        index: dispatched.len() + new_pending.len(),
                        detail,
                    })?
                };

            if lo < new_pending_start - T_EPSILON {
                let dispatch_hi = new_pending_start.min(hi);
                if dispatch_hi > lo + T_EPSILON {
                    let dispatch_seg = if (lo - seg.t_start).abs() < T_EPSILON
                        && (dispatch_hi - seg.t_end).abs() < T_EPSILON
                    {
                        seg.clone()
                    } else {
                        restrict_segment_lo_hi(&seg, lo, dispatch_hi).map_err(|detail| {
                            ShapeError::Algebra {
                                index: dispatched.len(),
                                detail,
                            }
                        })?
                    };
                    dispatched.push(dispatch_seg);
                }

                if hi > new_pending_start + T_EPSILON {
                    let pending_lo = new_pending_start;
                    let pending_hi = hi;
                    let pending_seg = restrict_segment_lo_hi(&seg, pending_lo, pending_hi)
                        .map_err(|detail| ShapeError::Algebra {
                            index: dispatched.len() + new_pending.len(),
                            detail,
                        })?;
                    new_pending.push(pending_seg);
                }
            } else {
                new_pending.push(seg_to_store);
            }
        }

        self.pending_freeze = new_pending;

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

        if self.planned_fitted.is_empty() && self.pending_freeze.is_empty() {
            return Ok(Vec::new());
        }

        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
        let target = self.t_appended;

        let mut dispatched: Vec<ShapedSegment> = Vec::new();

        if !self.pending_freeze.is_empty() {
            let pending = std::mem::take(&mut self.pending_freeze);
            for seg in pending {
                if seg.t_end <= self.t_dispatched + T_EPSILON {
                    continue;
                }
                let lo = self.t_dispatched.max(seg.t_start);
                let hi = target.min(seg.t_end);
                if hi <= lo + T_EPSILON {
                    continue;
                }
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    dispatched.push(seg);
                } else {
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
            }
        }

        if !self.planned_fitted.is_empty() {
            let t_freeze = self.t_dispatched + max_h;
            let emit_start = if !dispatched.is_empty() {
                t_freeze
            } else {
                self.t_dispatched
            };

            let history_storage = build_history_storage(&self.axes, emit_start);
            let history = PerAxisHistory {
                axes: [
                    history_storage[0].as_slice(),
                    history_storage[1].as_slice(),
                    history_storage[2].as_slice(),
                    history_storage[3].as_slice(),
                ],
            };

            let shaped = emit_shaped(
                &self.planned_fitted,
                &self.planned_meta,
                ctx.kernels,
                ctx.e_halos,
                &history,
                emit_start,
                self.t_appended,
            )?;

            for seg in shaped {
                if seg.t_end <= emit_start + T_EPSILON {
                    continue;
                }
                if seg.t_start >= target - T_EPSILON {
                    break;
                }
                let lo = emit_start.max(seg.t_start);
                let hi = target.min(seg.t_end);
                if hi <= lo + T_EPSILON {
                    continue;
                }
                if (lo - seg.t_start).abs() < T_EPSILON && (hi - seg.t_end).abs() < T_EPSILON {
                    dispatched.push(seg);
                } else {
                    let restricted = restrict_segment_lo_hi(&seg, lo, hi).map_err(|detail| {
                        ShapeError::Algebra {
                            index: dispatched.len(),
                            detail,
                        }
                    })?;
                    dispatched.push(restricted);
                }
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
    t_start: f64,
) -> [Vec<BezierPiece<f64>>; 4] {
    std::array::from_fn(|axis_idx| {
        axes[axis_idx]
            .pieces
            .iter()
            .filter(|p| p.u_start < t_start + T_EPSILON)
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

fn shaped_axis_velocity_at(axis: &nurbs::ScalarNurbs<f64>, t: f64) -> f64 {
    let d1 = nurbs::eval::derivative(axis);
    nurbs::eval::eval(&d1, t)
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
