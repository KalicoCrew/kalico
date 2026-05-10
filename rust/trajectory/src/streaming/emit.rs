// Phase 3 Task 3.2 — `ShaperState::emit_committed`.
//
// Produces shaped output for the dispatch-eligible region of the most
// recent replan's plan, advances the dispatch cursors, and trims old
// per-axis history that's no longer needed for future left-pad.
//
// See spec §3.3 ("Dispatch boundary: `t_decel_start − h`") and §3.7
// ("Cross-axis sync"): dispatch shaped output up to `t_decel_start −
// max(h_x, h_y)`. Passthrough axes (h=0) excluded from the max.

use nurbs::bezier::BezierPiece;

use super::{EmitContext, ShaperState};
use crate::emit_shaped::{emit_shaped, PerAxisHistory};
use crate::ShapeError;
use crate::ShapedSegment;

/// Absolute time-domain epsilon used for boundary comparisons. The same
/// scale `replace_uncommitted_axis_pieces` uses; cursor comparisons in
/// `emit_committed` would otherwise be tripped up by IEEE-754 rounding
/// in `t_offset + t_relative` additions inside `append_and_replan`.
const T_EPSILON: f64 = 1e-12;

impl ShaperState {
    /// **Phase 3 Task 3.2 — emit shaped output for the dispatch-eligible
    /// region of the most recent plan.**
    ///
    /// Per spec §3.3 the dispatch boundary is `t_decel_start − max_h` where
    /// `max_h = max(h_axis for axis in shaped_axes; passthrough excluded)`.
    /// Beyond that boundary, the convolution at any sample would reach into
    /// the speculative (un-committed) trailing decel ramp and must be held
    /// back until the next replan (which makes the speculative content real)
    /// or quiescence commit (which adopts the planned decel as the actual
    /// trajectory).
    ///
    /// This function:
    ///
    /// 1. Computes `target = t_decel_start − max_h`.
    /// 2. Shapes the full cached plan via [`emit_shaped`], supplying the
    ///    per-axis history (`pieces` entries with `u_end ≤ t_dispatched`)
    ///    so the left-pad reads from real prior motion.
    /// 3. Trims each shaped segment to `[t_dispatched, target]`: segments
    ///    fully past `target` are dropped; one segment may straddle the
    ///    boundary and is restricted to its head portion via
    ///    [`nurbs::algebra::restrict_to_domain`].
    /// 4. Advances `t_shaped` and `t_dispatched` to `target`.
    /// 5. Trims `axes[i].pieces` entries whose right edge is strictly
    ///    before `t_dispatched − max_h − δ_safety` — they can no longer
    ///    contribute to any future left-pad.
    ///
    /// **Why we restrict the straddling segment.** `plan_velocity` returns
    /// one [`FittedSegment`] per submitted move; the move's accel /
    /// cruise / decel structure lives inside that single segment. So on a
    /// single-move plan, `t_decel_start` is interior to the only segment
    /// and a strict "segment fully before target" filter would emit
    /// nothing. Restricting to the boundary preserves the dispatch
    /// semantic: every sample in the returned `ShapedSegment`'s domain
    /// reads its convolution support entirely from the committed region.
    ///
    /// Returns the newly-eligible shaped segments. Empty vector when
    /// `target ≤ t_dispatched` (nothing newly eligible since the last
    /// `emit_committed` call), including the fresh-state case before any
    /// `append_and_replan` has run.
    ///
    /// # Errors
    ///
    /// Forwards any [`ShapeError`] from [`emit_shaped`]. On error the
    /// state is left **unchanged** (no cursor advance, no history trim) so
    /// the caller can re-attempt or fall through to quiescence commit.
    pub fn emit_committed(
        &mut self,
        ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        // 1. Compute `max_h` across shaped axes. Passthrough axes (h=0)
        //    don't gate dispatch — they flow through to the multi-axis
        //    output immediately. If every axis is passthrough (unusual),
        //    `max_h = 0` and dispatch is gated solely by `t_decel_start`.
        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);

        // 2. Compute the dispatch target. If nothing new is eligible,
        //    return empty without touching the state.
        let target = self.t_decel_start - max_h;
        if target <= self.t_dispatched + T_EPSILON {
            return Ok(Vec::new());
        }

        // 3. Nothing to emit if the cached plan is empty (fresh state,
        //    or `append_and_replan` has not run since construction).
        if self.planned_fitted.is_empty() {
            return Ok(Vec::new());
        }

        // 4. Build the per-axis history from `axes[i].pieces` entries
        //    whose left edge precedes `t_dispatched`. The pad's left-pad
        //    loop trims each history piece to `[pad_target, t_dispatched]`,
        //    so a "straddling" piece (`u_start < t_dispatched < u_end`)
        //    contributes its pre-`t_dispatched` half — which is exactly
        //    the committed-but-not-yet-trimmed motion. Excluding the
        //    straddling piece would leave a `[history.tail.u_end,
        //    t_dispatched)` gap in the left-pad and the pad's
        //    `bezier_pieces_to_nurbs` final concatenation would panic
        //    on non-contiguous pieces.
        //
        //    We materialize each axis's history as a `Vec<BezierPiece<f64>>`
        //    so the slice borrow is valid for the duration of the
        //    `emit_shaped` call.
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

        // 5. Run `emit_shaped` over the full cached plan. The pad's
        //    neighbour scan can then read the trailing decel-to-zero as
        //    right-pad context for the last eligible segment; without it,
        //    the right-pad would fall back to constant-extension at
        //    `t_appended` and the convolution at the eligible boundary
        //    would mis-represent the post-decel cruise direction.
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

        // 6. Trim shaped output to `[t_dispatched, target]`:
        //    - Segment fully past `target` → drop.
        //    - Segment fully within `target` → keep as-is.
        //    - Segment straddling `target` → restrict to `[t_start, target]`.
        let mut dispatched: Vec<ShapedSegment> = Vec::with_capacity(shaped.len());
        for seg in shaped {
            if seg.t_start >= target - T_EPSILON {
                // Entirely past the boundary — held back for the next
                // emit_committed after the next replan extends `t_decel_start`.
                break; // segments are time-ordered; nothing more dispatchable follows.
            }
            if seg.t_end <= target + T_EPSILON {
                // Entirely within bounds — emit unchanged.
                dispatched.push(seg);
            } else {
                // Straddling — restrict each axis curve to `[t_start, target]`.
                // `restrict_to_domain` is exact under our cubic-Bézier-piecewise
                // representation (split at boundary via the same `split_piece_at`
                // used elsewhere in the algebra layer).
                let restricted = restrict_segment_to(&seg, target).map_err(|detail| {
                    ShapeError::Algebra {
                        index: dispatched.len(),
                        detail,
                    }
                })?;
                dispatched.push(restricted);
                // The straddling segment is the last partially-eligible one;
                // segments after it are entirely past the boundary.
                break;
            }
        }

        // 7. Advance the cursors. We advance to `target` unconditionally
        //    once we got past the early-exit at step 2 — the boundary
        //    itself moved and later calls must see it correctly.
        self.t_shaped = target;
        self.t_dispatched = target;

        // 8. Trim per-axis history. Anything with right edge strictly
        //    before `t_dispatched − max_h − δ_safety` is no longer
        //    reachable by any future convolution: the next emission's
        //    farthest left-pad target is `t_dispatched + epsilon − max_h`
        //    (segments produced just past the new boundary), and we keep
        //    an extra `δ_safety = max_h` buffer per spec open-question 2.
        let delta_safety = max_h; // spec §3.7 / open-question 2
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

    /// **Phase 4 Task 4.1 stub — quiescence commit handler.**
    ///
    /// Fires from `motion-bridge::planner::run_loop` when the inter-move
    /// `T_commit` quiescence timer elapses without a follow-on
    /// `PlannerMsg::Move`. Task 4.2 will replace this body with the real
    /// "commit decel-to-zero" implementation: shape the held-back trailing
    /// decel-to-zero ramp (the region `[t_decel_start − max_h,
    /// t_end_of_last_move]`) with constant-extension right-padding at
    /// `(end_pos, v = 0)`, append to `pending_dispatch`, advance
    /// `t_dispatched` to `t_end_of_last_move`, and return the freshly-shaped
    /// segments. Task 4.1 only wires the integration point — it returns an
    /// empty vector so the run-loop's commit branch can be exercised end-to-
    /// end (timer fires → handler runs → no segments produced yet) without
    /// the rest of Phase 4's machinery in place.
    ///
    /// The `_ctx` parameter mirrors [`Self::emit_committed`]'s signature so
    /// Task 4.2 can drop in the convolution without changing the call site.
    ///
    /// # Errors
    ///
    /// Returns `Ok(Vec::new())` unconditionally for now. Task 4.2 will
    /// forward [`ShapeError`]s from `emit_shaped`.
    pub fn commit_decel_to_zero(
        &mut self,
        _ctx: &EmitContext<'_>,
    ) -> Result<Vec<ShapedSegment>, ShapeError> {
        Ok(Vec::new())
    }
}

/// Restrict each of the segment's X/Y/Z NURBS curves to `[seg.t_start, t_hi]`.
/// Preserves E-mode metadata and `e_independent` (which is `None` in the
/// streaming pipeline anyway). Sets the resulting segment's `t_end = t_hi`.
fn restrict_segment_to(
    seg: &ShapedSegment,
    t_hi: f64,
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use nurbs::algebra::restrict_to_domain;

    let restricted_axes: [nurbs::ScalarNurbs<f64>; 3] = [
        restrict_to_domain(&seg.axes[0], seg.t_start, t_hi)?,
        restrict_to_domain(&seg.axes[1], seg.t_start, t_hi)?,
        restrict_to_domain(&seg.axes[2], seg.t_start, t_hi)?,
    ];
    Ok(ShapedSegment {
        axes: restricted_axes,
        e_mode: seg.e_mode,
        extrusion_per_xy_mm: seg.extrusion_per_xy_mm,
        e_independent: seg.e_independent.clone(),
        t_start: seg.t_start,
        t_end: t_hi,
    })
}
