// ShaperState construction + Phase-1 byte-identity shim + Phase-3 Task-3.1
// `append_and_replan` and its supporting helpers.

use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::bezier::{extract_bezier_pieces, BezierPiece};

use super::decel_finder::find_decel_start_time;
use super::{AxisShaperQueue, ReplanContext, ShaperState, UncommittedMove};
use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::plan_velocity::{plan_velocity, PlanInput, PlanSegment};
use crate::AxisShaper;
use crate::ShapeError;
use crate::ShapedSegment;

impl ShaperState {
    /// Construct a fresh streaming-shaper state at `home_pos` for each axis,
    /// with the per-axis kernels in `shapers`. Each axis queue is seeded with
    /// a `(home_pos[i], v=0)` rest extension covering `[-( h + δ_safety ), 0]`.
    ///
    /// `δ_safety` is set to `h` (so the initial seed spans `2 * h` of past)
    /// per open-question 2 in the spec. For a passthrough axis (`h = 0`) the
    /// seed has zero duration and is omitted.
    #[must_use]
    pub fn new(home_pos: [f64; 4], shapers: &[Option<AxisShaper>; 4]) -> Self {
        let axes: [AxisShaperQueue; 4] =
            std::array::from_fn(|i| build_axis_queue(home_pos[i], shapers[i]));

        Self {
            axes,
            uncommitted_moves: VecDeque::new(),
            t_appended: 0.0,
            t_decel_start: 0.0,
            t_shaped: 0.0,
            t_dispatched: 0.0,
            pending_dispatch: Vec::new(),
            planned_fitted: Vec::new(),
            planned_meta: Vec::new(),
        }
    }

    /// **Phase 1 shim.** Run the existing per-segment pad → shape → refit
    /// pipeline on `fitted` and stage the resulting `ShapedSegment` into
    /// `pending_dispatch`. The internal queue state (`axes`, `t_appended`,
    /// etc.) is intentionally left untouched here — Phase 3's
    /// `append_and_replan` + `emit_committed` are the history-aware path.
    ///
    /// Returns `Ok(())` on success. Errors from the algebra pipeline are
    /// surfaced via `nurbs::AlgebraError`; refit and other failures are
    /// flattened into `AlgebraError::DegreeMismatch` placeholders so the
    /// shim's signature stays narrow until Phase 2 widens it.
    pub fn append_batch(&mut self, fitted: &FittedSegment) -> Result<(), nurbs::AlgebraError> {
        let shaped = shape_single_segment(fitted, &self.axes)?;
        self.pending_dispatch.push(shaped);
        Ok(())
    }

    /// Drain `pending_dispatch`, returning all shaped segments that are
    /// ready for the wire. Clears the field.
    pub fn drain_committed(&mut self) -> Vec<ShapedSegment> {
        std::mem::take(&mut self.pending_dispatch)
    }

    /// **Phase 3 Task 3.1 — streaming-shaper replan-on-append entry point.**
    ///
    /// Replan the un-committed tail of the queue when a new move arrives.
    /// Per spec §3.2 / §3.4 the joined path "un-committed prior tail + new
    /// move" is fed to `plan_velocity` with:
    ///
    /// - `initial_v` = the velocity at `t_dispatched` as read off the
    ///   currently-planned `pieces` queue. Beginning-of-life (no pieces
    ///   covering `t_dispatched` yet) falls back to
    ///   `ctx.fallback_initial_v`.
    /// - `terminal_v` = `0.0` — the spec's "decel-to-zero default" so the
    ///   replanned tail is itself a safe rest if no further move arrives.
    ///
    /// TOPP-RA respects these boundary velocities (Step 0 of this task
    /// lifted the previous `(0, 0)` limitation), so the returned profile
    /// chains continuously through the move-to-move junction at the natural
    /// optimal junction velocity — no separate "junction deviation"
    /// computation is needed at this layer (junction-deviation caps are
    /// already folded into `temporal::multi::plan_batch`'s upfront velocity
    /// caps).
    ///
    /// On success the function:
    /// 1. Appends `new_segment` to `uncommitted_moves`.
    /// 2. Drops any `uncommitted_moves` entries with `t_end < t_dispatched`
    ///    (their planned `BezierPiece`s remain in `axes[i].pieces` as
    ///    history for the next `emit_shaped` left-pad).
    /// 3. Calls `plan_velocity` over the remaining un-committed tail.
    /// 4. Replaces all `axes[i].pieces` entries with `u_start ≥
    ///    t_dispatched` by the new plan's per-axis `BezierPiece`s, in
    ///    flattened-segment order. The committed history
    ///    (`u_start < t_dispatched`) is untouched.
    /// 5. Refreshes the per-`UncommittedMove` `t_start` / `t_end` from the
    ///    new plan's segment boundaries.
    /// 6. Updates `t_decel_start` to the new plan's last segment's
    ///    `t_start` (the decel-to-zero ramp's start time) and `t_appended`
    ///    to the new plan's overall `t_end`.
    /// 7. Caches the time-domain plan (`planned_fitted` / `planned_meta`)
    ///    so [`Self::emit_committed`] can shape its eligible head without
    ///    re-running TOPP-RA.
    ///
    /// # Errors
    ///
    /// Forwards any [`ShapeError`] from `plan_velocity`. On error the
    /// `ShaperState` is left unchanged (atomic — the new move is *not*
    /// pushed onto `uncommitted_moves` and no `pieces` content is touched).
    pub fn append_and_replan(
        &mut self,
        new_segment: CubicSegment,
        ctx: &ReplanContext,
    ) -> Result<(), ShapeError> {
        // 1. Determine initial_v at the existing replan boundary
        //    (`t_dispatched`). Sample the X / Y BezierPiece derivatives
        //    active at that absolute time and combine to a path speed.
        let initial_v = self.read_path_speed_at(self.t_dispatched, ctx.fallback_initial_v);

        // 2. Snapshot the prior `uncommitted_moves` so we can roll back on
        //    plan failure (per the "atomic on error" contract). Drop any
        //    entries whose `t_end < t_dispatched` — those are now history.
        let prior_uncommitted = self.uncommitted_moves.clone();
        let prior_t_appended = self.t_appended;
        let prior_t_decel_start = self.t_decel_start;

        self.uncommitted_moves
            .retain(|m| m.t_end > self.t_dispatched);

        // The new move appends after the currently-planned timeline.
        // Geometry doesn't have an absolute time yet — that's TOPP-RA's job.
        // We seed it with `t_start = prior_uncommitted_tail_end` so the
        // `uncommitted_moves` ordering reflects the input order; the
        // post-plan refresh overwrites both `t_start` and `t_end` from the
        // converged TOPP-RA profile.
        let pre_plan_t_start = self
            .uncommitted_moves
            .back()
            .map_or(self.t_dispatched, |m| m.t_end);
        self.uncommitted_moves.push_back(UncommittedMove {
            segment: new_segment,
            t_start: pre_plan_t_start,
            t_end: pre_plan_t_start, // refreshed post-plan
        });

        // 3. Build the planning input from the un-committed tail.
        //    Every entry contributes one `PlanSegment` (XY-coupled / travel —
        //    the streaming planner does not yet handle Independent E).
        let segs: Vec<UncommittedMove> = self.uncommitted_moves.iter().cloned().collect();
        let plan_segments: Vec<PlanSegment<'_>> = segs
            .iter()
            .map(|m| PlanSegment {
                temporal: temporal::multi::SegmentInput {
                    curve: &m.segment.xyz,
                    limits: ctx.limits,
                    trailing_junction_chord_tolerance_mm: ctx.junction_chord_tolerance_mm,
                },
                e_mode: m.segment.e_mode,
                extrusion_per_xy_mm: m.segment.extrusion_per_xy_mm,
                e_independent: m.segment.e_independent.as_ref(),
                feedrate_mm_s: m.segment.feedrate_mm_s,
            })
            .collect();

        let plan_input = PlanInput {
            segments: &plan_segments,
            grid_strategy: ctx.grid_strategy,
            worker_threads: ctx.worker_threads,
            kernels: ctx.kernels,
            fit_tolerance_mm: ctx.fit_tolerance_mm,
            beta_max_iters: ctx.beta_max_iters,
            beta_convergence_ratio: ctx.beta_convergence_ratio,
            e_limits: ctx.e_limits,
            initial_v,
            terminal_v: 0.0,
            safety_mode: ctx.safety_mode,
        };

        // 4. Run plan_velocity. The returned fitted segments are in time
        //    coordinates **relative to the batch** (i.e., `t_start = 0.0`
        //    on the first segment). We shift them up by `t_dispatched` so
        //    the queue's absolute-time invariant holds.
        let fitted = match plan_velocity(&plan_input) {
            Ok(f) => f,
            Err(e) => {
                // Roll back the uncommitted_moves mutation so the caller
                // sees an unchanged state on error.
                self.uncommitted_moves = prior_uncommitted;
                self.t_appended = prior_t_appended;
                self.t_decel_start = prior_t_decel_start;
                return Err(e);
            }
        };

        let time_offset = self.t_dispatched;

        // 5. Replace the un-committed region of each axis's `pieces`.
        //    "Un-committed" means `u_start >= t_dispatched`. Drop those
        //    entries; append the new plan's per-axis pieces (shifted into
        //    absolute time).
        for axis_idx in 0..3 {
            self.replace_uncommitted_axis_pieces(axis_idx, time_offset, &fitted);
        }

        // 6. Refresh `uncommitted_moves` timing from the new plan and
        //    update cursors. `plan_velocity` always returns one
        //    `FittedSegment` per XY-motion input segment (and we filter to
        //    `Travel` / `CoupledToXy` only above), and rejects empty
        //    inputs — so `fitted.len() == uncommitted_moves.len() ≥ 1`.
        debug_assert_eq!(fitted.len(), self.uncommitted_moves.len());
        for (m, f) in self.uncommitted_moves.iter_mut().zip(fitted.iter()) {
            m.t_start = f.t_start + time_offset;
            m.t_end = f.t_end + time_offset;
        }

        // `t_appended` = absolute end time of the new plan's last segment
        // (decel-to-zero terminus under `terminal_v = 0.0`).
        let last = fitted
            .last()
            .expect("fitted non-empty by plan_velocity contract");
        self.t_appended = last.t_end + time_offset;

        // `t_decel_start` = absolute time at which the terminal decel-to-zero
        // ramp starts. Walked backward from the path terminus through dense
        // velocity samples (see `find_decel_start_time` for the algorithm
        // and why "argmax velocity" is wrong on long-cruise moves).
        self.t_decel_start = find_decel_start_time(&fitted) + time_offset;

        // 7. Cache the time-domain plan for `emit_committed`. We shift each
        //    `FittedSegment` by `time_offset` so the cache is in the same
        //    absolute time line as `axes[i].pieces` and the cursors. The
        //    per-axis NURBS curves carry their own knot vectors in
        //    batch-relative time; we must shift those too so the pad layer's
        //    `extract_bezier_pieces` returns pieces in the correct absolute
        //    time domain (otherwise pad's `bezier_pieces_to_nurbs` panics on
        //    non-contiguous left-pad pieces — the seg's pieces are in batch
        //    coords but the history pieces are in absolute coords).
        //
        //    The per-segment `EmitSegmentMeta` is read off the parallel
        //    `uncommitted_moves` entries.
        self.planned_fitted = fitted
            .into_iter()
            .map(|f| FittedSegment {
                axes: [
                    shift_nurbs_in_time(&f.axes[0], time_offset),
                    shift_nurbs_in_time(&f.axes[1], time_offset),
                    shift_nurbs_in_time(&f.axes[2], time_offset),
                ],
                t_start: f.t_start + time_offset,
                t_end: f.t_end + time_offset,
            })
            .collect();
        self.planned_meta = self
            .uncommitted_moves
            .iter()
            .map(|m| EmitSegmentMeta {
                e_mode: m.segment.e_mode,
                extrusion_per_xy_mm: m.segment.extrusion_per_xy_mm,
            })
            .collect();

        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Phase 3 helpers (shared between `state.rs` and `emit.rs`)
// ---------------------------------------------------------------------------

impl ShaperState {
    /// Sample the path speed `||(dx/dt, dy/dt)||` at absolute time `t` by
    /// reading the active X / Y `BezierPiece`s' derivatives. Returns
    /// `fallback` when no piece covers `t` on either axis (e.g., the queue
    /// has only the rest-extension seed and we're sampling at the initial
    /// `t_dispatched = 0.0` boundary, or the cursor sits in a gap between
    /// pieces — defensive fall-back).
    pub(crate) fn read_path_speed_at(&self, t: f64, fallback: f64) -> f64 {
        let vx = self.axis_velocity_at(0, t);
        let vy = self.axis_velocity_at(1, t);
        match (vx, vy) {
            (Some(x), Some(y)) => (x * x + y * y).sqrt(),
            (Some(x), None) => x.abs(),
            (None, Some(y)) => y.abs(),
            (None, None) => fallback,
        }
    }

    /// Find the piece on axis `axis_idx` whose closed-open domain
    /// `[u_start, u_end)` contains `t` and return its derivative at `t`.
    /// Returns `None` if no piece covers `t`.
    ///
    /// Tie-break at piece boundaries: the right-hand piece wins
    /// (`u_start ≤ t < u_end`). The terminal `t == u_end` of the very last
    /// piece is accepted as well so the planner can read the queue's final
    /// velocity (which is `0.0` under the decel-to-zero invariant — the
    /// natural rest case).
    fn axis_velocity_at(&self, axis_idx: usize, t: f64) -> Option<f64> {
        let pieces = &self.axes[axis_idx].pieces;
        if pieces.is_empty() {
            return None;
        }

        // Last-piece terminal: clamp `t` to `u_end` (decel-to-zero ends at
        // `v = 0`; the derivative there is well-defined).
        let last = pieces.back().unwrap();
        if t >= last.u_end && t <= last.u_end + 1e-12 {
            return Some(last.differentiate().evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - 1e-12 <= t && t < p.u_end {
                return Some(p.differentiate().evaluate(t));
            }
        }
        None
    }

    /// Replace the `axes[axis_idx].pieces` entries whose `u_start ≥
    /// t_dispatched_now` (i.e., the un-committed tail) with the fresh plan's
    /// per-axis pieces. The plan's `BezierPiece`s are in batch-relative
    /// time (`t_start = 0.0` on the first segment); we shift by
    /// `time_offset = t_dispatched_now` so they land on the absolute time
    /// line shared with the rest of the queue.
    fn replace_uncommitted_axis_pieces(
        &mut self,
        axis_idx: usize,
        time_offset: f64,
        fitted: &[FittedSegment],
    ) {
        let t_keep_cutoff = self.t_dispatched;

        // Drop all pieces with `u_start ≥ t_keep_cutoff`. A piece that
        // straddles the cutoff (`u_start < cutoff < u_end`) is kept — its
        // pre-cutoff content was committed; anything beyond is moot because
        // dispatch never advances past `t_decel_start - h`, which is
        // necessarily ≤ `t_keep_cutoff` whenever a replan is welcome (the
        // committed region is by definition behind the dispatch cursor).
        let pieces = &mut self.axes[axis_idx].pieces;
        while let Some(back) = pieces.back() {
            if back.u_start >= t_keep_cutoff - 1e-12 {
                pieces.pop_back();
            } else {
                break;
            }
        }

        // Extract per-axis pieces from the fresh plan and shift onto the
        // absolute time line.
        for f in fitted {
            let axis_nurbs = &f.axes[axis_idx];
            let shifted = extract_bezier_pieces(axis_nurbs)
                .into_iter()
                .map(|mut p| {
                    p.u_start += time_offset;
                    p.u_end += time_offset;
                    p
                });
            pieces.extend(shifted);
        }
    }
}

/// Shift a `ScalarNurbs<f64>`'s time domain by `dt` seconds. Extracts the
/// piecewise-Bézier representation, shifts every piece's `u_start` / `u_end`,
/// and reassembles. Equivalent in effect to adding `dt` to every knot of the
/// underlying curve — but going through `extract_bezier_pieces` + shift +
/// `bezier_pieces_to_nurbs` avoids duplicating the curve's internal knot
/// machinery here.
fn shift_nurbs_in_time(curve: &nurbs::ScalarNurbs<f64>, dt: f64) -> nurbs::ScalarNurbs<f64> {
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces};
    let pieces: Vec<BezierPiece<f64>> = extract_bezier_pieces(curve)
        .into_iter()
        .map(|mut p| {
            p.u_start += dt;
            p.u_end += dt;
            p
        })
        .collect();
    bezier_pieces_to_nurbs(&pieces)
}

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

fn build_axis_queue(home_pos: f64, shaper: Option<AxisShaper>) -> AxisShaperQueue {
    let kernel = shaper.and_then(|s| s.to_kernel());
    let h = match shaper {
        Some(AxisShaper::SmoothZv { frequency_hz }) => 0.8025 / frequency_hz / 2.0,
        Some(AxisShaper::SmoothMzv { frequency_hz }) => 0.95625 / frequency_hz / 2.0,
        Some(AxisShaper::Passthrough) | None => 0.0,
    };

    let mut pieces = VecDeque::new();

    // Seed with a `(home_pos, v=0)` rest extension over `[-(h + δ_safety), 0]`.
    // `δ_safety = h` per spec open-question 2. For passthrough axes (`h = 0`)
    // the seed would be a zero-duration piece, which is degenerate; skip it.
    if h > 0.0 {
        let delta_safety = h;
        let total = h + delta_safety;
        pieces.push_back(BezierPiece {
            u_start: -total,
            u_end: 0.0,
            // Pascal-shifted monomial basis: a constant `home_pos` is just
            // `coeffs = [home_pos]`.
            coeffs: vec![home_pos],
        });
    }

    AxisShaperQueue { pieces, kernel, h }
}

// ---------------------------------------------------------------------------
// Phase 1 shim: delegate to the existing per-segment pipeline
// ---------------------------------------------------------------------------

/// Apply the existing pad → shape → refit pipeline to a single fitted
/// segment, mirroring exactly what `beta::run_one_iteration` does for a
/// stand-alone segment with no E gaps and no neighbours. The output is
/// byte-identical to that path; the streaming module's
/// `append_batch` is just a struct-shaped re-entry.
fn shape_single_segment(
    fitted: &FittedSegment,
    axes: &[AxisShaperQueue; 4],
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use crate::pad::pad_segment_axis;
    use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
    use crate::shaper::shape_axis;

    let t_start = fitted.t_start;
    let t_end = fitted.t_end;

    // Single-segment slice for the existing pad implementation.
    let fitted_slice = std::slice::from_ref(fitted);

    let mut shaped_axes: [Option<nurbs::ScalarNurbs<f64>>; 3] = [None, None, None];

    for axis in 0..3 {
        let q = &axes[axis];
        let axis_shaped = if let Some(kernel) = q.kernel.as_ref() {
            let padded = pad_segment_axis(0, axis, fitted_slice, &[], q.h, t_start, t_end);
            shape_axis(&padded, kernel, t_start, t_end)?
        } else {
            // Passthrough — use the fitted axis directly. Mirrors the
            // `kernels.z = None` branch in `beta::run_one_iteration`.
            fitted.axes[axis].clone()
        };

        // Match `beta::run_one_iteration`: refit *every* axis (including the
        // passthrough Z) to cubic Bézier. Without this the streaming shim's
        // output would diverge byte-for-byte from the existing pipeline.
        //
        // `refit_to_cubic` returns `nurbs::algebra::FitError`; surface it
        // through `AlgebraError::NotImplemented` until Phase 2 widens the
        // error type. Refit failures should not happen on the existing
        // production input (the shim's caller has already exercised this
        // path through `shape_batch`), so this is purely a defensive map.
        let refit = refit_to_cubic(&axis_shaped, REFIT_TOLERANCE_MM).map_err(|_| {
            nurbs::AlgebraError::NotImplemented(
                "streaming::append_batch: refit_to_cubic failed (Phase 1 shim)",
            )
        })?;
        shaped_axes[axis] = Some(refit);
    }

    Ok(ShapedSegment {
        axes: [
            shaped_axes[0].take().unwrap(),
            shaped_axes[1].take().unwrap(),
            shaped_axes[2].take().unwrap(),
        ],
        // Phase 1 has no E plumbing — match `beta::assemble_with_e_gaps`'s
        // default for `EMode::CoupledToXy`-with-zero-ratio; the planner-side
        // wiring (Task 1.2) will overwrite with the real input metadata.
        e_mode: geometry::segment::EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start,
        t_end,
    })
}
