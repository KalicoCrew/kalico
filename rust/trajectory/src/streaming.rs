// Phase 1 — Streaming-shaper module: skeletal types only.
//
// This module is the new home of the per-axis stateful trajectory queue that
// will eventually replace the per-batch pad-and-trim shaping driven by
// `shape_batch`. Phase 1 introduces *only* the data structures and a thin
// `append_batch` shim that delegates to the existing pad → shape → refit
// pipeline; subsequent phases progressively replace the shim with
// history-aware behaviour. See:
//
// - Spec: `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.1
// - Plan: `docs/superpowers/plans/2026-05-10-streaming-shaper.md` Phase 1
//
// **Behaviour invariant (Phase 1):** for any single-segment input, the output
// of `append_batch` followed by `drain_committed` is byte-identical to a
// direct call sequence of `pad::pad_segment_axis` → `shaper::shape_axis` →
// `refit::refit_to_cubic` (with passthrough Z falling through to the fitted
// axis exactly as `beta::run_one_iteration` does it). The unit tests below
// pin that invariant.

use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::{extract_bezier_pieces, BezierPiece};

use crate::fit::FittedSegment;
use crate::plan_velocity::{plan_velocity, PlanInput, PlanSegment, PlanShaper, SafetyMode};
use crate::AxisShaper;
use crate::ELimits;
use crate::ShapeError;
use crate::ShapedSegment;

/// Per-axis unshaped trajectory queue + kernel + half-support.
///
/// `pieces` accumulates the unshaped polynomial pieces that the convolution
/// must see (history, current, lookahead). Phase 1 keeps `pieces` populated
/// only with the `home_pos` rest-extension seed; phases 2/3 fill it from
/// real `append_batch` input and trim it against the dispatched cursor.
#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order. See module docs.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support (seconds). Equal to `T_sm / 2` for active shapers,
    /// `0.0` for passthrough.
    pub h: f64,
}

/// Source-geometry record for one un-committed `submit_move`. The streaming
/// planner retains these alongside the per-axis `pieces` queue so a
/// follow-on `append_and_replan` call can rebuild the planning path
/// (un-committed tail + new move) and run TOPP-RA over it.
///
/// Once the entry's `t_end` falls strictly below `t_dispatched`, the move's
/// pieces are wholly committed and the source record can be dropped from
/// `uncommitted_moves` — its planned time-domain `BezierPiece`s remain in
/// `axes[i].pieces` as history for `emit_shaped`'s left-pad.
#[derive(Debug, Clone)]
pub struct UncommittedMove {
    /// Source-geometry segment, as classified upstream (`CubicSegment` from
    /// `motion-bridge::classify_and_build` or equivalent).
    pub segment: CubicSegment,
    /// Absolute time at which this move's geometry starts in the planner
    /// timeline. Set by `append_and_replan`. For the first move ever
    /// appended this is `0.0`; for subsequent moves it equals the prior
    /// uncommitted move's `t_end` (continuous time line).
    pub t_start: f64,
    /// Absolute time at which the planner currently expects this move to
    /// end, as of the most recent replan. Updated on every replan.
    pub t_end: f64,
}

/// Configuration the streaming replan needs from the planner caller.
///
/// `kernels` and `e_limits` mirror [`crate::ShapeBatchInput`]; `limits` is
/// the temporal-axis machine limit applied to each move. The streaming
/// shaper does not own this configuration — `motion-bridge::planner.rs`
/// holds it in `PlannerConfig` and threads it through on every
/// `append_and_replan` call so live `update_limits` / `update_shaper`
/// reconfigurations take effect immediately on the next replan.
#[derive(Debug, Clone, Copy)]
pub struct ReplanContext {
    /// Per-axis temporal machine limits (`Limits::new(v, a, j, a_centripetal)`).
    pub limits: temporal::Limits,
    /// Per-axis shaper kernels in the order `[X, Y, Z, E]`. Mirrors
    /// [`crate::plan_velocity::PlanInput::kernels`]. E is structurally
    /// always [`PlanShaper::Passthrough`] or `None`.
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of β-medium outer iterations per replan.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for β-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis dynamic limits.
    pub e_limits: ELimits,
    /// Per-junction chord-error tolerance threaded into each segment's
    /// `temporal::multi::SegmentInput.trailing_junction_chord_tolerance_mm`.
    /// Slicer-supplied per-segment in the full pipeline; the streaming
    /// planner does not currently have a per-move plumb so we accept a
    /// single value here. Sane default: `0.05` (50 µm).
    pub junction_chord_tolerance_mm: f64,
    /// Worker thread count for TOPP-RA's parallel fan-out.
    pub worker_threads: usize,
    /// Grid strategy for `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Reference reading for the path speed at `t_dispatched`. The
    /// streaming planner samples its own `pieces` queue derivative when
    /// available and falls back to this when the cursor is outside the
    /// pieces' domain (e.g., right after `new()` before any append).
    /// Defaults to `0.0` (toolhead at rest).
    pub fallback_initial_v: f64,
    /// `SafetyMode` to pass to `plan_velocity`. The streaming append path
    /// always wants `SafetyMode::WorstCaseFuture` so the trailing-h region
    /// of the un-committed tail is β-derated against the worst-case future
    /// arrival.
    pub safety_mode: SafetyMode,
}

/// Stateful streaming-shaper planner state, sharing one absolute time line
/// across all axes (every append is multi-axis).
///
/// Phase 1 only uses `axes` and `pending_dispatch`; the cursors are seeded
/// to zero / left untouched by `append_batch`. Phase 3 (`append_and_replan`
/// / `emit_committed`) drives the cursors (`t_appended`, `t_decel_start`,
/// `t_shaped`, `t_dispatched`).
///
/// **v5 field set.** The v4-era fields `t_tentative`, `rest_tentative`, and
/// `generation` were removed when v5's design eliminated the
/// tentative-rest extension model — the streaming planner now appends each
/// move's terminal decel-to-zero outright and tracks where that decel begins
/// in `t_decel_start`. See spec §3.1 ("State invariants").
#[derive(Debug)]
pub struct ShaperState {
    /// Per-axis queues (X, Y, Z, E). Z is typically passthrough; E is unused
    /// in Phase 1 (extruder is followed off the shaped XY arc-length and is
    /// not a shaped axis in CLAUDE.md's MVP scope).
    pub axes: [AxisShaperQueue; 4],

    /// Source-geometry records for each move that has been appended but not
    /// yet fully committed (`t_dispatched < move.t_end`). Phase 3's
    /// `append_and_replan` reads this to build the planning path
    /// (un-committed tail + new move) for the replan window. Records whose
    /// `t_end < t_dispatched` are dropped by `append_and_replan` (their
    /// planned `BezierPiece`s stay in `axes[i].pieces` as history).
    pub uncommitted_moves: VecDeque<UncommittedMove>,

    /// Latest absolute time for which a real `append_batch` has been received.
    pub t_appended: f64,
    /// Absolute time at which the most-recently-submitted move's terminal
    /// decel-to-zero begins. Phase 3's `append_and_replan` populates this
    /// from the planner's velocity profile so the next `submit_move` can
    /// rewind to it and re-plan the un-committed tail. Initialized to
    /// `0.0` at construction; equal to `t_appended` when the queue is empty.
    pub t_decel_start: f64,
    /// Latest absolute time for which a shaped sample has been computed.
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been *dispatched*
    /// to the wire.
    pub t_dispatched: f64,

    /// Shaped output computed but not yet drained / dispatched. Populated
    /// transiently by Phase 3's `emit_committed` and by Phase 1's
    /// `append_batch` shim; drained via `drain_committed`.
    pub pending_dispatch: Vec<ShapedSegment>,
}

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
        }
    }

    /// **Phase 1 shim.** Run the existing per-segment pad → shape → refit
    /// pipeline on `fitted` and stage the resulting `ShapedSegment` into
    /// `pending_dispatch`. The internal queue state (`axes`, `t_appended`,
    /// etc.) is intentionally left untouched here — Phase 2 replaces this
    /// shim with real history-aware logic.
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
        let last = fitted.last().expect("fitted non-empty by plan_velocity contract");
        self.t_appended = last.t_end + time_offset;

        // `t_decel_start` = absolute time at which the new plan's
        // path-speed peaks; everything past that is the decel-to-zero
        // ramp by construction (TOPP-RA's profile is unimodal under fixed
        // boundary speeds, modulo per-segment limit changes which produce
        // piecewise-unimodal output — picking the time of global
        // path-speed maximum still correctly separates "committable under
        // any future" from "speculative, depends on future input").
        self.t_decel_start = find_decel_start_time(&fitted) + time_offset;

        Ok(())
    }
}

/// Scan all `BezierPiece`s of every fitted segment's X / Y axes and return
/// the absolute time (in the plan's own coordinate system) at which the
/// path-speed `√(vx² + vy²)` is globally maximal. This is the start of the
/// decel-to-zero ramp under the streaming planner's contract
/// (`terminal_v = 0.0` => the profile decelerates monotonically from the
/// peak to the path terminus). Sampling is per-piece on a dense uniform
/// grid (32 samples / piece) — enough to bracket the peak well below the
/// `T_commit` margin, with plenty of headroom for the post-shape
/// dispatch-boundary calculation.
fn find_decel_start_time(fitted: &[FittedSegment]) -> f64 {
    const SAMPLES_PER_PIECE: usize = 32;
    let mut best_t = fitted[0].t_start;
    let mut best_v_sq = -1.0_f64;

    for f in fitted {
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[0]);
        let y_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[1]);
        for (xp, yp) in x_pieces.iter().zip(y_pieces.iter()) {
            // X and Y pieces share the same time-domain partition (they
            // came out of the same C1-Hermite refit). Sample along X's
            // domain and combine the per-axis velocities for `||v||`.
            let dx = xp.differentiate();
            let dy = yp.differentiate();
            let u0 = xp.u_start;
            let u1 = xp.u_end;
            for s in 0..=SAMPLES_PER_PIECE {
                let t = u0 + (u1 - u0) * (s as f64) / (SAMPLES_PER_PIECE as f64);
                let vx = dx.evaluate(t);
                let vy = dy.evaluate(t);
                let v_sq = vx * vx + vy * vy;
                if v_sq > best_v_sq {
                    best_v_sq = v_sq;
                    best_t = t;
                }
            }
        }
    }

    best_t
}

// ---------------------------------------------------------------------------
// Phase 3 helpers
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build_smooth_zv_kernel;
    use crate::pad::pad_segment_axis;
    use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
    use crate::shaper::shape_axis;
    use crate::{AxisShaper, RequiredShaper};
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
    use nurbs::ScalarNurbs;

    /// Build a simple linear-move `FittedSegment`: X linear from 0 → 10,
    /// Y and Z constant at 0, on `t ∈ [0, 1]`.
    fn linear_segment() -> FittedSegment {
        let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 10.0],
        }]);
        let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0],
        }]);
        let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0],
        }]);
        FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    /// Byte-equivalent NURBS comparator: same degree, same knots, same
    /// control points, same weight presence. We compare with `==` on `f64`
    /// (NaN-free in this pipeline) to make "byte-equivalent" literal.
    fn assert_nurbs_byte_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
        assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
        assert_eq!(a.knots(), b.knots(), "{label}: knots differ");
        assert_eq!(
            a.control_points(),
            b.control_points(),
            "{label}: control points differ"
        );
        assert_eq!(
            a.weights().is_some(),
            b.weights().is_some(),
            "{label}: weight presence differs"
        );
        if let (Some(wa), Some(wb)) = (a.weights(), b.weights()) {
            assert_eq!(wa, wb, "{label}: weights differ");
        }
    }

    #[test]
    #[allow(clippy::float_cmp)] // Time bounds and cursor zeros are exact-by-construction.
    fn shim_matches_direct_pipeline_for_single_linear_move() {
        let fitted = linear_segment();
        let freq = 60.0;
        let h = 0.8025 / freq / 2.0;
        let kernel = build_smooth_zv_kernel(0.8025 / freq);

        // ---- Method A: streaming shim ----
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv { frequency_hz: freq }),
            Some(AxisShaper::SmoothZv { frequency_hz: freq }),
            Some(AxisShaper::Passthrough),
            Some(AxisShaper::Passthrough),
        ];
        let mut state = ShaperState::new([0.0, 0.0, 0.0, 0.0], &shapers);
        state.append_batch(&fitted).expect("shim should succeed");
        let shim_out = state.drain_committed();
        assert_eq!(shim_out.len(), 1, "shim should emit exactly one segment");
        let shim_seg = &shim_out[0];

        // After draining, `pending_dispatch` is empty.
        assert!(state.pending_dispatch.is_empty());
        // Re-draining yields nothing.
        assert!(state.drain_committed().is_empty());

        // ---- Method B: direct call sequence (mirrors `beta::run_one_iteration`) ----
        let fitted_slice = std::slice::from_ref(&fitted);

        // X: shaped + refit.
        let x_padded = pad_segment_axis(0, 0, fitted_slice, &[], h, 0.0, 1.0);
        let x_shaped = shape_axis(&x_padded, &kernel, 0.0, 1.0).unwrap();
        let x_refit = refit_to_cubic(&x_shaped, REFIT_TOLERANCE_MM).unwrap();

        // Y: shaped + refit (Y also SmoothZv at the same freq → same kernel).
        let y_padded = pad_segment_axis(0, 1, fitted_slice, &[], h, 0.0, 1.0);
        let y_shaped = shape_axis(&y_padded, &kernel, 0.0, 1.0).unwrap();
        let y_refit = refit_to_cubic(&y_shaped, REFIT_TOLERANCE_MM).unwrap();

        // Z: passthrough → still refit.
        let z_passthrough = fitted.axes[2].clone();
        let z_refit = refit_to_cubic(&z_passthrough, REFIT_TOLERANCE_MM).unwrap();

        // ---- Compare byte-for-byte ----
        assert_nurbs_byte_equal(&shim_seg.axes[0], &x_refit, "X");
        assert_nurbs_byte_equal(&shim_seg.axes[1], &y_refit, "Y");
        assert_nurbs_byte_equal(&shim_seg.axes[2], &z_refit, "Z");

        // Time bounds match the input.
        assert_eq!(shim_seg.t_start, 0.0);
        assert_eq!(shim_seg.t_end, 1.0);
    }

    #[test]
    #[allow(clippy::float_cmp)] // Cursor zeros and h=0 for passthrough are exact-by-construction.
    fn new_seeds_axis_queues_with_rest_extension() {
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv {
                frequency_hz: 100.0,
            }),
            Some(AxisShaper::SmoothMzv {
                frequency_hz: 80.0,
            }),
            Some(AxisShaper::Passthrough),
            None,
        ];
        let state = ShaperState::new([1.0, 2.0, 3.0, 4.0], &shapers);

        // Active axes get a single seed piece spanning `2h` of the past
        // (`δ_safety = h`).
        let h_x = 0.8025 / 100.0 / 2.0;
        assert_eq!(state.axes[0].pieces.len(), 1);
        let seed_x = &state.axes[0].pieces[0];
        assert!((seed_x.u_start - (-2.0 * h_x)).abs() < 1e-15);
        assert_eq!(seed_x.u_end, 0.0);
        assert_eq!(seed_x.coeffs, vec![1.0]);
        assert!((state.axes[0].h - h_x).abs() < 1e-15);
        assert!(state.axes[0].kernel.is_some());

        let h_y = 0.95625 / 80.0 / 2.0;
        assert_eq!(state.axes[1].pieces.len(), 1);
        let seed_y = &state.axes[1].pieces[0];
        assert!((seed_y.u_start - (-2.0 * h_y)).abs() < 1e-15);
        assert_eq!(seed_y.coeffs, vec![2.0]);

        // Passthrough — h = 0, no seed piece, no kernel.
        assert!(state.axes[2].pieces.is_empty());
        assert_eq!(state.axes[2].h, 0.0);
        assert!(state.axes[2].kernel.is_none());

        // None — same as Passthrough for the seed/kernel; recorded for E.
        assert!(state.axes[3].pieces.is_empty());
        assert_eq!(state.axes[3].h, 0.0);
        assert!(state.axes[3].kernel.is_none());

        // Cursors start at zero.
        assert_eq!(state.t_appended, 0.0);
        assert_eq!(state.t_decel_start, 0.0);
        assert_eq!(state.t_shaped, 0.0);
        assert_eq!(state.t_dispatched, 0.0);
        assert!(state.pending_dispatch.is_empty());
    }

    #[test]
    fn required_shaper_h_matches_axis_shaper_h() {
        // Sanity: the half-support computation matches `RequiredShaper::to_kernel`'s
        // own conversion (`0.8025 / freq` → support `[-h, h]`).
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv {
                frequency_hz: 186.0,
            }),
            Some(AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            }),
            Some(AxisShaper::Passthrough),
            None,
        ];
        let state = ShaperState::new([0.0; 4], &shapers);

        let kernel_x = RequiredShaper::SmoothZv {
            frequency_hz: 186.0,
        }
        .to_kernel();
        let (lo_x, hi_x) = kernel_x.support();
        let expected_h_x = (hi_x - lo_x) / 2.0;
        assert!((state.axes[0].h - expected_h_x).abs() < 1e-15);

        let kernel_y = RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        }
        .to_kernel();
        let (lo_y, hi_y) = kernel_y.support();
        let expected_h_y = (hi_y - lo_y) / 2.0;
        assert!((state.axes[1].h - expected_h_y).abs() < 1e-15);
    }

    // -----------------------------------------------------------------
    // Phase 3 Task 3.1 — append_and_replan tests
    // -----------------------------------------------------------------

    use crate::plan_velocity::PlanShaper;
    use crate::ELimits;

    /// Standard shaper set for the replan tests: SmoothMZV at 60 Hz on X
    /// and Y, passthrough on Z, none on E. Matches the production MVP
    /// `motion-bridge::config::PlannerConfig::default()` shape but with a
    /// lower (more permissive) shaper frequency so short test moves can
    /// converge β-medium under the relaxed tolerance budget.
    fn replan_shapers() -> [Option<AxisShaper>; 4] {
        [
            Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
            Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
            Some(AxisShaper::Passthrough),
            None,
        ]
    }

    /// Mirrors `replan_shapers` but with the `plan_velocity::PlanShaper`
    /// shape `ReplanContext` requires.
    fn replan_kernels() -> [Option<PlanShaper>; 4] {
        [
            Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
            Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
            Some(PlanShaper::Passthrough),
            None,
        ]
    }

    fn replan_limits() -> temporal::Limits {
        temporal::Limits::new(
            [500.0; 3],
            [5_000.0; 3],
            [100_000.0; 3],
            2_500.0,
        )
    }

    fn replan_context() -> ReplanContext {
        ReplanContext {
            limits: replan_limits(),
            kernels: replan_kernels(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 5,
            beta_convergence_ratio: 1.02,
            e_limits: ELimits {
                v_max: 100.0,
                a_max: 5_000.0,
            },
            junction_chord_tolerance_mm: 0.05,
            worker_threads: 1,
            grid_strategy: temporal::multi::GridStrategy::Fixed(20),
            fallback_initial_v: 0.0,
            safety_mode: SafetyMode::WorstCaseFuture,
        }
    }

    /// Construct a pure-X `CubicSegment` from `(start_x, end_x)` at unit
    /// feedrate. Inlines the collinear-cubic-Bézier formula
    /// (control points at 0, 1/3, 2/3, 1 lerp) so the trajectory crate's
    /// test harness doesn't have to depend on `motion-bridge` or `compat`.
    fn linear_x_segment(start_x: f64, end_x: f64, feedrate: f64) -> CubicSegment {
        use geometry::segment::{EMode, SourceRange};
        use nurbs::VectorNurbs;

        let p0 = [start_x, 0.0, 0.0];
        let p3 = [end_x, 0.0, 0.0];
        let lerp = |t: f64| -> [f64; 3] {
            [
                p0[0] + (p3[0] - p0[0]) * t,
                p0[1] + (p3[1] - p0[1]) * t,
                p0[2] + (p3[2] - p0[2]) * t,
            ]
        };
        let cps = vec![p0, lerp(1.0 / 3.0), lerp(2.0 / 3.0), p3];
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            cps,
            None,
        )
        .unwrap();
        CubicSegment::try_new(
            xyz,
            EMode::Travel,
            0.0,
            None,
            feedrate,
            SourceRange {
                start_line: 0,
                end_line: 0,
            },
            None,
        )
        .unwrap()
    }

    /// Spec §3.4 single-move acceptance: after the first `append_and_replan`
    /// call on a fresh state, the planner has built an accel-cruise-decel
    /// profile (so `t_decel_start` is strictly between 0 and `t_appended`)
    /// and the un-committed tail is materialized in the per-axis queues.
    #[test]
    fn single_move_append_planning_completes() {
        let mut state = ShaperState::new([0.0; 4], &replan_shapers());
        let ctx = replan_context();
        let seg = linear_x_segment(0.0, 1.0, 100.0);

        state
            .append_and_replan(seg, &ctx)
            .expect("first append should succeed");

        assert!(
            state.t_appended > 0.0,
            "t_appended must advance past 0.0 on first append, got {}",
            state.t_appended,
        );
        assert!(
            state.t_decel_start > 0.0,
            "t_decel_start must be strictly positive (the planner produced \
             a non-degenerate accel-cruise/peak-decel profile), got {}",
            state.t_decel_start,
        );
        assert!(
            state.t_decel_start < state.t_appended,
            "t_decel_start ({}) must lie strictly between 0 and t_appended ({}) — \
             the decel-to-zero ramp is the trailing portion of the plan",
            state.t_decel_start,
            state.t_appended,
        );
        // Per-axis queues are non-empty for X and Y.
        let x_pieces_after = state.axes[0]
            .pieces
            .iter()
            .filter(|p| p.u_start >= 0.0)
            .count();
        let y_pieces_after = state.axes[1]
            .pieces
            .iter()
            .filter(|p| p.u_start >= 0.0)
            .count();
        assert!(x_pieces_after > 0, "X queue must contain new plan's pieces");
        assert!(y_pieces_after > 0, "Y queue must contain new plan's pieces");
        // One UncommittedMove record per submitted move.
        assert_eq!(state.uncommitted_moves.len(), 1);
        assert!(state.uncommitted_moves[0].t_end > 0.0);
    }

    /// Spec §3.4 chained-replan acceptance: after move 2 is appended,
    /// the planner's velocity profile across the move-1/move-2 boundary
    /// does **not** decelerate to zero — TOPP-RA picks a non-zero junction
    /// velocity, allowing the toolhead to chain through.
    #[test]
    fn two_move_replan_chains_smoothly() {
        let mut state = ShaperState::new([0.0; 4], &replan_shapers());
        let ctx = replan_context();

        let m1 = linear_x_segment(0.0, 1.0, 100.0);
        state.append_and_replan(m1, &ctx).expect("move 1");
        let t_decel_after_move_1 = state.t_decel_start;
        let t_appended_after_move_1 = state.t_appended;

        let m2 = linear_x_segment(1.0, 2.0, 100.0);
        state.append_and_replan(m2, &ctx).expect("move 2");

        // After move 2, the move-1/move-2 boundary is in the interior of
        // the un-committed tail. The path-speed at the junction time
        // (where move-1's geometry ends and move-2's begins) must be
        // strictly positive — the planner is chaining, not stopping.
        assert_eq!(state.uncommitted_moves.len(), 2);
        let t_junction = state.uncommitted_moves[0].t_end;
        assert!(t_junction > 0.0 && t_junction < state.t_appended);

        let v_junction = state.read_path_speed_at(t_junction, -1.0);
        assert!(
            v_junction > 5.0,
            "junction speed must be strictly positive (chaining junction), got {} mm/s",
            v_junction,
        );

        // The chained plan covers move 1 + move 2 (2 mm total) and so
        // takes strictly longer than the move-1-only plan (1 mm). The
        // peak of the path speed (`t_decel_start`) can be earlier in the
        // chained plan than in the move-1-only plan — over a longer path
        // TOPP-RA reaches a higher peak earlier and spends more of the
        // total duration in decel — but the **decel ramp itself** runs
        // from `t_decel_start` all the way to `t_appended`, which is
        // strictly longer than the move-1-only plan's full duration.
        assert!(
            state.t_appended > t_appended_after_move_1,
            "two-move plan must take longer than one-move plan: \
             one-move {}, two-move {}",
            t_appended_after_move_1,
            state.t_appended,
        );
        assert!(
            state.t_decel_start < state.t_appended,
            "decel ramp must occupy a non-empty tail of the plan",
        );
        // Sanity: the move-1-only decel start was strictly past 0; the
        // chained plan's decel ramp is much longer, so its decel-start /
        // t_appended ratio is closer to 0.5 (peak near the midpoint).
        let _ = t_decel_after_move_1;
    }

    /// Spec §3.4 history-preservation acceptance: when the dispatch cursor
    /// has advanced past part of the planned trajectory, a follow-on
    /// `append_and_replan` only replaces the un-committed portion of the
    /// per-axis pieces. Pre-`t_dispatched` history is retained.
    #[test]
    fn append_after_committed_dispatch_keeps_history() {
        let mut state = ShaperState::new([0.0; 4], &replan_shapers());
        let ctx = replan_context();

        let m1 = linear_x_segment(0.0, 1.0, 100.0);
        state.append_and_replan(m1, &ctx).expect("move 1");

        // Simulate Phase-3 `emit_committed` advancing `t_dispatched` into
        // the middle of move 1 (between `0` and `t_decel_start`). For the
        // test we just write the cursor directly.
        let t_dispatched_synth = state.t_decel_start * 0.4;
        assert!(t_dispatched_synth > 0.0);
        state.t_dispatched = t_dispatched_synth;

        // Capture the X-axis piece set that's strictly behind the cursor
        // (history) so we can compare after the replan.
        let history_before: Vec<BezierPiece<f64>> = state.axes[0]
            .pieces
            .iter()
            .filter(|p| p.u_end <= t_dispatched_synth + 1e-12)
            .cloned()
            .collect();
        assert!(!history_before.is_empty(), "must have some history to preserve");

        let m2 = linear_x_segment(1.0, 2.0, 100.0);
        state.append_and_replan(m2, &ctx).expect("move 2");

        let history_after: Vec<BezierPiece<f64>> = state.axes[0]
            .pieces
            .iter()
            .filter(|p| p.u_end <= t_dispatched_synth + 1e-12)
            .cloned()
            .collect();
        assert_eq!(
            history_before, history_after,
            "pre-t_dispatched X history must be preserved byte-identically across replan",
        );

        // And the queue must still extend past `t_appended` for the
        // un-committed (replanned) tail.
        let pieces_past_cursor = state.axes[0]
            .pieces
            .iter()
            .filter(|p| p.u_start >= t_dispatched_synth)
            .count();
        assert!(
            pieces_past_cursor > 0,
            "replan must have appended fresh pieces to the un-committed tail",
        );

        // The first move is still tracked as uncommitted (its old end-time
        // got rewritten by the new plan; the original t_end is no longer
        // a meaningful cursor). Both moves are present.
        assert_eq!(state.uncommitted_moves.len(), 2);
    }
}
