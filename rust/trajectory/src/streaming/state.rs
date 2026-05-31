// ShaperState construction + Phase-1 byte-identity shim + Phase-3 Task-3.1
// `append_and_replan` and its supporting helpers.

use std::collections::VecDeque;

use geometry::segment::{split_cubic_bezier, CubicSegment};
use nurbs::bezier::{extract_bezier_pieces, BezierPiece};

use super::decel_finder::find_decel_start_time;
use super::{AxisShaperQueue, ReplanContext, ShaperState, UncommittedMove};
use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::plan_velocity::{plan_velocity, PlanInput, PlanSegment};
use crate::AxisShaper;
use crate::ShapeError;
use crate::ShapedSegment;

// ---------------------------------------------------------------------------
// Shared epsilon / tolerance constants
// ---------------------------------------------------------------------------

/// Lookup tolerance for absolute-time membership checks across the streaming
/// planner. Used as a half-open-interval slack so a piece's `u_start` reads as
/// covering `t` when `t` is bit-equal to the boundary, without admitting a
/// non-adjacent neighbouring piece. Sub-picosecond ≪ any meaningful timing
/// quantum on the wire (the MCU's tick rate is ~10 µs).
const TIME_LOOKUP_TOLERANCE: f64 = 1e-12;

/// Boundary slack for the `s_dispatched ∈ (0, 1)` interior check on the cubic
/// Bézier inverter result. Below this, the split would either trigger
/// [`split_cubic_bezier`]'s strict-interior panic or produce a degenerate
/// left/right half whose control polygon collapses to a single point. Wider
/// than [`TIME_LOOKUP_TOLERANCE`] because the Newton iterate has more
/// numerical wander than absolute-time arithmetic.
const SPLIT_BOUNDARY_TOLERANCE: f64 = 1e-9;

/// Threshold for the pure-X-axis check in [`invert_cubic_bezier_xyz_to_param`]
/// (control points on Y and Z must vanish to within this tolerance to enable
/// the closed-form `s` solve). One picometer is well inside the planner's
/// position resolution and is unaffected by refit noise (refits operate at
/// `5 µm` L∞ on position).
const PURE_AXIS_TOLERANCE: f64 = 1e-12;

/// Threshold for the collinear-cubic-Bézier check in
/// [`invert_cubic_bezier_xyz_to_param`]. A pure-X cubic whose middle control
/// points sit within this distance of the (1/3, 2/3) lerp of the endpoints
/// is treated as exactly collinear, allowing the analytic `s = (p − x0) /
/// (x3 − x0)` shortcut. One nanometer is below typical refit noise so this
/// short-circuit fires on every collinear-cubic input emitted by
/// `linear_x_segment` / `to_collinear_g5`.
const COLLINEAR_TOLERANCE: f64 = 1e-9;

/// Floor for `f_prime` in the Newton denominator. Below this, the iteration
/// is at a stationary point of the closest-point function; we break out and
/// accept whatever `s` we have rather than dividing by ~0.
const NEWTON_DENOMINATOR_FLOOR: f64 = 1e-18;

/// Convergence threshold for the Newton step size on the normalized
/// parameter `s ∈ [0, 1]`. 1 ULP of `s` corresponds to sub-nanometer position
/// on a 200 mm move; this is well below the refit's `5 µm` L∞ budget.
const NEWTON_PARAM_TOLERANCE: f64 = 1e-12;

/// Maximum Newton iterations before bailing. Quadratic convergence on the
/// closest-point function for a well-behaved single-piece cubic Bézier means
/// 6 iterations is typical; 12 is defensive insurance against numerical edge
/// cases.
const NEWTON_MAX_ITERS: usize = 12;

/// Residual budget for the Newton inverter's post-convergence position
/// check: `||xyz(s) − p_target|| < NEWTON_RESIDUAL_MM` is required for the
/// returned `s` to be accepted. Set to 10× the C¹ refit's L∞ tolerance
/// (`REFIT_TOLERANCE_MM` = 5 µm) so a successful Newton converge on a
/// genuinely-on-curve target lands well inside the budget, while a Newton
/// converge to a wrong root (self-intersecting / highly-curved geometry,
/// stationary-point trap) lies far enough off the curve to be rejected.
/// Documented inline at the call site.
const NEWTON_RESIDUAL_MM: f64 = 0.05;

/// Seed clamp band for the Newton initial guess `s_seed`. Keeps the
/// iteration off the closed-form boundary cases when the time fraction
/// `(t_d − t_start) / (t_end − t_start)` is numerically equal to 0 or 1.
const NEWTON_SEED_CLAMP: f64 = 1e-6;

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

    /// **Phase 5 Task 5.1** — reset the streaming state to a fresh
    /// `home_pos` seed without rebuilding kernels. Mirrors
    /// [`Self::new`]'s seeding behaviour: per-axis queues are cleared and
    /// re-seeded with a `(home_pos[i], v=0)` rest extension covering
    /// `[-( h + δ_safety ), 0]` using the **existing** per-axis `h`
    /// (kernel / half-support are preserved).
    ///
    /// All other planner state is wiped:
    /// - `uncommitted_moves` cleared (the in-flight tail is invalidated by
    ///   the reset — homing / underrun / klippy-reconnect by definition
    ///   abandons whatever was planned).
    /// - `planned_fitted` / `planned_meta` cleared (their alignment with
    ///   `uncommitted_moves` is the call-site invariant; both have to go).
    /// - `pending_dispatch` cleared (un-drained shaped output staged from
    ///   the previous timeline is no longer valid post-reset).
    /// - All cursors (`t_appended`, `t_decel_start`, `t_shaped`,
    ///   `t_dispatched`) zeroed — the absolute-time line restarts at 0
    ///   matching `ShaperState::new`'s conventions, so the run-loop's
    ///   `last_append_time` book-keeping behaves identically to a fresh
    ///   construction.
    ///
    /// Called on: `kalico_stream_open`, homing /
    /// `SET_KINEMATIC_POSITION`, engine `Underrun` fault, `force_idle`,
    /// klippy reconnect (spec §3.7).
    ///
    /// **Kernel preservation.** Unlike `UpdateShaper` — which rebuilds
    /// kernels — `reset` deliberately retains the existing per-axis
    /// kernel / `h`. The two paths are different events: shaper
    /// reconfiguration is a config update, reset is a position re-anchor.
    /// `UpdateShaper`'s job is to swap kernels; `reset`'s job is to seed
    /// the queue at a new home position.
    pub fn reset(&mut self, home_pos: [f64; 4]) {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            reseed_axis_queue(axis, home_pos[i]);
        }
        self.uncommitted_moves.clear();
        self.pending_dispatch.clear();
        self.planned_fitted.clear();
        self.planned_meta.clear();
        self.t_appended = 0.0;
        self.t_decel_start = 0.0;
        self.t_shaped = 0.0;
        self.t_dispatched = 0.0;
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

        // 2. Snapshot the prior state so we can roll back on plan failure
        //    (per the "atomic on error" contract). The snapshot covers every
        //    field this function mutates *before* the error point — that's
        //    `uncommitted_moves` (mutated in-place by `retain` + the partial-
        //    split rewrite + the new-move push) and the timeline cursors
        //    `t_appended` / `t_decel_start`. `planned_fitted` /
        //    `planned_meta` are not mutated until after `plan_velocity`
        //    returns Ok, but we snapshot them too as defense-in-depth
        //    against future edits introducing pre-error mutations — the
        //    next call's `split_partially_committed_at_t_dispatched` reads
        //    `planned_fitted` to find the partially-committed move's
        //    target position, so any inconsistency between `planned_fitted`
        //    and `uncommitted_moves` would silently mis-split the cubic.
        //    `Vec::clone` on these is cheap relative to TOPP-RA's runtime.
        let prior_uncommitted = self.uncommitted_moves.clone();
        let prior_t_appended = self.t_appended;
        let prior_t_decel_start = self.t_decel_start;
        let prior_planned_fitted = self.planned_fitted.clone();
        let prior_planned_meta = self.planned_meta.clone();

        // Resolve the partial-commit split against the **pre-mutation**
        // state. The lookup uses `planned_fitted` (un-committed unshaped
        // plan) to read the toolhead position at `t_dispatched`; it must
        // happen before we touch `uncommitted_moves` since the resolver's
        // 1:1 index alignment between `planned_fitted` and
        // `uncommitted_moves` is the search invariant.
        let partial_split = self.split_partially_committed_at_t_dispatched();

        self.uncommitted_moves
            .retain(|m| m.t_end > self.t_dispatched);

        // Apply the split (if any). Three cases:
        //
        //   * `None` — no straddling move (either no `planned_fitted`
        //     entry covered `t_dispatched`, or `s_dispatched` is near 0
        //     meaning the dispatched position equals the move's origin).
        //     Leave the queue untouched.
        //
        //   * `Some(Replace { new_segment })` — substitute the front
        //     move's `segment` with the right-half cubic so the planner
        //     sees only the un-committed path tail.
        //
        //   * `Some(DropFromQueue)` — `s_dispatched ≈ 1`: the move is
        //     essentially fully committed (the toolhead has been
        //     dispatched-through to ~its terminus). The `retain` above
        //     uses `t_end > t_dispatched`, which keeps the move when
        //     `t_d == t_end - ε` (the typical post-lookup geometry), so
        //     we must explicitly pop the front entry here. Leaving it
        //     intact would cause the planner to re-emit the entire move
        //     from its geometric origin at `t_dispatched` — the exact
        //     ~94 mm seam jump Phase 3 Task 3.1.5 was introduced to fix.
        match partial_split {
            Some(PartialCommitSplit::Replace { new_segment }) => {
                // The straddling move is the first remaining
                // `uncommitted_moves` entry by the time-ordering
                // invariant: prior moves were dropped by `retain` (since
                // their `t_end <= t_dispatched`), and later moves have
                // `t_start >= prior.t_end > t_dispatched` so they cannot
                // straddle. If the queue is empty after retain, the
                // straddling move was already dropped — in that case
                // `partial_split` would have been `None` (we cross-check
                // against `uncommitted_moves` in the resolver), so we
                // wouldn't reach this branch.
                if let Some(front) = self.uncommitted_moves.front_mut() {
                    front.segment = new_segment;
                    // `t_start` will be refreshed from the new plan; for
                    // now record that the un-committed portion starts at
                    // the dispatch boundary so ordering stays consistent.
                    front.t_start = self.t_dispatched;
                }
            }
            Some(PartialCommitSplit::DropFromQueue) => {
                // Drop the straddling move. `retain` leaves it in place
                // when `s ≈ 1` because by definition the lookup found a
                // `planned_fitted` entry with `t_d < t_end` (strict half-
                // open interval), so `t_end > t_d` and `retain` keeps
                // it. We pop manually to enforce "drop, not re-include."
                self.uncommitted_moves.pop_front();
            }
            None => {}
        }

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
        //
        //    Per-segment Limits override: the SOCP's path-frame jerk bound is
        //    `min(j_max[X], j_max[Y], j_max[Z])`
        //    (temporal/src/topp/constraints.rs:258). For a curve where some
        //    axes have zero displacement (e.g. pure-X G1 → collinear cubic
        //    Bezier), the inactive axes' j_max would otherwise dominate this
        //    min and clamp the path's d³s/dt³ to nonsense values. We bump
        //    inactive axes' j_max[axis] to the max across active axes so the
        //    min() reduces to the active-axis bound. Inactive axes have
        //    c'_ax(s) ≡ 0, so cartesian jerk on them is identically zero
        //    regardless of d³s/dt³ — the bump is correctness-preserving.
        //    Step 9 will replace this with per-axis Cartesian jerk SOCP
        //    relaxation; until then this caller-side patch unblocks high-
        //    accel printers whose Z (or E) has a much lower jerk than X/Y.
        let plan_segments: Vec<PlanSegment<'_>> = self
            .uncommitted_moves
            .iter()
            .map(|m| PlanSegment {
                temporal: temporal::multi::SegmentInput {
                    curve: &m.segment.xyz,
                    limits: per_segment_limits(
                        &m.segment.xyz,
                        ctx.limits,
                        m.segment.feedrate_mm_s,
                    ),
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
                // Roll back every field touched (or potentially touched) on
                // this code path so the caller sees an unchanged state on
                // error. See the snapshot site above for why
                // `planned_fitted` / `planned_meta` are restored too — the
                // next call's partial-commit resolver reads `planned_fitted`
                // and requires its 1:1 alignment with `uncommitted_moves` to
                // survive across a failed replan attempt.
                self.uncommitted_moves = prior_uncommitted;
                self.t_appended = prior_t_appended;
                self.t_decel_start = prior_t_decel_start;
                self.planned_fitted = prior_planned_fitted;
                self.planned_meta = prior_planned_meta;
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
        if t >= last.u_end && t <= last.u_end + TIME_LOOKUP_TOLERANCE {
            return Some(last.differentiate().evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - TIME_LOOKUP_TOLERANCE <= t && t < p.u_end {
                return Some(p.differentiate().evaluate(t));
            }
        }
        None
    }

    /// The settled toolhead position: each axis's unshaped curve evaluated at
    /// the end of the appended timeline (`t_appended`). After a `T_COMMIT`
    /// decel-to-zero commit this is the rest position, suitable for feeding to
    /// [`Self::reset`] to rewind the planner clock without moving the toolhead.
    ///
    /// Shaped axes (`h > 0`) always carry pieces covering `t_appended`, so they
    /// read exactly. Passthrough / none axes (`h == 0`) may have an empty queue;
    /// their fallback (`0.0`) is a don't-care because [`reseed_axis_queue`]
    /// discards the seed position for `h == 0` axes — an empty queue carries no
    /// position, and the next move on such an axis re-derives position from its
    /// own absolute geometry.
    #[must_use]
    pub fn current_position(&self) -> [f64; 4] {
        std::array::from_fn(|i| self.axis_position_at(i, self.t_appended).unwrap_or(0.0))
    }

    /// Advance the planner timeline from the current `t_appended` to `target_t`
    /// by inserting a "park at current position, v=0" rest segment on every
    /// shaped axis. Host-side only — nothing is dispatched; the MCU is genuinely
    /// at rest (it holds position with an empty queue), so the shaper history
    /// window stays valid with no reseed — the hold *is* the rest extension.
    ///
    /// No-op when `target_t <= t_appended` (caller's "queued-ahead" branch).
    ///
    /// **Precondition:** fully-committed state — `t_dispatched == t_appended` and
    /// `pending_dispatch` empty. The Move-arm placement rule commits the held-back
    /// tail via `run_commit_and_dispatch` immediately before calling this.
    ///
    /// After this call ALL FOUR cursors advance to `target_t`
    /// (`t_appended == t_decel_start == t_dispatched == t_shaped == target_t`),
    /// and `uncommitted_moves` / `planned_fitted` / `planned_meta` are cleared.
    /// `t_dispatched` MUST advance: `append_and_replan` plans the next move at
    /// `time_offset = t_dispatched` and `replace_uncommitted_axis_pieces` drops
    /// pieces with `u_start >= t_dispatched`. Leaving it behind would drop the
    /// hold (`u_start == old t_appended`) and plan the next move back at the old
    /// time — re-creating the -308. With `t_dispatched = target_t`, the hold
    /// pieces (`u_start < t_dispatched`) survive as committed shaping history and
    /// the next move plans from "now".
    pub fn advance_idle(&mut self, target_t: f64) {
        if target_t <= self.t_appended + 1e-12 {
            return;
        }
        debug_assert!(
            (self.t_dispatched - self.t_appended).abs() < 1e-9,
            "advance_idle requires fully-committed state: t_dispatched {} != t_appended {}",
            self.t_dispatched,
            self.t_appended,
        );
        debug_assert!(
            self.pending_dispatch.is_empty(),
            "advance_idle requires pending_dispatch drained before advancing",
        );
        let hold_start = self.t_appended;
        let hold_end = target_t;
        let end_pos: [f64; 4] =
            std::array::from_fn(|i| self.axis_position_at(i, hold_start).unwrap_or(0.0));

        for (i, axis) in self.axes.iter_mut().enumerate() {
            if axis.h > 0.0 {
                axis.pieces.push_back(BezierPiece {
                    u_start: hold_start,
                    u_end: hold_end,
                    coeffs: vec![end_pos[i]],
                });
            }
        }

        self.uncommitted_moves.clear();
        self.planned_fitted.clear();
        self.planned_meta.clear();
        self.t_appended = hold_end;
        self.t_decel_start = hold_end;
        self.t_dispatched = hold_end;
        self.t_shaped = hold_end;
    }

    /// Evaluate axis `axis_idx`'s unshaped position curve at time `t`. Mirrors
    /// [`Self::axis_velocity_at`] (same piece-walk and terminal clamp) but
    /// evaluates the piece itself rather than its derivative. `None` when the
    /// axis queue is empty or no piece covers `t`.
    fn axis_position_at(&self, axis_idx: usize, t: f64) -> Option<f64> {
        let pieces = &self.axes[axis_idx].pieces;
        if pieces.is_empty() {
            return None;
        }

        // Last-piece terminal: clamp `t` to `u_end` (the decel-to-zero ends at
        // the target position; evaluating at `u_end` returns it).
        let last = pieces.back().unwrap();
        if t >= last.u_end && t <= last.u_end + TIME_LOOKUP_TOLERANCE {
            return Some(last.evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - TIME_LOOKUP_TOLERANCE <= t && t < p.u_end {
                return Some(p.evaluate(t));
            }
        }
        None
    }

    /// **Phase 3 Task 3.1.5 — partial-commit replan.** Identify the move
    /// (if any) whose time domain straddles `t_dispatched`, read the
    /// unshaped toolhead position at `t_dispatched` from the **prior**
    /// `planned_fitted` cache, invert the move's source cubic Bézier to
    /// find the matching parameter `s_dispatched ∈ (0, 1)`, and return an
    /// owned [`PartialCommitSplit`] describing how the caller should
    /// rewrite the front of `uncommitted_moves`.
    ///
    /// Return value (owned, by value — *not* by reference):
    /// - `Some(PartialCommitSplit::Replace { new_segment })`: substitute
    ///   the partially-committed move's `segment` with the right-half cubic
    ///   covering `s ∈ [s_dispatched, 1]`. This is the typical case — a
    ///   non-trivial portion of the move remains un-committed.
    /// - `Some(PartialCommitSplit::DropFromQueue)`: `s_dispatched` lies in
    ///   the right-boundary band (`≥ 1 − SPLIT_BOUNDARY_TOLERANCE`); the
    ///   move is essentially fully committed and the caller should ensure
    ///   it is dropped rather than left intact (otherwise the new plan
    ///   would re-start that move from its geometric origin, the bug this
    ///   helper was introduced to fix). In practice the `retain` step in
    ///   `append_and_replan` already drops the move when
    ///   `t_dispatched ≥ t_end` — which is guaranteed at `s ≈ 1` for any
    ///   strictly-monotone time parameterization — so this variant is a
    ///   tracer for the caller's bookkeeping rather than a separate
    ///   removal request. We retain it as an explicit, documented case.
    ///
    /// Returns `None` when:
    /// - No `planned_fitted` entry covers `t_dispatched` (e.g., the very
    ///   first append after construction; or a prior emit dispatched all of
    ///   the front move and its `t_end` now equals `t_dispatched`).
    /// - The matching `UncommittedMove`'s `e_mode` is `Independent`. The
    ///   streaming planner does not currently feed Independent E moves
    ///   through `plan_velocity`, so a partial-commit there cannot arise
    ///   in production; we skip the split defensively.
    /// - `s_dispatched` lies in the left-boundary band
    ///   (`≤ SPLIT_BOUNDARY_TOLERANCE`): the unshaped position at
    ///   `t_dispatched` essentially equals the move's geometric origin, so
    ///   no split is required — the new plan can use the full move geometry
    ///   directly.
    /// - The Newton inverter's residual `||xyz(s_dispatched) − p_target||`
    ///   exceeds [`NEWTON_RESIDUAL_MM`]. This catches Newton converging
    ///   to a wrong root (self-intersecting or highly-curved cubics, or
    ///   stationary-point traps where `f_prime ≈ 0`). On a wrong-root
    ///   result we skip the split (same effect as out-of-bounds) rather
    ///   than substituting a geometrically-incorrect right-half cubic. The
    ///   `debug_assert!` makes the failure visible in test builds; release
    ///   builds silently fall back to "no split" which produces the same
    ///   ~94 mm seam jump the Phase 3 Task 3.1.5 fix was introduced to
    ///   eliminate — but on geometry where our split couldn't have been
    ///   trusted anyway, this is strictly better than a silent wrong
    ///   answer downstream.
    ///
    /// **Why we read position from `planned_fitted` rather than the post-
    /// shape `axes[i].pieces` history.** The replan's `plan_velocity` step
    /// produces an *unshaped* trajectory. To make that new unshaped
    /// trajectory continuous with the in-flight motion at `t_dispatched`,
    /// the splitting `s_dispatched` must correspond to the **unshaped**
    /// toolhead position the prior plan placed there — not the shaped
    /// position the kernel convolution produces. `planned_fitted` is the
    /// prior unshaped time-domain plan, exactly that value.
    ///
    /// Called before [`Self::append_and_replan`] mutates `uncommitted_moves`
    /// (specifically before `retain`), so the indices of `planned_fitted`
    /// and `uncommitted_moves` still align 1:1.
    fn split_partially_committed_at_t_dispatched(&self) -> Option<PartialCommitSplit> {
        // Find the prior plan's segment whose time domain contains
        // `t_dispatched`. Standard half-open interval `[t_start, t_end)`
        // semantics with a small lookup slack on the *left* side only —
        // the right side stays strict so `t_d == t_end` is unambiguously
        // *not* covered (it belongs to the next segment if there is one,
        // and otherwise means dispatch has caught up with the planned
        // terminus, which is "no straddle" territory).
        let t_d = self.t_dispatched;
        let (idx, planned) = self
            .planned_fitted
            .iter()
            .enumerate()
            .find(|(_, f)| f.t_start - TIME_LOOKUP_TOLERANCE <= t_d && t_d < f.t_end)?;

        // Cross-check with `uncommitted_moves` — the indices must match.
        let move_ref = self.uncommitted_moves.get(idx)?;

        // Independent-E moves never carry XY motion to split. Skip
        // defensively (the streaming planner does not feed these through
        // `plan_velocity` today, so this branch is unreachable in
        // production).
        if matches!(
            move_ref.segment.e_mode,
            geometry::segment::EMode::Independent
        ) {
            return None;
        }

        // Read the unshaped XYZ toolhead position at `t_dispatched`.
        let p_target = [
            nurbs::eval::eval(&planned.axes[0], t_d),
            nurbs::eval::eval(&planned.axes[1], t_d),
            nurbs::eval::eval(&planned.axes[2], t_d),
        ];

        // Invert the cubic Bézier to find `s_dispatched`. The point is
        // known to lie on the curve to within refit noise (≤ 5 µm by the
        // C¹ Hermite fit's tolerance) — so Newton converges in a handful
        // of iterations regardless of how curved the move is.
        //
        // Initial seed: the fraction of *time* dispatched within the
        // move. For an axis-aligned constant-cruise move that's already
        // exact; for accel/decel ramps it's within a few percent of the
        // true `s`, which Newton tightens fast.
        let move_span_t = planned.t_end - planned.t_start;
        let s_seed = if move_span_t > TIME_LOOKUP_TOLERANCE {
            ((t_d - planned.t_start) / move_span_t)
                .clamp(NEWTON_SEED_CLAMP, 1.0 - NEWTON_SEED_CLAMP)
        } else {
            0.5
        };
        let s_dispatched =
            invert_cubic_bezier_xyz_to_param(&move_ref.segment.xyz, p_target, s_seed)?;

        // Boundary triage. The split function panics on `s == 0` or `s == 1`
        // and produces a degenerate (zero-arc-length) half on near-boundary
        // values, so we route those cases away from `split_cubic_bezier`:
        //
        //   * `s ≤ SPLIT_BOUNDARY_TOLERANCE` — the dispatched position is
        //     essentially the move's origin. The new plan can use the full
        //     move geometry directly; return `None` so the caller leaves
        //     `uncommitted_moves` untouched.
        //
        //   * `s ≥ 1 − SPLIT_BOUNDARY_TOLERANCE` — the move is essentially
        //     fully committed. The new plan must *not* re-include the
        //     move's geometry (otherwise the seam-residue bug returns).
        //     The `retain` step in `append_and_replan` already drops moves
        //     with `t_end ≤ t_dispatched`, which by strict-monotonicity of
        //     the move's time parameterization is guaranteed whenever
        //     `s ≈ 1`. We return `DropFromQueue` as an explicit tracer so
        //     the call site documents the case rather than relying solely
        //     on `retain`'s side effect.
        if s_dispatched <= SPLIT_BOUNDARY_TOLERANCE {
            return None;
        }
        if s_dispatched >= 1.0 - SPLIT_BOUNDARY_TOLERANCE {
            return Some(PartialCommitSplit::DropFromQueue);
        }

        let (_left, right) = split_cubic_bezier(&move_ref.segment.xyz, s_dispatched);

        // Reconstruct a `CubicSegment` from the right half, carrying
        // through the move's metadata. `extrusion_per_xy_mm` is a *rate*
        // (mm E per mm XY), so it stays the same on the shorter tail.
        // `feedrate_mm_s` is the move's target cruise speed and is also
        // a rate.
        let new_segment = CubicSegment::try_new(
            right,
            move_ref.segment.e_mode,
            move_ref.segment.extrusion_per_xy_mm,
            move_ref.segment.e_independent.clone(),
            move_ref.segment.feedrate_mm_s,
            move_ref.segment.source,
            move_ref.segment.split_info,
        )
        .expect("split_cubic_bezier output is a valid single-piece cubic Bézier");

        Some(PartialCommitSplit::Replace { new_segment })
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
            if back.u_start >= t_keep_cutoff - TIME_LOOKUP_TOLERANCE {
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

/// Output of [`ShaperState::split_partially_committed_at_t_dispatched`]: an
/// owned description (not a borrow into `self`) of how the caller should
/// rewrite the front of `uncommitted_moves`.
enum PartialCommitSplit {
    /// `s_dispatched` is strictly interior to `(0, 1)`: substitute the
    /// front move's `segment` with `new_segment`, the right-half cubic
    /// Bézier covering `s ∈ [s_dispatched, 1]` of the original
    /// (re-parameterized to `[0, 1]`). All non-geometric metadata
    /// (`e_mode`, `extrusion_per_xy_mm`, `feedrate_mm_s`, etc.) is
    /// inherited from the original move.
    Replace { new_segment: CubicSegment },
    /// `s_dispatched ≥ 1 − SPLIT_BOUNDARY_TOLERANCE`: the partially-
    /// committed move is essentially fully committed. The caller must
    /// ensure the move is dropped from `uncommitted_moves` rather than
    /// re-included in the new plan input. See
    /// [`ShaperState::split_partially_committed_at_t_dispatched`] for the
    /// time-ordering invariant that lets the existing `retain` step
    /// satisfy this requirement automatically.
    DropFromQueue,
}

/// Invert a single-piece cubic Bézier in 3D for the parameter `s ∈ [0, 1]`
/// at which `xyz(s)` matches a known on-curve target point. Newton iteration
/// on `f(s) = (xyz(s) − p_target) · xyz'(s) = 0` (the closest-point criterion;
/// since `p_target` lies on the curve the closest-point and identity solutions
/// coincide).
///
/// Initialized at `s_seed`. Converges in ≤ 6 iterations on the test corpus;
/// we cap at [`NEWTON_MAX_ITERS`] as defensive insurance.
///
/// **Wrong-root guard.** For self-intersecting or highly-curved cubics
/// Newton's local convergence can land on a *different* root of `f(s) = 0`
/// — geometrically, the closest-point function has additional stationary
/// points where the curve folds back on itself, and Newton from a far-off
/// seed can be drawn to one of those. The closest-point function does not
/// distinguish "the point on the curve at this `s`" from "any other point
/// on the curve equidistant under the closest-point projection." We
/// therefore post-check that `||xyz(s) − p_target|| < NEWTON_RESIDUAL_MM`;
/// if the residual is too large we return `None` (caller treats this the
/// same as out-of-bounds — no split, skip the rewrite). A `debug_assert!`
/// surfaces the failure in test builds.
///
/// Returns `Some(s)` (clamped to `[0, 1]`) on a residual-checked convergence;
/// `None` on wrong-root detection.
fn invert_cubic_bezier_xyz_to_param(
    curve: &nurbs::VectorNurbs<f64, 3>,
    p_target: [f64; 3],
    s_seed: f64,
) -> Option<f64> {
    use nurbs::eval::vector_eval;

    // Build the curve's first and second derivatives once; both Newton
    // iterates use them. `differentiate` (degree → degree − 1) for vector
    // NURBS is available on `VectorNurbsView` via the algebra layer; for
    // single-piece cubic Béziers we compute them directly from the
    // control-point polygon for efficiency and clarity.
    let cps = curve.control_points();
    debug_assert_eq!(curve.degree(), 3);
    debug_assert_eq!(cps.len(), 4);

    // First-derivative control polygon: `3·(P_{i+1} − P_i)` for i in 0..3.
    // Result is a quadratic Bézier (degree 2, 3 control points).
    let d1_cps: [[f64; 3]; 3] = [
        [
            3.0 * (cps[1][0] - cps[0][0]),
            3.0 * (cps[1][1] - cps[0][1]),
            3.0 * (cps[1][2] - cps[0][2]),
        ],
        [
            3.0 * (cps[2][0] - cps[1][0]),
            3.0 * (cps[2][1] - cps[1][1]),
            3.0 * (cps[2][2] - cps[1][2]),
        ],
        [
            3.0 * (cps[3][0] - cps[2][0]),
            3.0 * (cps[3][1] - cps[2][1]),
            3.0 * (cps[3][2] - cps[2][2]),
        ],
    ];
    // Second-derivative control polygon: `2·(D1_{i+1} − D1_i)` for i in
    // 0..2. Result is a linear Bézier (degree 1, 2 control points).
    let d2_cps: [[f64; 3]; 2] = [
        [
            2.0 * (d1_cps[1][0] - d1_cps[0][0]),
            2.0 * (d1_cps[1][1] - d1_cps[0][1]),
            2.0 * (d1_cps[1][2] - d1_cps[0][2]),
        ],
        [
            2.0 * (d1_cps[2][0] - d1_cps[1][0]),
            2.0 * (d1_cps[2][1] - d1_cps[1][1]),
            2.0 * (d1_cps[2][2] - d1_cps[1][2]),
        ],
    ];

    // Bernstein-basis evaluators for the derivative polygons. We hand-roll
    // them rather than building `VectorNurbs` curves to keep this on the
    // straight-line cubic-Bézier hot path.
    let eval_d1 = |s: f64| -> [f64; 3] {
        let one_minus = 1.0 - s;
        let b0 = one_minus * one_minus;
        let b1 = 2.0 * one_minus * s;
        let b2 = s * s;
        [
            b0 * d1_cps[0][0] + b1 * d1_cps[1][0] + b2 * d1_cps[2][0],
            b0 * d1_cps[0][1] + b1 * d1_cps[1][1] + b2 * d1_cps[2][1],
            b0 * d1_cps[0][2] + b1 * d1_cps[1][2] + b2 * d1_cps[2][2],
        ]
    };
    let eval_d2 = |s: f64| -> [f64; 3] {
        let one_minus = 1.0 - s;
        [
            one_minus * d2_cps[0][0] + s * d2_cps[1][0],
            one_minus * d2_cps[0][1] + s * d2_cps[1][1],
            one_minus * d2_cps[0][2] + s * d2_cps[1][2],
        ]
    };

    let mut s = s_seed.clamp(0.0, 1.0);
    // Closed-form short-circuit for pure-X collinear cubic Béziers. Two
    // tolerances govern the test:
    //
    //   * [`PURE_AXIS_TOLERANCE`] — Y and Z control points all sit within
    //     ±1 pm of zero. Tight to make the "pure-X" classification
    //     unambiguous regardless of refit noise (refits operate at 5 µm
    //     L∞ on position, four orders of magnitude looser).
    //
    //   * [`COLLINEAR_TOLERANCE`] — middle X control points sit within
    //     ±1 nm of the (1/3, 2/3) lerp of the endpoint X coordinates.
    //     Loose enough to admit any refit-quality collinear cubic but
    //     tight enough to rule out a near-collinear cubic where the
    //     analytic `s = (p − x0) / (x3 − x0)` shortcut would mis-locate
    //     the target. (Near-collinear cubics fall through to Newton,
    //     which solves them just as exactly under the residual check.)
    //
    // The two constants used to be `1e-12` and `1e-9` respectively; they
    // were promoted to named constants so the rationale is documented.
    // The collinear bound is `1e-9` rather than `1e-12` because refit
    // noise on the *control polygon* is dominated by the refit's
    // sample-grid resolution, not by f64 round-off, and `1e-9` admits
    // every collinear-cubic the planner produces.
    let pure_x = cps.iter().all(|p| {
        p[1].abs() < PURE_AXIS_TOLERANCE && p[2].abs() < PURE_AXIS_TOLERANCE
    });
    if pure_x {
        let x0 = cps[0][0];
        let x3 = cps[3][0];
        let span = x3 - x0;
        let collinear_third = (cps[1][0] - (x0 + span / 3.0)).abs() < COLLINEAR_TOLERANCE
            && (cps[2][0] - (x0 + 2.0 * span / 3.0)).abs() < COLLINEAR_TOLERANCE;
        if collinear_third && span.abs() > PURE_AXIS_TOLERANCE {
            // Analytic solve, then residual-check. Even for a collinear
            // cubic the residual is meaningful: a Y/Z component on the
            // target point (e.g., a refit added a sub-pm wobble) would
            // produce a non-zero `dy / dz` that the analytic formula
            // ignores. The residual catches that.
            let s_closed = ((p_target[0] - x0) / span).clamp(0.0, 1.0);
            let xyz_s = vector_eval(curve, s_closed);
            let residual = ((xyz_s[0] - p_target[0]).powi(2)
                + (xyz_s[1] - p_target[1]).powi(2)
                + (xyz_s[2] - p_target[2]).powi(2))
            .sqrt();
            debug_assert!(
                residual < NEWTON_RESIDUAL_MM,
                "invert_cubic_bezier_xyz_to_param: pure-X collinear short-circuit \
                 produced residual {residual} mm > budget {NEWTON_RESIDUAL_MM} mm — \
                 the target point is not on the curve to within the planner's \
                 refit budget. Skipping split.",
            );
            if residual >= NEWTON_RESIDUAL_MM {
                return None;
            }
            return Some(s_closed);
        }
    }

    // Newton iteration on the closest-point function. `p_target` is known
    // to lie on `xyz(s)` to within refit noise; Newton converges in ≤ 6
    // iterations on the test corpus, and the contractive radius is large
    // for the well-behaved single-piece cubic Béziers the planner emits.
    for _ in 0..NEWTON_MAX_ITERS {
        let xyz_s = vector_eval(curve, s);
        let d1 = eval_d1(s);
        let d2 = eval_d2(s);

        let dx = [
            xyz_s[0] - p_target[0],
            xyz_s[1] - p_target[1],
            xyz_s[2] - p_target[2],
        ];
        // f(s)   = (xyz − p_target) · xyz'
        // f'(s)  = xyz' · xyz' + (xyz − p_target) · xyz''
        let f = dx[0] * d1[0] + dx[1] * d1[1] + dx[2] * d1[2];
        let f_prime =
            d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2] + dx[0] * d2[0] + dx[1] * d2[1]
                + dx[2] * d2[2];
        if !f.is_finite() || !f_prime.is_finite() || f_prime.abs() < NEWTON_DENOMINATOR_FLOOR {
            break;
        }
        let s_next = (s - f / f_prime).clamp(0.0, 1.0);
        // Quadratic convergence — terminate once the step is below curve
        // precision (1 ULP of normalized parameter ≈ sub-nanometer
        // position on a 200 mm move).
        if (s_next - s).abs() < NEWTON_PARAM_TOLERANCE {
            s = s_next;
            break;
        }
        s = s_next;
    }
    // Final sanity clamp.
    let s = s.clamp(0.0, 1.0);

    // Residual check — the wrong-root guard documented above. We evaluate
    // the curve at the converged `s` and reject if the distance from
    // `p_target` exceeds `NEWTON_RESIDUAL_MM` (10× the C¹ Hermite refit's
    // L∞ tolerance, since the target read off the prior `planned_fitted`
    // cache is itself a refit-quality value). The `debug_assert!` makes
    // wrong-root cases loud in test builds without panicking release builds
    // — the caller falls back to "no split" which is the same defensive
    // behaviour as out-of-bounds.
    let xyz_final = vector_eval(curve, s);
    let residual = ((xyz_final[0] - p_target[0]).powi(2)
        + (xyz_final[1] - p_target[1]).powi(2)
        + (xyz_final[2] - p_target[2]).powi(2))
    .sqrt();
    debug_assert!(
        residual < NEWTON_RESIDUAL_MM,
        "invert_cubic_bezier_xyz_to_param: Newton converged to s = {s} but the \
         residual ||xyz(s) − p_target|| = {residual} mm exceeds the wrong-root \
         budget {NEWTON_RESIDUAL_MM} mm. This indicates a wrong-root convergence \
         (self-intersecting / highly-curved cubic, or stationary-point trap). \
         Falling back to no-split; the caller will skip the rewrite.",
    );
    if residual >= NEWTON_RESIDUAL_MM {
        return None;
    }
    Some(s)
}

/// Shift a `ScalarNurbs<f64>`'s time domain by `dt` seconds. Extracts the
/// piecewise-Bézier representation, shifts every piece's `u_start` / `u_end`,
/// and reassembles. Equivalent in effect to adding `dt` to every knot of the
/// underlying curve — but going through `extract_bezier_pieces` + shift +
/// `bezier_pieces_to_nurbs` avoids duplicating the curve's internal knot
/// machinery here.
/// Build per-segment `Limits` from base `Limits` + curve geometry.
///
/// The SOCP relaxation in `temporal::topp::constraints` uses
/// `j_path = min(j_max[X], j_max[Y], j_max[Z])` as a single scalar bound on
/// `d³s/dt³`. For a curve where some axes have no displacement (e.g. pure-X
/// G1 → collinear cubic Bezier), the inactive axes' j_max would otherwise
/// pull this min down to nonsense values (a Voron with `max_z_accel=100`
/// has `j_max[Z]=200` vs `j_max[X]=140000` — pure-X jogs end up running at
/// effective ~700× slower than the X-axis is actually capable of).
///
/// Patch: bump inactive axes' `j_max[axis]` to the maximum across active
/// axes. Inactive means the curve's control points have a position span
/// below `AXIS_INACTIVE_SPAN_EPS_MM` on that axis. Since `c'_ax(s) ≡ 0` on
/// inactive axes, cartesian jerk on them is identically zero regardless of
/// `d³s/dt³`, so raising the SOCP's per-axis input limit is correctness-
/// preserving for the SOCP's `min()` reduction and the verifier's
/// per-axis Cartesian-jerk check.
///
/// Step 9 (proper per-axis Cartesian jerk SOCP relaxation, deferred per
/// `temporal/src/topp/constraints.rs` comment) will replace this helper.
fn per_segment_limits(
    curve: &nurbs::VectorNurbs<f64, 3>,
    base: temporal::Limits,
    feedrate_mm_s: f64,
) -> temporal::Limits {
    const AXIS_INACTIVE_SPAN_EPS_MM: f64 = 1e-6;

    let cps = curve.control_points();

    // Per-axis control-point span (active iff > eps).
    let mut span = [0.0_f64; 3];
    for ax in 0..3 {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for cp in cps {
            let v = cp[ax];
            if v < lo {
                lo = v;
            }
            if v > hi {
                hi = v;
            }
        }
        span[ax] = (hi - lo).max(0.0);
    }
    let chord_len = (span[0] * span[0] + span[1] * span[1] + span[2] * span[2]).sqrt();

    // --- j_max: bump inactive axes' jerk so SOCP's `min(j_max[X..Z])` doesn't
    //     collapse to a tiny per-Z value on a pure-XY move. Inactive axes have
    //     c'_ax(s) ≡ 0, so cartesian jerk on them is identically zero
    //     regardless of d³s/dt³.
    let max_active_j = (0..3)
        .filter_map(|ax| {
            if span[ax] > AXIS_INACTIVE_SPAN_EPS_MM {
                Some(base.j_max[ax])
            } else {
                None
            }
        })
        .fold(0.0_f64, f64::max);
    let mut j_max = base.j_max;
    if max_active_j > 0.0 {
        for ax in 0..3 {
            if span[ax] <= AXIS_INACTIVE_SPAN_EPS_MM {
                j_max[ax] = max_active_j;
            }
        }
    }

    // --- v_max: cap active axes by feedrate × direction_fraction so the
    //     planner respects commanded F. Inactive axes are LEFT AT BASE
    //     (setting them to 0 introduced numerical issues in the SOCP that
    //     broke the seam-residue test on a prior attempt — `c'_ax(s) ≡ 0`
    //     makes the constraint `|v_ax(t)| ≤ v_max[ax]` trivially-satisfied
    //     at any positive v_max[ax], so leaving inactive axes' v_max
    //     unchanged is correctness-preserving).
    let mut v_max = base.v_max;
    if feedrate_mm_s > 0.0 && chord_len > AXIS_INACTIVE_SPAN_EPS_MM {
        for ax in 0..3 {
            if span[ax] > AXIS_INACTIVE_SPAN_EPS_MM {
                let direction_fraction = span[ax] / chord_len;
                let feed_cap = feedrate_mm_s * direction_fraction;
                v_max[ax] = v_max[ax].min(feed_cap);
            }
        }
    }

    temporal::Limits::new(v_max, base.a_max, j_max, base.a_centripetal_max)
}

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

/// Phase 5 Task 5.1 — clear an existing axis queue and re-seed it with the
/// `(home_pos, v=0)` rest extension that `build_axis_queue` produces at
/// construction. `kernel` and `h` are preserved (reset is a position
/// re-anchor, not a shaper config swap — see [`ShaperState::reset`] for
/// the rationale). For passthrough axes (`h = 0`) no seed piece is added,
/// matching `build_axis_queue`'s behaviour.
fn reseed_axis_queue(axis: &mut AxisShaperQueue, home_pos: f64) {
    axis.pieces.clear();
    if axis.h > 0.0 {
        let delta_safety = axis.h;
        let total = axis.h + delta_safety;
        axis.pieces.push_back(BezierPiece {
            u_start: -total,
            u_end: 0.0,
            coeffs: vec![home_pos],
        });
    }
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
