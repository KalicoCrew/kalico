//! TIM5 ISR body — the unified motion evaluator (dispatch stage).
//!
//! Holds `dispatch_axis` and its two backends (`dispatch_pulse` /
//! `dispatch_phase`). The full per-tick driver (TIM5 ISR entry,
//! cycle-counter widening, XY-derived quantities, extruder-with-PA, …)
//! lands in Task 8; this module is intentionally narrow so Task 7's diff
//! is reviewable in isolation.
//!
//! Spec:
//! `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md`,
//! "TIM5 ISR — the unified evaluator" section.
//!
//! ### Plan deviations
//!
//! - `dispatch_axis` takes `shared: &SharedState` and
//!   `queue_ptr: *mut StepQueue` as explicit parameters rather than
//!   reaching into a global. There is no global `SHARED` static in this
//!   codebase — `SharedState` lives on `RuntimeContext` — and Task 5
//!   intentionally left `step_queues` C-owned (the per-axis queue
//!   storage lives in C, see `step_queue::step_queues`). Passing both in
//!   keeps `dispatch_axis` host-testable and matches how the engine /
//!   producer loop already thread `&SharedState` through to fault sites
//!   (see `engine::Engine::tick`).
//!
//! - Fault publication routes through `crate::fault_helpers::raise_*`
//!   instead of the plan's pseudo-symbols `shared::set_fault_*`. The
//!   helpers store detail+code in the canonical order documented on
//!   `fault_helpers`.
//!
//! - Phase branch only updates `last_coil_A` / `last_coil_B` /
//!   `last_phase_target` / `position_count`; the SPI write itself is
//!   deferred to Task 14 (when the SPI/DMA pipe lands). The bookkeeping
//!   side of phase dispatch needs to be in place by Task 7 so Task 8 can
//!   wire the full ISR.

// `step_queue::push` is `unsafe fn`; the caller is responsible for the
// SPSC discipline (one producer per queue, queue lifetime outlives the
// push). Workspace lints deny `unsafe_code` globally; the discipline
// rationale is documented at each call site below.
#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::{
    raise_math_non_finite, raise_piece_advance_underflow, raise_position_count_overflow,
    raise_step_queue_overflow,
};
use crate::phase_lut::PHASE_LUT;
use crate::state::SharedState;
use crate::step_queue::{push as queue_push, StepEntry, StepQueue};
use crate::stepping_state::{AxisConfig, StepMode};
use crate::sub_sample_timing::{
    compute_step_times, StepTimeInputs, StepTimingResult,
};

/// `|P_end - P_start|` below this triggers the uniform-spacing fallback
/// in `dispatch_pulse`. Default ≈ one tenth of a micron, well below the
/// physical microstep on every kinematic we ship.
pub const DISPLACEMENT_THRESHOLD_MM: f32 = 1e-4;

/// Dispatch one TIM5 sample for a single axis.
///
/// Reads `axis.mode` and routes to the appropriate backend. The caller
/// (Task 8's TIM5 ISR) is responsible for evaluating the cubic Bezier to
/// produce `p_end` / `v_end` and supplying the cached `p_sample_start`
/// from the previous tick; this function does not touch the curve.
///
/// `queue_ptr` is the per-axis [`StepQueue`] this axis pushes into.
/// Caller resolves it: on the MCU from the C-declared
/// `step_queues[axis_idx]`, on host from a test-owned buffer. The pointer
/// must outlive the call and the caller must be the sole producer
/// (TIM5 ISR is single-instance per axis by design).
///
/// `shared` is the cross-half [`SharedState`] used for fault publication
/// and per-axis telemetry counters.
///
/// `v_end` is currently unused (Task 7 leaves the secant-slope path
/// deriving its slope from the position pair); it is part of the
/// signature so Task 8 can wire it without re-cutting the call sites.
#[allow(clippy::too_many_arguments)] // Spec-pinned signature; structs add noise here.
pub fn dispatch_axis(
    axis_idx: usize,
    axis: &mut AxisConfig,
    queue_ptr: *mut StepQueue,
    shared: &SharedState,
    p_end: f32,
    v_end: f32,
    p_sample_start: f32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
) {
    // `v_end` is reserved for Task 8 (XY-derived velocity); silence the
    // unused-binding lint without changing the public signature.
    let _ = v_end;

    match axis.mode.load(Ordering::Acquire) {
        m if m == StepMode::Pulse as u8 => dispatch_pulse(
            axis_idx,
            axis,
            queue_ptr,
            shared,
            p_end,
            p_sample_start,
            sample_period_sec,
            sample_start_cycles,
            cycles_per_second,
        ),
        m if m == StepMode::Phase as u8 => dispatch_phase(axis_idx, axis, shared, p_end),
        // Invalid mode byte — the `mode` field is only ever written by the
        // foreground via `set_axis_mode` (Task 4), which enforces the enum
        // mapping. Treat as a no-op for this sample; if the byte is
        // genuinely corrupt the foreground-side `MathNonFinite` /
        // configuration faults will surface it.
        _ => {}
    }
}

/// Pulse-mode dispatch: schedule step pulses across this sample window.
///
/// Pipeline per spec §"TIM5 ISR — the unified evaluator":
/// 1. Quantize `p_end` to integer microsteps → `target_step_count`.
/// 2. Compute step count via `target - last` (signed).
/// 3. Hand off to `compute_step_times` (secant-slope or uniform fallback).
/// 4. Push each absolute cycle into the per-axis step queue.
/// 5. Bump each yoked stepper's `position_count` by the axis delta.
///
/// On queue-push failure the function publishes
/// [`FaultCode::StepQueueOverflow`](crate::error::FaultCode::StepQueueOverflow)
/// and returns; subsequent steps in the same sample are dropped. The
/// telemetry counter `queue_overflow_count[axis_idx]` is bumped by the
/// fault helper.
#[allow(clippy::too_many_arguments)]
fn dispatch_pulse(
    axis_idx: usize,
    axis: &mut AxisConfig,
    queue_ptr: *mut StepQueue,
    shared: &SharedState,
    p_end: f32,
    p_sample_start: f32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
) {
    // Guard against a zero / non-finite microstep distance — that would
    // make the quantization step divide by zero and the engine should
    // never have been armed in that state. Bail silently for Task 7;
    // Task 4's configure_axes is the proper gatekeeper.
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }

    let prev_step_count = axis.last_step_count;
    // round-to-nearest-int is the spec-canonical quantization. f32 cast
    // is bounded by `last_step_count`'s i32 range; any value outside
    // [-2^31, 2^31) would also fail the `checked_add` downstream and
    // raise PositionCountOverflow, which is the correct response.
    #[allow(clippy::cast_possible_truncation)]
    let target_step_count = (p_end / microstep_distance).round() as i32;
    let signed_steps = target_step_count.wrapping_sub(prev_step_count);
    // Update the axis cache regardless of whether we found any steps to
    // schedule — Phase-mode keeps it in lockstep too.
    axis.last_step_count = target_step_count;

    if signed_steps == 0 {
        return;
    }

    let inputs = StepTimeInputs {
        p_start: p_sample_start,
        p_end,
        prev_step_count,
        target_step_count,
        microstep_distance,
        sample_period_sec,
        sample_start_cycles,
        cycles_per_second,
        displacement_threshold: DISPLACEMENT_THRESHOLD_MM,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(t) | StepTimingResult::Uniform(t) => t,
        // signed_steps != 0 is already verified above, so NoSteps cannot
        // occur here — defensive return.
        StepTimingResult::NoSteps => return,
    };

    let dir: i8 = if signed_steps > 0 { 1 } else { -1 };
    // Count successful pushes so a partial-overflow scenario commits
    // `position_count` for steps that DID land in the queue (i.e., that
    // the C-side per-axis timer will fire as physical motion). Otherwise
    // the host's view of stepper position desyncs from physical reality.
    let mut steps_committed: i32 = 0;
    for cycle_abs in times.iter().copied() {
        let entry = StepEntry { cycle_abs, dir, _pad: [0; 3] };
        // SAFETY: `queue_ptr` is supplied by the caller (TIM5 ISR), who
        // owns the sole-producer role for this axis's step queue. The
        // queue's storage outlives this call (C-owned `.axi_bss` on the
        // MCU, stack/heap test buffer on host).
        let push_res = unsafe { queue_push(queue_ptr, entry) };
        if push_res.is_err() {
            // Commit the steps we already pushed before raising the
            // fault — the queue's contents are about to drive real GPIO
            // toggles regardless of fault state, so the position counter
            // must reflect that. `dir` is ±1, so the signed delta is just
            // ±steps_committed.
            let committed_delta = steps_committed * (i32::from(dir));
            commit_position_count(axis, axis_idx, shared, committed_delta);
            raise_step_queue_overflow(shared, axis_idx);
            // Rewrite `last_step_count` to match the partial commit, so
            // the next sample's `prev_step_count` matches the queue's
            // actual contribution rather than the full requested target.
            axis.last_step_count = prev_step_count + committed_delta;
            return;
        }
        steps_committed += 1;
    }

    // Full push success — commit the full requested delta.
    commit_position_count(axis, axis_idx, shared, signed_steps);
}

/// Bump `position_count` on every yoked stepper of `axis` by `delta`.
///
/// ISR is the sole writer, so `load + checked_add + store` (no CAS) is
/// the right shape; we lose no concurrency vs. `fetch_add` but gain
/// overflow detection. On overflow a `PositionCountOverflow` fault is
/// latched and the remaining steppers in the yoke are not updated.
fn commit_position_count(
    axis: &AxisConfig,
    axis_idx: usize,
    shared: &SharedState,
    delta: i32,
) {
    if delta == 0 {
        return;
    }
    for stepper in &axis.steppers {
        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(delta) else {
            raise_position_count_overflow(shared, axis_idx);
            return;
        };
        stepper.position_count.store(next, Ordering::Release);
    }
}

/// Phase-mode dispatch: update per-stepper coil-current state without
/// driving GPIO step pulses.
///
/// Bookkeeping only in Task 7 — the actual SPI write to TMC5160 XDIRECT
/// is deferred to Task 14. The fields updated here are exactly what the
/// SPI dispatcher will need to read on the next sample: the LUT lookup
/// result and the per-stepper target so delta computation stays
/// continuous across SPI cycles.
fn dispatch_phase(
    axis_idx: usize,
    axis: &mut AxisConfig,
    shared: &SharedState,
    p_end: f32,
) {
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }

    // Quantize the axis position to integer microsteps. This is the base
    // against which per-stepper phase offsets are added.
    #[allow(clippy::cast_possible_truncation)]
    let target_microsteps_axis = (p_end / microstep_distance).round() as i32;
    axis.last_step_count = target_microsteps_axis;

    for stepper in &axis.steppers {
        let phase_offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
        let target_stepper = target_microsteps_axis.wrapping_add(phase_offset);
        let prev_stepper = stepper.last_phase_target.load(Ordering::Acquire);
        // Wrap on the subtraction implies a configuration discontinuity
        // (a phase-offset jump or a re-arm): under normal motion
        // `last_phase_target` advances at most a few microsteps per
        // sample, so the wrapped difference equals the true difference.
        let delta_stepper = target_stepper.wrapping_sub(prev_stepper);
        stepper
            .last_phase_target
            .store(target_stepper, Ordering::Release);

        // Mask to the 10-bit electrical-cycle width. The signed→unsigned
        // bitcast wraps negative values modulo 2^32; `& 0x3FF` then
        // selects the low 10 bits, which equals the mathematical
        // remainder mod 1024 because 2^32 mod 1024 == 0. This is the
        // same identity the spec relies on.
        #[allow(clippy::cast_sign_loss)]
        let phase = (target_stepper as u32) & 0x3FF;
        // `phase` is bounded `0..1024 == PHASE_LUT.len()` by the mask
        // above, so the lookup cannot panic; the `get` keeps us out of
        // `clippy::indexing_slicing` (which is denied at the crate root).
        // safe: bounded by mask 0x3FF (= PHASE_LUT_SIZE - 1)
        let Some((coil_a, coil_b)) = PHASE_LUT.get(phase as usize).copied() else {
            continue;
        };

        stepper.last_coil_A.store(coil_a, Ordering::Release);
        stepper.last_coil_B.store(coil_b, Ordering::Release);

        // Bump `position_count` by the per-stepper delta (includes any
        // mid-sample offset change). Use a checked add so a runaway
        // offset latches a fault rather than silently wrapping.
        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(delta_stepper) else {
            raise_position_count_overflow(shared, axis_idx);
            return;
        };
        stepper.position_count.store(next, Ordering::Release);
    }
}

// =====================================================================
// Task 8 — full per-sample evaluator (phases 1-5 of the TIM5 ISR body).
// =====================================================================
//
// `runtime_tick_sample` is the single per-sample entry point that the TIM5
// ISR top-half (Task 6) and the host integration tests call. It runs the
// five canonical phases laid out in the spec:
//
//   1. Evaluate the cubic Bezier for motion axes A, B, Z; dispatch each.
//   2. Compute XY-derived quantities (motor-frame speed → cartesian XY
//      speed via `k_xy`; accumulate segment arc length).
//   3. Evaluate the extruder axis with the E-follows-XY + PA correction.
//   4. Publish (`p_end`, `v_end`) into `TickCaches` for the next tick.
//   5. (Deferred to Task 9) Segment retirement.
//
// ### Plan deviations
//
// - `TickContext` carries `queues: [*mut StepQueue; N_AXES]` instead of a
//   global queue array. This matches the explicit-pointer threading
//   landed in Task 7: on the MCU the caller resolves each entry from the
//   C-side `step_queues[axis_idx]`; on host the test owns the storage.
//   The pointer-array shape avoids re-borrowing `ctx.queues[axis_idx]`
//   while another borrow on `ctx.axes` is live.
// - `axes` is `&mut [AxisConfig; N_AXES]`; we index inside each phase
//   (`let axis = &mut ctx.axes[idx]`) rather than splitting the array
//   into per-axis borrows up front, because Phase 1 and Phase 3 access
//   non-overlapping indices in series.
// - Fault publication uses `raise_math_non_finite` from `fault_helpers`
//   (Task 7), not the plan's pseudo-symbol `shared::set_fault_...`.
// - `libm::sqrtf` provides arc-length computation; `f32::sqrt` is not in
//   `core`, so this keeps the body `no_std`-clean for the MCU build.

use crate::stepping_state::TickCaches;

pub use crate::stepping_state::N_AXES;
pub const AXIS_A: usize = 0;
pub const AXIS_B: usize = 1;
pub const AXIS_Z: usize = 2;
pub const AXIS_E: usize = 3;

/// Per-sample inputs to [`runtime_tick_sample`].
///
/// The caller (TIM5 ISR top-half on MCU, integration test on host) owns
/// all referenced storage for the duration of the call. `queues` is an
/// array of raw pointers because each per-axis [`StepQueue`] lives in a
/// disjoint backing store (C-owned `.axi_bss` on MCU, stack/heap on
/// host); a `&mut [&mut StepQueue; N_AXES]` form would force the caller
/// to materialize four simultaneous mutable references, which the C-side
/// layout cannot express.
///
/// Field semantics:
/// - `axes`: per-axis configuration + scratch state (Task 5 shape).
/// - `queues`: per-axis step queue pointer; producer-side, sole writer
///   is this ISR.
/// - `shared`: cross-half fault publication + telemetry counters.
/// - `caches`: tick-private scratch (Task 5's [`TickCaches`]).
/// - `sample_period_sec` / `sample_period_cycles` / `cycles_per_second`:
///   sample-window scalars used by dispatch + XY arc-length.
/// - `k_xy`: motor→cartesian XY speed scale. 1.0 for cartesian; 1/√2
///   for `CoreXY`.
/// - `advance_accel` / `advance_decel`: PA coefficients (s); the active
///   one is selected per-tick from `vdot_xy_accelerating`.
/// - `now_cycles`: sample-start absolute cycle counter (already widened
///   by Task 6); passed through to `dispatch_axis`.
/// - `t_sample_end_global`: wall-clock time at the end of this sample,
///   in seconds, in the same epoch as `piece_start_time_cycles`.
#[derive(Debug)]
pub struct TickContext<'a> {
    pub axes: &'a mut [AxisConfig; N_AXES],
    pub queues: [*mut StepQueue; N_AXES],
    pub shared: &'a SharedState,
    pub caches: &'a mut TickCaches,
    pub sample_period_sec: f32,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub k_xy: f32,
    pub advance_accel: f32,
    pub advance_decel: f32,
    pub now_cycles: u32,
    pub t_sample_end_global: f32,
}

/// Advance the axis's active piece if the sample time has moved past
/// the current piece's duration.
///
/// Returns `true` if at least one piece advance happened on this axis,
/// so the caller can use that as a hint for segment-retirement timing.
///
/// In the stepping-redesign, each `AxisConfig` carries a single active
/// piece; once exhausted, we clear `axis.piece` to `None` so the
/// foreground (Task 11) can supply the next piece on its next configure
/// call. The loop is bounded (`iters > 4` latches a fault) — that
/// upper bound also catches a non-finite or zero `piece.duration` that
/// would otherwise spin forever (`duration_cycles == 0` means
/// `piece_start_time_cycles` doesn't advance and `t_local` stays past
/// duration).
///
/// Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
/// "Piece advancement" section.
//
// `usize as u32` and `f32 as u64` casts are deliberate quantizations
// matching the spec; the lints would force a workaround that doesn't
// improve correctness on this hot path.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss, clippy::cast_sign_loss)]
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    axis_idx: usize,
    shared: &SharedState,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u8 = 0;
    loop {
        let Some(piece) = axis.piece else {
            break;
        };
        let piece_start_sec =
            (axis.piece_start_time_cycles as f32) / cycles_per_second;
        let t_local = t_sample_end_global - piece_start_sec;
        if t_local <= piece.duration {
            break;
        }
        // Advance: bump piece_start_time by piece.duration in cycles.
        // `duration` is non-negative seconds; converting to cycles via
        // `* cycles_per_second` and then `as u64` is a controlled
        // narrowing (the result fits in u64 for any realistic piece).
        let duration_cycles = (piece.duration * cycles_per_second) as u64;
        axis.piece_start_time_cycles =
            axis.piece_start_time_cycles.wrapping_add(duration_cycles);

        // Single active piece per axis; host pushes the next piece via
        // a Task-11 command handler. Mark axis idle so the foreground
        // can refill on its next configure call.
        axis.piece = None;
        advanced = true;

        iters = iters.saturating_add(1);
        if iters > 4 {
            raise_piece_advance_underflow(shared, axis_idx);
            break;
        }
    }
    advanced
}

/// Run one TIM5 sample across all four axes.
///
/// Caller responsibility:
/// - `TickContext::queues` entries are valid producer pointers for the
///   single-producer SPSC discipline (ISR is sole writer per axis).
/// - `TickContext::axes` is consistent across the call (the foreground
///   only mutates `mode` atomically and `piece` under the producer's
///   exclusive-access contract).
///
/// Behavior on a non-finite cubic evaluation: the offending axis's
/// dispatch is skipped and a [`MathNonFinite`](crate::error::FaultCode::MathNonFinite)
/// fault is latched via [`raise_math_non_finite`]. Other axes proceed.
///
/// Phase 5 (segment retirement) is deferred to Task 9.
//
// Every `[...]` index in this body is statically bounded by `N_AXES`:
// the iteration set `[AXIS_A, AXIS_B, AXIS_Z]` and the constant `AXIS_E`
// are all `< N_AXES`, and the bookkeeping loop's `0..N_AXES` bound is
// exactly the array length. The blanket `allow(clippy::indexing_slicing)`
// is justified — every individual index would otherwise need its own
// inline annotation, which would obscure the per-phase structure that
// the spec calls out.
#[allow(clippy::indexing_slicing)]
pub fn runtime_tick_sample(ctx: &mut TickContext) {
    let mut p_end_axis = [0.0_f32; N_AXES];
    let mut v_end_axis = [0.0_f32; N_AXES];

    // -----------------------------------------------------------------
    // Phase 1: evaluate motion axes A, B, Z and dispatch each.
    // -----------------------------------------------------------------
    for axis_idx in [AXIS_A, AXIS_B, AXIS_Z] {
        let axis = &mut ctx.axes[axis_idx];
        let p_sample_start = ctx.caches.p_prev[axis_idx];

        // Phase-5 prologue: advance / retire the active piece in lockstep
        // with sample time before any evaluation. After this call,
        // `axis.piece` is either still in-flight (`t_local <= duration`)
        // or `None` (exhausted; Phase 5 below will see all-axes-idle and
        // bump the retirement counter).
        advance_piece_if_needed(
            axis,
            axis_idx,
            ctx.shared,
            ctx.t_sample_end_global,
            ctx.cycles_per_second,
        );

        let Some(piece) = axis.piece else {
            // No active piece on this axis: hold position, zero velocity.
            // p_prev stays as last published (kept implicit by writing
            // p_sample_start back into p_end_axis below).
            p_end_axis[axis_idx] = p_sample_start;
            v_end_axis[axis_idx] = 0.0;
            continue;
        };

        // Local piece time = wall-clock now minus when this piece started.
        // Both quantities are in seconds; subtraction in f32 is fine at
        // sample granularity because Task 6 hands us already-aligned
        // values (and the piece duration is in milliseconds for a
        // typical 1mm-at-100mm/s move).
        let piece_start_sec =
            (axis.piece_start_time_cycles as f32) / ctx.cycles_per_second;
        let t_local = ctx.t_sample_end_global - piece_start_sec;

        let (p_end, v_end) =
            crate::monomial::eval_position_velocity(&piece, t_local);
        if !p_end.is_finite() || !v_end.is_finite() {
            raise_math_non_finite(ctx.shared, axis_idx);
            // Hold previous position to keep downstream caches sane.
            p_end_axis[axis_idx] = p_sample_start;
            v_end_axis[axis_idx] = 0.0;
            continue;
        }

        p_end_axis[axis_idx] = p_end;
        v_end_axis[axis_idx] = v_end;

        dispatch_axis(
            axis_idx,
            axis,
            ctx.queues[axis_idx],
            ctx.shared,
            p_end,
            v_end,
            p_sample_start,
            ctx.sample_period_sec,
            ctx.now_cycles,
            ctx.cycles_per_second,
        );
    }

    // -----------------------------------------------------------------
    // Phase 2: XY-derived quantities.
    //
    // Motor-frame speed → cartesian XY speed via `k_xy`. Arc length
    // accumulates per segment so the extruder follower (Phase 3) can
    // integrate it directly; pressure-advance polarity is derived from
    // the sign of dv_xy/dt.
    // -----------------------------------------------------------------
    let xy_active =
        ctx.axes[AXIS_A].piece.is_some() || ctx.axes[AXIS_B].piece.is_some();
    if xy_active {
        let va = v_end_axis[AXIS_A];
        let vb = v_end_axis[AXIS_B];
        let v_motor_sq = va * va + vb * vb;
        let v_xy_this = libm::sqrtf(v_motor_sq) * ctx.k_xy;
        ctx.caches.vdot_xy_accelerating = v_xy_this >= ctx.caches.v_xy_prev;
        ctx.caches.ds_xy_segment += v_xy_this * ctx.sample_period_sec;
        ctx.caches.v_xy_prev = v_xy_this;
        ctx.caches.v_xy_this = v_xy_this;
    } else {
        ctx.caches.v_xy_this = 0.0;
        ctx.caches.vdot_xy_accelerating = false;
        // Note: ds_xy_segment is *not* reset here. The segment-retirement
        // phase (Task 9) is what zeroes it; an XY-idle tick mid-segment
        // (e.g., a Z-only hop between extrusions) must not clobber the
        // accumulated arc length that the next segment's E follower may
        // still consume.
    }

    // -----------------------------------------------------------------
    // Phase 3: evaluate the extruder axis with E-follows-XY + PA.
    //
    // The intrinsic extrusion from the E NURBS piece (retract/prime/
    // filament-change) is summed with:
    //   - extrusion_per_xy_mm × ds_xy_segment   (E follows XY arc length)
    //   - pa_k × extrusion_per_xy_mm × v_xy_this (pressure advance)
    // where `pa_k` is `advance_accel` while v_xy is rising and
    // `advance_decel` while it is falling (asymmetric PA, see bleeding-
    // edge-v2 Step 9 lineage in the CLAUDE.md scope).
    // -----------------------------------------------------------------
    {
        let axis = &mut ctx.axes[AXIS_E];
        let p_sample_start = ctx.caches.p_prev[AXIS_E];

        // Phase-5 prologue for the extruder axis. Same rationale as the
        // motion-axis branch above: advance / retire before evaluation.
        advance_piece_if_needed(
            axis,
            AXIS_E,
            ctx.shared,
            ctx.t_sample_end_global,
            ctx.cycles_per_second,
        );

        if let Some(piece) = axis.piece {
            let piece_start_sec =
                (axis.piece_start_time_cycles as f32) / ctx.cycles_per_second;
            let t_local = ctx.t_sample_end_global - piece_start_sec;
            let (p_end_intrinsic, v_end) =
                crate::monomial::eval_position_velocity(&piece, t_local);

            if !p_end_intrinsic.is_finite() || !v_end.is_finite() {
                raise_math_non_finite(ctx.shared, AXIS_E);
                p_end_axis[AXIS_E] = p_sample_start;
                v_end_axis[AXIS_E] = 0.0;
            } else {
                let pa_k = if ctx.caches.vdot_xy_accelerating {
                    ctx.advance_accel
                } else {
                    ctx.advance_decel
                };
                let p_end = p_end_intrinsic
                    + axis.extrusion_per_xy_mm * ctx.caches.ds_xy_segment
                    + pa_k * axis.extrusion_per_xy_mm * ctx.caches.v_xy_this;

                p_end_axis[AXIS_E] = p_end;
                v_end_axis[AXIS_E] = v_end;

                dispatch_axis(
                    AXIS_E,
                    axis,
                    ctx.queues[AXIS_E],
                    ctx.shared,
                    p_end,
                    v_end,
                    p_sample_start,
                    ctx.sample_period_sec,
                    ctx.now_cycles,
                    ctx.cycles_per_second,
                );
            }
        } else {
            p_end_axis[AXIS_E] = p_sample_start;
            v_end_axis[AXIS_E] = 0.0;
        }
    }

    // -----------------------------------------------------------------
    // Phase 4: publish (p_end, v_end) into per-axis caches for the next
    // tick's secant-slope sub-sample timing.
    // -----------------------------------------------------------------
    ctx.caches.p_prev = p_end_axis;
    ctx.caches.v_prev = v_end_axis;

    // -----------------------------------------------------------------
    // Phase 5: segment retirement check.
    //
    // A segment is "retired" when every axis has advanced past its final
    // piece. The runtime doesn't carry a segment manifest at the engine
    // level (the spec defers that to the producer/host); this hook
    // publishes the retirement counter for the host and resets the
    // segment-local arc-length accumulator so the next segment's E
    // follower starts from zero.
    //
    // Heuristic: if every axis has `piece == None` AND the cached
    // `ds_xy_segment` is non-zero (so this sample saw the transition out
    // of an active segment), publish the retirement event.
    // -----------------------------------------------------------------
    let any_active = ctx.axes.iter().any(|a| a.piece.is_some());
    if !any_active && ctx.caches.ds_xy_segment > 0.0 {
        ctx.shared
            .retired_through_segment_id
            .fetch_add(1, Ordering::Release);
        ctx.caches.ds_xy_segment = 0.0;
    }
}

#[cfg(test)]
mod tests {
    //! Smoke tests that hit only the host-build path (queue allocated
    //! on the stack via `StepQueue::new()`). End-to-end ISR integration
    //! lives in Task 8's test suite.

    use super::{dispatch_axis, DISPLACEMENT_THRESHOLD_MM};
    use crate::monomial::BezierPieceMonomial;
    use crate::state::SharedState;
    use crate::step_queue::StepQueue;
    use crate::stepping_state::{AxisConfig, StepMode, StepperRef};
    use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
    use heapless::Vec;

    fn make_stepper() -> StepperRef {
        StepperRef {
            step_pin: 0,
            dir_pin: 0,
            dir_invert: false,
            position_count: AtomicI32::new(0),
            tmc_cs: None,
            last_coil_A: AtomicI16::new(0),
            last_coil_B: AtomicI16::new(0),
            phase_offset_microsteps: AtomicI32::new(0),
            phase_offset_target: AtomicI32::new(0),
            last_phase_target: AtomicI32::new(0),
        }
    }

    fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
        let mut steppers: Vec<StepperRef, 4> = Vec::new();
        let _ = steppers.push(make_stepper());
        AxisConfig {
            mode: AtomicU8::new(mode as u8),
            steppers,
            piece: None::<BezierPieceMonomial>,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance,
            extrusion_per_xy_mm: 0.0,
        }
    }

    /// Pulse mode with `p_end == p_sample_start` and matching
    /// `last_step_count` schedules zero steps and leaves the queue empty.
    #[test]
    fn pulse_zero_motion_no_steps_scheduled() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Pulse, 0.0125);

        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0, &mut axis, q_ptr, &shared,
            /* p_end */ 0.0,
            /* v_end */ 0.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
        );

        assert_eq!(q.tail, q.head, "no steps should be enqueued");
        assert_eq!(axis.last_step_count, 0);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "no fault should latch"
        );
    }

    /// Pulse mode with a clean +N-step displacement enqueues N entries
    /// and bumps `position_count` by exactly N for every yoked stepper.
    #[test]
    fn pulse_positive_motion_enqueues_n_steps() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Pulse, 0.0125);

        // Drive 4 microsteps forward = 4 × 0.0125 mm = 0.05 mm.
        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0, &mut axis, q_ptr, &shared,
            /* p_end */ 0.05,
            /* v_end */ 2000.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 1_000,
            /* cycles_per_second */ 520_000_000.0,
        );

        let enq = q.tail.wrapping_sub(q.head);
        assert_eq!(enq, 4, "expected 4 step entries, got {enq}");
        assert_eq!(axis.last_step_count, 4);
        assert_eq!(
            axis.steppers[0].position_count.load(Ordering::Acquire),
            4
        );
        assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    }

    /// Pulse mode with `|displacement| < threshold` still schedules
    /// `|n_steps|` entries via the uniform-spacing fallback.
    #[test]
    fn pulse_below_displacement_threshold_uses_uniform_fallback() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Pulse, 0.0125);

        // Two steps but P barely moves (within DISPLACEMENT_THRESHOLD_MM).
        // We force this by setting last_step_count to -2 and p_* near zero:
        // signed_steps = round(0 / 0.0125) - (-2) = 0 - (-2) = 2.
        axis.last_step_count = -2;
        let tiny = DISPLACEMENT_THRESHOLD_MM / 10.0;

        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0, &mut axis, q_ptr, &shared,
            /* p_end */ tiny,
            /* v_end */ 0.0,
            /* p_sample_start */ -tiny,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
        );

        let enq = q.tail.wrapping_sub(q.head);
        assert_eq!(enq, 2);
        assert_eq!(axis.last_step_count, 0);
    }

    /// Phase mode updates `last_coil_*`, `last_phase_target`, and
    /// `position_count` without touching the step queue.
    #[test]
    fn phase_mode_updates_coil_state_no_queue_writes() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Phase, 0.0125);

        // p_end = 256 microsteps → phase = 256 → PHASE_LUT[256] = (0, 248).
        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0, &mut axis, q_ptr, &shared,
            /* p_end */ 256.0 * 0.0125,
            /* v_end */ 0.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
        );

        assert_eq!(q.tail, q.head, "phase mode must not enqueue step pulses");
        assert_eq!(axis.last_step_count, 256);
        assert_eq!(
            axis.steppers[0].last_coil_A.load(Ordering::Acquire),
            0
        );
        assert_eq!(
            axis.steppers[0].last_coil_B.load(Ordering::Acquire),
            248
        );
        assert_eq!(
            axis.steppers[0].last_phase_target.load(Ordering::Acquire),
            256
        );
        assert_eq!(
            axis.steppers[0].position_count.load(Ordering::Acquire),
            256
        );
    }

    /// Phase mode honors `phase_offset_microsteps`: per-stepper target
    /// = axis position + offset, and `position_count` bumps by the
    /// per-stepper delta (which includes the offset).
    #[test]
    fn phase_mode_honors_phase_offset() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Phase, 0.0125);
        axis.steppers[0]
            .phase_offset_microsteps
            .store(7, Ordering::Release);

        // axis target = 256, stepper target = 263, phase = 263.
        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0, &mut axis, q_ptr, &shared,
            /* p_end */ 256.0 * 0.0125,
            /* v_end */ 0.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
        );

        assert_eq!(
            axis.steppers[0].last_phase_target.load(Ordering::Acquire),
            263
        );
        assert_eq!(
            axis.steppers[0].position_count.load(Ordering::Acquire),
            263
        );
    }
}
