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
    let target_step_count = libm::roundf(p_end / microstep_distance) as i32;
    let signed_steps = target_step_count.wrapping_sub(prev_step_count);
    // Update the axis cache regardless of whether we found any steps to
    // schedule — Phase-mode keeps it in lockstep too.
    axis.last_step_count = target_step_count;

    if signed_steps == 0 {
        return;
    }
    // 2026-05-21 diag: capture last non-zero signed_steps so the host
    // can see what the eval is producing. If this stays 0 across an
    // entire jog despite EA/ED bumping, the eval is never producing
    // enough p_end change to cross a microstep threshold.
    shared.isr_last_signed_steps.store(
        signed_steps.unsigned_abs(),
        core::sync::atomic::Ordering::Relaxed,
    );

    // 2026-05-21 FIX: bound check that the comment in compute_step_times
    // PROMISED but nothing enforced. If signed_steps is huge (e.g.,
    // target_step_count saturated to i32::MAX because p_end was a giant
    // finite f32), compute_step_times's `for k in 0..n_steps` would loop
    // billions of times — ~4 seconds of CPU spin, IWDG fires, MCU resets.
    // Bound here, capture the inputs via diag for further debugging.
    let abs_steps = signed_steps.unsigned_abs();
    if abs_steps > crate::sub_sample_timing::MAX_STEPS_PER_SAMPLE as u32 {
        // Diag: stash the inputs that produced the oversized
        // target_step_count. EE=abs_steps, then the f32 inputs:
        //   isr_last_p_end_bits = p_end.to_bits()
        //   isr_last_microstep_bits = microstep_distance.to_bits()
        // Host decodes via f32::from_bits().
        shared.isr_last_t_start_lo.store(abs_steps, core::sync::atomic::Ordering::Relaxed);
        shared.isr_last_p_end_bits.store(p_end.to_bits(), core::sync::atomic::Ordering::Relaxed);
        shared.isr_last_microstep_bits.store(microstep_distance.to_bits(), core::sync::atomic::Ordering::Relaxed);
        bump_relaxed(&shared.isr_overrun_count);
        axis.last_step_count = prev_step_count;
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
    let mut steps_committed: i32 = 0;
    #[allow(clippy::explicit_counter_loop)]
    for cycle_abs in times.iter().copied() {
        let entry = StepEntry { cycle_abs, dir, _pad: [0; 3] };
        // SAFETY: `queue_ptr` is supplied by the caller (TIM5 ISR), who
        // owns the sole-producer role for this axis's step queue. The
        // queue's storage outlives this call (C-owned `.axi_bss` on the
        // MCU, stack/heap test buffer on host).
        let push_res = unsafe { queue_push(queue_ptr, entry) };
        if push_res.is_ok() {
            bump_relaxed(&shared.isr_step_push_count);
        }
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

/// Phase-mode offset ramp: bring `phase_offset_microsteps` toward
/// `phase_offset_target` by at most `max_per_sample` per call.
///
/// Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
/// "Set stepper offset" — host requests a delta via `set_stepper_offset`
/// (Task 12), and the TIM5 ISR walks `phase_offset_microsteps` toward
/// `phase_offset_target` over multiple samples to avoid step rate spikes.
///
/// Early-return semantics:
/// - `max_per_sample == 0` is treated as "no ramp configured" (the boot
///   default). In that mode the target update goes through directly via
///   `set_stepper_offset` writing both fields, or via the next configure.
/// - `current == target` means nothing to do.
///
/// `wrapping_sub` is used on the i32 delta computation: `current` and
/// `target` are both i32 and either of them could be near `i32::MIN` /
/// `i32::MAX` under a pathological host request; wrapping is the safe shape
/// because we only inspect `delta.abs()` and `delta > 0` afterwards, and
/// a wrapped delta still selects the correct ramp direction for any
/// realistic step (max u16 == 65535 microsteps is many full revolutions).
fn ramp_phase_offset(stepper: &crate::stepping_state::StepperRef, max_per_sample: i32) {
    if max_per_sample == 0 {
        return;
    }
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    if current == target {
        return;
    }
    let delta = target.wrapping_sub(current);
    let step = if delta.abs() <= max_per_sample {
        delta
    } else if delta > 0 {
        max_per_sample
    } else {
        -max_per_sample
    };
    stepper
        .phase_offset_microsteps
        .store(current.wrapping_add(step), Ordering::Release);
}

/// Phase-mode dispatch: update per-stepper coil-current state without
/// driving GPIO step pulses.
///
/// Bookkeeping only in Task 7 — the actual SPI write to TMC5160 XDIRECT
/// is deferred to Task 14. The fields updated here are exactly what the
/// SPI dispatcher will need to read on the next sample: the LUT lookup
/// result and the per-stepper target so delta computation stays
/// continuous across SPI cycles.
///
/// Task 13: before reading `phase_offset_microsteps`, each stepper's
/// offset is ramped toward its `phase_offset_target` by at most
/// `shared.max_phase_offset_ramp_per_sample` per sample.
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
    let target_microsteps_axis = libm::roundf(p_end / microstep_distance) as i32;
    axis.last_step_count = target_microsteps_axis;

    // u16 -> i32 widening; cannot truncate or lose sign.
    let max_ramp = i32::from(
        shared
            .max_phase_offset_ramp_per_sample
            .load(Ordering::Acquire),
    );

    for stepper in &axis.steppers {
        ramp_phase_offset(stepper, max_ramp);
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

        // Task 14: push the new coil pair into the per-bus SPI write
        // queue. The foreground struct-timer in `src/runtime_tick.c`
        // pops these and dispatches the actual SPI transfer through
        // Klipper's bus driver (currently a stub; Stage-5 bench bring-up
        // wires real SPI). Stepper without a TMC chip-select (e.g. a
        // phase-stepped Z stepper without TMC5160) skips the push.
        //
        // Bus index is hardcoded to 0 in the MVP. Production needs a
        // per-stepper `bus_idx` field on `StepperRef` (or the upper byte
        // of `tmc_cs_oid` reinterpreted as bus | gpio_handle); deferred until
        // we have a multi-bus board to test against.
        if stepper.tmc_cs_oid.is_some() {
            #[allow(clippy::cast_sign_loss)] // sign-preserving u16->u32 then mask
            let packed = ((u32::from(coil_a as u16)) << 16) | (u32::from(coil_b as u16));
            let bus_idx: usize = 0;
            #[cfg(not(any(test, feature = "host")))]
            // SAFETY: `spi_queues` is a C-owned static of fixed length
            // `N_SPI_BUSES`; `bus_idx < N_SPI_BUSES` is enforced by the
            // hardcoded `0`. The cast yields a pointer to the `bus_idx`-th
            // element of the array — same provenance as the array base.
            let queue_ptr = unsafe {
                crate::spi_queue::spi_queues
                    .get()
                    .cast::<crate::spi_queue::SpiQueue>()
                    .add(bus_idx)
            };
            #[cfg(any(test, feature = "host"))]
            let queue_ptr: *mut crate::spi_queue::SpiQueue = core::ptr::null_mut();
            if !queue_ptr.is_null() {
                let entry = crate::spi_queue::SpiWrite {
                    tmc_cs_oid: stepper
                        .tmc_cs_oid
                        .unwrap_or(crate::stepping_state::TMC_CS_OID_NONE),
                    reg: 0x2D, // TMC5160 XDIRECT
                    _pad: [0; 2],
                    #[allow(clippy::cast_possible_wrap)] // packed is the bit-pattern we want
                    value: packed as i32,
                };
                // SAFETY: `queue_ptr` points to a live `SpiQueue` (C-owned
                // static); the TIM5 ISR is the sole producer for every
                // bus, satisfying the SPSC contract documented on `push`.
                if unsafe { crate::spi_queue::push(queue_ptr, entry) }.is_err() {
                    crate::fault_helpers::raise_spi_queue_overflow(shared, bus_idx);
                }
            }
        }

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
// `CurvePool` deliberately does not implement `Debug` (it carries large
// per-slot arrays and `UnsafeCell`-protected payloads). Skip the derive
// on `TickContext` — the ISR path doesn't format it, and tests can use
// `{:?}` on the individual fields if needed.
#[allow(missing_debug_implementations)]
pub struct TickContext<'a> {
    pub axes: &'a mut [AxisConfig; N_AXES],
    pub queues: [*mut StepQueue; N_AXES],
    pub shared: &'a SharedState,
    pub caches: &'a mut TickCaches,
    /// Curve pool reference for resolving `axis.curve_handle` →
    /// `LoadedCubicCurve` during the per-axis piece-cursor walk in
    /// `advance_piece_if_needed`. Borrowed `&` because the ISR is read-only
    /// against the pool; foreground is the sole writer under the
    /// `try_alloc_and_load` discipline.
    pub curve_pool: &'a crate::curve_pool::CurvePool,
    /// Segment-scoped XY arc-length accumulator (mm). Owned by `Engine`
    /// since Task 7 of the stepping-redesign finish; reset at segment-arm
    /// and read by retire for the final E delta. Threaded into the tick
    /// context via mutable borrow so Phase 2 can integrate and Phase 5
    /// can clear without reaching back through the engine.
    pub ds_xy_segment: &'a mut f32,
    /// Reference to `Engine::current` for the active segment. Phase 3's E-axis
    /// evaluator dispatches on `Segment::e_mode` + `Segment::extrusion_ratio`.
    /// `None` while no segment is armed (e.g. boot, between segments) — in
    /// which case Phase 3 returns `engine_segment_base_e` unchanged (no E
    /// motion).
    pub current_segment: Option<&'a crate::segment::Segment>,
    /// Snapshot of `Engine::e_accumulator` (truncated to f32) taken at
    /// `arm_segment` time. Phase 3 returns absolute E as
    /// `engine_segment_base_e + segment_local_e`. See spec §4.6.
    pub engine_segment_base_e: f32,
    pub sample_period_sec: f32,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub k_xy: f32,
    pub advance_accel: f32,
    pub advance_decel: f32,
    pub now_cycles: u32,
    pub t_sample_end_global: f32,
    /// 2026-05-21: full u64 widened-now. Required because
    /// `t_sample_end_global` (an f32) loses precision catastrophically
    /// after ~8 s of uptime — at 5e9 cycles the f32 mantissa can't
    /// distinguish a per-sample 13000-cycle increment, so `t_local`
    /// (derived as f32 subtraction of two large nearly-equal values)
    /// stays constant across samples → `p_end` constant → `signed_steps
    /// == 0` every sample → no steps ever pushed → motors silent. Pass
    /// the full u64 here so the consumer computes `t_local` from u64
    /// subtraction *before* the f32 conversion.
    pub now_cycles_u64: u64,
}

/// Walk the per-axis cursor forward past any sample-straddled pieces.
///
/// Returns `true` if at least one piece advance happened on this axis,
/// so the caller can use that as a hint for segment-retirement timing.
///
/// Spec §4.4. Per-axis loop only — does NOT make any retire/fault
/// decisions about other axes. The per-sample post-pass in
/// [`runtime_tick_sample`] owns `participating_mask` updates and the
/// early-exhaustion fault check (Task 10).
///
/// On curve exhaustion (`piece_cursor >= curve.piece_count`) this clears
/// both `axis.piece` and `axis.curve_handle` and breaks; the post-pass
/// later decides retire vs. fault.
///
/// The iteration cap is `MAX_PIECES_PER_CURVE` (the loop is structurally
/// bounded by curve length). Exceeding it implies internal corruption
/// (zero/non-finite duration, or a curve-pool race) — that case still
/// latches `PieceAdvanceUnderflow`.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §4.4.
//
// `usize as u32` and `f32 as u64` casts are deliberate quantizations
// matching the spec; the lints would force a workaround that doesn't
// improve correctness on this hot path.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss, clippy::cast_sign_loss)]
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    axis_idx: usize,
    curve_pool: &crate::curve_pool::CurvePool,
    shared: &SharedState,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u16 = 0;
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
        // Walk cursor forward by one piece. `duration` is non-negative
        // seconds; converting to cycles via `* cycles_per_second` and
        // then `as u64` is a controlled narrowing (the result fits in
        // u64 for any realistic piece). `saturating_add` on the cursor
        // means a runaway loop hits the iter cap rather than walking
        // past `piece_count` silently.
        let duration_cycles = (piece.duration * cycles_per_second) as u64;
        axis.piece_start_time_cycles = axis
            .piece_start_time_cycles
            .wrapping_add(duration_cycles);
        axis.piece_cursor = axis.piece_cursor.saturating_add(1);
        advanced = true;

        if let Some(handle) = axis.curve_handle {
            if let Some(curve_ptr) = curve_pool.lookup_active(handle) {
                // SAFETY: `lookup_active` Acquire-loaded the slot's
                // `current_gen` and confirmed it matches `handle`,
                // synchronizing with the foreground's Release store
                // of the populated curve. The ISR is the sole reader,
                // and `curve_ptr` aliases the slot's UnsafeCell payload
                // for the duration of this borrow (no `&mut` to the
                // slot exists). Dereferencing is sound under §10.5.
                #[allow(unsafe_code)]
                let curve = unsafe { &*curve_ptr };
                if (axis.piece_cursor as usize) < curve.piece_count as usize {
                    // SAFETY of index: piece_cursor < piece_count ≤
                    // MAX_PIECES_PER_CURVE = pieces array length.
                    #[allow(clippy::indexing_slicing)]
                    let next = curve.pieces[axis.piece_cursor as usize];
                    axis.piece = Some(next);
                } else {
                    // Curve exhausted. Per-sample post-pass (Task 10)
                    // owns the retire-vs-fault decision — clear local
                    // state only and let the loop exit.
                    axis.piece = None;
                    axis.curve_handle = None;
                    break;
                }
            } else {
                // Slot generation drift (defensive; shouldn't happen
                // unless host retired the curve mid-segment).
                axis.piece = None;
                axis.curve_handle = None;
                break;
            }
        } else {
            axis.piece = None;
            break;
        }

        iters = iters.saturating_add(1);
        if iters >= crate::curve_pool::MAX_PIECES_PER_CURVE as u16 {
            // Runaway loop (corrupted duration or curve_pool race).
            // Exceeding MAX_PIECES_PER_CURVE means something structural
            // is wrong — fault regardless of participation.
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
/// Phase-3 E-axis evaluator (spec §4.6, absolute-position model).
///
/// Returns the absolute E position in mm:
/// `engine_segment_base_e + segment_local`. The segment-local part
/// dispatches on [`Segment::e_mode`](crate::segment::Segment::e_mode):
///
/// - `CoupledToXy`: `intrinsic + extrusion_ratio * ds_xy_segment + pa_k *
///   extrusion_ratio * v_xy_this`. The follower term integrates
///   XY arc length since segment-arm; PA adds a velocity-proportional
///   correction whose sign (accelerating vs decelerating) the caller
///   has already baked into `pa_k`.
/// - `Independent`: `intrinsic` only — the E NURBS drives motion
///   (retract / prime / filament change). No XY contribution.
/// - `Travel`: `0`. E does not move.
///
/// `intrinsic` is the position from `axis_e.piece` (if any), evaluated at
/// `t_sample_end_global - piece_start_seconds`. Without an active piece
/// it is `0.0` — `Independent` mode then returns
/// `engine_segment_base_e` (no E motion this sample), which is exactly
/// the spec §4.6 "Independent with no intrinsic" behaviour.
///
/// `current == None` (no segment armed) likewise returns
/// `engine_segment_base_e` — E holds its accumulated position.
///
/// `&mut AxisConfig` is taken for parity with the rest of Phase 3 (the
/// caller already holds an exclusive borrow), but this evaluator does
/// not mutate the axis — `advance_piece_if_needed` does that in the
/// surrounding tick body.
//
// Permitted in this hot path: cast-precision-loss on `piece_start_time_cycles
// as f32` matches the rest of `runtime_tick_sample`; we live with the f32
// epoch's 24-bit mantissa for the same reasons documented on `TickContext`.
#[allow(clippy::cast_precision_loss)]
#[allow(clippy::too_many_arguments)] // Spec-pinned signature (§4.6 pseudocode).
pub fn evaluate_e_axis(
    axis_e: &mut AxisConfig,
    current: Option<&crate::segment::Segment>,
    engine_segment_base_e: f32,
    ds_xy_segment_accumulator: f32,
    pa_k: f32,
    v_xy_this: f32,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> f32 {
    let Some(seg) = current else {
        return engine_segment_base_e;
    };

    let intrinsic_local = if let Some(piece) = axis_e.piece {
        let piece_start_sec =
            (axis_e.piece_start_time_cycles as f32) / cycles_per_second;
        let t_local = t_sample_end_global - piece_start_sec;
        let (p, _v) = crate::monomial::eval_position_velocity(&piece, t_local);
        p
    } else {
        0.0
    };

    let segment_local = match seg.e_mode {
        crate::config::EMode::CoupledToXy => {
            let follower = seg.extrusion_ratio * ds_xy_segment_accumulator;
            let pa_correction = pa_k * seg.extrusion_ratio * v_xy_this;
            intrinsic_local + follower + pa_correction
        }
        crate::config::EMode::Independent => intrinsic_local,
        crate::config::EMode::Travel => 0.0,
    };

    engine_segment_base_e + segment_local
}

// `runtime_tick_sample` is the per-sample ISR core. Spec §4 makes the
// per-phase block structure load-bearing for review; splitting it into
// helpers would obscure the ABXZ / A2 / E phase ordering documented in the
// design. The 108-line body remains a deliberate single function.
#[allow(clippy::indexing_slicing, clippy::too_many_lines)]
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
            ctx.curve_pool,
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

        // 2026-05-21: compute `t_local` from u64 cycle SUBTRACT first,
        // then convert to f32 — otherwise f32 catastrophic cancellation
        // (two large nearly-equal values subtracted) makes t_local
        // constant across samples after ~8 s of uptime, freezing p_end
        // and making signed_steps == 0 forever (motors silent despite
        // engine running cleanly).
        let t_local_cycles = ctx
            .now_cycles_u64
            .wrapping_sub(axis.piece_start_time_cycles);
        let t_local = (t_local_cycles as f32) / ctx.cycles_per_second;

        // 2026-05-21 diag: capture c0 (start-of-piece position) and
        // t_local on every eval. Last value before the dispatch_pulse
        // bound check trips is what the host reads via diag tags.
        ctx.shared.isr_last_c0_bits.store(piece.coeffs[0].to_bits(), core::sync::atomic::Ordering::Relaxed);
        ctx.shared.isr_last_t_local_bits.store(t_local.to_bits(), core::sync::atomic::Ordering::Relaxed);

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

        // Step 7: dispatch_axis re-enabled. dispatch_pulse now bails
        // before the queue_push loop (compute_step_times runs).
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
        *ctx.ds_xy_segment += v_xy_this * ctx.sample_period_sec;
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
    // Absolute-position model (spec §4.6). `evaluate_e_axis` returns
    // `engine_segment_base_e + segment_local`, dispatching on the
    // current segment's `e_mode`:
    //   - CoupledToXy: intrinsic + extrusion_ratio × ds_xy_segment
    //                  + pa_k × extrusion_ratio × v_xy_this
    //   - Independent: intrinsic (E NURBS only; no XY contribution)
    //   - Travel:      0 (no E motion)
    // `pa_k` is `advance_accel` while v_xy is rising and `advance_decel`
    // while falling (asymmetric PA, bleeding-edge-v2 Step 9 lineage in
    // the CLAUDE.md scope).
    //
    // `dispatch_axis` for E reuses `caches.p_prev[AXIS_E]` as the prior
    // absolute position; the per-axis cache is updated below in Phase 4
    // so subsequent samples see this sample's absolute E.
    // -----------------------------------------------------------------
    {
        let axis = &mut ctx.axes[AXIS_E];
        let p_sample_start = ctx.caches.p_prev[AXIS_E];

        // Phase-5 prologue for the extruder axis. Same rationale as the
        // motion-axis branch above: advance / retire before evaluation.
        advance_piece_if_needed(
            axis,
            AXIS_E,
            ctx.curve_pool,
            ctx.shared,
            ctx.t_sample_end_global,
            ctx.cycles_per_second,
        );

        let pa_k = if ctx.caches.vdot_xy_accelerating {
            ctx.advance_accel
        } else {
            ctx.advance_decel
        };
        let p_end = evaluate_e_axis(
            axis,
            ctx.current_segment,
            ctx.engine_segment_base_e,
            *ctx.ds_xy_segment,
            pa_k,
            ctx.caches.v_xy_this,
            ctx.t_sample_end_global,
            ctx.cycles_per_second,
        );

        if p_end.is_finite() {
            p_end_axis[AXIS_E] = p_end;
            // Phase-3 doesn't surface a separate E velocity (the secant-
            // slope sub-sample timing uses position differencing).
            v_end_axis[AXIS_E] = 0.0;
            dispatch_axis(
                AXIS_E,
                axis,
                ctx.queues[AXIS_E],
                ctx.shared,
                p_end,
                /* v_end */ 0.0,
                p_sample_start,
                ctx.sample_period_sec,
                ctx.now_cycles,
                ctx.cycles_per_second,
            );
        } else {
            raise_math_non_finite(ctx.shared, AXIS_E);
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
    // Phase 5: segment retirement.
    //
    // Owned by `Engine::post_pass_exhaustion` + `Engine::retire_if_complete`
    // (Task 10). The post-pass updates `pending_mask` from the per-axis
    // exhaustion state set by `advance_piece_if_needed` above, and the
    // retire publishes `retired_through_segment_id`, the `SEGMENT_END`
    // trace marker, and rolls forward `e_accumulator` when
    // `pending_mask == 0`. `Engine::tick_sample` invokes both immediately
    // after this function returns — the call site needs the trace
    // producer borrow, which is held outside `TickContext`.
    // -----------------------------------------------------------------
}

// ─── ISR sample entry — Codex M1 + M2 (2026-05-20) ───────────────────────
//
// Single sample-tick body invoked from `kalico_runtime_tick_sample`. Owns
// three responsibilities the pre-redesign code distributed across the FFI
// shim, `Engine::tick`, and the removed `producer_step`:
//
// 1. **Widen + publish the MCU clock (Codex M2).** Reads the raw 32-bit
//    DWT/CYCCNT, extends it to u64 through the ISR-owned `WidenState`, and
//    publishes the widened value through the §11.4 seqlock so
//    `Engine::tick_sample`'s `widened_now_lo.load()` and any foreground
//    reader see a coherent (lo, hi) pair. Without this step the
//    `widened_now_*` cells stayed at zero on the H7 / F4 because nothing in
//    the new stepping-redesign FFI surface ever called `WidenState::widen`
//    or `publish_widened_now` — the engine's sample-time math read 0 every
//    tick and the piece evaluator never advanced.
//
// 2. **Arm queued segments (Codex M1).** When `engine.current` is `None`,
//    dequeue the next producer-side segment (or promote a previously
//    deferred one) and hand it to `Engine::arm_segment`. The §6.3 host
//    `arm()` predicate already verified `first_t_start >= now + arm_lead`
//    against the foreground's view of the clock, so the ISR's job here is
//    purely the mechanical "current slot empty → install the next segment"
//    transition. If the segment's `t_start` is still in the future relative
//    to the freshly widened now, we park it in `IsrState::pending_segment`
//    until a later tick promotes it — the C-backed SPSC has no put-back
//    primitive, so we must hold the dequeued segment in ISR-owned state to
//    preserve order.
//
// 3. **Run the per-sample evaluator.** Same `Engine::tick_sample` call the
//    FFI used pre-fix, just rehosted here so the cyccnt widen / clock
//    publish / arm steps all sit on the same exclusive `&mut IsrState`
//    borrow and there is no field-disjoint-borrow gymnastics in the FFI
//    shim.
//
// All three steps must run under a single `&mut IsrState` borrow because
// `Engine::arm_segment`, `WidenState::widen`, and the queue/pending-segment
// mutation are mutually exclusive writers to ISR-owned state. The §11.1
// ownership discipline says TIM5 is the sole writer; this function is the
// ISR's entry point and therefore the natural home for those mutations.
/// Single-call ISR body that widens the clock, arms a queued segment if
/// the engine is idle, and evaluates the per-sample stepping math.
/// `raw_cyccnt` is the freshly-sampled 32-bit DWT counter (zero on host).
///
/// # Caller contract
///
/// Must run under exclusive `&mut IsrState` access. The FFI shim's
/// `kalico_runtime_tick_sample` is the production caller; host integration
/// tests reach this through the published kalico-c-api FFI entry, which
/// projects the rt_storage half-split and calls this function under the
/// same single-writer discipline.
pub fn isr_sample_tick(
    isr: &mut crate::state::IsrState,
    shared: &SharedState,
    curve_pool: &crate::curve_pool::CurvePool,
    raw_cyccnt: u32,
) {
    // 2026-05-21 bench diag — per-stage cycle counters.
    // `cyccnt_read()` here is a private host-stub-able extern (declared
    // below); on MCU it's the same DWT->CYCCNT read the C side uses,
    // so deltas are in clock cycles (520 MHz on H7 / 180 MHz on F4).
    // Skipped on host/test builds — the stub returns 0 so all four reads
    // give zero deltas, max counters stay at 0.
    let body_start = unsafe { cyccnt_read() };

    // Bench bring-up diagnostic (2026-05-21): `Engine::tick_counter`
    // existed but had no production increment site — the FFI accessor
    // returned 0 forever, so the bench diag tag "is TIM5 firing?" was
    // unreadable. Bump per-call here so the counter snapshots whether
    // the ISR body executed at all, independent of the sample-period /
    // arm-from-queue guards below. Wrapping is benign (u32 at 40 kHz
    // wraps every ~30 hours).
    //
    // 2026-05-21 follow-up: rebound through `bump_relaxed` (load+store)
    // because the original `increment()` (fetch_add(1, Relaxed)) was the
    // first counter to show the "writes not visible to FFI reader"
    // symptom — even ruby's documented "this fixes the latent gap"
    // increment reads 0 from kalico_runtime_get_tick_counter despite
    // isr_sample_tick demonstrably running (E6 widen_max non-zero).
    bump_relaxed(isr.engine.tick_counter.inner_atomic());

    // 1. Widen the raw DWT sample and publish the §11.4 seqlock.
    let now = isr.widen_state.widen(raw_cyccnt);
    crate::clock::publish_widened_now(shared, now);
    let after_widen = unsafe { cyccnt_read() };
    update_max(&shared.isr_widen_cycles_max, after_widen.wrapping_sub(body_start));

    // 2026-05-21 bisection step 2: 948c6fdc6 (widen+publish only)
    // survived the jog (state=ready post-jog, no crash, F7=589K IRQs,
    // E5=314 cycles per ISR, E3=45K tick_counter, all atomics fine).
    // That confirmed the 4-second freeze is in dequeue/arm/eval. Now
    // re-enable dequeue + park-check but still skip arm_segment +
    // tick_sample evaluator. If bench survives:
    //   freeze is in arm_segment or evaluator (next bisection step).
    // If bench crashes:
    //   freeze is in queue_consumer.dequeue() itself — less likely
    //   but possible if the SPSC interaction with the C ring is
    //   doing something pathological under load.

    // 2. Promote-or-dequeue when the engine's current slot is empty.
    //    Take `pending_segment` out by value so the subsequent
    //    `arm_segment` / `dequeue` calls don't fight for the IsrState
    //    field borrows.
    if isr.engine.current.is_none() {
        let candidate = match isr.pending_segment.take() {
            Some(parked) => Some(parked),
            None => {
                let dequeued = isr.queue_consumer.dequeue();
                if dequeued.is_some() {
                    shared
                        .producer_segment_dequeued_total
                        .fetch_add(1, Ordering::AcqRel);
                    bump_relaxed(&shared.isr_deq_some_count);
                } else {
                    bump_relaxed(&shared.isr_deq_none_count);
                }
                dequeued
            }
        };
        if let Some(seg) = candidate {
            shared.isr_last_t_start_lo.store(seg.t_start as u32, Ordering::Relaxed);
            shared.isr_last_widened_lo.store(now as u32, Ordering::Relaxed);
            if seg.t_start <= now {
                shared
                    .current_segment_id
                    .store(seg.id, Ordering::Release);
                bump_relaxed(&shared.isr_armed_count);
                // 2026-05-21 bisection step 4: arm_segment ON, evaluator OFF.
                // bd62f20bb (dequeue+park yes, arm+eval no) survived. EA=2
                // dequeues, EC=0 parks (widening seed correct, seg.t_start
                // <= now), ED=2 would-be-arms, EE=EF=0xBD47F5 (host time
                // domain matches MCU's widened). Now actually call
                // arm_segment but still bail before tick_sample. If bench
                // survives, freeze is in tick_sample evaluator. If crashes,
                // freeze is in arm_segment (likely curve_pool lookup or
                // axis piece initialization).
                isr.engine.arm_segment(seg, curve_pool);
            } else {
                bump_relaxed(&shared.isr_parked_count);
                isr.pending_segment = Some(seg);
            }
        }
    }

    let after_arm = unsafe { cyccnt_read() };
    update_max(&shared.isr_arm_cycles_max, after_arm.wrapping_sub(after_widen));

    // 2026-05-21 bisection step 5: re-enable tick_sample call. Bisection
    // inside runtime_tick_sample (via _BISECT_RTS env at compile time) is
    // controlled separately — see the early-return below `pub fn
    // runtime_tick_sample`. Step 4 (4a15dcf78) with arm_segment ON + this
    // call OFF survived (ED=1, E7=1154). Now adding back the engine.tick_sample
    // call; if bench crashes, the freeze is in runtime_tick_sample's body.

    // 2026-05-21 circuit breaker. The previous bench attempts crashed
    // with a 4-second IRQ tying up the CPU (prior_diag tim5_max_cyc =
    // 2147491011 ≈ 4.1s, IWDG fires at 500ms → MCU reset → USB drop
    // → klippy aborts). Bail out before tick_sample evaluator runs so
    // the ISR returns within a sane budget regardless of what the
    // evaluator does. If the bench survives the next jog with this in
    // place, the 4-second freeze is in tick_sample (or arm_segment's
    // far-end). Counter `isr_eval_skipped_count` (NEW field) tracks
    // bails.
    let elapsed = after_arm.wrapping_sub(body_start);
    if elapsed > 20000 {
        // ~38 µs at 520 MHz. Whatever happened above already burned
        // the per-tick budget; skip the evaluator and return so IWDG
        // doesn't fire. Subsequent ticks may still fault, but the
        // bench should survive long enough for diag tags to capture
        // the state.
        bump_relaxed(&shared.isr_overrun_count);
        return;
    }

    // 3. Hand the per-sample evaluator the trace producer it needs. The
    //    field-disjoint borrow here is the same pattern the FFI used
    //    before the M1/M2 fix landed.
    let crate::state::IsrState {
        engine,
        trace_producer,
        ..
    } = isr;
    engine.tick_sample(shared, curve_pool, trace_producer);
    let body_end = unsafe { cyccnt_read() };
    update_max(&shared.isr_eval_cycles_max, body_end.wrapping_sub(after_arm));

    // Circuit-breaker: if the whole body exceeded ~58 µs on H7 (30000
    // cycles at 520 MHz; ~167 µs on F4 at 180 MHz), bump the overrun
    // counter so the host can see we're starving foreground.
    let body_cycles = body_end.wrapping_sub(body_start);
    if body_cycles > 30000 {
        shared.isr_overrun_count.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

// CYCCNT extern. MCU = DWT->CYCCNT via the C helper; host = always 0
// so test builds don't break (cyccnt deltas in tests are meaningless
// anyway — sample times come from `runtime_tick_sample` inputs).
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn runtime_cyccnt_read() -> u32;
}
#[cfg(not(any(test, feature = "host")))]
#[inline]
unsafe fn cyccnt_read() -> u32 {
    unsafe { runtime_cyccnt_read() }
}
#[cfg(any(test, feature = "host"))]
#[inline]
unsafe fn cyccnt_read() -> u32 { 0 }

#[inline]
fn update_max(slot: &core::sync::atomic::AtomicU32, val: u32) {
    use core::sync::atomic::Ordering;
    let prev = slot.load(Ordering::Relaxed);
    if val > prev {
        // Best-effort: a racy update from another writer can lose an
        // intermediate max, but the ISR is the sole writer to these
        // counters (TIM5 = single producer), so the race is theoretical.
        slot.store(val, Ordering::Relaxed);
    }
}

// 2026-05-21: Workaround for the "fetch_add not visible to FFI reader"
// codegen symptom seen in earlier bench. Tags 0xE3/0xEA/0xEB/0xEC/0xED
// all read 0 despite E6 (update_max-driven) reading 2831 cycles in the
// same ISR call — meaning fetch_add(1, Relaxed) writes weren't visible
// while load+store WERE. Same fingerprint as the 2026-05-18 LLVM
// miscompile of heapless::spsc::Consumer (qlen_sd=6 qlen_ps=1 from the
// same instance). Mirror update_max's load+store pattern explicitly so
// the same codegen path the working counter uses applies to all writers.
// ISR is the single writer for these counters; race-free.
#[inline]
fn bump_relaxed(slot: &core::sync::atomic::AtomicU32) {
    use core::sync::atomic::Ordering;
    let prev = slot.load(Ordering::Relaxed);
    slot.store(prev.wrapping_add(1), Ordering::Relaxed);
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
            position_count: AtomicI32::new(0),
            tmc_cs_oid: None,
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
            curve_handle: None,
            piece_cursor: 0,
            piece: None::<BezierPieceMonomial>,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance,
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

    /// Task 13: Phase mode ramps `phase_offset_microsteps` toward
    /// `phase_offset_target` at `max_phase_offset_ramp_per_sample` per
    /// call, clamping on the final step.
    #[test]
    fn phase_mode_ramps_offset_toward_target_at_max_per_sample() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Phase, 0.0125);
        // current = 0, target = 10, max = 4 → expect 4, 8, 10.
        axis.steppers[0]
            .phase_offset_target
            .store(10, Ordering::Release);
        shared
            .max_phase_offset_ramp_per_sample
            .store(4, Ordering::Release);

        let q_ptr: *mut StepQueue = &mut q;
        for expected in [4_i32, 8, 10] {
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
                axis.steppers[0]
                    .phase_offset_microsteps
                    .load(Ordering::Acquire),
                expected,
                "ramp should advance to {expected}",
            );
        }
    }

    /// Task 13: `max_phase_offset_ramp_per_sample == 0` disables the
    /// ramp — `phase_offset_microsteps` is left untouched even when
    /// `phase_offset_target` differs.
    #[test]
    fn phase_mode_ramp_disabled_when_max_per_sample_is_zero() {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Phase, 0.0125);
        axis.steppers[0]
            .phase_offset_microsteps
            .store(3, Ordering::Release);
        axis.steppers[0]
            .phase_offset_target
            .store(99, Ordering::Release);
        // max_phase_offset_ramp_per_sample defaults to 0 (no ramp).

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
            axis.steppers[0]
                .phase_offset_microsteps
                .load(Ordering::Acquire),
            3,
            "ramp should be a no-op when max_per_sample == 0",
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
