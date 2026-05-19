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
    raise_position_count_overflow, raise_step_queue_overflow,
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
    for cycle_abs in times.iter().copied() {
        let entry = StepEntry { cycle_abs, dir, _pad: [0; 3] };
        // SAFETY: `queue_ptr` is supplied by the caller (TIM5 ISR), who
        // owns the sole-producer role for this axis's step queue. The
        // queue's storage outlives this call (C-owned `.axi_bss` on the
        // MCU, stack/heap test buffer on host).
        let push_res = unsafe { queue_push(queue_ptr, entry) };
        if push_res.is_err() {
            raise_step_queue_overflow(shared, axis_idx);
            return;
        }
    }

    // Per-stepper position bookkeeping. ISR is the sole writer, so
    // load + checked_add + store (no CAS) is the right shape; we lose
    // no concurrency vs. `fetch_add` but gain overflow detection.
    for stepper in &axis.steppers {
        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(signed_steps) else {
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
