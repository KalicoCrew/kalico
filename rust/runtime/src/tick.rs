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
use crate::step_queue::{StepEntry, StepQueue, push as queue_push};
use crate::stepping_state::{AxisConfig, StepMode};
use crate::sub_sample_timing::{StepTimeInputs, StepTimingResult, compute_step_times};

// FFI declaration for the C-side SPI write function. Enabled on MCU builds
// and MACH_LINUX sim builds (kalico-sim feature). The C implementation in
// `src/stm32/phase_stepping_spi.c` (MCU) and `src/linux/phase_stepping_spi.c`
// (sim) performs a skip-not-block transfer: if `phase_spi_try_acquire()`
// fails (bus held by foreground TMC register access), the write is skipped
// and a skip counter is incremented — no blocking, no queue needed.
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn phase_stepping_write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16);
}

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

    let mode = axis.mode.load(Ordering::Acquire);
    shared.isr_last_axis_mode_packed.store(
        ((axis_idx as u32) << 16) | u32::from(mode),
        Ordering::Relaxed,
    );
    match mode {
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
        m if m == StepMode::Phase as u8 => {
            bump_relaxed(&shared.isr_phase_call_count);
            dispatch_phase(axis_idx, axis, shared, p_end);
        }
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
    bump_relaxed(&shared.isr_pulse_call_count);
    // Guard against a zero / non-finite microstep distance — that would
    // make the quantization step divide by zero and the engine should
    // never have been armed in that state. Bail silently for Task 7;
    // Task 4's configure_axes is the proper gatekeeper.
    let microstep_distance = axis.microstep_distance;
    if axis_idx == AXIS_A {
        shared
            .isr_last_microstep_bits
            .store(microstep_distance.to_bits(), Ordering::Relaxed);
    }
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        bump_relaxed(&shared.isr_pulse_bad_mstep_count);
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
    if axis_idx == AXIS_A {
        shared
            .isr_last_p_end_bits
            .store(p_end.to_bits(), Ordering::Relaxed);
        shared.isr_last_step_counts_packed.store(
            ((target_step_count as u32) << 16) | ((prev_step_count as u32) & 0xFFFF),
            Ordering::Relaxed,
        );
    }
    // Update the axis cache regardless of whether we found any steps to
    // schedule — Phase-mode keeps it in lockstep too.
    axis.last_step_count = target_step_count;

    if signed_steps == 0 {
        bump_relaxed(&shared.isr_pulse_zero_step_count);
        return;
    }
    // 2026-05-21 diag: capture last non-zero signed_steps so the host
    // can see what the eval is producing. If this stays 0 across an
    // entire jog despite EA/ED bumping, the eval is never producing
    // enough p_end change to cross a microstep threshold.
    if axis_idx == AXIS_A {
        shared.isr_last_signed_steps.store(
            signed_steps.unsigned_abs(),
            core::sync::atomic::Ordering::Relaxed,
        );
    }
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
        shared
            .isr_last_t_start_lo
            .store(abs_steps, core::sync::atomic::Ordering::Relaxed);
        if axis_idx == AXIS_A {
            shared
                .isr_last_p_end_bits
                .store(p_end.to_bits(), core::sync::atomic::Ordering::Relaxed);
            shared.isr_last_microstep_bits.store(
                microstep_distance.to_bits(),
                core::sync::atomic::Ordering::Relaxed,
            );
        }
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
        let entry = StepEntry {
            cycle_abs,
            dir,
            _pad: [0; 3],
        };
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
fn commit_position_count(axis: &AxisConfig, axis_idx: usize, shared: &SharedState, delta: i32) {
    if delta == 0 {
        return;
    }
    // When step_modes[axis] == Modulated, the TMC5160 is in XDIRECT mode
    // and ignores GPIO step pulses — they produce no physical motion. Only
    // dispatch_phase (via direct position_count updates) should advance the
    // counter. Suppressing here makes the sim faithful to hardware.
    if shared
        .step_modes
        .get(axis_idx)
        .map_or(false, |m| {
            m.load(Ordering::Acquire) == crate::state::StepMode::Modulated as u8
        })
    {
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
fn dispatch_phase(axis_idx: usize, axis: &mut AxisConfig, shared: &SharedState, p_end: f32) {
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
        // Klipper's bus driver. Steppers without a TMC chip-select (e.g.
        // a phase-stepped Z stepper without TMC5160) skip the push.
        //
        // `motor_idx` is resolved from `shared.phase_slot_idx`: scan
        // entries 0..phase_motor_count for ones whose slot matches
        // `axis_idx`, collecting them into a local cursor so the j-th
        // stepper in this axis maps to the j-th matching motor entry.
        if stepper.tmc_cs_oid.is_some() {
            // Resolve motor_idx by scanning phase_slot_idx for this axis.
            // We track `stepper_cursor` (count of steppers with tmc_cs_oid
            // seen so far) to match the j-th stepper to the j-th motor.
            // This variable is incremented below after each tmc_cs_oid stepper.
            // Inline scan: find the motor_cursor-th entry in phase_slot_idx
            // that maps to axis_idx.
            let phase_motor_count =
                shared.phase_motor_count.load(Ordering::Acquire) as usize;
            let mut found_motor_idx: Option<u8> = None;
            {
                // Count how many tmc_cs_oid-bearing steppers in this axis
                // come before `stepper` in `axis.steppers` to derive the
                // j-th slot we need to look up.
                let mut j: usize = 0;
                for earlier in &axis.steppers {
                    if core::ptr::eq(earlier as *const _, stepper as *const _) {
                        break;
                    }
                    if earlier.tmc_cs_oid.is_some() {
                        j += 1;
                    }
                }
                // Walk phase_slot_idx to find the j-th entry matching axis_idx.
                let mut match_count: usize = 0;
                for m in 0..phase_motor_count.min(crate::state::MAX_STEPPER_OIDS) {
                    let slot = shared.phase_slot_idx[m].load(Ordering::Acquire);
                    if slot as usize == axis_idx {
                        if match_count == j {
                            #[allow(clippy::cast_possible_truncation)]
                            {
                                found_motor_idx = Some(m as u8);
                            }
                            break;
                        }
                        match_count += 1;
                    }
                }
            }

            let motor_idx = found_motor_idx.unwrap_or(0xFF);

            #[cfg(any(test, feature = "host"))]
            crate::test_xdirect_capture::record(motor_idx, coil_a, coil_b);

            // Call phase_stepping_write_xdirect directly from the ISR.
            // On MCU builds this is always active. On MACH_LINUX sim builds
            // (kalico-sim feature), it's also active so the call exercises
            // the real SPI path through the shim → emulator.
            // The C function implements skip-not-block semantics: it calls
            // phase_spi_try_acquire() and skips the transfer (incrementing a
            // skip counter) if the SPI bus is held by a foreground TMC
            // register access — no fault, no queue needed.
            #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
            // SAFETY: motor_idx, coil_a, coil_b are computed from validated
            // LUT/config state above. The C function is reentrant-safe for
            // concurrent foreground access via its internal try-acquire guard.
            unsafe {
                phase_stepping_write_xdirect(motor_idx, coil_a, coil_b);
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
///   sample-window scalars used by dispatch.
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
    pub sample_period_sec: f32,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
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
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    axis_idx: usize,
    curve_pool: &crate::curve_pool::CurvePool,
    shared: &SharedState,
    now_cycles_u64: u64,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u16 = 0;
    loop {
        let Some(piece) = axis.piece else {
            break;
        };
        // 2026-05-21 fix: subtract in u64 cycle domain BEFORE converting to
        // f32 seconds. The prior `(now as f32) - (start as f32)` form lost
        // precision after ~8 s of uptime (both operands rounded to the same
        // f32 value), making t_local ≈ 0 and never crossing any piece
        // duration → loop walks past every piece on first sample → axis.piece
        // becomes None → per-axis eval skips silently. Same root cause as
        // commit 5be894004 fixed for the eval path; this caller was missed.
        let t_local_cycles = now_cycles_u64.wrapping_sub(axis.piece_start_time_cycles);
        let t_local = (t_local_cycles as f32) / cycles_per_second;
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
        axis.piece_start_time_cycles = axis.piece_start_time_cycles.wrapping_add(duration_cycles);
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
/// All four axes (A, B, Z, E) are evaluated identically. The host
/// pre-computes E as a regular Bezier curve; the MCU treats E the same
/// as any other axis with no PA, no arc-length integration, no XY-derived
/// quantities.
//
// Every `[...]` index in this body is statically bounded by `N_AXES`.
// The blanket `allow(clippy::indexing_slicing)` is justified — each
// index is within [0, N_AXES) by construction.
// `libm::fabsf(v) * 65536.0 as u32` for endstop Q16.16 encoding:
// `fabsf` always returns non-negative; `as u32` saturates on overflow.
#[allow(
    clippy::indexing_slicing,
    clippy::too_many_lines,
    clippy::cast_sign_loss
)]
pub fn runtime_tick_sample(ctx: &mut TickContext) {
    let mut p_end_axis = [0.0_f32; N_AXES];
    let mut v_end_axis = [0.0_f32; N_AXES];

    // -----------------------------------------------------------------
    // Phase 1: evaluate ALL axes (A, B, Z, E) uniformly.
    //
    // All four axes share identical evaluation logic: advance piece cursor,
    // evaluate Bezier polynomial, dispatch steps. E is no longer special.
    // -----------------------------------------------------------------
    for axis_idx in [AXIS_A, AXIS_B, AXIS_Z, AXIS_E] {
        let axis = &mut ctx.axes[axis_idx];
        let p_sample_start = ctx.caches.p_prev[axis_idx];

        advance_piece_if_needed(
            axis,
            axis_idx,
            ctx.curve_pool,
            ctx.shared,
            ctx.now_cycles_u64,
            ctx.cycles_per_second,
        );

        let Some(piece) = axis.piece else {
            // No active piece on this axis: hold position, zero velocity.
            p_end_axis[axis_idx] = p_sample_start;
            v_end_axis[axis_idx] = 0.0;
            continue;
        };

        // Compute t_local from u64 cycle subtraction BEFORE converting to f32
        // to avoid catastrophic cancellation after ~8 s uptime.
        let t_local_cycles = ctx
            .now_cycles_u64
            .wrapping_sub(axis.piece_start_time_cycles);
        let t_local = (t_local_cycles as f32) / ctx.cycles_per_second;

        if axis_idx == AXIS_A {
            ctx.shared.isr_last_c0_bits.store(
                piece.coeffs[0].to_bits(),
                core::sync::atomic::Ordering::Relaxed,
            );
            ctx.shared
                .isr_last_t_local_bits
                .store(t_local.to_bits(), core::sync::atomic::Ordering::Relaxed);
        }

        let (p_end, v_end) = crate::monomial::eval_position_velocity(&piece, t_local);
        if !p_end.is_finite() || !v_end.is_finite() {
            raise_math_non_finite(ctx.shared, axis_idx);
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
    // Phase 2: publish (p_end, v_end) into per-axis caches for the next
    // tick's secant-slope sub-sample timing.
    // -----------------------------------------------------------------
    ctx.caches.p_prev = p_end_axis;
    ctx.caches.v_prev = v_end_axis;

    // -----------------------------------------------------------------
    // Phase 3: Endstop trip evaluation (sensorless homing).
    //
    // Q16.16 per-axis motor-frame speed magnitudes for the velocity-gated
    // endstop policy (`IgnoreUntilMoving`). Derived from the just-evaluated
    // v_end_axis values for A, B, Z.
    // -----------------------------------------------------------------
    {
        // Compute Q16.16 speeds for axes A, B, Z (endstop uses motor-frame).
        let v_motor_q16: [u32; 3] = [
            (libm::fabsf(v_end_axis[AXIS_A]) * 65536.0) as u32,
            (libm::fabsf(v_end_axis[AXIS_B]) * 65536.0) as u32,
            (libm::fabsf(v_end_axis[AXIS_Z]) * 65536.0) as u32,
        ];

        let mut stepper_counts = [0_i32; crate::state::MAX_STEPPER_OIDS];
        for axis in ctx.axes.iter() {
            for stepper in axis.steppers.iter() {
                if let Some(dst) = stepper_counts.get_mut(usize::from(stepper.stepper_oid)) {
                    *dst = stepper.position_count.load(Ordering::Acquire);
                }
            }
        }
        if crate::endstop::tick(ctx.now_cycles_u64, v_motor_q16, &stepper_counts)
            == crate::endstop::TripAction::AbortNow
        {
            for axis in ctx.axes.iter_mut() {
                axis.piece = None;
                axis.curve_handle = None;
            }
        }
        for (i, &val) in stepper_counts.iter().enumerate() {
            if let Some(dst) = ctx.shared.stepper_counts.get(i) {
                dst.store(val, Ordering::Release);
            }
        }
    }

    // -----------------------------------------------------------------
    // Phase 4: segment retirement.
    //
    // Owned by `Engine::post_pass_exhaustion` + `Engine::retire_if_complete`
    // (Task 10). Invoked immediately after this function returns in
    // `Engine::tick_sample`.
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
    update_max(
        &shared.isr_widen_cycles_max,
        after_widen.wrapping_sub(body_start),
    );

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
        if let Some(mut seg) = candidate {
            shared
                .isr_last_t_start_lo
                .store(seg.t_start as u32, Ordering::Relaxed);
            shared
                .isr_last_widened_lo
                .store(now as u32, Ordering::Relaxed);
            // 2026-05-21 epoch-diagnosis: also capture HIGH 32 bits and the
            // saturating cycle-delta so we can distinguish hypothesis (a) wrong
            // epoch from (b) u32-narrowed t_start on the wire.
            shared
                .isr_last_t_start_hi
                .store((seg.t_start >> 32) as u32, Ordering::Relaxed);
            shared
                .isr_last_widened_hi
                .store((now >> 32) as u32, Ordering::Relaxed);
            let delta = now.saturating_sub(seg.t_start);
            shared
                .isr_arm_delta_lo
                .store(delta as u32, Ordering::Relaxed);
            shared
                .isr_arm_delta_hi
                .store((delta >> 32) as u32, Ordering::Relaxed);
            if seg.t_start <= now {
                shared.current_segment_id.store(seg.id, Ordering::Release);
                bump_relaxed(&shared.isr_armed_count);
                // Host/USB command latency can make a freshly dequeued
                // terminal segment start a few milliseconds in the past by
                // the time the TIM5 ISR arms it. If we keep that stale
                // epoch, `advance_piece_if_needed` may walk every short
                // piece to exhaustion before the first evaluation and the
                // move retires without emitting steps. Rebase late arms to
                // the current ISR clock while preserving segment duration.
                let lateness = now.saturating_sub(seg.t_start);
                if lateness > 0 {
                    seg.t_start = now;
                    seg.t_end = seg.t_end.saturating_add(lateness);
                }
                // 2026-05-21 bisection step 4: arm_segment ON, evaluator OFF.
                // bd62f20bb (dequeue+park yes, arm+eval no) survived. EA=2
                // dequeues, EC=0 parks (widening seed correct, seg.t_start
                // <= now), ED=2 would-be-arms, EE=EF=0xBD47F5 (host time
                // domain matches MCU's widened). Now actually call
                // arm_segment but still bail before tick_sample. If bench
                // survives, freeze is in tick_sample evaluator. If crashes,
                // freeze is in arm_segment (likely curve_pool lookup or
                // axis piece initialization).
                isr.engine.arm_segment_with_diag(seg, curve_pool, shared);
            } else {
                bump_relaxed(&shared.isr_parked_count);
                isr.pending_segment = Some(seg);
            }
        }
    }

    let after_arm = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_arm_cycles_max,
        after_arm.wrapping_sub(after_widen),
    );

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
    update_max(
        &shared.isr_eval_cycles_max,
        body_end.wrapping_sub(after_arm),
    );

    // Circuit-breaker: if the whole body exceeded ~58 µs on H7 (30000
    // cycles at 520 MHz; ~167 µs on F4 at 180 MHz), bump the overrun
    // counter so the host can see we're starving foreground.
    let body_cycles = body_end.wrapping_sub(body_start);
    if body_cycles > 30000 {
        shared
            .isr_overrun_count
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
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
unsafe fn cyccnt_read() -> u32 {
    0
}

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
mod tests;
