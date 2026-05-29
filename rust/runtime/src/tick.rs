//! TIM5 ISR body — the unified motion evaluator (dispatch stage).
//!
//! Task 5: `advance_piece_if_needed`, `TickContext`, and `runtime_tick_sample`
//! have been removed (they depended on `CurvePool`). Task 6 will reintroduce
//! them against the new `PieceRing` architecture.
//!
//! Retained: `dispatch_axis`, `dispatch_pulse`, `dispatch_phase`,
//! `isr_sample_tick`, and the axis-index constants.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::{raise_position_count_overflow, raise_step_queue_overflow};
use crate::phase_lut::PHASE_LUT;
use crate::state::SharedState;
use crate::step_queue::{StepEntry, StepQueue, push as queue_push};
use crate::stepping_state::{AxisConfig, StepMode};
use crate::sub_sample_timing::{StepTimeInputs, StepTimingResult, compute_step_times};

// FFI declaration for the C-side SPI write function.
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn phase_stepping_write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16);
}

/// `|P_end - P_start|` below this triggers the uniform-spacing fallback
/// in `dispatch_pulse`.
pub const DISPLACEMENT_THRESHOLD_MM: f32 = 1e-4;

pub use crate::stepping_state::N_AXES;
pub const AXIS_A: usize = 0;
pub const AXIS_B: usize = 1;
pub const AXIS_Z: usize = 2;
pub const AXIS_E: usize = 3;

/// Dispatch one TIM5 sample for a single axis.
#[allow(clippy::too_many_arguments)]
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
        _ => {}
    }
}

/// Pulse-mode dispatch: schedule step pulses across this sample window.
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
    axis.last_step_count = target_step_count;

    if signed_steps == 0 {
        bump_relaxed(&shared.isr_pulse_zero_step_count);
        return;
    }
    if axis_idx == AXIS_A {
        shared.isr_last_signed_steps.store(
            signed_steps.unsigned_abs(),
            core::sync::atomic::Ordering::Relaxed,
        );
    }
    let abs_steps = signed_steps.unsigned_abs();
    if abs_steps > crate::sub_sample_timing::MAX_STEPS_PER_SAMPLE as u32 {
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
        // SAFETY: `queue_ptr` is supplied by the TIM5 ISR, sole producer.
        let push_res = unsafe { queue_push(queue_ptr, entry) };
        if push_res.is_ok() {
            bump_relaxed(&shared.isr_step_push_count);
        }
        if push_res.is_err() {
            let committed_delta = steps_committed * (i32::from(dir));
            commit_position_count(axis, axis_idx, shared, committed_delta);
            raise_step_queue_overflow(shared, axis_idx);
            axis.last_step_count = prev_step_count + committed_delta;
            return;
        }
        steps_committed += 1;
    }

    commit_position_count(axis, axis_idx, shared, signed_steps);
}

fn commit_position_count(axis: &AxisConfig, axis_idx: usize, shared: &SharedState, delta: i32) {
    if delta == 0 {
        return;
    }
    if shared.step_modes.get(axis_idx).map_or(false, |m| {
        m.load(Ordering::Acquire) == crate::state::StepMode::Modulated as u8
    }) {
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

fn dispatch_phase(axis_idx: usize, axis: &mut AxisConfig, shared: &SharedState, p_end: f32) {
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }

    #[allow(clippy::cast_possible_truncation)]
    let target_microsteps_axis = libm::roundf(p_end / microstep_distance) as i32;
    axis.last_step_count = target_microsteps_axis;

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
        let delta_stepper = target_stepper.wrapping_sub(prev_stepper);
        stepper
            .last_phase_target
            .store(target_stepper, Ordering::Release);

        #[allow(clippy::cast_sign_loss)]
        let phase = (target_stepper as u32) & 0x3FF;
        let Some((coil_a, coil_b)) = PHASE_LUT.get(phase as usize).copied() else {
            continue;
        };

        stepper.last_coil_A.store(coil_a, Ordering::Release);
        stepper.last_coil_B.store(coil_b, Ordering::Release);

        if stepper.tmc_cs_oid.is_some() {
            let phase_motor_count = shared.phase_motor_count.load(Ordering::Acquire) as usize;
            let mut found_motor_idx: Option<u8> = None;
            {
                let mut j: usize = 0;
                for earlier in &axis.steppers {
                    if core::ptr::eq(earlier as *const _, stepper as *const _) {
                        break;
                    }
                    if earlier.tmc_cs_oid.is_some() {
                        j += 1;
                    }
                }
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

            #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
            // SAFETY: motor_idx, coil_a, coil_b are validated above.
            unsafe {
                phase_stepping_write_xdirect(motor_idx, coil_a, coil_b);
            }
        }

        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(delta_stepper) else {
            raise_position_count_overflow(shared, axis_idx);
            return;
        };
        stepper.position_count.store(next, Ordering::Release);
    }
}

// ─── ISR sample entry ─────────────────────────────────────────────────────

/// DIAG(sip): low 32 bits of `(rust_now - c_clock)` captured each ISR tick,
/// where `c_clock` is the C/Klipper widened clock (`timer_read_time()` +
/// `stats_send_time_high`) that the clock-sync response returns to the host.
/// This measures the divergence between the engine's evaluation clock and the
/// host's scheduling clock. Read by the -308 fault site. REVERT after.
pub(crate) static CLK_DOMAIN_OFFSET_CYC: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Single-call ISR body for the piece-ring walker engine (Task 6).
///
/// Widens the 32-bit DWT clock, publishes `widened_now`, then delegates to
/// `Engine::tick` which walks each configured axis's ring, evaluates the
/// Horner polynomial, and calls `dispatch_axis`.
///
/// `storage` is projected from `RuntimeContext::piece_storage` by the FFI
/// caller (`kalico_runtime_tick_sample`).
pub fn isr_sample_tick(
    isr: &mut crate::state::IsrState,
    shared: &SharedState,
    storage: &mut [crate::piece_ring::PieceEntry],
    raw_cyccnt: u32,
) {
    let body_start = unsafe { cyccnt_read() };

    bump_relaxed(isr.engine.tick_counter.inner_atomic());

    let now = isr.widen_state.widen(raw_cyccnt);
    crate::clock::publish_widened_now(shared, now);
    // DIAG(sip): measure divergence between the engine's Rust widened clock
    // (`now`) and the C/Klipper clock that the clock-sync response feeds the
    // host scheduler. Same DWT low bits, independently-widened high bits.
    #[cfg(not(any(test, feature = "host")))]
    {
        unsafe extern "C" {
            fn timer_read_time() -> u32;
            static stats_send_time: u32;
            static stats_send_time_high: u32;
        }
        // SAFETY: single u32 reads of Klipper-owned globals (same access the
        // clock-sync FFI `kalico_runtime_clock_sync_request` already performs).
        let c_clock = unsafe {
            let low = timer_read_time();
            let high = stats_send_time_high + ((low < stats_send_time) as u32);
            (u64::from(high) << 32) | u64::from(low)
        };
        CLK_DOMAIN_OFFSET_CYC.store(
            now.wrapping_sub(c_clock) as u32,
            core::sync::atomic::Ordering::Relaxed,
        );
    }
    let after_widen = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_widen_cycles_max,
        after_widen.wrapping_sub(body_start),
    );

    // No segment-arm stage — the new engine manages its own piece advancement.
    let after_arm = after_widen;
    update_max(
        &shared.isr_arm_cycles_max,
        after_arm.wrapping_sub(after_widen),
    );

    let elapsed = after_arm.wrapping_sub(body_start);
    if elapsed > 20000 {
        bump_relaxed(&shared.isr_overrun_count);
        return;
    }

    let crate::state::IsrState { engine, .. } = isr;
    engine.tick(now, shared, storage);

    let body_end = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_eval_cycles_max,
        body_end.wrapping_sub(after_arm),
    );

    let body_cycles = body_end.wrapping_sub(body_start);
    if body_cycles > 30000 {
        shared
            .isr_overrun_count
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

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
        slot.store(val, Ordering::Relaxed);
    }
}

#[inline]
fn bump_relaxed(slot: &core::sync::atomic::AtomicU32) {
    use core::sync::atomic::Ordering;
    let prev = slot.load(Ordering::Relaxed);
    slot.store(prev.wrapping_add(1), Ordering::Relaxed);
}

#[cfg(test)]
mod tests;
