#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::{
    raise_position_count_overflow, raise_step_queue_overflow, raise_steps_per_sample_exceeded,
    raise_unknown_step_mode,
};
use crate::phase_lut::{PHASE_LUT, PHASE_LUT_SIZE};
use crate::state::SharedState;

const _: () = assert!(
    0x3FF < PHASE_LUT_SIZE,
    "PHASE_LUT_SIZE must be > 0x3FF (1023) for the phase-mask indexing in dispatch_phase to be infallible",
);
use crate::step_queue::{StepEntry, StepQueue, peek as queue_peek, push as queue_push};
use crate::stepping_state::{AxisConfig, StepMode};
use crate::sub_sample_timing::{StepTimeInputs, StepTimingResult, compute_step_times};
use crate::tick::bump_relaxed;

// FFI declaration for the C-side SPI write function.
//
// Gate: fire the real C SPI write on bare-metal MCU firmware
// (`not(any(test, feature = "host"))`) AND on Linux-MCU firmware
// (`feature = "mcu-linux"`).  The `mcu-linux` feature implies `host`, so a
// plain `not(any(test, feature = "host"))` would NOT catch a Linux firmware
// build — `mcu-linux` is the explicit discriminator.
#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn phase_stepping_write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16);
}

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn kalico_kick_step_output(axis_idx: u8, cycle_abs: u32);
}

#[inline]
fn kick_per_axis_timer(axis_idx: usize, cycle_abs: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: writes only a timer compare register and an owned-mask bit;
    // same NVIC priority as the step-output ISR so cannot interleave.
    unsafe {
        kalico_kick_step_output(axis_idx as u8, cycle_abs);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = (axis_idx, cycle_abs);
    }
}

pub const DISPLACEMENT_THRESHOLD_MM: f32 = 1e-4;

pub use crate::stepping_state::N_AXES;
pub const AXIS_A: usize = 0;
pub const AXIS_B: usize = 1;
pub const AXIS_Z: usize = 2;
pub const AXIS_E: usize = 3;

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
        _ => {
            raise_unknown_step_mode(shared, axis_idx, mode);
        }
    }
}

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
        raise_steps_per_sample_exceeded(shared, axis_idx, abs_steps);
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

    // If the queue was empty, the consumer timer is parked; kick it to the
    // first step after pushing. Non-empty means it is already scheduled.
    // SAFETY: sole consumer at same NVIC priority — cannot race with peek.
    let was_empty = unsafe { queue_peek(queue_ptr) }.is_none();
    let first_cycle_abs = times.first().copied();

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
            if was_empty && steps_committed > 0 {
                if let Some(wt) = first_cycle_abs {
                    kick_per_axis_timer(axis_idx, wt);
                }
            }
            raise_step_queue_overflow(shared, axis_idx);
            axis.last_step_count = prev_step_count + committed_delta;
            return;
        }
        steps_committed += 1;
    }

    if was_empty && steps_committed > 0 {
        if let Some(wt) = first_cycle_abs {
            kick_per_axis_timer(axis_idx, wt);
        }
    }

    commit_position_count(axis, axis_idx, shared, signed_steps);
}

pub(crate) fn commit_position_count(
    axis: &AxisConfig,
    axis_idx: usize,
    shared: &SharedState,
    delta: i32,
) {
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
        #[allow(clippy::indexing_slicing)] // infallible: phase < PHASE_LUT_SIZE by construction
        let (coil_a, coil_b) = PHASE_LUT[phase as usize];

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
                    // SAFETY: `m < phase_motor_count.min(MAX_STEPPER_OIDS)`, so
                    // `m < MAX_STEPPER_OIDS == phase_slot_idx.len()`.
                    #[allow(clippy::indexing_slicing)]
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

            #[cfg(all(any(test, feature = "host"), not(feature = "mcu-linux")))]
            crate::test_xdirect_capture::record(motor_idx, coil_a, coil_b);

            // SAFETY: `phase_stepping_write_xdirect` accepts any
            // (motor_idx, coil_a, coil_b) triple; motor_idx 0xFF is the
            // "no slot found" sentinel the C side skips gracefully.
            // coil_a/coil_b are PHASE_LUT values, always within i16 range.
            #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
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

#[cfg(test)]
mod tests;
