use core::sync::atomic::Ordering;

use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};

pub const PHASE_PERIOD: i32 = 1024;
pub const PHASE_MASK: i32 = PHASE_PERIOD - 1;

pub struct PhaseQuery {
    pub axis_idx: u8,
    pub mode: u8,
    pub phase: u16,
    pub settled: bool,
}

pub fn shortest_phase_delta(current_phase: u16, target_phase: u16) -> i32 {
    let raw =
        (i32::from(target_phase) - i32::from(current_phase)).rem_euclid(PHASE_PERIOD);
    if raw > PHASE_PERIOD / 2 {
        raw - PHASE_PERIOD
    } else {
        raw
    }
}

pub fn find_stepper(
    axes: &[Option<AxisState>],
    stepper_oid: u8,
) -> Option<(usize, &AxisState, &StepperRef)> {
    for (axis_idx, axis_opt) in axes.iter().enumerate() {
        let Some(axis) = axis_opt else { continue };
        for stepper in &axis.steppers {
            if stepper.stepper_oid == stepper_oid {
                return Some((axis_idx, axis, stepper));
            }
        }
    }
    None
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
fn phase_of(last_step_count: i32, offset: i32) -> u16 {
    (last_step_count.wrapping_add(offset) & PHASE_MASK) as u16
}

pub fn jog_to(
    axes: &[Option<AxisState>],
    shared: &SharedState,
    stepper_oid: u8,
    target_phase: u16,
    max_microsteps_per_sample: u16,
) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    };
    if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
        return -3;
    }
    let pending_target = stepper.phase_offset_target.load(Ordering::Acquire);
    let pending_phase = phase_of(axis.last_step_count, pending_target);
    let delta = shortest_phase_delta(pending_phase, target_phase);
    stepper
        .phase_offset_target
        .store(pending_target.wrapping_add(delta), Ordering::Release);
    shared
        .max_phase_offset_ramp_per_sample
        .store(max_microsteps_per_sample, Ordering::Release);
    0
}

pub fn align_to(axes: &[Option<AxisState>], stepper_oid: u8, target_phase: u16) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        return -1;
    }
    let motion_active = axes
        .iter()
        .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()));
    if motion_active {
        return -2;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        return -1;
    };
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let current_phase = phase_of(axis.last_step_count, current);
    let new_offset =
        current.wrapping_add(shortest_phase_delta(current_phase, target_phase));
    stepper
        .phase_offset_microsteps
        .store(new_offset, Ordering::Release);
    stepper
        .phase_offset_target
        .store(new_offset, Ordering::Release);
    0
}

pub fn query(axes: &[Option<AxisState>], stepper_oid: u8) -> Option<PhaseQuery> {
    let (axis_idx, axis, stepper) = find_stepper(axes, stepper_oid)?;
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    #[allow(clippy::cast_possible_truncation)]
    Some(PhaseQuery {
        axis_idx: axis_idx as u8,
        mode: axis.mode.load(Ordering::Acquire),
        phase: phase_of(axis.last_step_count, current),
        settled: current == target,
    })
}

#[cfg(test)]
mod tests;
