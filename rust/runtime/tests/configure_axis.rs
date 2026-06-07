#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const _ASSERT_MAX_AXES: () = assert!(MAX_AXES == 8);

fn new_engine() -> Engine {
    Engine::new(520_000_000, 40_000)
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

#[test]
fn configure_axis_publishes_mode_and_scalars() {
    let mut e = new_engine();

    let binding = pulse_binding();
    let rc = e.configure_axis(0, StepMode::Pulse, 0.0125, 256, &[binding], 512);
    assert_eq!(rc, 0, "configure_axis returned non-zero");

    let axis = e.stepping_axes[0]
        .as_ref()
        .expect("axis should be configured");
    assert_eq!(axis.mode.load(Ordering::Acquire), StepMode::Pulse as u8);
    assert!((axis.microstep_distance - 0.0125).abs() < 1e-9);
    assert!(axis.armed.is_none());
    assert_eq!(axis.last_step_count, 0);
    assert_eq!(axis.steppers.len(), 1);
    assert_eq!(axis.steppers[0].stepper_oid, 0);
    assert!(axis.steppers[0].tmc_cs_oid.is_none());
}

#[test]
fn configure_axis_rejects_invalid_inputs() {
    let mut e = new_engine();
    let b = pulse_binding();

    assert_ne!(e.configure_axis(8, StepMode::Pulse, 0.01, 64, &[b], 512), 0);
    assert_ne!(
        e.configure_axis(255, StepMode::Pulse, 0.01, 64, &[b], 512),
        0
    );

    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, f32::NAN, 64, &[b], 512),
        0
    );
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, f32::INFINITY, 64, &[b], 512),
        0
    );
    assert_ne!(e.configure_axis(0, StepMode::Pulse, 0.0, 64, &[b], 512), 0);
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, -0.01, 64, &[b], 512),
        0
    );
    assert_eq!(e.configure_axis(0, StepMode::Phase, 0.01, 64, &[b], 512), 0);
}

#[test]
fn configure_kinematics_returns_ok_for_valid_inputs() {
    let mut e = new_engine();
    assert_eq!(e.configure_kinematics(1.0), 0);
    let inv_sqrt2 = 1.0_f32 / 2.0_f32.sqrt();
    assert_eq!(e.configure_kinematics(inv_sqrt2), 0);
}

#[test]
fn configure_kinematics_rejects_invalid_inputs() {
    let mut e = new_engine();
    assert_ne!(e.configure_kinematics(0.0), 0);
    assert_ne!(e.configure_kinematics(-1.0), 0);
    assert_ne!(e.configure_kinematics(f32::NAN), 0);
    assert_ne!(e.configure_kinematics(f32::INFINITY), 0);
}

#[test]
fn configure_pressure_advance_returns_ok_for_valid_inputs() {
    let mut e = new_engine();
    assert_eq!(e.configure_pressure_advance(0.05, 0.05), 0);
    assert_eq!(e.configure_pressure_advance(0.08, 0.04), 0);
    assert_eq!(e.configure_pressure_advance(0.0, 0.0), 0);
}

#[test]
fn configure_pressure_advance_rejects_invalid_inputs() {
    let mut e = new_engine();
    assert_ne!(e.configure_pressure_advance(f32::NAN, 0.0), 0);
    assert_ne!(e.configure_pressure_advance(0.0, f32::INFINITY), 0);
    assert_ne!(e.configure_pressure_advance(-0.01, 0.0), 0);
    assert_ne!(e.configure_pressure_advance(0.0, -0.01), 0);
}
