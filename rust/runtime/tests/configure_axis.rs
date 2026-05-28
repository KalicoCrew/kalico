//! Stepping-redesign Task 11 — unit-level coverage of the three
//! `configure_*` methods on `Engine`. These exercise the validation +
//! state-publish behaviour directly through the engine surface; the
//! FFI-shim layer (`kalico_runtime_configure_axis` etc.) is covered at
//! Task 18 integration time.
//!
//! Task 14 extends `configure_axis` to take a `&[StepperBindingRust]`
//! slice instead of the old `extrusion_per_xy_mm + stepper_count` pair.
//! Tests here cover the Pulse path; the Phase-rejection path and the
//! stepper-binding population are covered in `configure_axis_two_phase.rs`.
//!
//! Why an integration test (`tests/`) instead of `#[cfg(test)] mod tests`
//! inside `engine.rs`: the engine module is closed to in-file tests
//! (4036+ lines, no existing `mod tests`), and the test-only `Default`
//! impl on `Engine<P, I>` makes construction from a sibling crate test
//! ergonomic without reaching into private fields.

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

/// Compile-time guard: the unified per-axis array has exactly 8 entries.
const _ASSERT_MAX_AXES: () = assert!(MAX_AXES == 8);

fn new_engine() -> Engine {
    // 520 MHz matches the H723 Kconfig default; any positive freq works
    // for the configure_* surface (it touches cycles_per_second only as
    // a cached scalar).
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

    // X axis (index 0): Pulse mode with a 0.0125 mm microstep
    // (TMC5160 256-microstep, 1.8° motor on 20-tooth GT2 belt ≈ 0.0125 mm).
    // Task 14: pass a single Pulse-only binding (TMC_CS_OID_NONE).
    let binding = pulse_binding();
    let rc = e.configure_axis(
        0,
        StepMode::Pulse,
        0.0125, // microstep_distance
        256,    // ring_depth
        &[binding],
        512, // total_ring_pieces
    );
    assert_eq!(rc, 0, "configure_axis returned non-zero");

    let axis = e.stepping_axes[0]
        .as_ref()
        .expect("axis should be configured");
    assert_eq!(axis.mode.load(Ordering::Acquire), StepMode::Pulse as u8);
    assert!((axis.microstep_distance - 0.0125).abs() < 1e-9);
    // After configure, no piece is active and counters are zeroed so the
    // next segment-arrival path can re-seed cleanly.
    assert!(!axis.has_piece);
    assert_eq!(axis.last_step_count, 0);
    // Stepper binding populated.
    assert_eq!(axis.steppers.len(), 1);
    assert_eq!(axis.steppers[0].stepper_oid, 0);
    assert!(axis.steppers[0].tmc_cs_oid.is_none());
}

#[test]
fn configure_axis_rejects_invalid_inputs() {
    let mut e = new_engine();
    let b = pulse_binding();

    // Out-of-range axis index.
    assert_ne!(e.configure_axis(8, StepMode::Pulse, 0.01, 64, &[b], 512), 0);
    assert_ne!(
        e.configure_axis(255, StepMode::Pulse, 0.01, 64, &[b], 512),
        0
    );

    // Non-finite microstep_distance.
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, f32::NAN, 64, &[b], 512),
        0
    );
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, f32::INFINITY, 64, &[b], 512),
        0
    );
    // Zero / negative microstep_distance.
    assert_ne!(e.configure_axis(0, StepMode::Pulse, 0.0, 64, &[b], 512), 0);
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, -0.01, 64, &[b], 512),
        0
    );
    // Phase mode with a valid microstep_distance is now accepted.
    assert_eq!(e.configure_axis(0, StepMode::Phase, 0.01, 64, &[b], 512), 0);
}

// configure_kinematics and configure_pressure_advance are now no-op stubs —
// k_xy and advance_accel/advance_decel fields were removed when the
// E-follows-XY arc-length integrator was removed. The stubs still validate
// their inputs and return 0 on success / -1 on invalid, preserving the FFI ABI.

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
