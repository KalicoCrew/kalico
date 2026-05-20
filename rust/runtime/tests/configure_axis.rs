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
use runtime::slot::{NoopIs, NoopPa};
use runtime::stepping_state::{N_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

type EngineImpl = Engine<NoopPa, NoopIs>;

/// Compile-time guard: the unified per-axis array has exactly 4 entries.
/// If this ever changes (e.g. AB-CoreXY-with-independent-A2-B2 industrial
/// configs grow it to 8), every site that hand-initializes a 4-element
/// array literal in `Engine::new` must be revisited.
const _ASSERT_N_AXES: () = assert!(N_AXES == 4);

fn new_engine() -> EngineImpl {
    // 520 MHz matches the H723 Kconfig default; any positive freq works
    // for the configure_* surface (it touches cycles_per_second only as
    // a cached scalar).
    EngineImpl::new(520_000_000)
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust { tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 3] }
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
        &[binding],
    );
    assert_eq!(rc, 0, "configure_axis returned non-zero");

    let axis = &e.stepping_axes[0];
    assert_eq!(axis.mode.load(Ordering::Acquire), StepMode::Pulse as u8);
    assert!((axis.microstep_distance - 0.0125).abs() < 1e-9);
    // After configure, no piece is active and counters are zeroed so the
    // next segment-arrival path can re-seed cleanly.
    assert!(axis.piece.is_none());
    assert_eq!(axis.piece_start_time_cycles, 0);
    assert_eq!(axis.last_step_count, 0);
    assert!(axis.curve_handle.is_none());
    assert_eq!(axis.piece_cursor, 0);
    // Stepper binding populated.
    assert_eq!(axis.steppers.len(), 1);
    assert!(axis.steppers[0].tmc_cs_oid.is_none());
}

#[test]
fn configure_axis_rejects_invalid_inputs() {
    let mut e = new_engine();
    let b = pulse_binding();

    // Out-of-range axis index.
    assert_ne!(e.configure_axis(4, StepMode::Pulse, 0.01, &[b]), 0);
    assert_ne!(e.configure_axis(255, StepMode::Pulse, 0.01, &[b]), 0);

    // Non-finite microstep_distance.
    assert_ne!(e.configure_axis(0, StepMode::Pulse, f32::NAN, &[b]), 0);
    assert_ne!(
        e.configure_axis(0, StepMode::Pulse, f32::INFINITY, &[b]),
        0,
    );
    // Zero / negative microstep_distance.
    assert_ne!(e.configure_axis(0, StepMode::Pulse, 0.0, &[b]), 0);
    assert_ne!(e.configure_axis(0, StepMode::Pulse, -0.01, &[b]), 0);
    // Phase mode — rejected with KALICO_ERR_PHASE_MODE_NOT_AVAILABLE.
    assert_ne!(e.configure_axis(0, StepMode::Phase, 0.01, &[b]), 0);
}

#[test]
fn configure_kinematics_accepts_cartesian_and_corexy() {
    let mut e = new_engine();

    // Cartesian.
    assert_eq!(e.configure_kinematics(1.0), 0);
    assert!((e.k_xy - 1.0).abs() < 1e-9);

    // CoreXY: 1 / sqrt(2) ≈ 0.7071067811865476.
    let inv_sqrt2 = 1.0_f32 / 2.0_f32.sqrt();
    assert_eq!(e.configure_kinematics(inv_sqrt2), 0);
    assert!((e.k_xy - inv_sqrt2).abs() < 1e-7);
}

#[test]
fn configure_kinematics_rejects_invalid_inputs() {
    let mut e = new_engine();
    let baseline = e.k_xy;

    assert_ne!(e.configure_kinematics(0.0), 0);
    assert_ne!(e.configure_kinematics(-1.0), 0);
    assert_ne!(e.configure_kinematics(f32::NAN), 0);
    assert_ne!(e.configure_kinematics(f32::INFINITY), 0);

    // State unchanged after each rejection.
    assert!((e.k_xy - baseline).abs() < 1e-9);
}

#[test]
fn configure_pressure_advance_accepts_symmetric_and_asymmetric() {
    let mut e = new_engine();

    // Symmetric Klipper-style PA: same K on accel and decel.
    assert_eq!(e.configure_pressure_advance(0.05, 0.05), 0);
    assert_eq!(e.advance_accel, 0.05);
    assert_eq!(e.advance_decel, 0.05);

    // Asymmetric (Kalico bleeding-edge Step 9).
    assert_eq!(e.configure_pressure_advance(0.08, 0.04), 0);
    assert_eq!(e.advance_accel, 0.08);
    assert_eq!(e.advance_decel, 0.04);

    // PA off.
    assert_eq!(e.configure_pressure_advance(0.0, 0.0), 0);
    assert_eq!(e.advance_accel, 0.0);
    assert_eq!(e.advance_decel, 0.0);
}

#[test]
fn configure_pressure_advance_rejects_invalid_inputs() {
    let mut e = new_engine();
    let _ = e.configure_pressure_advance(0.05, 0.05); // seed baseline

    // Non-finite either side.
    assert_ne!(e.configure_pressure_advance(f32::NAN, 0.0), 0);
    assert_ne!(e.configure_pressure_advance(0.0, f32::INFINITY), 0);

    // Negative — PA is never physically negative.
    assert_ne!(e.configure_pressure_advance(-0.01, 0.0), 0);
    assert_ne!(e.configure_pressure_advance(0.0, -0.01), 0);
}
