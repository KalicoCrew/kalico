//! Task 14 — `configure_axis` two-phase validation tests.
//!
//! Covers: Phase mode acceptance, out-of-range axis index, valid Pulse
//! success with stepper bindings, OID 0 as a legal SPI binding, and
//! state-preservation on validation failure.

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::error::{KALICO_ERR_INVALID_ARG, KALICO_ERR_MOTION_IN_PROGRESS, KALICO_OK};
use runtime::slot::{NoopIs, NoopPa};
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

type EngineImpl = Engine<NoopPa, NoopIs>;

fn build_engine() -> EngineImpl {
    // 520 MHz matches the H723 Kconfig default. Any positive freq is fine
    // for the configure_* surface (it only touches `cycles_per_second`).
    EngineImpl::new(520_000_000, 40_000)
}

fn no_tmc_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn tmc_binding(oid: u8) -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: oid,
        _pad: [0; 2],
    }
}

// ─── Test 1: Phase mode is accepted ──────────────────────────────────────────

/// `configure_axis(mode=Phase)` must succeed (`KALICO_OK`) and publish
/// the axis mode as `Phase`, with the stepper binding stored.
#[test]
fn phase_mode_accepted() {
    let mut e = build_engine();
    let b = no_tmc_binding();

    let rc = e.configure_axis(0, StepMode::Phase, 0.0125, &[b]);
    assert_eq!(rc, KALICO_OK, "Phase configure_axis must succeed, got {rc}");

    // Axis mode must be Phase and the stepper binding written.
    let axis = &e.stepping_axes[0];
    assert_eq!(
        axis.mode.load(Ordering::Acquire),
        StepMode::Phase as u8,
        "mode must be Phase after Phase configure_axis"
    );
    assert_eq!(axis.steppers.len(), 1, "stepper binding must be stored");
    assert!((axis.microstep_distance - 0.0125).abs() < 1e-9);
}

// ─── Test 2: Out-of-range axis index ─────────────────────────────────────────

/// `configure_axis` with `axis_idx >= N_AXES` must return
/// `KALICO_ERR_INVALID_ARG` without touching any axis.
#[test]
fn out_of_range_axis_idx_rejected() {
    let mut e = build_engine();
    let b = no_tmc_binding();

    // N_AXES is 4, so 4 and 255 are out of range.
    let rc4 = e.configure_axis(4, StepMode::Pulse, 0.01, &[b]);
    assert_eq!(rc4, KALICO_ERR_INVALID_ARG, "axis 4 must be rejected");

    let rc255 = e.configure_axis(255, StepMode::Pulse, 0.01, &[b]);
    assert_eq!(rc255, KALICO_ERR_INVALID_ARG, "axis 255 must be rejected");

    // No axis state should have been modified.
    for axis_idx in 0..runtime::stepping_state::N_AXES {
        let axis = &e.stepping_axes[axis_idx];
        assert!(
            axis.steppers.is_empty(),
            "axis {axis_idx} steppers must be empty after out-of-range rejection"
        );
        assert_eq!(
            axis.microstep_distance, 0.0,
            "axis {axis_idx} microstep_distance must be 0 after rejection"
        );
    }
}

// ─── Test 3: Valid Pulse configuration succeeds ───────────────────────────────

/// A well-formed `configure_axis(mode=Pulse, ...)` call with a single
/// Pulse-only binding must succeed (`KALICO_OK`) and publish all fields.
#[test]
fn valid_pulse_succeeds() {
    let mut e = build_engine();
    let binding = no_tmc_binding();

    let rc = e.configure_axis(1, StepMode::Pulse, 0.00625, &[binding]);
    assert_eq!(rc, KALICO_OK, "valid Pulse configure must succeed");

    let axis = &e.stepping_axes[1];
    assert_eq!(
        axis.mode.load(Ordering::Acquire),
        StepMode::Pulse as u8,
        "mode must be Pulse"
    );
    assert!(
        (axis.microstep_distance - 0.00625).abs() < 1e-9,
        "microstep_distance must be published"
    );
    assert_eq!(axis.steppers.len(), 1, "one stepper binding must be stored");
    assert_eq!(axis.steppers[0].stepper_oid, 0);
    assert!(
        axis.steppers[0].tmc_cs_oid.is_none(),
        "TMC_CS_OID_NONE binding must decode to None"
    );
    // Piece state cleared for clean re-seed.
    assert!(axis.piece.is_none());
    assert_eq!(axis.piece_start_time_cycles, 0);
    assert_eq!(axis.last_step_count, 0);
}

// ─── Test 4: OID 0 is a legal SPI binding ────────────────────────────────────

/// SPI chip-select OID 0 is a valid OID (the first `command_config_spi`
/// object on the firmware command table). It must NOT be treated as
/// `TMC_CS_OID_NONE` (0xFF). Verify that `configure_axis` stores it as
/// `Some(0)`.
#[test]
fn binding_tmc_cs_oid_zero_is_legal() {
    let mut e = build_engine();
    let binding = tmc_binding(0);

    let rc = e.configure_axis(2, StepMode::Pulse, 0.01, &[binding]);
    assert_eq!(rc, KALICO_OK, "OID 0 binding must be accepted");

    let axis = &e.stepping_axes[2];
    assert_eq!(axis.steppers.len(), 1);
    assert_eq!(
        axis.steppers[0].tmc_cs_oid,
        Some(0),
        "OID 0 must decode to Some(0), not None"
    );
}

// ─── Test 5: Switching from Pulse to Phase mode succeeds ─────────────────────

/// A valid `configure_axis(mode=Phase)` call on an axis that was previously
/// configured as `Pulse` must succeed and overwrite the axis state.
#[test]
fn switch_pulse_to_phase_succeeds() {
    let mut e = build_engine();
    let good_binding = no_tmc_binding();

    // Seed a valid Pulse configuration on axis 3.
    let rc_ok = e.configure_axis(3, StepMode::Pulse, 0.05, &[good_binding]);
    assert_eq!(rc_ok, KALICO_OK, "first configure must succeed");

    // Now configure as Phase with a different microstep_distance.
    let phase_binding = no_tmc_binding();
    let rc_phase = e.configure_axis(3, StepMode::Phase, 0.01, &[phase_binding]);
    assert_eq!(rc_phase, KALICO_OK, "Phase configure must succeed");

    // Axis state updated to Phase.
    let axis = &e.stepping_axes[3];
    assert_eq!(
        axis.mode.load(Ordering::Acquire),
        StepMode::Phase as u8,
        "mode must be Phase after Phase configure_axis"
    );
    assert!((axis.microstep_distance - 0.01).abs() < 1e-9, "microstep_distance updated");

    // Validation failures on a subsequent call still leave state untouched.
    let rc_bad = e.configure_axis(3, StepMode::Pulse, 0.0, &[no_tmc_binding()]);
    assert_ne!(rc_bad, KALICO_OK, "zero microstep_distance must still be rejected");
    // Mode stays Phase (the bad call touched nothing).
    assert_eq!(
        e.stepping_axes[3].mode.load(Ordering::Acquire),
        StepMode::Phase as u8,
        "mode must remain Phase after rejected call"
    );
}

// ─── Test 6: Multiple bindings populate all stepper slots ─────────────────────

/// A two-stepper axis (e.g. dual-Z on a CoreXY) must store both bindings
/// with the correct OIDs.
#[test]
fn multiple_bindings_stored() {
    let mut e = build_engine();
    let bindings = [tmc_binding(3), tmc_binding(7)];

    let rc = e.configure_axis(0, StepMode::Pulse, 0.01, &bindings);
    assert_eq!(rc, KALICO_OK);

    let axis = &e.stepping_axes[0];
    assert_eq!(axis.steppers.len(), 2, "both bindings must be stored");
    assert_eq!(axis.steppers[0].tmc_cs_oid, Some(3));
    assert_eq!(axis.steppers[1].tmc_cs_oid, Some(7));
}

// ─── Test 7: KALICO_ERR_MOTION_IN_PROGRESS returned during motion ─────────────

/// `configure_axis` must return `KALICO_ERR_MOTION_IN_PROGRESS` when a
/// segment is currently armed. Seeding `current` is done through the
/// `arm_segment` path; we verify the code path without needing a full
/// curve pool by checking the error code directly from the engine method.
///
/// Implementation note: `Engine::arm_segment` sets `self.current = Some(seg)`
/// even when all handles are `UNUSED_SENTINEL`. That is the exact condition
/// `configure_axis` checks, so we use the sentinel path to arm the engine
/// without needing a populated `CurvePool`.
#[test]
fn motion_in_progress_rejected() {
    use runtime::config::EMode;
    use runtime::curve_pool::{CurveHandle, CurvePool};
    use runtime::segment::{KinematicTag, Segment};

    let mut e = build_engine();

    // Minimal segment: all handles UNUSED_SENTINEL. `arm_segment` still
    // writes `self.current = Some(seg)` so the motion-in-progress gate fires.
    let seg = Segment {
        id: 1,
        x_handle: CurveHandle::UNUSED_SENTINEL,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 1_000_000,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        flags: 0,
        _pad: [0; 1],
        extrusion_ratio: 0.0,
        consumers_remaining: 0,
    };
    let pool = CurvePool::new();
    e.arm_segment(seg, &pool);

    // Engine is now mid-segment.
    assert!(
        e.debug_current_is_some(),
        "engine must have an armed segment"
    );

    let b = no_tmc_binding();
    let rc = e.configure_axis(0, StepMode::Pulse, 0.01, core::slice::from_ref(&b));
    assert_eq!(
        rc, KALICO_ERR_MOTION_IN_PROGRESS,
        "configure_axis must refuse while a segment is armed"
    );
}
