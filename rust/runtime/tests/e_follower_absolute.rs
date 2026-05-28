#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Tests for Phase-3 E follower math — absolute-E position model.
//!
//! Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §4.6.
//!
//! These exercise `runtime::tick::evaluate_e_axis` directly, which is the
//! pure function Phase 3 of `runtime_tick_sample` delegates to. The
//! evaluator returns absolute E position in mm:
//!
//!   p_end_E = engine_segment_base_e + segment_local
//!
//! where `segment_local` dispatches on `Segment::e_mode`:
//!   - CoupledToXy: intrinsic + extrusion_ratio * ds_xy_segment
//!                  + pa_k * extrusion_ratio * v_xy_this
//!   - Independent: intrinsic (no XY contribution)
//!   - Travel:      0
//!
//! `intrinsic` evaluates `axis_e.piece` if present, else 0.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;

use runtime::config::EMode;
use runtime::curve_pool::CurveHandle;
use runtime::monomial::{BezierPieceMonomial, bernstein_to_monomial_with_duration};
use runtime::segment::{KinematicTag, Segment};
use runtime::stepping_state::{AxisConfig, StepMode, StepperRef};
use runtime::tick::evaluate_e_axis;

// ---------------------------------------------------------------------
// Test helpers — minimal AxisConfig + Segment fixtures.
// ---------------------------------------------------------------------

fn make_stepper() -> StepperRef {
    StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn make_axis() -> AxisConfig {
    let mut steppers: Vec<StepperRef, 4> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(StepMode::Pulse as u8),
        steppers,
        curve_handle: None,
        piece_cursor: 0,
        piece: None::<BezierPieceMonomial>,
        piece_start_time_cycles: 0,
        last_step_count: 0,
        microstep_distance: 0.0125,
    }
}

fn segment_with(e_mode: EMode, extrusion_ratio: f32) -> Segment {
    Segment {
        id: 1,
        x_handle: CurveHandle::UNUSED_SENTINEL,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 1_000_000,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode,
        flags: 0,
        _pad: [0; 1],
        extrusion_ratio,
        consumers_remaining: 0,
    }
}

const CPS: f32 = 520_000_000.0;

// ---------------------------------------------------------------------
// Spec §4.6 — CoupledToXy follower term with no intrinsic E piece.
// ---------------------------------------------------------------------

/// `e_mode == CoupledToXy`, no E intrinsic curve (handle would be
/// `UNUSED_SENTINEL` upstream). The follower picks up
/// `extrusion_ratio * ds_xy_segment`.
#[test]
fn coupled_to_xy_intrinsic_zero_e_handle_sentinel() {
    let mut axis_e = make_axis(); // piece = None
    let seg = segment_with(EMode::CoupledToXy, 0.05);
    let engine_segment_base_e: f32 = 0.0;
    let ds_xy_segment: f32 = 0.1; // 100 µm of XY arc this segment so far
    let p = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        /* pa_k */ 0.0,
        /* v_xy_this */ 0.0,
        /* t_sample_end_global */ 0.0,
        CPS,
    );
    // segment_base_e (0) + extrusion_ratio (0.05) * ds_xy_segment (0.1)
    //                                          = 0.005 mm
    assert!((p - 0.005).abs() < 1e-7, "got {p}");
}

// ---------------------------------------------------------------------
// Spec §4.6 — continuity across consecutive segments.
// ---------------------------------------------------------------------

/// First segment's final E delta has been rolled into the engine's
/// `e_accumulator` (here modelled as a base of 0.05). The second
/// segment starts with `engine_segment_base_e == 0.05` and its own
/// `ds_xy_segment` counter resets to zero at arm time.
#[test]
fn coupled_to_xy_position_continuous_across_segments() {
    let mut axis_e = make_axis();
    let seg = segment_with(EMode::CoupledToXy, 0.05);
    // Engine retired the previous CoupledToXy segment; e_accumulator
    // (snapshot into segment_base_e here) holds the carried-forward
    // absolute E.
    let engine_segment_base_e: f32 = 0.05;
    let ds_xy_segment: f32 = 0.5;
    let p = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        0.0,
        0.0,
        0.0,
        CPS,
    );
    // 0.05 (carry) + 0.05 * 0.5 (this segment) = 0.075 mm — monotonic
    // forward, no backwards motion.
    assert!((p - 0.075).abs() < 1e-7, "got {p}");
    assert!(p > engine_segment_base_e, "E must advance, not regress");
}

// ---------------------------------------------------------------------
// Spec §4.6 — Independent: no XY contribution, intrinsic E only.
// ---------------------------------------------------------------------

/// `e_mode == Independent`. The E NURBS owns motion (retract / prime /
/// filament change); XY arc length must be ignored even when nonzero.
#[test]
fn independent_mode_no_xy_contribution() {
    let mut axis_e = make_axis();
    // Linear 2 mm in 25 µs (the duration is arbitrary — we evaluate at
    // `t_local == duration` to land at the end of the piece).
    let piece = bernstein_to_monomial_with_duration([0.0, 2.0 / 3.0, 4.0 / 3.0, 2.0], 25e-6);
    axis_e.piece = Some(piece);
    axis_e.piece_start_time_cycles = 0;

    let seg = segment_with(EMode::Independent, /* ratio (ignored) */ 0.05);
    let engine_segment_base_e: f32 = 0.10; // carried forward
    let ds_xy_segment: f32 = 10.0; // 10 mm XY arc — would massively
    // perturb output in CoupledToXy.
    let t_sample_end_global: f32 = 25e-6; // exactly the piece end

    let p = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        /* pa_k */ 0.1,
        /* v_xy_this */ 100.0,
        t_sample_end_global,
        CPS,
    );
    // base (0.10) + intrinsic (2.0). XY/PA contribute nothing in
    // Independent mode.
    assert!((p - 2.10).abs() < 1e-4, "got {p}");
}

// ---------------------------------------------------------------------
// Spec §4.6 — Travel: zero E motion regardless of intrinsic / XY state.
// ---------------------------------------------------------------------

/// `e_mode == Travel`. Even a non-zero intrinsic E piece and a non-zero
/// XY arc length must not produce E motion this segment.
#[test]
fn travel_mode_zero_motion() {
    let mut axis_e = make_axis();
    // Spurious E intrinsic — must be ignored.
    let piece = bernstein_to_monomial_with_duration([0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0], 25e-6);
    axis_e.piece = Some(piece);

    let seg = segment_with(EMode::Travel, /* ratio (ignored) */ 0.5);
    let engine_segment_base_e: f32 = 0.25;
    let ds_xy_segment: f32 = 5.0; // would matter only in CoupledToXy
    let p = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        /* pa_k */ 0.1,
        /* v_xy_this */ 50.0,
        /* t_sample_end_global */ 25e-6,
        CPS,
    );
    // Travel returns engine_segment_base_e unchanged — segment_local
    // is hard-zeroed.
    assert!((p - engine_segment_base_e).abs() < 1e-7, "got {p}");
}

// ---------------------------------------------------------------------
// Spec §4.6 — pressure-advance sign-of-acceleration.
// ---------------------------------------------------------------------

/// CoupledToXy with non-zero `pa_k`. On acceleration the PA correction
/// pushes extra E forward; on deceleration the same coefficient applied
/// with a negative `v_xy_this` (relative to the caller's sign
/// convention) pulls it back. The caller (Phase 3) selects pa_k from
/// `advance_accel` vs `advance_decel` based on `vdot_xy_accelerating`.
/// Here we feed positive vs negative pa_k directly to capture the
/// sign-of-acceleration switching cleanly.
#[test]
fn pa_correction_signed_by_acceleration() {
    let mut axis_e = make_axis(); // piece = None
    let seg = segment_with(EMode::CoupledToXy, 0.05);
    let engine_segment_base_e: f32 = 0.0;
    let ds_xy_segment: f32 = 0.0; // isolate PA term
    let v_xy_this: f32 = 100.0; // mm/s

    // Accelerating: pa_k = advance_accel = +5 ms.
    let p_accel = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        /* pa_k */ 0.005,
        v_xy_this,
        0.0,
        CPS,
    );
    // 0 + 0.005 * 0.05 * 100 = 0.025 mm forward (PA push).
    assert!((p_accel - 0.025).abs() < 1e-7, "got {p_accel}");
    assert!(p_accel > 0.0, "accelerating PA must be additive");

    // Decelerating: pa_k = advance_decel, but the *coefficient* itself
    // is positive in spec terms — Phase 3's PA polarity comes from the
    // selection, not the sign. To exercise the sign-flip path in the
    // evaluator we feed pa_k negative as a stand-in for "deceleration
    // pulls E back."
    let p_decel = evaluate_e_axis(
        &mut axis_e,
        Some(&seg),
        engine_segment_base_e,
        ds_xy_segment,
        /* pa_k */ -0.005,
        v_xy_this,
        0.0,
        CPS,
    );
    assert!((p_decel - (-0.025)).abs() < 1e-7, "got {p_decel}");
    assert!(p_decel < 0.0, "decelerating PA must be subtractive");

    // Symmetry: equal-and-opposite around the base when pa_k flips
    // sign with everything else held fixed.
    assert!((p_accel + p_decel).abs() < 1e-7);
}

// ---------------------------------------------------------------------
// Spec §4.6 — `current == None` shortcut.
// ---------------------------------------------------------------------

/// No segment armed: evaluator returns `engine_segment_base_e` so E
/// holds its accumulated absolute position. This is the cold-boot /
/// between-segments state.
#[test]
fn no_current_segment_returns_base() {
    let mut axis_e = make_axis();
    let engine_segment_base_e: f32 = 1.234;
    let p = evaluate_e_axis(
        &mut axis_e,
        /* current */ None,
        engine_segment_base_e,
        /* ds_xy_segment */ 99.0,
        /* pa_k */ 0.1,
        /* v_xy_this */ 100.0,
        /* t_sample_end_global */ 1.0,
        CPS,
    );
    assert!((p - engine_segment_base_e).abs() < 1e-7, "got {p}");
}
