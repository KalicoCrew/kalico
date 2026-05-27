//! Tests for `Engine::arm_segment` (Task 8 of stepping-redesign-finish).
//!
//! Covers the §3.3 + §4.5 invariants:
//! - per-axis state is armed for non-sentinel handles, idle otherwise,
//! - `participating_mask` honours `e_mode` (CoupledToXy E non-participating,
//!   Independent E participating, Travel E excluded),
//! - `pending_mask` initialises equal to `participating_mask`,
//! - `segment_base_e` snapshots `e_accumulator` (f32 truncation),
//! - `ds_xy_segment` resets to 0.0,
//! - `current` is set to `Some(seg)`.
//!
//! Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md.

use runtime::config::EMode;
use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};

type EngineImpl = Engine<NoopPa, NoopIs>;

fn new_engine() -> EngineImpl {
    EngineImpl::new(520_000_000, 40_000)
}

fn make_linear_wire(delta_mm: f32, duration_s: f32) -> WirePiece {
    // Linear (constant-velocity) Bernstein control points 0, d/3, 2d/3, d.
    WirePiece {
        bp0_bits: 0.0f32.to_bits(),
        bp1_bits: (delta_mm / 3.0).to_bits(),
        bp2_bits: (2.0 * delta_mm / 3.0).to_bits(),
        bp3_bits: delta_mm.to_bits(),
        duration_bits: duration_s.to_bits(),
    }
}

/// Construct a default Segment with all handles as `UNUSED_SENTINEL`.
/// Tests adjust only the fields they care about.
fn idle_segment() -> Segment {
    Segment {
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
    }
}

#[test]
fn arms_per_axis_state_for_valid_segment() {
    let mut engine = new_engine();
    let pool = CurvePool::new();
    // Load a linear 10mm/25µs into slot 0:
    let handle = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 25e-6)])
        .expect("alloc");
    let mut seg = idle_segment();
    seg.x_handle = handle;
    seg.t_start = 12_345;

    engine.arm_segment(seg, &pool);

    assert!(engine.stepping_axes[0].curve_handle.is_some());
    assert_eq!(engine.stepping_axes[0].piece_cursor, 0);
    assert!(engine.stepping_axes[0].piece.is_some());
    assert_eq!(engine.stepping_axes[0].piece_start_time_cycles, 12_345);
    assert!(engine.stepping_axes[1].piece.is_none());
    assert!(engine.stepping_axes[2].piece.is_none());
    assert!(engine.stepping_axes[3].piece.is_none());
    assert_eq!(engine.participating_mask, 0b0001);
    assert_eq!(engine.pending_mask, 0b0001);
}

#[test]
fn idle_axis_stays_none_for_unused_sentinel() {
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let handle = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 25e-6)])
        .expect("alloc");
    let mut seg = idle_segment();
    seg.x_handle = handle;
    // y, z, e remain UNUSED_SENTINEL

    engine.arm_segment(seg, &pool);

    assert!(engine.stepping_axes[1].curve_handle.is_none());
    assert!(engine.stepping_axes[2].curve_handle.is_none());
    assert!(engine.stepping_axes[3].curve_handle.is_none());
}

#[test]
fn participating_mask_for_coupled_e_includes_e_bit() {
    // After E-unification: EMode::CoupledToXy no longer excludes E from
    // participating_mask. All axes with a valid curve handle participate,
    // regardless of e_mode. The host pre-computes E as a regular Bezier.
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let x = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 25e-6)])
        .expect("alloc x");
    let e = pool
        .try_alloc_and_load(1, &[make_linear_wire(0.5, 25e-6)])
        .expect("alloc e");
    let mut seg = idle_segment();
    seg.x_handle = x;
    seg.e_handle = e;
    seg.e_mode = EMode::CoupledToXy;
    seg.extrusion_ratio = 0.05;

    engine.arm_segment(seg, &pool);

    // Both X and E have handles → both participate in retire bookkeeping.
    assert_eq!(engine.participating_mask, 0b1001); // X + E
    assert_eq!(engine.pending_mask, 0b1001);
    assert!(engine.stepping_axes[3].curve_handle.is_some());
}

#[test]
fn participating_mask_for_independent_e_includes_e_bit() {
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let x = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 25e-6)])
        .expect("alloc x");
    let e = pool
        .try_alloc_and_load(1, &[make_linear_wire(0.5, 25e-6)])
        .expect("alloc e");
    let mut seg = idle_segment();
    seg.x_handle = x;
    seg.e_handle = e;
    seg.e_mode = EMode::Independent;

    engine.arm_segment(seg, &pool);

    assert_eq!(engine.participating_mask, 0b1001); // X + E
    assert_eq!(engine.pending_mask, 0b1001);
}

#[test]
fn participating_mask_for_travel_excludes_e_bit() {
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let x = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 25e-6)])
        .expect("alloc x");
    let mut seg = idle_segment();
    seg.x_handle = x;
    // e_handle stays UNUSED_SENTINEL
    seg.e_mode = EMode::Travel;

    engine.arm_segment(seg, &pool);

    assert_eq!(engine.participating_mask, 0b0001);
    assert_eq!(engine.pending_mask, 0b0001);
}

// segment_base_e_snapshotted_from_accumulator and ds_xy_segment_resets_to_zero
// were removed: the E-accumulator and ds_xy_segment fields no longer exist.
// E is now a regular Bezier axis pre-computed by the host; the MCU evaluates
// all four axes (A/B/Z/E) uniformly with no per-segment arc-length integration.
