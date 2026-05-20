//! Tests for the per-sample post-pass exhaustion check (§4.4 + §4.5) and the
//! Phase-5 retire (Task 10 of stepping-redesign-finish).
//!
//! Coverage:
//! - Simultaneous A+B exhaustion does NOT fault — `pending_mask` drops to 0
//!   in the same sample and `retire_if_complete` fires.
//! - A-exhausts-while-B-pending DOES fault with `PieceAdvanceUnderflow`.
//! - `CoupledToXy` E exhausting while XY pending does NOT fault — E is not
//!   in `participating_mask` under `CoupledToXy`.
//!
//! Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md`.

use core::sync::atomic::Ordering;
use heapless::spsc::Queue;

use runtime::config::EMode;
use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_SEGMENT_END, TRACE_RING_N, TraceSample};

type EngineImpl = Engine<NoopPa, NoopIs>;

const CLOCK_FREQ: u32 = 520_000_000;

fn new_engine() -> EngineImpl {
    EngineImpl::new(CLOCK_FREQ)
}

fn make_linear_wire(delta_mm: f32, duration_s: f32) -> WirePiece {
    WirePiece {
        bp0_bits: 0.0f32.to_bits(),
        bp1_bits: (delta_mm / 3.0).to_bits(),
        bp2_bits: (2.0 * delta_mm / 3.0).to_bits(),
        bp3_bits: delta_mm.to_bits(),
        duration_bits: duration_s.to_bits(),
    }
}

fn idle_segment(id: u32) -> Segment {
    Segment {
        id,
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

/// Helper: simulate a per-axis curve exhaustion by clearing the
/// `curve_handle` on the named axis index. Mirrors what
/// `advance_piece_if_needed` does when the cursor walks off the end of the
/// curve.
fn mark_axis_exhausted(engine: &mut EngineImpl, axis_idx: usize) {
    engine.stepping_axes[axis_idx].curve_handle = None;
    engine.stepping_axes[axis_idx].piece = None;
}

/// Allocate a trace producer + consumer pair backed by a leaked queue.
/// Returns the pair as `'static` references so the test body owns them
/// without lifetime gymnastics.
fn make_trace_pair() -> (
    heapless::spsc::Producer<'static, TraceSample, TRACE_RING_N>,
    heapless::spsc::Consumer<'static, TraceSample, TRACE_RING_N>,
) {
    let queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    queue.split()
}

#[test]
fn simultaneous_xy_exhaustion_does_not_fault() {
    // Arm a segment with X and Y curves of equal length. Then simulate
    // both axes exhausting on the same sample. Post-pass must NOT fault
    // (pending_mask drops to 0 → retire path); retire then clears
    // `current` and publishes `retired_through_segment_id`.
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let (mut t_producer, mut t_consumer) = make_trace_pair();

    let hx = pool
        .try_alloc_and_load(0, &[make_linear_wire(1.0, 25e-6)])
        .expect("alloc x");
    let hy = pool
        .try_alloc_and_load(1, &[make_linear_wire(1.0, 25e-6)])
        .expect("alloc y");
    let mut seg = idle_segment(7);
    seg.x_handle = hx;
    seg.y_handle = hy;
    // Travel: E not participating, only X+Y bits set.
    seg.e_mode = EMode::Travel;

    engine.arm_segment(seg, &pool);
    assert_eq!(engine.participating_mask, 0b0011, "X+Y participating");
    assert_eq!(engine.pending_mask, 0b0011);

    // Both X and Y "advance off the end" in the same sample.
    mark_axis_exhausted(&mut engine, 0);
    mark_axis_exhausted(&mut engine, 1);

    engine.post_pass_exhaustion(&shared);

    // Pending mask drops to 0 in one go → no fault.
    assert_eq!(engine.pending_mask, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "simultaneous exhaustion of every participating axis must NOT fault",
    );

    // Retire should now fire.
    let retired = engine.retire_if_complete(&shared, &mut t_producer);
    assert!(retired, "retire_if_complete must fire when pending_mask==0");
    assert!(!engine.debug_current_is_some(), "current cleared on retire");
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        7,
        "retired_through_segment_id published the segment id",
    );

    // SEGMENT_END trace sample observable.
    let mut found_end = false;
    while let Some(s) = t_consumer.dequeue() {
        if s.flags & TRACE_FLAG_SEGMENT_END != 0 && s.segment_id == 7 {
            found_end = true;
        }
    }
    assert!(found_end, "SEGMENT_END trace for seg.id=7 must be enqueued");
}

#[test]
fn early_exhaustion_x_while_y_pending_faults() {
    // X exhausts but Y is still in flight on this sample. Post-pass
    // must latch `PieceAdvanceUnderflow` with axis_idx=0 in
    // `fault_detail` (bits 16..24).
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    let hx = pool
        .try_alloc_and_load(0, &[make_linear_wire(1.0, 25e-6)])
        .expect("alloc x");
    let hy = pool
        .try_alloc_and_load(1, &[make_linear_wire(10.0, 250e-6)])
        .expect("alloc y");
    let mut seg = idle_segment(11);
    seg.x_handle = hx;
    seg.y_handle = hy;
    seg.e_mode = EMode::Travel;

    engine.arm_segment(seg, &pool);
    assert_eq!(engine.participating_mask, 0b0011);

    // Only X exhausts; Y still has a curve_handle.
    mark_axis_exhausted(&mut engine, 0);

    engine.post_pass_exhaustion(&shared);

    // Y still pending; X bit cleared.
    assert_eq!(
        engine.pending_mask, 0b0010,
        "Y bit still pending after X exhaustion",
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceAdvanceUnderflow.as_i32(),
        "early exhaustion of X while Y pending must latch PieceAdvanceUnderflow",
    );
    // axis_idx encoded into bits 16..24 of fault_detail.
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        (detail >> 16) & 0xFF,
        0,
        "axis_idx 0 (X) encoded into fault_detail",
    );
}

#[test]
fn coupled_e_exhausting_early_does_not_fault() {
    // CoupledToXy: only X+Y are participating; E is a follower (not in
    // `participating_mask`). Even if E's curve "exhausts" first, the
    // post-pass should not fault — E isn't tracked in `pending_mask`.
    let mut engine = new_engine();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    let hx = pool
        .try_alloc_and_load(0, &[make_linear_wire(10.0, 250e-6)])
        .expect("alloc x");
    let hy = pool
        .try_alloc_and_load(1, &[make_linear_wire(10.0, 250e-6)])
        .expect("alloc y");
    let he = pool
        .try_alloc_and_load(2, &[make_linear_wire(0.5, 25e-6)])
        .expect("alloc e");
    let mut seg = idle_segment(13);
    seg.x_handle = hx;
    seg.y_handle = hy;
    seg.e_handle = he;
    seg.e_mode = EMode::CoupledToXy;
    seg.extrusion_ratio = 0.05;

    engine.arm_segment(seg, &pool);
    // Only X+Y, not E.
    assert_eq!(engine.participating_mask, 0b0011, "CoupledToXy: X+Y only");

    // E exhausts early.
    mark_axis_exhausted(&mut engine, 3);

    engine.post_pass_exhaustion(&shared);

    // pending_mask unchanged: E isn't in participating_mask.
    assert_eq!(
        engine.pending_mask, 0b0011,
        "E exhaustion under CoupledToXy must NOT clear participating bits",
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "CoupledToXy E exhaustion must NOT raise PieceAdvanceUnderflow",
    );
}
