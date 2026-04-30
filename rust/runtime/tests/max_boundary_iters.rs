//! Phase 12.2: Step-5 carryover — `MAX_BOUNDARY_ITERS` test-only injection.
//!
//! With `Q_N = 8` (heapless effective capacity 7) plus the engine's in-flight
//! `current`, the natural reachable boundary-loop iteration count per tick is
//! 8 carries — equal to the bound, so the fault path is dead defense-in-depth
//! through the public producer API. The test injects an artificial starting
//! count via `Engine::inject_iter_count` so the next single-segment carry
//! trips `MAX_BOUNDARY_ITERS` and latches `BoundaryLoopExhausted`.
//!
//! Phase 12 alignment: `MAX_BOUNDARY_ITERS = Q_N - 1 = 7`. Setting
//! `injected_iter_start = 7` means the first carry increments to 8 > 7 → fault.
//!
//! Gating: `inject_iter_count` is exposed under `cfg(any(test, feature =
//! "test-injection"))`. Inline unit tests get it via `cfg(test)`; integration
//! tests like this one get it because the runtime crate is built (as a
//! dependency of the test binary) with `--features test-injection`. The
//! `[cfg_attr(...)]` below requires that feature so a bare `cargo test` (no
//! features) skips this binary cleanly rather than failing to compile.
#![cfg(feature = "test-injection")]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::items_after_statements
)]

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::error::RuntimeError;
use runtime::queue::Q_N;
use runtime::config::EMode;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_FAULT_MARKER, TRACE_RING_N, TraceSample};

const CLOCK_FREQ: u32 = 520_000_000;

#[test]
fn injected_iter_start_trips_boundary_loop_fault() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> = Box::leak(Box::new(Queue::new()));
    let (mut t_producer, mut t_consumer) = trace.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut widen = WidenState::default();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // Inject so the first carry increments iters from 7 → 8, exceeding
    // MAX_BOUNDARY_ITERS = 7 and tripping the fault.
    engine.inject_iter_count(Q_N as u32 - 1);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // Two segments: the first will retire (now > duration), the second is
    // never reached because the boundary loop faults.
    let seg = Segment {
        id: 1,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: tc, // 1 tick duration
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
    };
    let seg2 = Segment {
        id: 2,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: tc,
        t_end: tc * 2,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
    };
    q_producer.enqueue(seg).unwrap();
    q_producer.enqueue(seg2).unwrap();

    // Tick at t = tc * 5 — way past the first segment's t_end so the
    // boundary loop tries to carry. The first carry increments iters
    // from 7 to 8, exceeding the bound.
    let r = engine.tick(
        (tc * 5) as u32,
        &mut widen,
        &pool,
        &mut q_consumer,
        &mut t_producer,
        &shared,
    );

    assert!(matches!(r, Err(RuntimeError::BoundaryLoopExhausted)));
    assert_eq!(engine.status(), RuntimeStatus::Fault);

    // Trace should contain a fault-marker sample.
    let mut found_fault_marker = false;
    while let Some(sample) = t_consumer.dequeue() {
        if sample.flags & TRACE_FLAG_FAULT_MARKER != 0 {
            found_fault_marker = true;
            assert_eq!(sample.segment_id, 1);
        }
    }
    assert!(
        found_fault_marker,
        "boundary-loop fault must emit a TRACE_FLAG_FAULT_MARKER sample"
    );
}

#[test]
fn no_injection_default_path_does_not_fault_on_single_carry() {
    // Sanity: without injection, a single boundary carry (1 segment carries
    // into 1 next segment) is well under MAX_BOUNDARY_ITERS and must not fault.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> = Box::leak(Box::new(Queue::new()));
    let (mut t_producer, _t_consumer) = trace.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut widen = WidenState::default();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // No injection.
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // First segment is a hold so we can avoid loading curve fixtures.
    use runtime::segment::SEGMENT_FLAG_HOLD_SEGMENT;
    let seg = Segment {
        id: 1,
        x_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        y_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        z_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        e_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        t_start: 0,
        t_end: tc, // 1 tick
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: SEGMENT_FLAG_HOLD_SEGMENT,
        _pad: [0; 1],
    };
    let seg2 = Segment {
        id: 2,
        x_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        y_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        z_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        e_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        t_start: tc,
        t_end: tc * 2,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: SEGMENT_FLAG_HOLD_SEGMENT,
        _pad: [0; 1],
    };
    q_producer.enqueue(seg).unwrap();
    q_producer.enqueue(seg2).unwrap();

    // Tick at tc — boundary loop carries seg1 → seg2 (one iteration), no fault.
    let r = engine.tick(
        (tc + 1) as u32,
        &mut widen,
        &pool,
        &mut q_consumer,
        &mut t_producer,
        &shared,
    );
    assert!(r.is_ok(), "single-carry boundary loop must succeed: {r:?}");
    assert_ne!(engine.status(), RuntimeStatus::Fault);
}
