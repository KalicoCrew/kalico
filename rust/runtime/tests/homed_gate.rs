//! Homed gate tests for the engine evaluator.
//! Validates that the engine refuses to run when not homed.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::config::EMode;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::error::RuntimeError;
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_FAULT_MARKER, TRACE_RING_N, TraceSample};

mod fixtures;

const CLOCK_FREQ: u32 = 520_000_000;

#[test]
fn engine_refuses_to_run_when_not_homed() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (mut t_producer, mut t_consumer) = trace.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut widen = WidenState::default();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // homed defaults to false; open stream to trigger the fault path.
    shared.stream_open.store(true, Ordering::Release);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Load a valid curve and push a segment.
    let (deg, knots, cps) = fixtures::linear_scalar(0.0, 10.0);
    let x_handle = fixtures::load_scalar(&pool, 0, deg, &knots, &cps);

    q_producer
        .enqueue(Segment {
            id: 1,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: tc * 4,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    let r = engine.tick(0, &mut widen, &pool, &mut q_consumer, &mut t_producer, &shared);
    assert!(matches!(r, Err(RuntimeError::NotHomed)), "expected NotHomed, got {r:?}");
    assert_eq!(engine.status(), RuntimeStatus::Fault);

    // Fault marker should be in trace.
    let mut found_fault = false;
    while let Some(sample) = t_consumer.dequeue() {
        if sample.flags & TRACE_FLAG_FAULT_MARKER != 0 {
            found_fault = true;
        }
    }
    assert!(found_fault, "NotHomed fault should emit a fault marker trace sample");
}

#[test]
fn engine_runs_when_homed() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (mut t_producer, _t_consumer) = trace.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut widen = WidenState::default();
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // Set homed = true.
    shared.homed.store(true, Ordering::Release);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    let (deg, knots, cps) = fixtures::linear_scalar(0.0, 10.0);
    let x_handle = fixtures::load_scalar(&pool, 0, deg, &knots, &cps);

    q_producer
        .enqueue(Segment {
            id: 1,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: tc * 4,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    let r = engine.tick(0, &mut widen, &pool, &mut q_consumer, &mut t_producer, &shared);
    assert!(r.is_ok(), "engine should run when homed: {r:?}");
    assert_eq!(engine.status(), RuntimeStatus::Running);
}

#[test]
fn engine_idle_when_not_homed_and_stream_closed() {
    // When homed=false and stream_open=false (no stream), the engine should
    // return Ok(()) without faulting — this is the normal pre-homing idle.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (_q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (mut t_producer, _t_consumer) = trace.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut widen = WidenState::default();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    // homed = false (default), stream_open = false (default).

    let r = engine.tick(0, &mut widen, &pool, &mut q_consumer, &mut t_producer, &shared);
    assert!(r.is_ok(), "should not fault when not homed with stream closed");
    assert_ne!(engine.status(), RuntimeStatus::Fault);
}
