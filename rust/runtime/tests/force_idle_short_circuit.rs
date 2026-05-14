//! Step-emission §3.10 / Task 11: synchronous foreground `runtime_force_idle`.
//!
//! Replaces the legacy ISR-side short-circuit test. The old contract was
//! "foreground sets `shared.force_idle=true`; next `Engine::tick` clears
//! `current`, sets `acked_force_idle`, returns." The new contract is
//! "foreground calls `Engine::runtime_force_idle` directly; the method
//! drains the queue, retires every in-flight pool slot, resets step rings,
//! clears producer state, and zeroes per-motor accumulators."
//!
//! These tests pin the new behaviour: post-flush state must be the
//! "fresh, no work" state regardless of pre-flush in-flight state.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::doc_markdown,
    unsafe_code
)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::config::EMode;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;

const CLOCK_FREQ: u32 = 520_000_000;

fn make_segment(id: u32, x_handle: CurveHandle) -> Segment {
    Segment {
        id,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 1_000_000,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    }
}

#[test]
fn force_idle_drains_queue_and_retires_slot() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // Load a real curve so the slot has `current_gen != last_retired_gen`
    // (i.e., is non-trivially "in use" from the pool's perspective).
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    let cps = [0.0_f32, 10.0];
    let handle = pool
        .validate_and_load(0, 1, &knots, &cps)
        .expect("load curve");
    assert!(
        !pool.is_slot_free(handle.slot_idx),
        "slot must be in-use pre-flush"
    );

    q_producer.enqueue(make_segment(1, handle)).unwrap();
    assert!(q_consumer.ready());

    // Pre-flush: set producer_pending to model an in-flight kick.
    shared.producer_pending.store(true, Ordering::Release);

    engine.runtime_force_idle(&pool, &mut q_consumer, &shared);

    // Queue drained.
    assert!(!q_consumer.ready(), "queue must be empty post-flush");
    // Producer-pending cleared.
    assert!(
        !shared.producer_pending.load(Ordering::Acquire),
        "producer_pending must be cleared post-flush"
    );
    // The slot's last_retired_gen now matches current_gen — the queued
    // segment's handle was retired by the synchronous flush.
    assert!(
        pool.is_slot_free(handle.slot_idx),
        "pool slot must be retired post-flush"
    );
    // Transition-period courtesy: `acked_force_idle` set so the legacy
    // `stream::flush` polling path observes the ack.
    assert!(
        shared.acked_force_idle.load(Ordering::Acquire),
        "acked_force_idle must be set as transition courtesy"
    );
}

#[test]
fn force_idle_resets_step_rings_and_producer_state() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (_q_producer, mut q_consumer) = queue.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // Push a step entry on motor 0 so the ring has `available > 0`.
    engine.step_rings[0].push(0xDEAD_BEEF, 1);
    assert_eq!(engine.step_rings[0].available(), 1);

    // Mark a producer state non-idle by starting a fake curve.
    engine.producer_states[0].start_curve(7);
    assert!(!engine.producer_states[0].is_idle());

    // Stash a "currently filling" segment id on motor 0.
    engine.motor_current_segment_id[0] = Some(42);
    engine.motor_curve_cursor[0] = 5;

    engine.runtime_force_idle(&pool, &mut q_consumer, &shared);

    // Step ring drained: head + cursor both back to 0, `available == 0`.
    assert_eq!(
        engine.step_rings[0].available(),
        0,
        "step ring must be empty post-flush"
    );
    // Producer state idle.
    assert!(
        engine.producer_states[0].is_idle(),
        "producer state must be idle post-flush"
    );
    // Per-motor cursors / segment-id slots reset.
    assert_eq!(engine.motor_current_segment_id[0], None);
    assert_eq!(engine.motor_curve_cursor[0], 0);
}

#[test]
fn force_idle_retires_producer_current_segment() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (_q_producer, mut q_consumer) = queue.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let pool = CurvePool::new();
    let shared = SharedState::new();

    // Load a curve and stash it as `producer_current` (the lockstep
    // wall-clock segment the producer is currently filling).
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    let cps = [0.0_f32, 10.0];
    let handle = pool
        .validate_and_load(0, 1, &knots, &cps)
        .expect("load curve");
    engine.producer_current = Some(make_segment(7, handle));
    assert!(!pool.is_slot_free(handle.slot_idx));

    engine.runtime_force_idle(&pool, &mut q_consumer, &shared);

    // producer_current cleared.
    assert!(engine.producer_current.is_none());
    // Slot retired.
    assert!(
        pool.is_slot_free(handle.slot_idx),
        "producer_current's slot must be retired post-flush"
    );
}

#[test]
fn force_idle_settles_status_to_idle() {
    // Engine::new starts at Idle; force_idle preserves Idle. (Driving
    // the engine to Running requires an active tick path which would
    // pull in widen_state + trace plumbing; the post-flush invariant
    // we care about — "Idle status, not Running, not Fault" — is the
    // same in either case as long as the engine wasn't pre-faulted.)
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (_q_producer, mut q_consumer) = queue.split();

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let pool = CurvePool::new();
    let shared = SharedState::new();

    assert_eq!(engine.status(), runtime::engine::RuntimeStatus::Idle);
    engine.runtime_force_idle(&pool, &mut q_consumer, &shared);
    assert_eq!(engine.status(), runtime::engine::RuntimeStatus::Idle);
}
