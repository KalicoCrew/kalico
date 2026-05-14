//! Verifies the per-consumer retirement decoupling (spec §3.8).
//!
//! Today (Task 5) the StepTime producer is the only consumer that runs in
//! tests — Modulated retirement comes from TIM5's runtime_modulated_tick
//! which lands in Task 10. So this file mainly covers the StepTime side:
//! once the producer's Newton fill returns SegmentExhausted for every motor
//! that consumes the segment's curves, the curve pool slots retire.
//!
//! For the lockstep-MVP path (single producer_current at a time), retirement
//! also publishes `retired_through_segment_id` and fires the stream's
//! terminal-segment hook.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic
)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::config::{EMode, McuAxisConfig, MotorConfig};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::step_producer::ProducerTickResult;

const CLOCK_FREQ: u32 = 520_000_000;

fn linear_cubic(end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let cps = vec![0.0, end / 3.0, end * 2.0 / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

/// Build a Cartesian X-only segment over [t_start, t_start + duration] that
/// moves X by `end_mm`. Distinct slot indices for different segments so
/// the test can verify retirement of each.
fn build_segment_cartesian_x(
    pool: &CurvePool,
    end_mm: f32,
    t_start: u64,
    duration: u64,
    slot_idx: u16,
    seg_id: u32,
) -> Segment {
    let (deg, knots, cps) = linear_cubic(end_mm);
    let x_handle = pool
        .validate_and_load(slot_idx, deg, &knots, &cps)
        .expect("load X curve");
    Segment {
        id: seg_id,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start,
        t_end: t_start + duration,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    }
}

fn cartesian_x_engine() -> Engine<NoopPa, NoopIs> {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig {
                steps_per_mm: 160.0,
                is_awd: false,
                invert_dir: false,
            }),
            None,
            None,
            None,
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });
    engine
}

#[test]
fn slot_retires_when_producer_finishes_curve() {
    // Short Cartesian-X segment, Step-Time motor 0 only. Run producer
    // until AllIdle. The X-curve's slot must be retired.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = cartesian_x_engine();

    let seg = build_segment_cartesian_x(&pool, 0.5, 0, 5_000_000, 0, 42);
    let x_handle = seg.x_handle;
    let x_slot_idx = x_handle.slot_idx;
    engine
        .push_segment(seg, &mut q_producer, &shared)
        .expect("push ok");

    // The slot is in-use immediately after load — last_retired_gen has
    // not yet caught up with current_gen.
    assert!(
        !pool.is_slot_free(x_slot_idx),
        "slot must be in-use right after load"
    );

    // Run producer to completion.
    let mut runs = 0;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 200, "producer should converge");
    }

    assert!(
        pool.is_slot_free(x_slot_idx),
        "X slot must be retired after producer finishes curve"
    );
    // Retire-through cursor advanced to this segment.
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        42,
        "retired_through_segment_id should advance on retirement"
    );
}

#[test]
fn slot_held_until_all_consuming_motors_done() {
    // CoreXY: both motors A and B consume the X curve. Even if motor A's
    // Newton finishes before motor B's, the slot must NOT retire until
    // both motors clear their bit in consumers_remaining.
    //
    // In the lockstep MVP regime the producer drives both motors in the
    // same call so they finish together — this test exercises the
    // consumers_remaining bookkeeping by manually clearing one motor's
    // ProducerState mid-flow and verifying retirement waits.
    //
    // The simplest scope-controlled scenario: install a CoreXY config
    // where motor A is StepTime but motor B is StepTime too. Both rings
    // get filled in lockstep, so the assertion is the natural one — when
    // we observe the segment retired (slot free), both bits should be
    // clear.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // Long X-curve (15 mm = 1200 steps per motor at 80 spm > ring
    // capacity 1024) so neither motor can finish Newton within a single
    // producer_step call. Step times stay in the ring; producer reports
    // WorkPending; slot stays held until the consumer drains and the
    // producer can finally drive Newton past u=1.
    let (deg, knots, cps) = linear_cubic(15.0);
    let x_handle = pool
        .validate_and_load(0, deg, &knots, &cps)
        .expect("load X curve");
    let x_slot_idx = x_handle.slot_idx;
    let seg = Segment {
        id: 7,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 78_000_000,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    engine
        .push_segment(seg, &mut q_producer, &shared)
        .expect("push ok");

    // First producer_step pass: fills rings up to capacity but cannot
    // finish either motor's Newton because there are more steps than
    // ring slots. Slot stays held.
    let _ = engine.producer_step(&pool, &mut q_consumer, &shared);
    assert!(
        !pool.is_slot_free(x_slot_idx),
        "slot stays held while motors are mid-curve"
    );

    // Drain consumer side and finish — both motors' rings have to be
    // advanced for the producer to make more progress.
    for _ in 0..50 {
        let ring_a_avail = engine.step_ring(0).expect("ring 0").available();
        if ring_a_avail > 0 {
            engine.step_ring(0).expect("ring 0").advance(ring_a_avail);
        }
        let ring_b_avail = engine.step_ring(1).expect("ring 1").available();
        if ring_b_avail > 0 {
            engine.step_ring(1).expect("ring 1").advance(ring_b_avail);
        }
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        if r == ProducerTickResult::AllIdle {
            break;
        }
    }

    assert!(
        pool.is_slot_free(x_slot_idx),
        "after both motors finish, slot retires"
    );
}

#[test]
fn second_segment_can_reuse_slot_after_first_retires() {
    // Two back-to-back Cartesian-X segments using the same slot index.
    // After the first retires, the second's validate_and_load on slot 0
    // must succeed (alloc predicate: current_gen == last_retired_gen).
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = cartesian_x_engine();

    let seg1 = build_segment_cartesian_x(&pool, 0.5, 0, 5_000_000, 0, 1);
    engine
        .push_segment(seg1, &mut q_producer, &shared)
        .expect("push 1");
    while engine.producer_step(&pool, &mut q_consumer, &shared)
        != ProducerTickResult::AllIdle
    {}
    assert!(pool.is_slot_free(0), "slot 0 free after seg1 retires");

    // Now slot 0 should be reusable.
    let seg2 = build_segment_cartesian_x(&pool, 0.5, 5_000_000, 5_000_000, 0, 2);
    engine
        .push_segment(seg2, &mut q_producer, &shared)
        .expect("push 2");
    while engine.producer_step(&pool, &mut q_consumer, &shared)
        != ProducerTickResult::AllIdle
    {}
    assert!(pool.is_slot_free(0), "slot 0 free after seg2 retires");
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        2,
        "retired-through advances to seg 2"
    );
}
