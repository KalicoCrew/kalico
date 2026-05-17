//! Smoke tests for `Engine::runtime_modulated_tick` (spec §3.2, T10).
//!
//! Verifies the polled-tick StepAccumulator dispatch:
//!   - A Modulated motor playing a wall-clock segment increments its
//!     `stepper_counts` entry over a few ticks.
//!   - When wall-clock crosses `t_end`, the Modulated motor's bit clears
//!     from the segment's `consumers_remaining` mask. With a single
//!     Modulated motor on a Cartesian-X segment that has no other
//!     consumers, the segment retires and `producer_current` is cleared.
//!
//! These tests construct the engine directly and seed `producer_current`
//! to mirror the lockstep regime the FFI path uses (the producer side
//! and the modulated side share the same wall-clock segment cursor under
//! the MVP simplification per spec §7 question 2).
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
use runtime::state::{SharedState, StepMode};

const CLOCK_FREQ: u32 = 520_000_000;

/// Linear-X cubic from 0 to `end` mm in `u ∈ [0, 1]`.
fn linear_cubic(end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let cps = vec![0.0, end / 3.0, end * 2.0 / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

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
    let mut seg = Segment {
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
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );
    seg
}

/// Empty queue Consumer for tests that pre-seed `producer_current` directly
/// and don't exercise the lazy-dequeue path. Leaks the Queue so the Consumer
/// holds a `'static` borrow (matches `Engine`'s API).
fn empty_queue_consumer() -> heapless::spsc::Consumer<'static, Segment, Q_N> {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (_p, c) = queue.split();
    c
}

fn cartesian_x_engine_modulated_motor0() -> (Engine<NoopPa, NoopIs>, SharedState) {
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
    let shared = SharedState::new();
    // Flip motor 0 to Modulated so runtime_modulated_tick actually drives
    // its StepAccumulator. The other three slots stay at whatever default
    // SharedState::new() picks (Modulated == 0, but motors 1..3 aren't
    // configured so they're filtered out at the step_state.get/motors.get
    // lookup).
    shared.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);
    (engine, shared)
}

#[test]
fn modulated_tick_advances_stepper_counter_mid_segment() {
    let (mut engine, shared) = cartesian_x_engine_modulated_motor0();
    let pool = CurvePool::new();

    // 10 mm X jog over a comfortable duration (`MAX_STEPS_PER_TICK` is 16
    // on the accumulator side; 10 mm × 160 sm = 1600 steps / 200 ticks = 8
    // steps/tick, safely under).
    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_cartesian_x(&pool, 10.0, 0, DURATION, 0, 1);
    engine.producer_current = Some(seg);

    let counter = &shared.stepper_counts[0];
    assert_eq!(counter.load(Ordering::Acquire), 0);

    // Step a few ticks mid-segment and confirm the accumulator advances.
    let mut q = empty_queue_consumer();
    for i in 1..=10 {
        let now = (DURATION / 200) * (i as u64);
        engine.runtime_modulated_tick(now, &mut q, &pool, &shared);
    }

    let count = counter.load(Ordering::Acquire);
    assert!(
        count > 0,
        "modulated tick should have emitted some step pulses, got {count}"
    );
    assert!(
        engine.producer_current.is_some(),
        "segment should still be active mid-flight"
    );
}

#[test]
fn modulated_tick_clears_consumer_bits_and_retires_segment_at_t_end() {
    let (mut engine, shared) = cartesian_x_engine_modulated_motor0();
    let pool = CurvePool::new();

    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_cartesian_x(&pool, 10.0, 0, DURATION, 0, 7);
    let mask_before = seg.consumers_remaining;
    assert!(
        mask_before != 0,
        "test segment must declare at least one consumer (got mask=0)"
    );
    engine.producer_current = Some(seg);

    // Tick past t_end — the modulated path should clear motor 0's bits
    // and, because no other motors consume the X curve, retire the segment.
    let mut q = empty_queue_consumer();
    engine.runtime_modulated_tick(DURATION + 1, &mut q, &pool, &shared);

    assert!(
        engine.producer_current.is_none(),
        "segment should have retired (consumers_done) at t_end"
    );
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        7,
        "retired_through_segment_id should advance to the retired segment id"
    );
}

#[test]
fn modulated_tick_no_op_when_no_producer_current() {
    let (mut engine, shared) = cartesian_x_engine_modulated_motor0();
    let pool = CurvePool::new();

    // Pre-condition: no active wall-clock segment.
    assert!(engine.producer_current.is_none());

    let mut q = empty_queue_consumer();
    engine.runtime_modulated_tick(1_000, &mut q, &pool, &shared);

    // Should remain idle and not touch any motor counters.
    assert!(engine.producer_current.is_none());
    assert_eq!(shared.stepper_counts[0].load(Ordering::Acquire), 0);
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        0
    );
}

#[test]
fn modulated_tick_skips_steptime_motors() {
    // All motors default to Modulated=0; set motor 0 explicitly to StepTime
    // and verify it does NOT emit pulses through the modulated path.
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
    let shared = SharedState::new();
    shared.step_modes[0].store(StepMode::StepTime as u8, Ordering::Release);

    let pool = CurvePool::new();
    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_cartesian_x(&pool, 10.0, 0, DURATION, 0, 1);
    engine.producer_current = Some(seg);

    let mut q = empty_queue_consumer();
    for i in 1..=10 {
        let now = (DURATION / 200) * (i as u64);
        engine.runtime_modulated_tick(now, &mut q, &pool, &shared);
    }

    let count = shared.stepper_counts[0].load(Ordering::Acquire);
    assert_eq!(
        count, 0,
        "StepTime motor should not emit pulses through the modulated path"
    );
}

/// **Disabled: the lazy-dequeue this test pinned was an SPSC race.**
///
/// 2026-05-17 bench root-cause: the lazy `queue.dequeue()` this test
/// asserted (originally added by 081ab4a3b for F446 pure-Modulated Z)
/// is a second consumer-side dequeue site on the same `Consumer` half
/// of the SPSC queue. `fetch_segment_for_motor` (StepTime producer
/// timer, foreground) is the first; this ISR site is the second. SPSC's
/// "single consumer" invariant is broken whenever an MCU has ANY mix
/// of Modulated + StepTime motors (H7's X/Y Modulated + E StepTime
/// config triggered the wedge ~7–30 s after `ConfigureAxes` enables
/// TIM5; the corrupted internal state cascades through the firmware
/// until USB OTG silently disconnects and the kernel drops the device
/// with `acm_start_wb: usb_submit_urb -19`).
///
/// Diagnostic confirmation: patching `runtime_tick_enable()` to a
/// no-op (TIM5 never armed) eliminates the wedge entirely — 0 restarts
/// in 150 s on the same printer config that previously crash-looped
/// every 7–30 s. So the failure path lives inside the TIM5 ISR; the
/// only thing the cluster changed in that path is this lazy dequeue.
///
/// The original problem the lazy dequeue tried to solve (pure-Modulated
/// MCU never advancing its queue) needs a different fix that keeps the
/// foreground side as the sole Consumer — e.g. driving `producer_step`
/// even when no StepTime motor is present so the foreground always
/// owns dequeue. Ignored until that lands.
#[test]
#[ignore = "lazy-dequeue removed — see comment above; pure-Modulated dequeue fix \
            must keep foreground as sole SPSC consumer"]
fn modulated_tick_dequeues_from_queue_when_producer_current_is_none() {
    // F446-like: only motor 2 (Z) is configured, set to Modulated.
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            None,
            None,
            Some(MotorConfig {
                steps_per_mm: 800.0,
                is_awd: false,
                invert_dir: false,
            }),
            None,
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });
    let shared = SharedState::new();
    // Mirror configure_axes blob writing: motor_idx==2 → Modulated; others
    // explicitly StepTime so the count_modulated_steppers loop matches the
    // real F446 post-configure_axes state.
    shared.step_modes[0].store(StepMode::StepTime as u8, Ordering::Release);
    shared.step_modes[1].store(StepMode::StepTime as u8, Ordering::Release);
    shared.step_modes[2].store(StepMode::Modulated as u8, Ordering::Release);
    shared.step_modes[3].store(StepMode::StepTime as u8, Ordering::Release);

    let pool = CurvePool::new();

    // Build a Z-axis segment, push it through the real push_segment path
    // (not the producer_current seeding shortcut the other tests use).
    let (deg, knots, cps) = linear_cubic(5.0);
    let z_handle = pool
        .validate_and_load(0, deg, &knots, &cps)
        .expect("load Z curve");
    const DURATION: u64 = 200 * 13_000;
    let seg = Segment {
        id: 11,
        x_handle: CurveHandle::UNUSED_SENTINEL,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: DURATION,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };

    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    engine
        .push_segment(seg, &mut q_producer, &shared)
        .expect("push ok");

    // Pre-condition: producer_current is empty. The modulated tick must
    // populate it from the queue on its next fire — without any
    // intervening producer_step call (the F446 has no StepTime motor to
    // drive it).
    assert!(engine.producer_current.is_none());

    engine.runtime_modulated_tick(DURATION / 4, &mut q_consumer, &pool, &shared);

    assert!(
        engine.producer_current.is_some(),
        "modulated tick must lazily dequeue from the queue when no StepTime motor exists to do it"
    );
    assert_eq!(
        engine.producer_current.unwrap().id,
        11,
        "dequeued segment id should match the pushed segment"
    );

    // Continue ticking until the segment retires; assert motor 2's counter
    // advanced (Z move actually happened) and retirement published.
    for i in 1..=200 {
        let now = (DURATION / 200) * (i as u64);
        engine.runtime_modulated_tick(now, &mut q_consumer, &pool, &shared);
    }
    engine.runtime_modulated_tick(DURATION + 1, &mut q_consumer, &pool, &shared);

    assert!(
        engine.producer_current.is_none(),
        "segment should have retired after wall-clock crossed t_end"
    );
    assert_eq!(
        shared.retired_through_segment_id.load(Ordering::Acquire),
        11,
        "retired_through_segment_id should advance to the retired segment"
    );
    let z_steps = shared.stepper_counts[2].load(Ordering::Acquire);
    assert!(
        z_steps > 0,
        "motor 2 (Z) should have emitted step pulses, got {z_steps}"
    );
}
