//! End-to-end integration test for the Task-5 producer path.
//!
//! Pushes a synthetic Cartesian-X segment, calls `engine.producer_step`,
//! verifies the per-motor ring receives step times in monotonically
//! increasing order. Also exercises the `producer_pending` kick contract
//! and the `producer_runs_total` heartbeat counter.
//!
//! Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md
//! §3.3 + §3.4 + §3.8.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
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

/// 4-CP degree-3 Bézier with collinear control points so position(u) = end*u.
fn linear_cubic(end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let cps = vec![0.0, end / 3.0, end * 2.0 / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

/// 4-CP degree-3 Bézier with collinear control points moving from `start` to
/// `end` (in mm). Lets us exercise curves that don't start at the coordinate
/// origin — the realistic case for any jog after the toolhead has moved.
fn linear_cubic_from_to(start: f32, end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let d = end - start;
    let cps = vec![start, start + d / 3.0, start + 2.0 * d / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

/// Build a Cartesian X-only segment over [t_start, t_start + duration] that
/// moves X from 0 to `end_mm`. Y/Z/E unused. EMode::Travel.
fn build_segment_cartesian_x(
    pool: &CurvePool,
    end_mm: f32,
    t_start: u64,
    duration: u64,
    slot_idx: u16,
) -> Segment {
    let (deg, knots, cps) = linear_cubic(end_mm);
    let x_handle = pool
        .validate_and_load(slot_idx, deg, &knots, &cps)
        .expect("load X curve");
    Segment {
        id: 1,
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
        // Recomputed by Engine::push_segment, value here is irrelevant.
        consumers_remaining: 0,
    }
}

/// Make an Engine configured for a Cartesian X-only setup at 160 steps/mm
/// on motor 0. Motors 1/2/3 unconfigured so their producer_states stay
/// at step_distance=0 and producer_step skips them.
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

struct Harness {
    engine: Engine<NoopPa, NoopIs>,
    pool: CurvePool,
    shared: SharedState,
    q_producer: heapless::spsc::Producer<'static, Segment, Q_N>,
    q_consumer: heapless::spsc::Consumer<'static, Segment, Q_N>,
}

impl Harness {
    fn cartesian_x() -> Self {
        let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
        let (q_producer, q_consumer) = queue.split();
        Self {
            engine: cartesian_x_engine(),
            pool: CurvePool::new(),
            shared: SharedState::new(),
            q_producer,
            q_consumer,
        }
    }
}

#[test]
fn one_segment_one_motor_fills_ring() {
    let mut h = Harness::cartesian_x();
    // X 0 → 5 mm over 50 ms (=800 steps at 160 spm) — comfortably below
    // the per-motor StepRing capacity (1024) so the producer can finish
    // the whole curve without back-pressure.
    let seg = build_segment_cartesian_x(&h.pool, 5.0, 0, 26_000_000, 0);
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");

    // Drive producer to completion. PRODUCER_BATCH_CAP = 32; 800 / 32 ≈
    // 25 calls expected.
    let mut runs = 0_u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 200, "producer_step should converge well within 200 calls");
    }

    let ring = h.engine.step_ring(0).expect("motor 0 ring");
    let avail = ring.available();
    // 5 mm × 160 steps/mm = 800 steps; allow ±2 for Newton boundary at u=1.
    assert!(
        (798..=802).contains(&avail),
        "expected ~800 entries in motor 0 ring; got {avail}"
    );

    // Other motors stay idle (their step_distance is 0 because unconfigured).
    for m in 1..4 {
        let ring = h.engine.step_ring(m).expect("ring");
        assert_eq!(ring.available(), 0, "motor {m} should not have entries");
    }
}

#[test]
fn jog_from_nonzero_position_produces_step_pulses() {
    // Architectural regression test. Previously `initial_step` was seeded
    // from `shared.stepper_counts[i]`, which is 0 at boot and only grows
    // when pulses fire. For a realistic jog where the toolhead is at, say,
    // X=100 mm and the bridge sends a curve with `curve(0) = 100`, the
    // Newton target `(0 + dir) * step_distance ≈ ±step_distance` lay
    // 100 mm away from where the curve actually evaluates. Newton's first
    // iteration overshot out of `[0, 1]` → `SegmentExhausted` → zero pulses
    // → stepper_counts stayed at 0 forever → permanent dead-lock.
    //
    // Fix anchors `initial_step` to `eval(0.0) / step_distance`, so the
    // Newton target lands within the curve's value range and Newton
    // converges normally. This test pins that behavior.
    let mut h = Harness::cartesian_x();
    // X 100 → 101 mm. 160 steps at 160 spm. Fits in one ring fill.
    let (deg, knots, cps) = linear_cubic_from_to(100.0, 101.0);
    let x_handle = h.pool.validate_and_load(0, deg, &knots, &cps).unwrap();
    let seg = Segment {
        id: 1,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 26_000_000,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");

    // Drive the producer until idle.
    let mut runs = 0_u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 200, "producer_step should converge");
    }

    let ring = h.engine.step_ring(0).expect("motor 0 ring");
    let avail = ring.available();
    // 1 mm × 160 spm = 160 steps. Allow ±2 for Newton edge.
    assert!(
        (158..=162).contains(&avail),
        "expected ~160 entries for a 1 mm jog from X=100 to X=101; got {avail}. \
         If avail==0, the regression is back (initial_step seeded from \
         stepper_counts instead of curve(0)/step_distance).",
    );

    // stepper_counts should reflect the curve's coordinate frame after the
    // first curve, so subsequent curves (which the planner emits with
    // continuity at the previous curve's endpoint) stay coherent.
    let counter = h.shared.stepper_counts[0].load(Ordering::Acquire);
    // The producer seeds stepper_counts to `round(curve(0) / step_distance)`,
    // then the consumer (not running in this test) would increment it as
    // pulses fire. With no consumer running, the value should be exactly the
    // seed: round(100.0 / 0.00625) = 16000.
    assert_eq!(
        counter, 16000,
        "stepper_counts[0] should equal round(100/0.00625) after producer \
         seeds it from curve(0); got {counter}",
    );
}

#[test]
fn long_segment_fills_ring_to_capacity_then_back_pressures() {
    // A segment that produces more steps than the ring capacity (1024).
    // After enough producer_step calls the ring is full and producer
    // returns WorkPending without finishing the curve — the consumer
    // must drain entries to unblock further fills. We don't have a
    // consumer running in this test, so we simulate one by manually
    // calling `advance` on the ring and verifying the producer can fill
    // again.
    let mut h = Harness::cartesian_x();
    // X 0 → 10 mm over 100 ms = 1600 steps > 1024 (ring capacity).
    let seg = build_segment_cartesian_x(&h.pool, 10.0, 0, 52_000_000, 0);
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");
    // Fill rounds until WorkPending stalls (ring full).
    let mut last_avail = 0_u32;
    for _ in 0..100 {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        let avail = h.engine.step_ring(0).expect("ring").available();
        if r == ProducerTickResult::WorkPending && avail == last_avail {
            // Wedged on ring-full: the producer made no progress despite
            // WorkPending. That's the expected back-pressure state.
            break;
        }
        last_avail = avail;
    }
    let ring = h.engine.step_ring(0).expect("ring");
    assert!(
        ring.available() >= 1000,
        "ring should be near capacity (1024); got {}",
        ring.available()
    );

    // Simulate consumer draining 500 entries; producer can refill.
    ring.advance(500);
    for _ in 0..50 {
        let _ = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    }
    assert!(
        h.engine.step_ring(0).expect("ring").available() > 1000,
        "after consumer drain, producer should refill toward capacity"
    );
}

#[test]
fn empty_queue_returns_all_idle() {
    let mut h = Harness::cartesian_x();
    let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    assert_eq!(r, ProducerTickResult::AllIdle);
    for m in 0..4 {
        let ring = h.engine.step_ring(m).expect("ring");
        assert_eq!(ring.available(), 0);
    }
}

#[test]
fn push_segment_sets_producer_pending() {
    let mut h = Harness::cartesian_x();
    assert!(
        !h.shared.producer_pending.load(Ordering::Acquire),
        "pending flag starts clear"
    );
    let seg = build_segment_cartesian_x(&h.pool, 1.0, 0, 1_000_000, 0);
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");
    assert!(
        h.shared.producer_pending.load(Ordering::Acquire),
        "push_segment must CAS-set producer_pending"
    );
}

#[test]
fn producer_step_clears_pending_on_entry() {
    let mut h = Harness::cartesian_x();
    h.shared.producer_pending.store(true, Ordering::Release);
    let _ = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    assert!(
        !h.shared.producer_pending.load(Ordering::Acquire),
        "producer_step must clear producer_pending at entry"
    );
}

#[test]
fn producer_runs_total_increments() {
    let mut h = Harness::cartesian_x();
    let before = h.shared.producer_runs_total.load(Ordering::Acquire);
    let _ = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    let _ = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    let after = h.shared.producer_runs_total.load(Ordering::Acquire);
    assert_eq!(after, before + 2, "heartbeat advances per call");
}

#[test]
fn ring_entries_monotonic_in_time() {
    // Stronger property check: the cycle timestamps the producer pushes
    // must be monotonically non-decreasing within a single curve. The
    // ring's consumer is keyed off these timestamps, so any out-of-order
    // pair would translate to motion that runs backwards in time.
    let mut h = Harness::cartesian_x();
    // Short segment so PRODUCER_BATCH_CAP fills it in 1-2 calls.
    let seg = build_segment_cartesian_x(&h.pool, 0.1, 1_000_000, 5_000_000, 0);
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");
    let mut runs = 0;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle || runs > 20 {
            break;
        }
    }

    let ring = h.engine.step_ring(0).expect("motor 0 ring");
    let mut last = ring.peek_head().expect("ring should not be empty").0;
    ring.advance(1);
    while let Some((t, _dir)) = ring.peek_head() {
        // The 32-bit wrap is non-monotonic globally but the segment
        // starts at 1_000_000 cycles and runs for 5_000_000 — far below
        // the 2^32 wrap boundary, so naive < comparison is safe here.
        assert!(
            t >= last,
            "ring entries must be monotonic: {t} < {last}"
        );
        last = t;
        ring.advance(1);
    }
}

#[test]
fn corexy_motor0_and_motor1_both_step_on_x_jog() {
    // CoreXY pure-X jog: both motors A and B step (A = X+Y = +X, B = X−Y = +X).
    // Verifies the per-motor kinematic transform inside producer_step.
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

    // X 0 → 10 mm; Y handle is UNUSED (sentinel) so the closure treats
    // it as identically zero. CoreXY motors 0/1 see motor pos = X + 0 = X
    // and X − 0 = X respectively.
    let (deg, knots, cps) = linear_cubic(10.0);
    let x_handle = pool
        .validate_and_load(0, deg, &knots, &cps)
        .expect("load X curve");
    let seg = Segment {
        id: 1,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 52_000_000,
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

    let mut runs = 0;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle || runs > 200 {
            break;
        }
    }

    let avail_a = engine.step_ring(0).expect("ring 0").available();
    let avail_b = engine.step_ring(1).expect("ring 1").available();
    // 10 mm × 80 steps/mm = 800 steps for each motor.
    assert!(
        (798..=802).contains(&avail_a),
        "motor A should have ~800 entries; got {avail_a}"
    );
    assert!(
        (798..=802).contains(&avail_b),
        "motor B should have ~800 entries; got {avail_b}"
    );
}

/// **IWDG-spin regression** — pinned 2026-05-14.
///
/// Before the `motor_has_remaining_work` guard in `fetch_segment_for_motor`,
/// a Cartesian X-only jog where motors 1 (Y) and 3 (E) get constant curves
/// (dy=0, de=0) caused the producer to manufacture fake
/// `motor_finished_curve=true` on every fire AFTER those motors finished:
///
///   1. Fire 1: motor 1's `compute_next_step_time` returns `SegmentExhausted`
///      (constant curve, no motion). `motor_finished_curve=true` → clear
///      motor 1's consumer bit, increment cursor, `made_progress=true`.
///   2. Fire 2: `producer_states[1].is_idle()=true` (was cleared at
///      `engine.rs:1949`). `fetch_segment_for_motor` returns the same
///      `producer_current` segment because the old code only checked
///      `handle.is_unused_sentinel()`, not whether motor 1's consumer bit
///      was still set in `seg.consumers_remaining`. `start_curve` runs.
///      `compute_next_step_time` returns `SegmentExhausted` again. Bit-clear
///      is idempotent. `made_progress=true` (spuriously).
///   3. Producer returns `WorkPending` → C side self-reschedules at
///      `SF_RESCHEDULE_FLOOR=100 µs` → 10 kHz SysTick storm → foreground
///      `watchdog_reset` task can't run → IWDG fires at 511 ms → MCU resets.
///
/// This test must converge in a bounded number of `producer_step` calls and
/// observe that motors 1 and 3 do NOT report progress on subsequent fires
/// once they've finished their constant curves.
#[test]
fn fake_finish_loop_does_not_spin_on_finished_constant_axes() {
    // Cartesian engine with motors 0 (X) and 1 (Y) and 3 (E) all configured.
    // Motor 0 has a real X jog; motors 1 and 3 get constant curves at their
    // current positions; motor 2 (Z) unconfigured.
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig {
                steps_per_mm: 160.0,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig {
                steps_per_mm: 160.0,
                is_awd: false,
                invert_dir: false,
            }),
            None,
            Some(MotorConfig {
                steps_per_mm: 2207.0,
                is_awd: false,
                invert_dir: false,
            }),
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();

    // X: 0 → 1 mm (160 steps at 160 spm); Y: constant at 50 mm; E: constant
    // at 0 mm. This matches the host's "send every kinematic axis's curve
    // including constants" policy from dispatch.rs.
    let x_handle = {
        let (deg, knots, cps) = linear_cubic(1.0);
        pool.validate_and_load(0, deg, &knots, &cps).expect("x")
    };
    let y_handle = {
        let (deg, knots, cps) = linear_cubic_from_to(50.0, 50.0);
        pool.validate_and_load(1, deg, &knots, &cps).expect("y")
    };
    let e_handle = {
        let (deg, knots, cps) = linear_cubic_from_to(0.0, 0.0);
        pool.validate_and_load(2, deg, &knots, &cps).expect("e")
    };
    let seg = Segment {
        id: 1,
        x_handle,
        y_handle,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle,
        t_start: 0,
        t_end: 26_000_000, // 50 ms at 520 MHz
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    engine
        .push_segment(seg, &mut q_producer, &shared)
        .expect("push ok");

    // Drive producer until idle. Before the fix this loop ran forever
    // (WorkPending every fire, producer_runs_total grew unbounded). After
    // the fix it must converge in well under PRODUCER_CONVERGE_CAP calls
    // — motor 0 alone needs ~160/32 ≈ 5 fills.
    const PRODUCER_CONVERGE_CAP: u32 = 100;
    let mut runs = 0_u32;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(
            runs < PRODUCER_CONVERGE_CAP,
            "producer_step must converge to AllIdle; reached {runs} calls \
             with WorkPending still set — the fake-finish spin is back. \
             motor 0 avail={}, producer_runs_total={}",
            engine.step_ring(0).map(|r| r.available()).unwrap_or(0),
            shared.producer_runs_total.load(Ordering::Acquire),
        );
    }

    // Motor 0 filled its ring with ~160 entries; motors 1 and 3 produced 0.
    let m0 = engine.step_ring(0).expect("ring 0").available();
    let m1 = engine.step_ring(1).expect("ring 1").available();
    let m3 = engine.step_ring(3).expect("ring 3").available();
    assert!(
        (158..=162).contains(&m0),
        "motor 0 should have ~160 X-step entries; got {m0}"
    );
    assert_eq!(m1, 0, "motor 1 (constant Y) must not produce step pulses");
    assert_eq!(m3, 0, "motor 3 (constant E) must not produce step pulses");

    // Sanity-check on convergence speed: 100 producer_step calls is plenty
    // for 160 X steps at PRODUCER_BATCH_CAP=32 per call. If runs > 50 we
    // are wasting CPU on spurious refires even if we did terminate.
    assert!(
        runs <= 50,
        "producer_step converged but took {runs} calls — that's still \
         far more than needed for 160 X-steps + 2 trivial finishes. \
         Suggests the fake-finish guard is leaking some work."
    );
}
