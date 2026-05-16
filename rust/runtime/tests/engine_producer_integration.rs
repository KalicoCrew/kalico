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
use runtime::state::{SharedState, StepMode};
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
fn initial_step_seed_is_direction_aware_for_negative_motion() {
    // Live-printer regression (2026-05-15 trident-printer reproducer:
    // 0xB3002424 fault_detail: dequeued=36, retired=36, pushed=0).
    //
    // Before this fix `initial_step` was seeded via `(pos0 / step_distance)
    // as i32` (truncate toward zero == floor for positive pos0). The
    // producer then computed `target = (initial_step + dir) * step_distance`.
    //
    //   * For positive motion: target = (floor + 1) * sd is the next step
    //     boundary above pos0. Correct.
    //
    //   * For negative motion: target = (floor - 1) * sd is the step boundary
    //     BELOW the actual next negative boundary. When the curve's first
    //     piece spanned less than 2 * step_distance (typical for shaped
    //     curves at the start of a move), this target fell below the piece's
    //     minimum value → SegmentExhausted on the first call → motor finishes
    //     curve without emitting any step pulses. The MCU dequeued segments,
    //     retired them, and produced zero motion.
    //
    // Fix: seed `initial_step` via ceil() for negative-direction curves so
    // target = (ceil - 1) * sd = the first downward step boundary.
    //
    // This test exercises the seed path directly: a single-piece cubic from
    // pos0=99.998 down to 99.0 (negative). The stepper_counts atomic captures
    // the producer's seed value, so we can assert it without relying on
    // multi-piece curve construction (which is geometry-fragile to inline).
    let mut h = Harness::cartesian_x();
    let (deg, knots, cps) = linear_cubic_from_to(99.998, 99.0);
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

    // One producer_step call is enough to fire the seeding path. The seed
    // is written to stepper_counts before any pulses are pushed.
    h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);

    // ceil(99.998 / 0.00625) = ceil(15999.68) = 16000.
    // Pre-fix this was 15999 (floor), which combined with the
    // `target = (cs + dir) * sd` formula gave first-step target = 99.9875.
    // Post-fix: 16000 → first-step target = 99.99375 (the actual next
    // downward step boundary from a motor sitting at the top of step
    // range [99.99375, 100.0)).
    let counter = h.shared.stepper_counts[0].load(Ordering::Acquire);
    assert_eq!(
        counter, 16000,
        "stepper_counts[0] should be ceil(99.998/0.00625) = 16000 for negative \
         motion seed; got {counter} (pre-fix: 15999 = floor)",
    );

    // Sanity: positive-direction seeding stays at floor. Verifies we didn't
    // accidentally invert the contract for the regular case.
    let mut h2 = Harness::cartesian_x();
    let (deg2, knots2, cps2) = linear_cubic_from_to(99.998, 100.5);
    let x_handle2 = h2.pool.validate_and_load(0, deg2, &knots2, &cps2).unwrap();
    let seg2 = Segment {
        id: 2,
        x_handle: x_handle2,
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
    h2.engine
        .push_segment(seg2, &mut h2.q_producer, &h2.shared)
        .expect("push ok");
    h2.engine.producer_step(&h2.pool, &mut h2.q_consumer, &h2.shared);
    let counter2 = h2.shared.stepper_counts[0].load(Ordering::Acquire);
    assert_eq!(
        counter2, 15999,
        "positive motion seed should remain floor(99.998/0.00625) = 15999; \
         got {counter2}",
    );
}

#[test]
fn corexy_xonly_jog_with_real_handle_constant_y_emits_step_pulses() {
    // Live-printer regression (2026-05-15 trident-printer reproducer:
    // post-flash 0xB3000505 on H723 / mcu: dequeued=5, retired=5, pushed=0
    // across an X jog of +25mm then -25mm).
    //
    // Root cause: the bridge's 2026-05-11 fix to `dispatch.rs` stopped
    // filtering trivially-constant curves; the bridge now sends every
    // kinematic axis curve as a real handle every segment so the runtime
    // can "hold prev value" semantics. For an X-only jog on CoreXY, that
    // means:
    //
    //   * X handle → real handle to shaped X curve (smooth_mzv ⇒ multi-piece)
    //   * Y handle → real handle to CONSTANT Y curve (1 piece, all CPs equal)
    //
    // The producer's CoreXY-only piece-count-match check (engine.rs:
    // ~1944-1955) bails when `n_pieces_primary != n_pieces_secondary`
    // unless secondary is UNUSED. Constant-Y on a real handle has 1 piece
    // while shaped-X has many, so the check fires for motors 0 and 1, both
    // motors retire the segment without pushing any step events.
    //
    // Bug symptom: every CoreXY jog energizes the steppers but produces
    // zero physical motion. F446 (Z) is unaffected because Z is single-axis
    // (no secondary curve), so the check never fires there. Bench
    // `bench_repro::first_jog_after_stream_open_emits_step_pulses` passes
    // because the test harness still filters constants the old way and
    // leaves the Y handle as UNUSED_SENTINEL, which secondary_is_zero=true
    // hides the bug.
    //
    // Expected post-fix behavior: when secondary is trivially constant,
    // the producer should use its single value as a per-piece offset to
    // primary's pieces, regardless of piece-count mismatch.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);

    // CoreXY: motors 0 (A) and 1 (B) configured at 160 steps/mm.
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // Multi-piece X cubic: 100 → 101 mm over 2 piecewise-Bezier pieces.
    // n_cps = 3*N + 1 = 7 for N=2; n_knots = n_cps + degree + 1 = 11.
    // Knots layout for piecewise-Bezier degree-3, N=2:
    //   [u0;4][u1;3][u2;4] = [0,0,0,0, 0.5,0.5,0.5, 1,1,1,1].
    // Within each piece, CPs are collinear along the piece's slope so the
    // result is the same straight line 100 → 101 as a single-piece would
    // give, but the runtime sees TWO pieces — mirroring what the smooth_mzv
    // shaper would produce on real input.
    let x_knots: Vec<f32> = vec![0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0];
    let x_cps: Vec<f32> = vec![
        100.0, 100.166_67, 100.333_336, 100.5, 100.666_67, 100.833_336, 101.0,
    ];
    let x_handle = pool
        .validate_and_load(0, 3, &x_knots, &x_cps)
        .expect("load X curve");

    // Constant Y cubic: 100 → 100 mm. Single-piece. This is what
    // `dispatch::CurveLoadParams::from_scalar_nurbs_normalized` emits for
    // a trivially-constant Y curve on an X-only jog after the 2026-05-11
    // bridge fix to send every kinematic axis.
    let y_knots: Vec<f32> = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let y_cps: Vec<f32> = vec![100.0, 100.0, 100.0, 100.0];
    let y_handle = pool
        .validate_and_load(1, 3, &y_knots, &y_cps)
        .expect("load constant Y curve");

    let seg = Segment {
        id: 1,
        x_handle,
        y_handle, // REAL handle, not UNUSED_SENTINEL — matches live behavior.
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 26_000_000,
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

    // Drive producer until idle. For 160 steps with PRODUCER_BATCH_CAP=32,
    // expect ~5–6 fills with WorkPending then AllIdle.
    let mut runs = 0_u32;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 500, "producer_step should converge well within 500 calls");
    }

    // For a +1 mm X jog with Y constant on CoreXY:
    //   motor 0 (A = X + Y): position 200 → 201   ⇒ ~160 step pulses
    //   motor 1 (B = X − Y): position   0 →   1   ⇒ ~160 step pulses
    let m0 = engine.step_ring(0).expect("motor 0 ring").available();
    let m1 = engine.step_ring(1).expect("motor 1 ring").available();

    assert!(
        (140..=180).contains(&m0),
        "motor 0 should push ~160 step events for 1mm X with constant Y on CoreXY; \
         got {m0}. If m0==0, the constant-secondary piece-count-mismatch bug is back \
         (engine.rs piece_coeffs bails on mismatched non-zero piece counts)."
    );
    assert!(
        (140..=180).contains(&m1),
        "motor 1 should push ~160 step events for 1mm X with constant Y on CoreXY; \
         got {m1}. If m1==0, the constant-secondary piece-count-mismatch bug is back."
    );
}

#[test]
fn corexy_yonly_jog_with_real_handle_constant_x_emits_step_pulses() {
    // Symmetric case to `corexy_xonly_jog_with_real_handle_constant_y_*`:
    // here the SHAPED axis is Y and the CONSTANT axis is X. Without the
    // 2026-05-15 fix the producer iterates primary's pieces (X has 1, the
    // constant) and never reaches secondary's (Y has N, the shaped one),
    // so both CoreXY motors retire the segment without pushing — same bug,
    // different axis pair.
    //
    // Post-fix the iteration is driven by the shaped side regardless of
    // which axis label it lives under.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);

    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // Constant X cubic at 100 mm — single piece.
    let x_knots: Vec<f32> = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let x_cps: Vec<f32> = vec![100.0, 100.0, 100.0, 100.0];
    let x_handle = pool
        .validate_and_load(0, 3, &x_knots, &x_cps)
        .expect("load constant X");

    // Multi-piece Y cubic: 100 → 101 mm over 2 pieces — same shape as the
    // X-only test mirrored to Y.
    let y_knots: Vec<f32> = vec![0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0];
    let y_cps: Vec<f32> = vec![
        100.0, 100.166_67, 100.333_336, 100.5, 100.666_67, 100.833_336, 101.0,
    ];
    let y_handle = pool
        .validate_and_load(1, 3, &y_knots, &y_cps)
        .expect("load Y curve");

    let seg = Segment {
        id: 1,
        x_handle,
        y_handle,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 26_000_000,
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

    let mut runs = 0_u32;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle { break; }
        assert!(runs < 500, "producer_step should converge well within 500 calls");
    }

    // For a +1 mm Y jog with X constant on CoreXY:
    //   motor 0 (A = X + Y): position 200 → 201   ⇒ ~160 step pulses
    //   motor 1 (B = X − Y): position 0   → -1    ⇒ ~160 step pulses (negative)
    let m0 = engine.step_ring(0).expect("motor 0 ring").available();
    let m1 = engine.step_ring(1).expect("motor 1 ring").available();
    assert!(
        (140..=180).contains(&m0),
        "motor 0 should push ~160 step events for 1mm Y with constant X; got {m0}"
    );
    assert!(
        (140..=180).contains(&m1),
        "motor 1 should push ~160 step events for 1mm Y with constant X; got {m1}"
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

/// **Multi-piece NURBS regression** — pinned 2026-05-14.
///
/// The host bridge sends curves in piecewise-Bézier form (per
/// `trajectory/src/refit.rs::refit_to_cubic` →
/// `nurbs/src/bezier.rs::bezier_pieces_to_nurbs`): degree-3, multiplicity-3
/// interior knots, `n_cps = 3N + 1` for `N` pieces. Bench observation:
/// a 1 mm X jog arrives as 2–10 pieces depending on the input shaper.
///
/// Pre-fix, the producer's `extract_uniform_cubic_bezier_coeffs` required
/// EXACTLY 4 CPs degree 3. For multi-piece (N≥2) curves it returned `None`,
/// the producer fell back to zero-coeffs, and `compute_next_step_time`
/// returned `SegmentExhausted` on the first call — zero step pulses emitted,
/// segments retired immediately, host slot pool drained, no motion.
///
/// This test pushes a synthetic 3-piece curve (4-CP-per-piece, shared
/// boundary CPs, 10 total CPs) and asserts the producer fills the ring
/// with step entries across ALL pieces.
#[test]
fn producer_handles_multi_piece_cubic_nurbs() {
    use core::sync::atomic::Ordering as _Order;

    let mut h = Harness::cartesian_x();

    // Build a 3-piece piecewise-Bézier degree-3 NURBS that moves X
    // monotonically from 0 to 3 mm:
    //   piece 0: u ∈ [0.0, 0.33], X from 0.0 to 1.0
    //   piece 1: u ∈ [0.33, 0.67], X from 1.0 to 2.0
    //   piece 2: u ∈ [0.67, 1.0], X from 2.0 to 3.0
    // Each piece is collinear (P0, P0+d/3, P0+2d/3, P3 = P0+d), so
    // CPs are the boundary values and the 1/3 / 2/3 interpolants.
    // Shared boundary CPs collapse to: [0.0, 0.333, 0.667, 1.0, 1.333,
    // 1.667, 2.0, 2.333, 2.667, 3.0] — 10 CPs = 3·3 + 1. ✓
    let cps: Vec<f32> = vec![
        0.0, 1.0 / 3.0, 2.0 / 3.0,
        1.0, 4.0 / 3.0, 5.0 / 3.0,
        2.0, 7.0 / 3.0, 8.0 / 3.0,
        3.0,
    ];
    // Knot vector: [0,0,0,0, 0.33,0.33,0.33, 0.67,0.67,0.67, 1,1,1,1]
    // (14 entries = 10 CPs + degree(3) + 1).
    let third = 1.0_f32 / 3.0;
    let two_thirds = 2.0_f32 / 3.0;
    let knots: Vec<f32> = vec![
        0.0, 0.0, 0.0, 0.0,
        third, third, third,
        two_thirds, two_thirds, two_thirds,
        1.0, 1.0, 1.0, 1.0,
    ];
    assert_eq!(knots.len(), cps.len() + 4, "knot vector well-formed");
    let x_handle = h.pool.validate_and_load(0, 3, &knots, &cps).expect("load");
    let seg = Segment {
        id: 1,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        // 78 ms at 520 MHz — 3 mm × 160 steps/mm = 480 steps; @100 mm/s
        // step rate → 16 kHz, well within ring capacity of 1024.
        t_end: 40_600_000,
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

    // Drive producer to completion. With N=3 pieces and 480 total steps,
    // PRODUCER_BATCH_CAP=32 means ~15 fires expected. Cap loosely at 100.
    let mut runs = 0_u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(
            runs < 100,
            "multi-piece producer must converge to AllIdle within 100 calls. \
             Got runs={runs}, ring_avail={}, steps_pushed_total={}",
            h.engine.step_ring(0).map(|r| r.available()).unwrap_or(0),
            h.shared.producer_steps_pushed_total.load(_Order::Acquire),
        );
    }

    // 3 mm × 160 spm = 480 steps. Allow ±5 for Cardano boundary rounding.
    let avail = h.engine.step_ring(0).expect("ring 0").available();
    assert!(
        (475..=485).contains(&avail),
        "expected ~480 entries from a 3 mm jog across 3 cubic pieces; got \
         {avail}. If 0, the multi-piece walker bailed early. If <100, only \
         the first piece was processed."
    );

    // Ring entries must be monotonically increasing across piece boundaries.
    let ring = h.engine.step_ring(0).expect("ring");
    let mut last = ring.peek_head().expect("ring non-empty").0;
    ring.advance(1);
    let mut prev_dir: i8 = 0;
    while let Some((t, dir)) = ring.peek_head() {
        assert!(t >= last, "ring time monotonicity broken across pieces: {t} < {last}");
        if prev_dir != 0 {
            // Single-direction jog — all entries should have same direction.
            assert_eq!(dir, prev_dir, "direction flipped mid-jog");
        }
        prev_dir = dir;
        last = t;
        ring.advance(1);
    }
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

// CoreXY +1mm X jog with SHAPER-NOISY Y (mimics smooth_mzv on a stationary
// axis). The Y curve has piece-level oscillations summing to ~zero net
// displacement, but with per-piece |Δy| ≈ 12 µm — about 2× step_distance
// at 160 spm. Mirrors what the planner's shaper convolution emits for an
// "idle" axis during a single-axis jog.
//
// Pass = motor A receives a NET +1mm worth of pulses (Σ dir ≈ +160),
//        not just |total| ≈ 160 with mixed directions.
//
// If this fails, step_time's per-piece dir-from-velocity picker is being
// fooled by shaper noise, producing alternating-dir pulses that cancel
// the real X motion. Hardware symptom: motor energizes but doesn't move.
#[test]
fn corexy_xonly_jog_with_noisy_y_emits_net_directional_pulses() {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);

    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // X: 100 → 101 mm over 4 pieces (matching Y's piece count below, which
    // is what the planner emits when both axes go through the same shaper
    // kernel). Collinear cps within each piece ⇒ straight line.
    let x_knots: Vec<f32> = vec![
        0.0, 0.0, 0.0, 0.0,
        0.25, 0.25, 0.25,
        0.5, 0.5, 0.5,
        0.75, 0.75, 0.75,
        1.0, 1.0, 1.0, 1.0,
    ];
    // Shaper-pad style: piece 0 has near-zero X motion (smooth_mzv pad),
    // pieces 1-3 carry the actual jog. Piece 0 only +1 µm, pieces 1-3
    // distribute the remaining 0.999 mm. Mirrors what smooth_mzv produces
    // for the first ~1 ms of a jog (shaper pre-burst).
    let x_cps: Vec<f32> = vec![
        // piece 0: 100.000 → 100.001 (1 µm shaper-pad)
        100.000, 100.0003, 100.0007,
        // piece 1: 100.001 → 100.334 (0.333 mm)
        100.001, 100.112, 100.223,
        // piece 2: 100.334 → 100.667 (0.333 mm)
        100.334, 100.445, 100.556,
        // piece 3: 100.667 → 101.000 (0.333 mm)
        100.667, 100.778, 100.889,
        101.000,
    ];
    let x_handle = pool
        .validate_and_load(0, 3, &x_knots, &x_cps)
        .expect("load X curve");

    // Noisy Y: nominally constant 100mm, but with per-piece oscillation.
    // 4 pieces, each with end-to-start displacement < 1 step_distance, but
    // some pieces have inner cps shifted ±12 µm (shaper-pad noise). Net
    // start = end = 100.0.
    // Knots: [0;4][1/4;3][2/4;3][3/4;3][1;4] for N=4 pieces.
    let y_knots: Vec<f32> = vec![
        0.0, 0.0, 0.0, 0.0,
        0.25, 0.25, 0.25,
        0.5, 0.5, 0.5,
        0.75, 0.75, 0.75,
        1.0, 1.0, 1.0, 1.0,
    ];
    // Per-piece cps: [start, inner1, inner2, end]. Inner cps shifted ±12 µm
    // around each piece's straight line for "shaper-like" noise.
    let y_cps: Vec<f32> = vec![
        // piece 0: 100.000 → 100.000 with +12 µm bump in inner CPs
        100.000, 100.012, 100.012,
        // piece 1: 100.000 → 100.000 with -12 µm dip
        100.000, 99.988, 99.988,
        // piece 2: 100.000 → 100.000 with +12 µm bump
        100.000, 100.012, 100.012,
        // piece 3: 100.000 → 100.000 with -12 µm dip
        100.000, 99.988, 99.988,
        100.000,
    ];
    assert_eq!(y_cps.len(), 13, "N=4 pieces ⇒ 3*4+1 = 13 cps");
    let y_handle = pool
        .validate_and_load(1, 3, &y_knots, &y_cps)
        .expect("load noisy Y curve");

    let seg = Segment {
        id: 1,
        x_handle,
        y_handle,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: 26_000_000,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    engine.push_segment(seg, &mut q_producer, &shared).expect("push ok");

    let mut runs = 0_u32;
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        runs += 1;
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 500, "producer_step should converge");
    }

    // Drain motor A's ring, sum directions.
    let m0_ring = engine.step_ring(0).expect("motor 0 ring");
    let m0_total = m0_ring.available();
    let mut m0_net: i32 = 0;
    let mut m0_pos: u32 = 0;
    let mut m0_neg: u32 = 0;
    for _ in 0..m0_total {
        let (_t, dir) = m0_ring.peek_head().expect("ring not empty");
        m0_net += dir as i32;
        if dir > 0 {
            m0_pos += 1;
        } else if dir < 0 {
            m0_neg += 1;
        }
        m0_ring.advance(1);
    }

    eprintln!(
        "[noisy-y test] motor 0: total={} pos={} neg={} net={}",
        m0_total, m0_pos, m0_neg, m0_net,
    );

    // For a +1mm X-only jog on CoreXY: motor A = X+Y net displacement is
    // +1mm × 160 spm = 160 microsteps. Allow ±30 tolerance for shaper noise
    // affecting boundary-region steps. The key: NET must be strongly
    // positive — not balanced (which would mean dir-flip cancellation).
    assert!(
        m0_net >= 100,
        "motor A net direction must be strongly positive (real X motion); \
         got net={} (pos={} neg={}). If neg ≈ pos, step_time dir-picker is \
         being fooled by shaper noise on the constant Y axis.",
        m0_net, m0_pos, m0_neg,
    );
}

// Direct side-by-side comparison: feed the SAME segment + curve geometry to
// `producer_step` (StepTime emission) and `runtime_modulated_tick`
// (Modulated emission). Count what each emits.
//
// Why this matters: on the live bench, modulated mode MOVES the toolhead
// but step_time mode does not. If both modes are correct, they should emit
// the SAME total step count with the SAME net direction. Any divergence
// here pins down where step_time diverges from the working modulated path.
fn build_corexy_xjog_noisy_y_segment(
    pool: &CurvePool,
    x_slot: u16,
    y_slot: u16,
    seg_id: u32,
    t_start: u64,
    duration: u64,
) -> Segment {
    let x_knots: Vec<f32> = vec![
        0.0, 0.0, 0.0, 0.0,
        0.25, 0.25, 0.25,
        0.5, 0.5, 0.5,
        0.75, 0.75, 0.75,
        1.0, 1.0, 1.0, 1.0,
    ];
    // Start at X=0 (matches engine's default prev_x=0). Shaper-pad piece 0
    // (1 µm), main motion in pieces 1-3 to reach X=1.0 mm.
    let x_cps: Vec<f32> = vec![
        0.000, 0.0003, 0.0007,
        0.001, 0.112, 0.223,
        0.334, 0.445, 0.556,
        0.667, 0.778, 0.889,
        1.000,
    ];
    let x_handle = pool
        .validate_and_load(x_slot, 3, &x_knots, &x_cps)
        .expect("load X");
    let y_knots: Vec<f32> = vec![
        0.0, 0.0, 0.0, 0.0,
        0.25, 0.25, 0.25,
        0.5, 0.5, 0.5,
        0.75, 0.75, 0.75,
        1.0, 1.0, 1.0, 1.0,
    ];
    // Y oscillates ±12 µm per piece, net zero. Starts at Y=0 to match prev_y.
    let y_cps: Vec<f32> = vec![
        0.000, 0.012, 0.012,
        0.000, -0.012, -0.012,
        0.000, 0.012, 0.012,
        0.000, -0.012, -0.012,
        0.000,
    ];
    let y_handle = pool
        .validate_and_load(y_slot, 3, &y_knots, &y_cps)
        .expect("load Y");
    let mut seg = Segment {
        id: seg_id,
        x_handle,
        y_handle,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start,
        t_end: t_start + duration,
        kinematics: KinematicTag::CoreXyAndE,
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

#[test]
fn compare_modulated_vs_step_time_for_corexy_xjog_noisy_y() {
    // --- StepTime side ---
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool_st = CurvePool::new();
    let shared_st = SharedState::new();
    let mut engine_st = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine_st.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });
    // step_modes default is StepTime — explicit for clarity:
    shared_st.step_modes[0].store(StepMode::StepTime as u8, Ordering::Release);
    shared_st.step_modes[1].store(StepMode::StepTime as u8, Ordering::Release);

    const T_START: u64 = 0;
    const DURATION: u64 = 26_000_000; // 50 ms @ 520 MHz
    let seg_st =
        build_corexy_xjog_noisy_y_segment(&pool_st, 0, 1, 1, T_START, DURATION);
    engine_st.push_segment(seg_st, &mut q_producer, &shared_st).expect("push");
    loop {
        if engine_st.producer_step(&pool_st, &mut q_consumer, &shared_st)
            == ProducerTickResult::AllIdle
        {
            break;
        }
    }
    let st_m0_ring = engine_st.step_ring(0).expect("m0 ring");
    let st_m0_total = st_m0_ring.available();
    let mut st_m0_net: i32 = 0;
    let mut st_m0_pos: u32 = 0;
    let mut st_m0_neg: u32 = 0;
    for _ in 0..st_m0_total {
        let (_t, dir) = st_m0_ring.peek_head().expect("entry");
        st_m0_net += dir as i32;
        if dir > 0 {
            st_m0_pos += 1;
        } else if dir < 0 {
            st_m0_neg += 1;
        }
        st_m0_ring.advance(1);
    }
    let st_m1_ring = engine_st.step_ring(1).expect("m1 ring");
    let st_m1_total = st_m1_ring.available();
    let mut st_m1_net: i32 = 0;
    for _ in 0..st_m1_total {
        let (_t, dir) = st_m1_ring.peek_head().expect("entry");
        st_m1_net += dir as i32;
        st_m1_ring.advance(1);
    }

    // --- Modulated side ---
    let pool_md = CurvePool::new();
    let shared_md = SharedState::new();
    let mut engine_md = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine_md.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 160.0, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });
    shared_md.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);
    shared_md.step_modes[1].store(StepMode::Modulated as u8, Ordering::Release);

    let seg_md =
        build_corexy_xjog_noisy_y_segment(&pool_md, 0, 1, 1, T_START, DURATION);
    // Modulated path reads from producer_current directly; pre-seed and
    // pass an empty queue Consumer (the lazy-dequeue path is exercised by
    // engine_modulated_tick.rs's dedicated regression test).
    engine_md.producer_current = Some(seg_md);
    let md_queue: &'static mut heapless::spsc::Queue<
        runtime::segment::Segment,
        { runtime::queue::Q_N },
    > = Box::leak(Box::new(heapless::spsc::Queue::new()));
    let (_md_qp, mut md_qc) = md_queue.split();

    // Tick at 40 kHz across the segment.
    const TICK_HZ: u64 = 40_000;
    const TICK_PERIOD_CYCLES: u64 = CLOCK_FREQ as u64 / TICK_HZ; // 13_000
    let n_ticks = DURATION / TICK_PERIOD_CYCLES + 5; // overshoot t_end to trigger retire
    for i in 1..=n_ticks {
        let now = T_START + i * TICK_PERIOD_CYCLES;
        engine_md.runtime_modulated_tick(now, &mut md_qc, &pool_md, &shared_md);
    }
    let md_m0_net = shared_md.stepper_counts[0].load(Ordering::Acquire);
    let md_m1_net = shared_md.stepper_counts[1].load(Ordering::Acquire);

    eprintln!(
        "[compare] STEP_TIME   m0: total={} pos={} neg={} net={}, m1: net={}",
        st_m0_total, st_m0_pos, st_m0_neg, st_m0_net, st_m1_net,
    );
    eprintln!(
        "[compare] MODULATED   m0: net={}, m1: net={} (signed step count via stepper_counts)",
        md_m0_net, md_m1_net,
    );

    // Both modes should produce the same NET direction (+160 microsteps on
    // motor A and motor B for a +1mm X jog with constant-mean Y). If they
    // diverge, that's the divergence point we need to explain.
    let st_net = st_m0_net;
    let md_net = md_m0_net;
    eprintln!(
        "[compare] m0 net delta: step_time={} modulated={} diff={}",
        st_net, md_net, st_net - md_net,
    );
    eprintln!(
        "[compare] m1 net delta: step_time={} modulated={} diff={}",
        st_m1_net, md_m1_net, st_m1_net - md_m1_net,
    );

    // The test PURPOSE is observation, not pass/fail: print the diff and
    // let humans interpret. But assert that BOTH produce non-zero motion in
    // the correct (positive) direction, since otherwise neither side works.
    assert!(
        md_net >= 100,
        "modulated must emit ≥100 net positive microsteps on motor A (real X motion)",
    );
    assert!(
        st_net >= 100,
        "step_time must emit ≥100 net positive microsteps on motor A; got {}",
        st_net,
    );
}
