//! Host-side reproduction of the bench's "jog doesn't move motors" failure.
//!
//! Built 2026-05-20 in response to the live-bench symptom: after the
//! stepping-redesign Tasks 1-21 + the post-bench-bring-up fixes (caps wire
//! schema, SPI rx mode, per-axis-timer host stubs), klippy reaches `ready`,
//! accepts `_CLIENT_LINEAR_MOVE X=-10 F=6000`, the planner shapes a curve,
//! the bridge dispatch closure pushes `LoadCurveCubic` + `PushSegment` to the
//! H7, and the engine's status frame stays `engine_status=0` with `queue_depth=0`
//! — no step pulses ever fire.
//!
//! Codex's 2026-05-20 analysis pointed at five real gaps; this harness pins
//! the first two as failing tests so we can drive them green without flashing
//! between iterations:
//!
//!   2. `Engine::tick_sample` returns at the `sample_period_sec <= 0.0` guard
//!      because nothing publishes the sample period. Both `Engine::new` and
//!      `init_in_place` write `0.0`; neither `configure_kinematics` nor any
//!      sibling setter promotes it. → `tick_sample_no_op_proves_sample_period_gate`.
//!
//!   1. The H7's `runtime_tick_enable` gates TIM5 on
//!      `count_modulated_steppers > 0`, but klippy sends all-StepTime
//!      configure_axes; so even after #2 is fixed, no per-sample tick fires.
//!      The "tick_sample_pushes_step_entries..." test below works around this
//!      by driving `tick_sample` from the harness loop directly (the unit-of-
//!      reproduction is the per-sample evaluator, not TIM5 itself), and
//!      `Engine::test_install_step_queues` replaces the `[null; N_AXES]` host
//!      stub so the SPSC step queues become observable.
//!
//! ### Iteration loop
//!
//! `cargo test -p runtime --test jog_repro` reproduces the bench failure in
//! ~2 seconds. Each Codex point becomes a `#[test]`; fixing production code
//! is "make the test pass", no firmware flash required.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TraceSample, TRACE_RING_N};

type EngineImpl = Engine<NoopPa, NoopIs>;

const H7_CLOCK_HZ: u32 = 520_000_000;
const SAMPLE_RATE_HZ: u32 = 40_000;

/// 10mm X-only linear move at 100 mm/s. Single cubic piece with linear
/// position profile P(t) = 100·t (mm) over t∈[0, 0.1]s.
///
/// Bernstein control points for a linear curve from 0 to `displacement_mm`
/// (with `t` already in seconds, not normalised — the runtime's evaluator
/// uses t_local in seconds, see tick_integration.rs comments) are
/// `[0, displacement/3, 2·displacement/3, displacement]` after time-scaling
/// by `duration`. For a `displacement = velocity · duration` linear move
/// that simplifies to `[0, v·dur/3, 2·v·dur/3, v·dur]`.
fn linear_jog_curve(displacement_mm: f32, duration_sec: f32) -> WirePiece {
    WirePiece {
        bp0_bits: (0.0_f32).to_bits(),
        bp1_bits: (displacement_mm / 3.0).to_bits(),
        bp2_bits: (2.0 * displacement_mm / 3.0).to_bits(),
        bp3_bits: displacement_mm.to_bits(),
        duration_bits: duration_sec.to_bits(),
    }
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust { tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 3] }
}

fn configured_engine() -> EngineImpl {
    let mut e = EngineImpl::new(H7_CLOCK_HZ);
    // X axis (0): Pulse mode, 0.0125 mm/microstep — matches the bench TMC5160
    // setup at 256-microstep on a 1.8° motor + 20-tooth GT2 belt.
    let binding = pulse_binding();
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, &[binding]), 0);
    // Y / Z / E left unconfigured. configure_kinematics(k_xy=1.0) is the
    // Cartesian default.
    assert_eq!(e.configure_kinematics(1.0), 0);
    e
}

#[test]
fn tick_sample_no_op_proves_sample_period_gate() {
    // Reproduces Codex point 2: with `Engine::new`'s default
    // `sample_period_sec = 0.0`, `tick_sample` short-circuits before
    // evaluating any loaded piece. Even with a fully-armed segment, the
    // X axis's `last_step_count` stays at zero across N ticks.
    let mut engine = configured_engine();
    // Deliberately DO NOT call `test_set_sample_period` — that's the
    // reproduction: production behaves the same way.

    let pool = CurvePool::new();
    let piece = linear_jog_curve(10.0, 0.1);
    let handle = pool
        .try_alloc_and_load(0, &[piece])
        .expect("slot 0 alloc");
    let mut seg = Segment {
        id: 1,
        x_handle: handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: ((0.1_f64) * f64::from(H7_CLOCK_HZ)) as u64,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: runtime::config::EMode::Travel,
        flags: 0,
        _pad: [0; 1],
        extrusion_ratio: 0.0,
        consumers_remaining: 0,
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );
    engine.arm_segment(seg, &pool);

    let shared = SharedState::new();
    let mut trace_storage: Queue<TraceSample, TRACE_RING_N> = Queue::new();
    let (mut trace_producer, _trace_consumer) = trace_storage.split();

    // Drive 100 samples — at 40 kHz that's 2.5 ms of simulated time, plenty
    // for the linear curve to advance multiple microsteps if the evaluator
    // were running.
    for _ in 0..100 {
        engine.tick_sample(&shared, &pool, &mut trace_producer);
    }

    let last_step_count = engine.stepping_axes[0].last_step_count;
    assert_eq!(
        last_step_count, 0,
        "Codex point 2 reproduction: tick_sample IS supposed to no-op here. \
         If this test fails with last_step_count > 0, sample_period_sec is \
         being set somewhere — update this test to flip to a 'should fail \
         pre-fix' marker."
    );
}

#[test]
fn tick_sample_pushes_step_entries_after_sample_period_set() {
    // Companion to the test above: once `sample_period_sec` is wired (via
    // `test_set_sample_period` — production needs an equivalent path), the
    // same setup should produce step entries on the X queue. This is the
    // "make it green" target — currently fails because beyond Codex 2, the
    // host-side queues aren't observable (Codex 5: stubbed-out path), and
    // Engine::tick_sample's host branch uses [null; N_AXES]. We install real
    // host queues via `test_install_step_queues` to close that gap.
    let mut engine = configured_engine();
    engine.test_set_sample_period(SAMPLE_RATE_HZ);

    // Install observable host-side queues so dispatch_axis can push.
    let mut queues = [StepQueue::new(), StepQueue::new(), StepQueue::new(), StepQueue::new()];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];
    engine.test_install_step_queues(queue_ptrs);

    let pool = CurvePool::new();
    let piece = linear_jog_curve(10.0, 0.1);
    let handle = pool
        .try_alloc_and_load(0, &[piece])
        .expect("slot 0 alloc");
    let mut seg = Segment {
        id: 1,
        x_handle: handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: 0,
        t_end: ((0.1_f64) * f64::from(H7_CLOCK_HZ)) as u64,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: runtime::config::EMode::Travel,
        flags: 0,
        _pad: [0; 1],
        extrusion_ratio: 0.0,
        consumers_remaining: 0,
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );
    engine.arm_segment(seg, &pool);

    let shared = SharedState::new();
    let mut trace_storage: Queue<TraceSample, TRACE_RING_N> = Queue::new();
    let (mut trace_producer, _trace_consumer) = trace_storage.split();

    // Drive the widened MCU clock manually. On the MCU,
    // `runtime_widened_host_clock` (src/runtime_tick.c) republishes
    // `timer_read_time()` widened into `SharedState::widened_now_lo` at the
    // producer Klipper-timer cadence (~1 kHz); the ISR's
    // `tick_sample` reads that value to derive `t_sample_end_global` for the
    // piece evaluator. Host builds have no such producer, so without an
    // explicit advance here the engine sees t=0 every iteration and the
    // Bezier evaluator never advances past the piece start.
    let cycles_per_sample = (H7_CLOCK_HZ / SAMPLE_RATE_HZ) as u32;
    let mut now_cycles: u32 = 0;
    // 100ms at 40 kHz = 4000 samples — exactly the curve's duration.
    for _ in 0..4000 {
        now_cycles = now_cycles.wrapping_add(cycles_per_sample);
        shared.widened_now_lo.store(now_cycles, Ordering::Release);
        engine.tick_sample(&shared, &pool, &mut trace_producer);
    }

    // 10 mm @ 0.0125 mm/microstep = 800 steps. The Newton-iterated step
    // emission may differ by ±1 vs the closed-form count.
    let stepper_count = engine.stepping_axes[0].steppers[0]
        .position_count
        .load(Ordering::Acquire);
    assert!(
        stepper_count.abs() >= 700,
        "after 100ms at 40 kHz over a 10mm move, the X stepper's \
         position_count should be near 800 microsteps, got {stepper_count}"
    );

    // Observe step queue depth: any non-zero tail proves the producer wrote
    // entries the per-axis timer would consume.
    let q_x_depth = unsafe {
        let q = queue_ptrs[0];
        core::ptr::read_volatile(&(*q).tail).wrapping_sub(
            core::ptr::read_volatile(&(*q).head),
        )
    };
    assert!(
        q_x_depth > 0,
        "step_queues[0] should have entries pending consumer pop after \
         producer ticks; got depth={q_x_depth}"
    );
}
