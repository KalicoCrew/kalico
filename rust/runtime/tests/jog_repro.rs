#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

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

use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TRACE_RING_N, TraceSample};

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

fn absolute_linear_curve(start_mm: f32, end_mm: f32, duration_sec: f32) -> WirePiece {
    let delta = end_mm - start_mm;
    WirePiece {
        bp0_bits: start_mm.to_bits(),
        bp1_bits: (start_mm + delta / 3.0).to_bits(),
        bp2_bits: (start_mm + 2.0 * delta / 3.0).to_bits(),
        bp3_bits: end_mm.to_bits(),
        duration_bits: duration_sec.to_bits(),
    }
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn configured_engine() -> EngineImpl {
    // Engine::new now accepts (clock_hz, sample_rate_hz) and computes
    // sample_period_sec at construction time — the Codex 2026-05-20 gap #2
    // fix. Both values come from C-side constants in production
    // (runtime_clock_freq / runtime_sample_rate_hz in src/runtime_tick.c).
    let mut e = EngineImpl::new(H7_CLOCK_HZ, SAMPLE_RATE_HZ);
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
fn engine_new_publishes_sample_period() {
    // Verifies Codex gap #2 fix: `Engine::new(clock_hz, sample_rate_hz)` now
    // derives and stores `sample_period_sec` + `sample_period_cycles` at
    // construction time, so `tick_sample`'s `sample_period_sec <= 0.0` guard
    // never fires in production.
    //
    // Before this fix both `Engine::new` and `init_in_place` wrote `0.0`,
    // leaving `tick_sample` permanently stuck at the guard — motors never moved
    // regardless of segment queue depth.
    let engine = configured_engine();

    let expected_sec = 1.0_f32 / SAMPLE_RATE_HZ as f32;
    let expected_cycles = H7_CLOCK_HZ / SAMPLE_RATE_HZ; // integer division matches impl

    assert!(
        engine.sample_period_sec > 0.0,
        "sample_period_sec must be positive after Engine::new; got {}",
        engine.sample_period_sec
    );
    assert!(
        (engine.sample_period_sec - expected_sec).abs() < 1e-9,
        "sample_period_sec expected {expected_sec} (1/{SAMPLE_RATE_HZ}), got {}",
        engine.sample_period_sec
    );
    assert_eq!(
        engine.sample_period_cycles, expected_cycles,
        "sample_period_cycles expected {expected_cycles} ({H7_CLOCK_HZ}/{SAMPLE_RATE_HZ})"
    );
}

#[test]
fn isr_sample_tick_arms_queued_segment_and_pushes_steps() {
    // Codex M1 + M2 regression test (2026-05-20). Drives the same scenario
    // the bench failed on — a 10 mm X jog with a single segment + single
    // cubic-Bezier piece — but exercises the **production ISR sample path**
    // (`runtime::tick::isr_sample_tick`) instead of bypassing it via direct
    // `Engine::arm_segment` calls. That bypass is what the pre-fix harness
    // did, and it's the reason this test passed even while the real bench
    // was stuck at "queue depth=0, no steps." With the new wiring the only
    // segment-activation path is the dequeue-and-arm inside
    // `isr_sample_tick`; if that path were removed, every assertion below
    // would fall back to the pre-2026-05-20 silent-no-motion behaviour.
    //
    // Pre-fix expectation: `engine.current` stays `None`, `position_count`
    // stays at 0, this `assert!(>= 700)` fails.
    // Post-fix expectation: arm-from-queue + widen-then-publish drive the
    // same ~800-step output that the unit-level evaluator produced
    // historically.

    use core::ptr::addr_of_mut;

    use heapless::spsc::Queue;
    use runtime::c_segment_queue;
    use runtime::clock::WidenState;
    use runtime::state::IsrState;

    let mut engine = configured_engine();

    // Install observable host-side step queues so `dispatch_axis` has
    // somewhere to push entries.
    let mut step_queues = [
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
    ];
    let queue_ptrs = [
        addr_of_mut!(step_queues[0]),
        addr_of_mut!(step_queues[1]),
        addr_of_mut!(step_queues[2]),
        addr_of_mut!(step_queues[3]),
    ];
    engine.test_install_step_queues(queue_ptrs);

    // Build the ISR-side envelope by hand. `c_segment_queue` is a singleton
    // backed by a host `Mutex<VecDeque<Segment>>`; reset it so this test
    // does not see leftovers from a sibling test in the same crate.
    c_segment_queue::reset();
    let queue_consumer = c_segment_queue::Consumer::<Segment>::new();
    let mut queue_producer = c_segment_queue::Producer::<Segment>::new();
    // `heapless::spsc::Queue::split` needs a `'static mut`. `Box::leak`
    // gives us that without growing the test stack — the leaked queue
    // outlives the test process, which is fine.
    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (trace_producer, _trace_consumer) = trace_queue.split();

    let mut isr = IsrState {
        queue_consumer,
        trace_producer,
        engine,
        widen_state: WidenState::default(),
        pending_segment: None,
    };

    // Load the cubic-Bezier curve into the pool + enqueue a segment via
    // the **producer** side (the production foreground path) — never call
    // `arm_segment` from the test.
    let pool = CurvePool::new();
    let piece = linear_jog_curve(10.0, 0.1);
    let handle = pool.try_alloc_and_load(0, &[piece]).expect("slot 0 alloc");
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
    queue_producer.enqueue(seg).expect("queue producer enqueue");

    let shared = SharedState::new();
    // The new tick path widens the raw cyccnt INSIDE `isr_sample_tick` and
    // publishes via the §11.4 seqlock; the test only needs to feed a
    // monotonically advancing `raw` value. We advance one sample-period
    // worth of cycles each iteration — exactly what the H7 TIM5 ISR
    // observes between firings.
    let cycles_per_sample = H7_CLOCK_HZ / SAMPLE_RATE_HZ;
    let mut raw_cyccnt: u32 = 0;
    // 100 ms at 40 kHz = 4 000 samples — the segment's full duration. We
    // drain the per-axis X StepQueue each iteration to mimic the per-axis
    // SysTick consumer (`kalico_per_axis_step_event`); without that drain
    // the depth-32 ring saturates inside ~4 ms and the producer silently
    // drops trailing entries, which is a test-setup artefact, not a
    // production bug. Tail past 4 000 by ~5 ms so the curve's t_local
    // crosses `piece.duration` (the `t_local <= duration` guard in
    // `advance_piece_if_needed` requires strict `>`); the retire path
    // fires on the next post-pass after exhaustion.
    for _ in 0..4200 {
        raw_cyccnt = raw_cyccnt.wrapping_add(cycles_per_sample);
        runtime::tick::isr_sample_tick(&mut isr, &shared, &pool, raw_cyccnt);
        unsafe { while runtime::step_queue::pop(queue_ptrs[0]).is_some() {} }
    }

    // Production observable: the X stepper's signed pulse counter.
    // 10 mm at 0.0125 mm/microstep = 800 microsteps; Newton sub-sample
    // emission may differ by a handful at the edges.
    let stepper_count = isr.engine.stepping_axes[0].steppers[0]
        .position_count
        .load(Ordering::Acquire);
    assert!(
        stepper_count.abs() >= 700,
        "FFI-equivalent isr_sample_tick path: 10mm @ 0.0125 mm/microstep \
         over 100ms should drive ~800 microsteps on axis X; got {stepper_count}. \
         If this is 0, the ISR's arm-from-queue wiring (Codex M1) regressed: \
         the segment sat in the queue forever, engine.current stayed None, \
         and dispatch_axis never pushed step entries."
    );

    // Cross-check: the widened-now seqlock must have been ticking — Codex
    // M2 regression surface. If `widened_now_lo` is still zero after 4 000
    // ticks the ISR's `publish_widened_now` call is gone.
    let widened_now = runtime::clock::read_widened_now(&shared);
    assert!(
        widened_now > 0,
        "Codex M2 regression: isr_sample_tick must call publish_widened_now \
         every sample; widened-now is still 0 after 4 000 ticks."
    );

    // The segment should have retired by the end of the 100 ms loop.
    let retired = shared.retired_through_segment_id.load(Ordering::Acquire);
    assert_eq!(
        retired, 1,
        "segment 1 should have retired by sample 4000; \
         retired_through_segment_id = {retired}. Likely cause: arm-from-queue \
         never fired so retire bookkeeping had no current to clear."
    );
}

#[test]
fn seeded_absolute_jog_after_set_kinematic_position_pushes_steps() {
    use core::ptr::addr_of_mut;

    use heapless::spsc::Queue;
    use runtime::c_segment_queue;
    use runtime::clock::WidenState;
    use runtime::state::IsrState;

    let mut engine = configured_engine();
    engine.seed_position([100.0, 100.0, 10.0]);

    let seed_count = engine.stepping_axes[0].last_step_count;
    assert_eq!(
        seed_count, 8000,
        "X seed at 100mm with 0.0125mm/microstep must initialize last_step_count"
    );
    assert_eq!(
        engine.tick_caches.p_prev[0], 100.0,
        "SET_KINEMATIC_POSITION must seed the secant-slope position cache"
    );

    let mut step_queues = [
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
    ];
    let queue_ptrs = [
        addr_of_mut!(step_queues[0]),
        addr_of_mut!(step_queues[1]),
        addr_of_mut!(step_queues[2]),
        addr_of_mut!(step_queues[3]),
    ];
    engine.test_install_step_queues(queue_ptrs);

    c_segment_queue::reset();
    let queue_consumer = c_segment_queue::Consumer::<Segment>::new();
    let mut queue_producer = c_segment_queue::Producer::<Segment>::new();
    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (trace_producer, _trace_consumer) = trace_queue.split();

    let mut isr = IsrState {
        queue_consumer,
        trace_producer,
        engine,
        widen_state: WidenState::default(),
        pending_segment: None,
    };

    let pool = CurvePool::new();
    let piece = absolute_linear_curve(100.0, 110.0, 0.1);
    let handle = pool.try_alloc_and_load(0, &[piece]).expect("slot 0 alloc");
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
    queue_producer.enqueue(seg).expect("queue producer enqueue");

    let shared = SharedState::new();
    let cycles_per_sample = H7_CLOCK_HZ / SAMPLE_RATE_HZ;
    let mut raw_cyccnt: u32 = 0;
    for _ in 0..4200 {
        raw_cyccnt = raw_cyccnt.wrapping_add(cycles_per_sample);
        runtime::tick::isr_sample_tick(&mut isr, &shared, &pool, raw_cyccnt);
        unsafe { while runtime::step_queue::pop(queue_ptrs[0]).is_some() {} }
    }

    let final_count = isr.engine.stepping_axes[0].steppers[0]
        .position_count
        .load(Ordering::Acquire);
    assert!(
        (final_count - seed_count).abs() >= 700,
        "absolute 100mm->110mm jog after SET_KINEMATIC_POSITION should emit \
         about 800 new X microsteps; seed={seed_count}, final={final_count}"
    );
    assert!(
        shared.isr_step_push_count.load(Ordering::Acquire) > 0,
        "seeded absolute jog must push step entries instead of tripping the \
         oversized catch-up guard every sample"
    );
}
