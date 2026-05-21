//! Regression test: `isr_step_push_count` must be non-zero after a jog move.
//!
//! ### Bench symptom being reproduced
//!
//! On `sota-motion` HEAD, the H7's diagnostic telemetry shows:
//!   - `isr_armed_count > 0`  (EA tag fires — segment IS dequeued + armed)
//!   - `isr_deq_some_count > 0` (ED tag fires — dequeue path executed)
//!   - `isr_step_push_count == 0` (no step entries ever pushed to the queue)
//!   - `isr_last_signed_steps == 0` (every dispatch_pulse call sees signed_steps=0)
//!
//! Motors don't move. The engine is armed and evaluating, but position never
//! advances. This test reproduces that failure locally, without firmware.
//!
//! ### Root cause
//!
//! `dispatch_pulse` computes:
//!
//! ```text
//! t_local_cycles = now_cycles_u64.wrapping_sub(piece_start_time_cycles)
//! t_local        = (t_local_cycles as f32) / cycles_per_second
//! p_end          = eval_bezier(t_local)   // = 0.0 when t_local = 0.0
//! signed_steps   = round(p_end / microstep_distance) - position_count  // = 0
//! ```
//!
//! If `raw_cyccnt` fed to `isr_sample_tick` never advances, `WidenState::widen`
//! returns the same value every call. The segment is armed with
//! `piece_start_time_cycles = seg.t_start = 0`. On every subsequent tick:
//!
//! ```text
//! now_cycles_u64          = widen(0) = 0
//! t_local_cycles          = 0.wrapping_sub(0) = 0
//! t_local                 = 0.0
//! p_end                   = 0.0  (curve evaluated at its start)
//! signed_steps            = 0 - 0 = 0   ← dispatch_pulse returns early
//! isr_step_push_count     += 0  (never bumped)
//! ```
//!
//! ### What this test asserts
//!
//! After 4 200 calls to `isr_sample_tick` with a 10 mm X-axis cubic Bézier
//! segment in the queue **and `raw_cyccnt` held constant at 0**:
//!
//!   - `shared.isr_step_push_count > 0`     — **FAILS on current HEAD**
//!   - `shared.isr_last_signed_steps != 0`  — **FAILS on current HEAD**
//!
//! To fix: advance `raw_cyccnt` by `H7_CLOCK_HZ / SAMPLE_RATE_HZ` (13 000
//! cycles) each iteration — exactly what `jog_repro.rs`'s passing test does.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;
use runtime::c_segment_queue;
use runtime::clock::WidenState;
use runtime::config::EMode;
use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::{IsrState, SharedState};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TraceSample, TRACE_RING_N};

type EngineImpl = Engine<NoopPa, NoopIs>;

const H7_CLOCK_HZ: u32 = 520_000_000;
const SAMPLE_RATE_HZ: u32 = 40_000;

/// Linear 10 mm X-only cubic Bézier piece over `duration_sec`.
///
/// Bernstein control points: `[0, d/3, 2d/3, d]` where `d = displacement_mm`.
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
    let mut e = EngineImpl::new(H7_CLOCK_HZ, SAMPLE_RATE_HZ);
    let binding = pulse_binding();
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, &[binding]), 0);
    assert_eq!(e.configure_kinematics(1.0), 0);
    e
}

/// Reproduces the bench symptom: segment IS armed (EA/ED tags fire), but
/// `isr_step_push_count` stays 0 because `raw_cyccnt` never advances past
/// the segment's `t_start`, leaving `t_local = 0` on every tick.
///
/// **This test FAILS on `sota-motion` HEAD.**
///
/// To make it pass: advance `raw_cyccnt` by `H7_CLOCK_HZ / SAMPLE_RATE_HZ`
/// per iteration so `now_u64` grows and `t_local` increases over the curve.
#[test]
fn step_push_emits_pieces_for_g5_move() {
    use core::ptr::addr_of_mut;

    let mut engine = configured_engine();

    // Install observable host-side step queues.
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

    // Build the ISR envelope.
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

    // Load a 10 mm linear X-axis move: 100 ms at 100 mm/s.
    let pool = CurvePool::new();
    let piece = linear_jog_curve(10.0, 0.1);
    let handle = pool.try_alloc_and_load(0, &[piece]).expect("slot 0 alloc");

    let mut seg = Segment {
        id: 1,
        x_handle: handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        // t_start = 0 so that the armed piece_start_time_cycles = 0 and
        // widen(0) - 0 = 0 every tick when raw_cyccnt is held constant.
        t_start: 0,
        t_end: ((0.1_f64) * f64::from(H7_CLOCK_HZ)) as u64,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
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
    queue_producer.enqueue(seg).expect("enqueue segment");

    let shared = SharedState::new();

    // -----------------------------------------------------------------------
    // KEY DIFFERENCE from jog_repro.rs's PASSING test: raw_cyccnt does NOT
    // advance between iterations. It stays at 0 the entire loop.
    //
    // Effect: WidenState::widen(0) returns 0 every call.
    //         now_cycles_u64 = 0 every tick.
    //         t_local_cycles = 0.wrapping_sub(0) = 0 every tick.
    //         t_local        = 0.0 every tick.
    //         p_end          = eval_bezier(0.0) = 0.0 every tick.
    //         signed_steps   = 0 every tick.
    //         dispatch_pulse returns early → isr_step_push_count never bumped.
    //
    // The segment is armed (isr_armed_count > 0) — the ISR dequeues it,
    // checks t_start (0) ≤ now (0), and calls arm_segment. EA/ED tags fire.
    // But no steps are pushed. Exactly the bench symptom.
    // -----------------------------------------------------------------------
    let raw_cyccnt: u32 = 0; // constant — does NOT advance

    for _ in 0..4_200 {
        runtime::tick::isr_sample_tick(&mut isr, &shared, &pool, raw_cyccnt);
        // Drain the step queue so it doesn't saturate (irrelevant here since
        // nothing pushes, but mirrors the jog_repro harness for consistency).
        unsafe { while runtime::step_queue::pop(queue_ptrs[0]).is_some() {} }
    }

    // -----------------------------------------------------------------------
    // These two assertions FAIL on sota-motion HEAD.
    //
    // isr_armed_count > 0 would pass (segment was armed), confirming the
    // EA/ED tags fire. But the step-push diagnostics never advance.
    // -----------------------------------------------------------------------

    let push_count = shared.isr_step_push_count.load(Ordering::Acquire);
    assert!(
        push_count > 0,
        "isr_step_push_count must be > 0 after a 10 mm X jog segment is armed \
         and the engine ticks for 100 ms worth of samples; got {push_count}. \
         \n\nBench symptom reproduced: segment is armed (EA/ED tags fire) but \
         dispatch_pulse sees signed_steps=0 every tick because now_cycles_u64 \
         never advances past piece_start_time_cycles (both = 0), so t_local = 0 \
         and p_end = 0 every sample. Fix: advance raw_cyccnt by \
         cycles_per_sample (H7_CLOCK_HZ / SAMPLE_RATE_HZ = 13 000) each tick."
    );

    let last_signed = shared.isr_last_signed_steps.load(Ordering::Acquire);
    assert_ne!(
        last_signed,
        0,
        "isr_last_signed_steps must be non-zero (some sample must have produced \
         a non-zero step demand) during a 10 mm X jog; got {last_signed}. \
         \n\nThis stays 0 because t_local is frozen at 0.0 every tick — \
         the Bézier is evaluated at its starting point (p_end = 0 mm) on every \
         sample, so the signed step delta is always 0."
    );
}
