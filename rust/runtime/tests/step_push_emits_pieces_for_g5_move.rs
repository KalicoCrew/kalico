//! Regression test: `isr_step_push_count` must be non-zero after a jog move.
//!
//! ### Bench symptom (historical)
//!
//! On `sota-motion` HEAD before the t_local f32-cancellation fix
//! (`5be894004`) and the u32→u64 widening fix (`de160ee0c`), the H7's
//! diagnostic telemetry showed:
//!   - `isr_armed_count > 0`  (EA tag fires — segment IS dequeued + armed)
//!   - `isr_deq_some_count > 0` (ED tag fires — dequeue path executed)
//!   - `isr_step_push_count == 0` (no step entries ever pushed to the queue)
//!   - `isr_last_signed_steps == 0` (every dispatch_pulse call sees signed_steps=0)
//!
//! Motors don't move. The engine is armed and evaluating, but position never
//! advances.
//!
//! ### Root cause (frozen clock)
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
//! segment in the queue **and `raw_cyccnt` advancing by `cycles_per_sample`
//! each iteration** (exactly as the real H7 TIM5 ISR observes between
//! firings):
//!
//!   - `shared.isr_step_push_count > 0`    — step entries were pushed
//!   - `shared.isr_last_signed_steps != 0` — at least one sample had
//!     non-zero step demand
//!
//! The clock advancement is the single structural difference between this
//! test and the degenerate frozen-clock case. A frozen clock (constant
//! `raw_cyccnt = 0`) produces `t_local = 0` every tick → no steps. An
//! advancing clock drives `t_local` forward across the Bézier → ~800
//! microsteps over 100 ms.

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

/// Verifies the engine emits step pulses for a 10 mm X-axis G5 jog when the
/// mock cycle counter advances by `cycles_per_sample` each iteration —
/// matching how the real H7 TIM5 ISR advances between firings.
///
/// With an advancing clock `WidenState::widen` returns a monotonically
/// increasing `now_cycles_u64`, `t_local` grows from 0 to 0.1 s across the
/// segment's Bézier, and ~800 microsteps are produced (10 mm /
/// 0.0125 mm·microstep⁻¹). The test asserts `isr_step_push_count > 0` and
/// `isr_last_signed_steps != 0` — both of which would be 0 if the clock were
/// frozen.
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
        // t_start = 0 so the segment arms on the first tick (now = widen(0)
        // satisfies seg.t_start <= now → 0 <= 0 on the very first call).
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
    // KEY FIX vs. the frozen-clock degenerate case:
    //
    // Advance `raw_cyccnt` by `cycles_per_sample` (H7_CLOCK_HZ /
    // SAMPLE_RATE_HZ = 13 000 cycles) before each `isr_sample_tick` call —
    // exactly what the H7 TIM5 ISR observes between firings. This makes
    // `WidenState::widen` return a monotonically increasing `now_cycles_u64`
    // so `t_local` grows from 0 to 0.1 s across the segment's Bézier.
    //
    // Without this advancement: widen(0) = 0 every tick, t_local = 0,
    // p_end = 0, signed_steps = 0 → no steps pushed (the frozen-clock bug).
    // -----------------------------------------------------------------------
    let cycles_per_sample = H7_CLOCK_HZ / SAMPLE_RATE_HZ; // 13 000

    // 100 ms at 40 kHz = 4 000 samples. Tail past 4 000 by ~5 ms so the
    // curve's t_local crosses `piece.duration` (the `t_local <= duration`
    // guard in `advance_piece_if_needed` requires strict `>`); the retire
    // path fires on the next post-pass after exhaustion.
    let mut raw_cyccnt: u32 = 0;
    for _ in 0..4_200 {
        raw_cyccnt = raw_cyccnt.wrapping_add(cycles_per_sample);
        runtime::tick::isr_sample_tick(&mut isr, &shared, &pool, raw_cyccnt);
        // Drain the step queue so it doesn't saturate (the depth-32 ring
        // would fill in ~4 ms at 800 steps / 100 ms = 8 steps/ms). Mirrors
        // the jog_repro harness discipline.
        unsafe { while runtime::step_queue::pop(queue_ptrs[0]).is_some() {} }
    }

    let push_count = shared.isr_step_push_count.load(Ordering::Acquire);
    assert!(
        push_count > 0,
        "isr_step_push_count must be > 0 after a 10 mm X jog segment is armed \
         and the engine ticks for 100 ms worth of samples with an advancing \
         clock; got {push_count}. \
         \n\nIf this is 0, the engine path has regressed: either the segment \
         was never armed, or dispatch_pulse sees signed_steps=0 every tick \
         despite now_cycles_u64 advancing."
    );

    let last_signed = shared.isr_last_signed_steps.load(Ordering::Acquire);
    assert_ne!(
        last_signed,
        0,
        "isr_last_signed_steps must be non-zero (some sample must have produced \
         a non-zero step demand) during a 10 mm X jog with an advancing clock; \
         got {last_signed}."
    );
}
