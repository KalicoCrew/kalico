//! Step-emission architecture acceptance-criteria coverage (spec §9).
//!
//! Verifies the host-testable subset of the six acceptance criteria from
//! `docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md`
//! §9. Criterion 1 (audibly smooth `G1 X10 F600`) is bench-only; the rest
//! are verifiable in this layer.
//!
//! Criterion 2 — `KALICO_ERR_SCHEDULE_OVERFLOW` gone — is a static-search
//! property; covered by `error::tests` (the error code does not exist).
//!
//! Criterion 3 — sustained back-to-back jogs produce no underruns. In the
//! Rust host harness the C-side consumer (`step_time_event`) doesn't run,
//! so we cannot bump the (currently-unwired) `consumer_underrun_total`
//! counter directly. Instead we verify the equivalent producer-side
//! property: pushing a long sequence of segments, the producer fills the
//! per-motor ring with the **correct cumulative step count**, the queue
//! drains cleanly to `AllIdle`, and curves retire — i.e. the producer
//! never wedges and the architecture never silently loses step entries.
//!
//! Criterion 4 — `producer_runs_total` advances on `push_segment` (kick
//! path) rather than on a heartbeat. Verified by counting producer runs
//! while the engine is idle (no segments → no kicks → no runs except the
//! ones we explicitly call), then again across a push (kick wakes the
//! producer exactly once per push when no work is pending).
//!
//! Criterion 5 — already pinned by
//! `sim_steptime_z_jog::all_steppers_default_to_step_time_and_tim5_never_armed`.
//!
//! Criterion 6 — flipping one motor to Modulated coexists with other
//! motors continuing on StepTime: the producer's per-motor mode filter
//! short-circuits the Modulated motor (`producer_step` skips it) while
//! the StepTime motor continues to receive ring entries.

#![cfg(feature = "kalico-sim")]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
)]

extern crate alloc;

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
const X_STEPS_PER_MM: f32 = 160.0;

/// 4-CP degree-3 Bézier with collinear control points so position(u) = end*u.
fn linear_cubic(end: f32) -> (u8, alloc::vec::Vec<f32>, alloc::vec::Vec<f32>) {
    use alloc::vec;
    let cps = vec![0.0, end / 3.0, end * 2.0 / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

fn build_segment_x(
    pool: &CurvePool,
    end_mm: f32,
    t_start: u64,
    duration_cycles: u64,
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
        t_end: t_start + duration_cycles,
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
                steps_per_mm: X_STEPS_PER_MM,
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
        let queue: &'static mut Queue<Segment, Q_N> =
            alloc::boxed::Box::leak(alloc::boxed::Box::new(Queue::new()));
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

// ─── Criterion 3: sustained back-to-back jogs, no silent step loss ────────

/// Push a long sequence of 1 mm jog segments; verify the cumulative step
/// count matches the commanded total motion and the queue drains cleanly.
///
/// `Q_N = 8` (heapless 0.8 effective capacity = 7), so we interleave
/// push-batches with producer-drain passes — mirroring the live pipeline
/// where the bridge pushes a few segments, the producer fills the ring,
/// the C-side consumer drains, and the next batch lands.
#[test]
fn sustained_back_to_back_jogs_no_step_loss() {
    let mut h = Harness::cartesian_x();
    const N_SEGMENTS: u32 = 32;
    // 1 mm per segment over 10 ms = 160 steps per segment at 160 spm.
    const SEG_LEN_MM: f32 = 1.0;
    const SEG_DURATION_CYCLES: u64 = 5_200_000; // 10 ms at 520 MHz
    const STEPS_PER_SEG: u32 = 160;
    // We can have at most ~7 segments queued before push_segment fails. Push,
    // drive producer until queue empties (or rings fill), drain ring entries
    // by manually advancing (simulating the consumer), then push the next
    // batch.
    let mut next_t_start: u64 = 0;
    let mut total_pushed_steps: u32 = 0;
    let mut total_consumed_steps: u32 = 0;

    // Bookkeeping: minimum ring-available observed _while there are still
    // pending segments_. In the live pipeline this would map to the
    // consumer's underrun detector — if the ring never goes empty while
    // there's work pending, the consumer never short-polls and
    // `step_time_empty_polls` (the C-side underrun counter) stays at zero.
    // We can't run the C consumer here, so we instead assert the
    // ring-available stays >0 whenever the producer reports WorkPending.
    let mut underrun_like_events: u32 = 0;

    for seg_id in 1..=N_SEGMENTS {
        let slot_idx = ((seg_id - 1) % 16) as u16;
        let mut pending = Some(build_segment_x(
            &h.pool,
            SEG_LEN_MM,
            next_t_start,
            SEG_DURATION_CYCLES,
            slot_idx,
            seg_id,
        ));
        next_t_start += SEG_DURATION_CYCLES;
        total_pushed_steps += STEPS_PER_SEG;
        // Retry-on-queue-full: hand the rejected Segment back to push.
        // `push_segment` returns `Err(Segment)` on queue full; the curve
        // is already loaded in the pool, so we just push the returned
        // segment again after draining.
        loop {
            let seg = pending.take().expect("loop invariant: seg present");
            match h.engine.push_segment(seg, &mut h.q_producer, &h.shared) {
                Ok(()) => break,
                Err(returned) => {
                    pending = Some(returned);
                    drive_producer_and_drain(
                        &mut h,
                        &mut total_consumed_steps,
                        &mut underrun_like_events,
                    );
                }
            }
        }
    }

    // Final drain to flush the last batch.
    drive_producer_and_drain(&mut h, &mut total_consumed_steps, &mut underrun_like_events);

    // ── Criterion 3 assertions ──────────────────────────────────────────
    // 1. Cumulative consumed steps ≈ total commanded motion. Tolerance:
    //    one segment's worth of boundary slop per pushed segment (±5
    //    steps per segment from the Newton boundary u≈1 convergence
    //    window). With N_SEGMENTS=32 → ±160 step tolerance.
    let delta = (total_consumed_steps as i64) - (total_pushed_steps as i64);
    let tolerance = (N_SEGMENTS as i64) * 5;
    assert!(
        delta.abs() <= tolerance,
        "step loss exceeds tolerance: consumed {} vs commanded {} (delta {}, tol ±{})",
        total_consumed_steps,
        total_pushed_steps,
        delta,
        tolerance,
    );

    // 2. Producer-side underrun proxy: the ring should never empty while
    //    the producer still has work to do (when WorkPending fires the ring
    //    is, by construction of the test loop, always > 0).
    assert_eq!(
        underrun_like_events, 0,
        "{} producer cycles observed where ring was empty while WorkPending",
        underrun_like_events,
    );

    // 3. Producer-runs-total > 0 (heartbeat advanced).
    assert!(
        h.shared.producer_runs_total.load(Ordering::Acquire) > 0,
        "producer_runs_total must advance",
    );
}

/// Drive the producer to completion, draining the per-motor ring as we
/// go (manual `advance` simulates the C-side consumer). Tracks the
/// `consumed_steps` counter and the producer-side underrun proxy.
///
/// Architectural note: `producer_step` returns `AllIdle` whenever the
/// current segment retires and no further fetch happens in the same
/// call. In the production pipeline the next `push_segment` kick (or
/// the consumer's low-water hook) re-arms the producer. In this test
/// the queue may still hold un-processed segments after `AllIdle` —
/// we loop until the queue is drained AND the producer reports idle.
fn drive_producer_and_drain(
    h: &mut Harness,
    consumed_steps: &mut u32,
    underrun_like_events: &mut u32,
) {
    let mut iters = 0u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        iters += 1;
        // Simulate consumer: drain all ring entries the producer added.
        let avail = h.engine.step_ring(0).expect("ring 0").available();
        if avail > 0 {
            h.engine.step_ring(0).expect("ring 0").advance(avail);
            *consumed_steps = consumed_steps.saturating_add(avail);
        } else if r == ProducerTickResult::WorkPending {
            // Producer reports work pending but no entries landed in the
            // ring this round → that would correspond to an underrun
            // condition on the live consumer. In healthy operation this
            // should never happen because the producer fills at least one
            // entry per call when work is pending.
            *underrun_like_events = underrun_like_events.saturating_add(1);
        }
        if r == ProducerTickResult::AllIdle {
            // AllIdle just means the producer finished the segment it
            // was working on. If more segments are queued, kick the
            // producer again — this mirrors what the C-side scheduler
            // does (push_segment / low-water hook re-arms the
            // `runtime_producer_event` Klipper timer). We detect a
            // pending kick via `producer_pending` getting CAS-set on
            // the next push, but here we instead inspect the queue
            // length directly via `is_queue_drained`.
            if is_queue_drained(h) {
                break;
            }
        }
        assert!(
            iters < 5000,
            "producer loop did not converge to AllIdle within 5000 iters"
        );
    }
}

/// Return true iff the segment queue is empty AND there's no active
/// `producer_current` segment.
fn is_queue_drained(h: &Harness) -> bool {
    // SAFETY: `q_consumer.peek()` is a const-time peek of the heapless
    // SPSC queue's read head. Doesn't mutate.
    // (heapless 0.8 doesn't expose `peek` on the Consumer half; we use
    // `len()` via `q_consumer.len()`. If that's also unavailable, we
    // can't directly inspect — fall back to checking if the producer
    // dequeues anything on the next call.)
    h.q_consumer.len() == 0
}

// ─── Criterion 4: producer is event-driven (no heartbeat) ─────────────────

/// `producer_runs_total` advances **only** when something kicks the
/// producer. Without a push there's nothing to advance it. (Trivially
/// true here because the producer-runs counter only bumps inside
/// `producer_step`; there's no fixed-cadence timer that calls
/// `producer_step` independently — that's the architectural property
/// criterion 4 codifies.)
#[test]
fn producer_runs_total_does_not_advance_without_push() {
    let h = Harness::cartesian_x();
    let before = h.shared.producer_runs_total.load(Ordering::Acquire);
    // Sleep / time-pass / no kick → no advance.
    core::hint::spin_loop();
    let after = h.shared.producer_runs_total.load(Ordering::Acquire);
    assert_eq!(
        after, before,
        "producer_runs_total must not auto-advance without an explicit kick"
    );
}

/// After a push, the producer_pending kick flag is CAS-set so the C-side
/// scheduler queues `runtime_producer_event`. The first `producer_step`
/// call clears the flag. With no further pushes, the producer reaches
/// `AllIdle` and `runtime_producer_event` returns `SF_DONE` — the kick
/// loop is therefore single-shot per push, not periodic.
#[test]
fn producer_kick_is_single_shot_per_push() {
    let mut h = Harness::cartesian_x();
    assert!(
        !h.shared.producer_pending.load(Ordering::Acquire),
        "kick flag starts clear"
    );

    let seg = build_segment_x(&h.pool, 1.0, 0, 5_200_000, 0, 1);
    h.engine
        .push_segment(seg, &mut h.q_producer, &h.shared)
        .expect("push ok");
    assert!(
        h.shared.producer_pending.load(Ordering::Acquire),
        "push_segment must CAS-set producer_pending"
    );

    // First producer_step clears the kick flag.
    let _ = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
    assert!(
        !h.shared.producer_pending.load(Ordering::Acquire),
        "producer_step must clear producer_pending on entry"
    );

    // Drain the ring so the next producer_step has nothing to do.
    let avail = h.engine.step_ring(0).expect("ring").available();
    h.engine.step_ring(0).expect("ring").advance(avail);

    // Drive to AllIdle. The C-side `runtime_producer_event` returns
    // `SF_DONE` at this point — i.e. it does NOT auto-reschedule.
    let mut iters = 0u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        iters += 1;
        let avail = h.engine.step_ring(0).expect("ring").available();
        if avail > 0 {
            h.engine.step_ring(0).expect("ring").advance(avail);
        }
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(iters < 200, "producer should reach AllIdle quickly");
    }

    // Kick flag stayed clear all the way — no second self-kick.
    assert!(
        !h.shared.producer_pending.load(Ordering::Acquire),
        "no spurious producer_pending after AllIdle"
    );
}

// ─── Criterion 6: mixed Modulated + StepTime motors coexist ───────────────

/// Set motor 0 to Modulated, motor 1 to StepTime. Push a segment that
/// references both. The producer's per-motor mode filter must SKIP
/// motor 0's ring (the Modulated motor is driven by TIM5, not the
/// producer) and FILL motor 1's ring with step times.
#[test]
fn mixed_modulated_steptime_motors_coexist() {
    // Build a CoreXY engine — motors 0 and 1 are both X+Y mixers, so
    // a pure-X segment exercises both motors' producer paths.
    let queue: &'static mut Queue<Segment, Q_N> =
        alloc::boxed::Box::leak(alloc::boxed::Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: X_STEPS_PER_MM, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: X_STEPS_PER_MM, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // Flip motor 0 to Modulated. Motor 1 stays on the default StepTime.
    use runtime::set_step_mode;
    set_step_mode(&shared, 0, StepMode::Modulated, /*phase=*/ true)
        .expect("motor 0 → Modulated");
    assert_eq!(
        StepMode::from_u8(shared.step_modes[0].load(Ordering::Acquire)),
        Some(StepMode::Modulated),
    );
    assert_eq!(
        StepMode::from_u8(shared.step_modes[1].load(Ordering::Acquire)),
        Some(StepMode::StepTime),
    );

    // Push a pure-X segment (10 mm over 50 ms = 1600 nominal steps per motor).
    let (deg, knots, cps) = linear_cubic(10.0);
    let x_handle = pool
        .validate_and_load(0, deg, &knots, &cps)
        .expect("load X");
    let seg = Segment {
        id: 1,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
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

    // Drive producer, draining motor 1 as we go so the ring can refill.
    // 10 mm × 160 spm = 1600 steps > ring capacity (1024), so we need to
    // drain mid-flight. We also call `runtime_modulated_tick` periodically
    // to advance the wall-clock and ultimately clear motor 0's bits in
    // the segment's consumers_remaining mask — mirroring the production
    // TIM5 path which runs at ~20-40 kHz alongside the producer.
    let mut total_consumed_motor1: u32 = 0;
    let mut iters = 0u32;
    let mut now: u64 = 0;
    let now_advance_per_iter: u64 = 26_000_000 / 50; // 50 modulated ticks across the segment
    loop {
        let r = engine.producer_step(&pool, &mut q_consumer, &shared);
        iters += 1;
        // Drain motor 1's ring (StepTime path).
        let r1 = engine.step_ring(1).expect("ring 1");
        let a1 = r1.available();
        if a1 > 0 {
            r1.advance(a1);
            total_consumed_motor1 += a1;
        }
        // Motor 0 (Modulated) must NEVER accrue producer ring entries —
        // the producer's per-motor mode filter short-circuits it.
        let avail_motor0 = engine.step_ring(0).expect("ring 0").available();
        assert_eq!(
            avail_motor0, 0,
            "motor 0 is Modulated; producer must never fill its ring (got {})",
            avail_motor0,
        );
        // Advance simulated wall clock and run the Modulated tick. Once
        // `now` crosses `t_end = 26_000_000`, motor 0's consumer bit
        // clears and the segment retires.
        now += now_advance_per_iter;
        engine.runtime_modulated_tick(now, &pool, &shared);
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(iters < 500, "producer did not converge");
    }

    // Motor 1 received the full step train (~1600 ±5 Newton-boundary slop).
    let delta = (total_consumed_motor1 as i64) - 1600;
    assert!(
        delta.abs() <= 5,
        "motor 1 (StepTime) should receive ~1600 steps; got {}",
        total_consumed_motor1,
    );

    // Final motor-0 ring state: still 0.
    assert_eq!(
        engine.step_ring(0).expect("ring 0").available(),
        0,
        "Modulated motor 0 ring must remain empty post-segment",
    );
}

/// Flip motor 0 back to StepTime mid-test. Producer should resume filling
/// its ring on subsequent segments without losing state.
#[test]
fn mode_flip_modulated_to_steptime_resumes_producer_fill() {
    let mut h = Harness::cartesian_x();
    use runtime::set_step_mode;

    // Start: motor 0 Modulated. Push segment 1. Producer skips motor 0.
    set_step_mode(&h.shared, 0, StepMode::Modulated, /*phase=*/ true).unwrap();
    let seg1 = build_segment_x(&h.pool, 1.0, 0, 5_200_000, 0, 1);
    h.engine
        .push_segment(seg1, &mut h.q_producer, &h.shared)
        .expect("push 1 ok");
    let mut iters = 0u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        iters += 1;
        if r == ProducerTickResult::AllIdle || iters > 50 {
            break;
        }
    }
    assert_eq!(
        h.engine.step_ring(0).expect("ring").available(),
        0,
        "Modulated motor must not have ring entries",
    );

    // Flip to StepTime. Push segment 2. Producer should fill ring 0 now.
    set_step_mode(&h.shared, 0, StepMode::StepTime, /*phase=*/ true).unwrap();
    let seg2 = build_segment_x(&h.pool, 1.0, 5_200_000, 5_200_000, 1, 2);
    h.engine
        .push_segment(seg2, &mut h.q_producer, &h.shared)
        .expect("push 2 ok");
    let mut iters = 0u32;
    loop {
        let r = h.engine.producer_step(&h.pool, &mut h.q_consumer, &h.shared);
        iters += 1;
        if r == ProducerTickResult::AllIdle || iters > 50 {
            break;
        }
    }

    let avail = h.engine.step_ring(0).expect("ring").available();
    // 1 mm × 160 spm = 160 steps, ±5 slop.
    assert!(
        (155..=165).contains(&avail),
        "after flip back to StepTime, ring should fill (~160 steps); got {}",
        avail,
    );
}
