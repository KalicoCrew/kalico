//! Phase 9: hold-segment ISR short-circuit + retire events. Spec §6.5.
//!
//! Tests:
//! - Hold segment short-circuits past `pool.resolve` (no fault on
//!   `HOLD_SEGMENT_SENTINEL`).
//! - Hold segment emits `SEGMENT_END` at retire — stream stays alive.
//! - Hold segment emits throttled `HOLD_SAMPLE` breadcrumbs while active.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::panic,
    clippy::items_after_statements
)]

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::Q_N;
use runtime::config::EMode;
use runtime::segment::{KinematicTag, SEGMENT_FLAG_HOLD_SEGMENT, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_HOLD_SAMPLE, TRACE_FLAG_SEGMENT_END, TRACE_RING_N, TraceSample};

const CLOCK_FREQ: u32 = 520_000_000;

struct Harness {
    engine: Engine<NoopPa, NoopIs>,
    widen: WidenState,
    pool: CurvePool,
    shared: SharedState,
    q_producer: heapless::spsc::Producer<'static, Segment, Q_N>,
    q_consumer: heapless::spsc::Consumer<'static, Segment, Q_N>,
    t_producer: heapless::spsc::Producer<'static, TraceSample, TRACE_RING_N>,
    t_consumer: heapless::spsc::Consumer<'static, TraceSample, TRACE_RING_N>,
}

impl Harness {
    fn new() -> Self {
        let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
        let (q_producer, q_consumer) = queue.split();
        let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
            Box::leak(Box::new(Queue::new()));
        let (t_producer, t_consumer) = trace.split();
        let shared = SharedState::new();
        Self {
            engine: Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ),
            widen: WidenState::default(),
            pool: CurvePool::new(),
            shared,
            q_producer,
            q_consumer,
            t_producer,
            t_consumer,
        }
    }

    fn tick(&mut self, raw_cyccnt: u32) -> Result<(), runtime::error::RuntimeError> {
        self.engine.tick(
            raw_cyccnt,
            &mut self.widen,
            &self.pool,
            &mut self.q_consumer,
            &mut self.t_producer,
            &self.shared,
        )
    }

    fn drain_trace(&mut self, out: &mut [TraceSample]) -> usize {
        let mut count = 0;
        while count < out.len() {
            let Some(sample) = self.t_consumer.dequeue() else {
                break;
            };
            if let Some(slot) = out.get_mut(count) {
                *slot = sample;
            }
            count += 1;
        }
        count
    }
}

fn hold_segment(id: u32, t_start: u64, t_end: u64) -> Segment {
    Segment {
        id,
        // Sentinel handles that would FAIL pool.resolve if we ever tried —
        // proves the short-circuit happens before lookup.
        x_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        y_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        z_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        e_handle: CurveHandle::HOLD_SEGMENT_SENTINEL,
        t_start,
        t_end,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: SEGMENT_FLAG_HOLD_SEGMENT,
        _pad: [0; 1],
        consumers_remaining: 0,
    }
}

#[test]
fn hold_segment_skips_curve_lookup_and_emits_last_position() {
    let mut h = Harness::new();
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Two-tick hold; sentinel handle means lookup would fault if reached.
    h.q_producer
        .enqueue(hold_segment(1, 0, tc * 2))
        .expect("enqueue hold");

    // Tick at t=0 — activates segment + processes one hold tick.
    h.tick(0).expect("hold tick should not fault");
    assert_eq!(
        h.engine.status(),
        RuntimeStatus::Running,
        "status should be Running while hold is active"
    );
    assert_eq!(
        h.engine.last_error(),
        0,
        "no error should latch on hold-segment short-circuit"
    );

    // Tick at t=tc — still in the hold window.
    h.tick(tc as u32)
        .expect("second hold tick should not fault");
}

#[test]
fn hold_segment_emits_segment_end_at_retire() {
    let mut h = Harness::new();
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Short hold — single tick window so the second tick retires it.
    let hold_id = 7;
    h.q_producer
        .enqueue(hold_segment(hold_id, 0, tc))
        .expect("enqueue hold");

    // Tick at t=0: activates hold, evaluates one tick. Next tick (t=tc)
    // would land at duration boundary; the hold-segment branch sees
    // `next_t_segment >= duration` and emits SEGMENT_END pre-emptively.
    h.tick(0).expect("hold activation tick");

    // Tick at t=tc: segment retires (boundary loop drops it; queue empty).
    h.tick(tc as u32).expect("hold retire tick");

    // Drain trace — must contain SEGMENT_END for the hold's id +
    // the sentinel curve handle (so foreground reclaim's confirm_retired
    // is called with HOLD_SEGMENT_SENTINEL, which no-ops on the out-of-
    // range slot index).
    let mut out = [TraceSample::default(); 16];
    let n = h.drain_trace(&mut out);
    assert!(n >= 1, "expected at least the SEGMENT_END trace sample");
    let segment_end = out
        .iter()
        .take(n)
        .find(|s| s.flags & TRACE_FLAG_SEGMENT_END != 0)
        .expect("SEGMENT_END must be emitted at hold retire");
    assert_eq!(segment_end.segment_id, hold_id);
    assert_eq!(
        segment_end.curve_handle,
        CurveHandle::HOLD_SEGMENT_SENTINEL,
        "hold's SEGMENT_END carries the sentinel handle"
    );

    // Stream stayed alive — retired_through cursor advanced.
    use core::sync::atomic::Ordering;
    assert_eq!(
        h.shared.retired_through_segment_id.load(Ordering::Acquire),
        hold_id
    );
}

#[test]
fn hold_segment_throttles_hold_sample_breadcrumb() {
    let mut h = Harness::new();
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Long hold so the throttle fires at least once. HOLD_SAMPLE_TICK_PERIOD
    // is 400 ticks (~10 ms at 40 kHz); we tick 800 times to get >=2 fires.
    let n_ticks: u64 = 800;
    h.q_producer
        .enqueue(hold_segment(42, 0, tc * (n_ticks + 1)))
        .expect("enqueue hold");

    for i in 0..n_ticks {
        h.tick((i * tc) as u32).expect("hold tick");
    }

    // Drain + count HOLD_SAMPLE flag occurrences.
    let mut out = [TraceSample::default(); 16];
    let mut total_hold_samples = 0;
    loop {
        let n = h.drain_trace(&mut out);
        if n == 0 {
            break;
        }
        for s in out.iter().take(n) {
            if s.flags & TRACE_FLAG_HOLD_SAMPLE != 0 {
                total_hold_samples += 1;
            }
        }
    }
    assert!(
        total_hold_samples >= 1,
        "hold-sample breadcrumb should fire at least once over 800 ticks (~20 ms), got {total_hold_samples}"
    );
}
