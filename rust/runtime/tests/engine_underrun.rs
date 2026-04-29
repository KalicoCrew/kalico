//! Step-6 §8.2 boundary-drain branch tests:
//!   queue empty + stream_open=false → Drained (or Idle initially).
//!   queue empty + stream_open=true  → KALICO_FAULT_UNDERRUN.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::doc_markdown
)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::clock::WidenState;
use runtime::curve_pool::CurvePool;
use runtime::engine::{Engine, RuntimeStatus};
use runtime::error::{KALICO_ERR_UNDERRUN, RuntimeError};
use runtime::queue::Q_N;
use runtime::segment::Segment;
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_RING_N, TraceSample};

const CLOCK_FREQ: u32 = 520_000_000;

struct Harness {
    engine: Engine<NoopPa, NoopIs>,
    widen: WidenState,
    pool: CurvePool,
    shared: SharedState,
    #[allow(dead_code)]
    q_producer: heapless::spsc::Producer<'static, Segment, Q_N>,
    q_consumer: heapless::spsc::Consumer<'static, Segment, Q_N>,
    t_producer: heapless::spsc::Producer<'static, TraceSample, TRACE_RING_N>,
    #[allow(dead_code)]
    t_consumer: heapless::spsc::Consumer<'static, TraceSample, TRACE_RING_N>,
}

impl Harness {
    fn new() -> Self {
        let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
        let (q_producer, q_consumer) = queue.split();
        let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
            Box::leak(Box::new(Queue::new()));
        let (t_producer, t_consumer) = trace.split();
        Self {
            engine: Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ),
            widen: WidenState::default(),
            pool: CurvePool::new(),
            shared: SharedState::new(),
            q_producer,
            q_consumer,
            t_producer,
            t_consumer,
        }
    }

    fn tick(&mut self, raw_cyccnt: u32) -> Result<(), RuntimeError> {
        self.engine.tick(
            raw_cyccnt,
            &mut self.widen,
            &self.pool,
            &mut self.q_consumer,
            &mut self.t_producer,
            &self.shared,
        )
    }
}

#[test]
fn empty_queue_stream_closed_yields_idle() {
    // Initial state: stream_open=false (default), queue empty. Engine stays
    // in Idle (its initial post-init state) — Drained is set only after a
    // segment has been retired.
    let mut h = Harness::new();
    assert!(!h.shared.stream_open.load(Ordering::Acquire));
    h.tick(0).expect("tick should succeed when stream closed");
    assert_eq!(h.engine.status(), RuntimeStatus::Idle);
    assert_eq!(h.engine.last_error(), 0);
}

#[test]
fn empty_queue_stream_open_yields_underrun_fault() {
    let mut h = Harness::new();
    h.shared.stream_open.store(true, Ordering::Release);
    let r = h.tick(0);
    assert!(matches!(r, Err(RuntimeError::Underrun)));
    assert_eq!(h.engine.status(), RuntimeStatus::Fault);
    assert_eq!(h.engine.last_error(), KALICO_ERR_UNDERRUN);
}
