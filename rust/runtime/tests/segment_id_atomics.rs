//! `SharedState` segment-id atomic writers — Round-2 review B14.
//!
//! `Engine::tick` must publish `current_segment_id` on activation and
//! `retired_through_segment_id` on retirement. The push-side
//! `accepted_segment_id` atomic is exercised by the kalico-c-api FFI tests
//! (it lives in `push_segment_impl`, not `Engine`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::queue::Q_N;
use runtime::config::EMode;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_RING_N, TraceSample};

const CLOCK_FREQ: u32 = 520_000_000;

struct Harness {
    engine: Engine<NoopPa, NoopIs>,
    widen: WidenState,
    pool: CurvePool,
    shared: SharedState,
    q_producer: heapless::spsc::Producer<'static, Segment, Q_N>,
    q_consumer: heapless::spsc::Consumer<'static, Segment, Q_N>,
    t_producer: heapless::spsc::Producer<'static, TraceSample, TRACE_RING_N>,
    _t_consumer: heapless::spsc::Consumer<'static, TraceSample, TRACE_RING_N>,
}

impl Harness {
    fn new() -> Self {
        let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
        let (q_producer, q_consumer) = queue.split();
        let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
            Box::leak(Box::new(Queue::new()));
        let (t_producer, t_consumer) = trace.split();
        let shared = SharedState::new();
        // Step 7-B: homed gate — set homed=true so segment activation works.
        shared.homed.store(true, core::sync::atomic::Ordering::Release);
        Self {
            engine: Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ),
            widen: WidenState::default(),
            pool: CurvePool::new(),
            shared,
            q_producer,
            q_consumer,
            t_producer,
            _t_consumer: t_consumer,
        }
    }

    fn tick(&mut self, raw_cyccnt: u32) {
        let _ = self.engine.tick(
            raw_cyccnt,
            &mut self.widen,
            &self.pool,
            &mut self.q_consumer,
            &mut self.t_producer,
            &self.shared,
        );
    }
}

fn straight_line_curve_handle(pool: &CurvePool, slot: u16) -> CurveHandle {
    let cps = [0.0_f32, 1.0];
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    pool.validate_and_load(slot, 1, &knots, &cps)
        .unwrap()
}

#[test]
fn current_segment_id_set_on_activation() {
    let mut h = Harness::new();
    let handle = straight_line_curve_handle(&h.pool, 0);
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    h.q_producer
        .enqueue(Segment {
            id: 42,
            x_handle: handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: 4 * tc,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Pre-activation, current_segment_id is the initial 0.
    assert_eq!(
        h.shared.current_segment_id.load(Ordering::Acquire),
        0,
        "current_segment_id starts at 0"
    );

    // First tick activates the segment.
    h.tick(0);
    assert_eq!(
        h.shared.current_segment_id.load(Ordering::Acquire),
        42,
        "activation must publish the current segment id"
    );
}

#[test]
fn retired_through_segment_id_advances_on_retire() {
    let mut h = Harness::new();
    let handle = straight_line_curve_handle(&h.pool, 0);
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    h.q_producer
        .enqueue(Segment {
            id: 7,
            x_handle: handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: 2 * tc,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick across the whole segment + one past, drives retirement.
    for tick_idx in 0..=3u64 {
        // SAFETY (truncation): test fixtures stay well below 2^32 cycles.
        #[allow(clippy::cast_possible_truncation)]
        h.tick((tick_idx * tc) as u32);
    }

    let retired = h.shared.retired_through_segment_id.load(Ordering::Acquire);
    assert_eq!(
        retired, 7,
        "retired_through_segment_id must follow the just-finished segment id"
    );
}

#[test]
fn retired_through_segment_id_monotonic_across_two_segments() {
    let mut h = Harness::new();
    let handle = straight_line_curve_handle(&h.pool, 0);
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    let d1 = tc * 2;
    let d2 = tc * 2;
    h.q_producer
        .enqueue(Segment {
            id: 3,
            x_handle: handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: d1,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();
    // Stage a second segment that uses the same slot once gen advances —
    // emulate the retire-then-realloc cycle by enqueuing with the existing
    // handle (still valid since the slot's current_gen hasn't changed yet).
    h.q_producer
        .enqueue(Segment {
            id: 4,
            x_handle: handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: d1,
            t_end: d1 + d2,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick across both segments.
    for tick_idx in 0..=5u64 {
        #[allow(clippy::cast_possible_truncation)]
        h.tick((tick_idx * tc) as u32);
    }

    let retired = h.shared.retired_through_segment_id.load(Ordering::Acquire);
    assert!(
        retired >= 3,
        "retired_through_segment_id must advance, got {retired}"
    );
}
