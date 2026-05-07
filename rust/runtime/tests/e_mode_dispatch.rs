//! E-mode dispatch tests for the per-axis scalar engine evaluator.
//! Validates CoupledToXy, Independent, and Travel E modes.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
)]

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::config::EMode;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_SEGMENT_END, TRACE_RING_N, TraceSample};

mod fixtures;

const CLOCK_FREQ: u32 = 520_000_000;

/// Test scaffolding matching engine_tick.rs pattern.
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

#[allow(clippy::cast_possible_truncation)]
fn raw_cyccnt(now: u64) -> u32 {
    now as u32
}

#[test]
fn coupled_e_accumulates_arc_length() {
    let mut h = Harness::new();

    // X linear: 0 → 50 mm. Y constant at 0 (sentinel).
    let (deg, knots, cps) = fixtures::linear_scalar(0.0, 50.0);
    let x_handle = fixtures::load_scalar(&h.pool, 0, deg, &knots, &cps);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n_ticks = 100u64;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: n_ticks * tc,
            kinematics: KinematicTag::CartesianXyzAndE, // identity transform for simplicity
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.04,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    for tick_idx in 0..=n_ticks {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed");
    }

    // Drain trace and get the last sample.
    let mut out = [TraceSample::default(); 256];
    let n = h.drain_trace(&mut out);
    assert!(n > 0, "expected at least one trace sample");
    let last = &out[n - 1];

    // E_final should be approximately 0.04 * 50 = 2.0 mm.
    // Cartesian identity: motor_e = e.
    let expected_e = 0.04 * 50.0;
    let tolerance = 0.05; // 50 um tolerance for discretization
    assert!(
        (last.motor_e - expected_e).abs() < tolerance,
        "expected E ~ {expected_e}, got {}",
        last.motor_e,
    );
}

#[test]
fn independent_e_tracks_nurbs() {
    let mut h = Harness::new();

    // X/Y constant (sentinels). E NURBS linear 10 → 5.
    let (e_deg, e_knots, e_cps) = fixtures::linear_scalar(10.0, 5.0);
    let e_handle = fixtures::load_scalar(&h.pool, 0, e_deg, &e_knots, &e_cps);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n_ticks = 20u64;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle: CurveHandle::UNUSED_SENTINEL,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle,
            t_start: 0,
            t_end: n_ticks * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Independent,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    for tick_idx in 0..=n_ticks {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed");
    }

    let mut out = [TraceSample::default(); 64];
    let n = h.drain_trace(&mut out);
    assert!(n > 0);

    // First sample (u~0): E near 10. Last sample (u~1): E near 5.
    let first = &out[0];
    let last = &out[n - 1];
    assert!(
        (first.motor_e - 10.0).abs() < 0.5,
        "first E should be near 10, got {}",
        first.motor_e,
    );
    assert!(
        (last.motor_e - 5.0).abs() < 0.5,
        "last E should be near 5, got {}",
        last.motor_e,
    );
}

#[test]
fn travel_e_stays_constant() {
    let mut h = Harness::new();

    // Pre-set the E accumulator by running a short coupled segment first.
    let (x_deg, x_knots, x_cps) = fixtures::linear_scalar(0.0, 10.0);
    let x_handle1 = fixtures::load_scalar(&h.pool, 0, x_deg, &x_knots, &x_cps);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n1 = 10u64;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle: x_handle1,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: n1 * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.04,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Second segment: travel (X moving, E should stay constant).
    let (x_deg2, x_knots2, x_cps2) = fixtures::linear_scalar(10.0, 30.0);
    let x_handle2 = fixtures::load_scalar(&h.pool, 1, x_deg2, &x_knots2, &x_cps2);
    let n2 = 10u64;
    h.q_producer
        .enqueue(Segment {
            id: 2,
            x_handle: x_handle2,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: n1 * tc,
            t_end: (n1 + n2) * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick through both segments.
    for tick_idx in 0..=(n1 + n2) {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed");
    }

    let mut out = [TraceSample::default(); 64];
    let n = h.drain_trace(&mut out);

    // Find the last sample of seg1 and all samples of seg2.
    let mut last_seg1_e: Option<f32> = None;
    let mut seg2_e_values = Vec::new();
    for s in out.iter().take(n) {
        if s.segment_id == 1 {
            last_seg1_e = Some(s.motor_e);
        }
        if s.segment_id == 2 {
            seg2_e_values.push(s.motor_e);
        }
    }

    let e_at_travel_start = last_seg1_e.expect("expected seg1 samples");
    assert!(e_at_travel_start > 0.0, "E should be non-zero after coupled segment");

    // All seg2 (Travel) E values should equal the e_at_travel_start.
    for (i, &e_val) in seg2_e_values.iter().enumerate() {
        assert!(
            (e_val - e_at_travel_start).abs() < 0.001,
            "seg2 tick {i}: E should be constant at {e_at_travel_start}, got {e_val}",
        );
    }
}

#[test]
fn xy_seed_prevents_spurious_extrusion() {
    let mut h = Harness::new();

    // First segment starts at X=100 mm. Without the XY seed, the first tick
    // would compute dx = 100 - 0 = 100, producing a large spurious E delta.
    let (x_deg, x_knots, x_cps) = fixtures::linear_scalar(100.0, 110.0);
    let x_handle = fixtures::load_scalar(&h.pool, 0, x_deg, &x_knots, &x_cps);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n_ticks = 10u64;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: n_ticks * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.04,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick once.
    h.tick(raw_cyccnt(0)).expect("first tick should succeed");

    let mut out = [TraceSample::default(); 4];
    let n = h.drain_trace(&mut out);
    assert!(n >= 1, "expected at least one trace sample");

    // First-tick E should be near zero (not 0.04 * 100 = 4.0).
    // At u=0 the XY seed sets prev_x = X(0) = 100, so dx=0 on the first eval.
    let first_e = out[0].motor_e;
    assert!(
        first_e.abs() < 0.01,
        "first-tick E should be ~0 (XY seed active), got {first_e}",
    );

    // Run remaining ticks.
    for tick_idx in 1..=n_ticks {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed");
    }

    // Final E should be 0.04 * (110 - 100) = 0.4, NOT 0.04 * 110.
    let mut out2 = [TraceSample::default(); 64];
    let n2 = h.drain_trace(&mut out2);
    let last = &out2[n2 - 1];
    let expected_e = 0.04 * 10.0; // 0.4
    assert!(
        (last.motor_e - expected_e).abs() < 0.05,
        "final E should be ~{expected_e}, got {}",
        last.motor_e,
    );
}

/// Regression: the boundary loop must emit TRACE_FLAG_SEGMENT_END for motion
/// segments (not just hold segments). Without this, the reclaim pipeline never
/// calls `confirm_retired` for motion-segment curve pool handles.
#[test]
fn boundary_loop_emits_segment_end_for_motion_segments() {
    let mut h = Harness::new();

    // Create a motion segment whose duration is less than one tick cycle, so
    // it expires entirely within the boundary loop on the first tick.
    let (x_deg, x_knots, x_cps) = fixtures::linear_scalar(0.0, 10.0);
    let x_handle = fixtures::load_scalar(&h.pool, 0, x_deg, &x_knots, &x_cps);

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // Duration < 1 tick — the segment is already expired by t_start + tc.
    let short_duration = tc / 2;

    // Need a second segment to land in after the boundary loop retires seg 1.
    let (x_deg2, x_knots2, x_cps2) = fixtures::linear_scalar(10.0, 20.0);
    let x_handle2 = fixtures::load_scalar(&h.pool, 1, x_deg2, &x_knots2, &x_cps2);

    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: short_duration,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    h.q_producer
        .enqueue(Segment {
            id: 2,
            x_handle: x_handle2,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: short_duration,
            t_end: short_duration + 100 * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick once at t = tc (past seg 1's t_end). Seg 1 should be retired in the
    // boundary loop and a SEGMENT_END trace emitted for it.
    h.tick(raw_cyccnt(tc)).expect("tick should succeed");

    let mut out = [TraceSample::default(); 16];
    let n = h.drain_trace(&mut out);
    assert!(n > 0, "expected trace samples");

    // Find SEGMENT_END for segment id=1 (the motion segment retired in the
    // boundary loop).
    let seg_end_for_1 = out[..n]
        .iter()
        .any(|s| s.segment_id == 1 && (s.flags & TRACE_FLAG_SEGMENT_END) != 0);
    assert!(
        seg_end_for_1,
        "boundary loop must emit SEGMENT_END for motion segments (seg id=1)"
    );
}

/// Regression: the boundary loop must sync e_accumulator when retiring an
/// Independent-E segment, so a subsequent CoupledToXy segment starts E
/// integration from the Independent segment's endpoint (not stale state).
#[test]
fn boundary_loop_syncs_e_accumulator_for_independent_segments() {
    let mut h = Harness::new();

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Segment 1: Independent E with NURBS going from 0 to 7.5 mm.
    // Duration < 1 tick so it expires entirely in the boundary loop.
    let (e_deg, e_knots, e_cps) = fixtures::linear_scalar(0.0, 7.5);
    let e_handle = fixtures::load_scalar(&h.pool, 0, e_deg, &e_knots, &e_cps);

    let short_duration = tc / 2;

    // Segment 2: CoupledToXy — X moves 0 → 10 mm, ratio = 0.04.
    // E should start from 7.5 (the Independent endpoint) and accumulate
    // 0.04 * 10 = 0.4, ending at ~7.9.
    let (x_deg, x_knots, x_cps) = fixtures::linear_scalar(0.0, 10.0);
    let x_handle = fixtures::load_scalar(&h.pool, 1, x_deg, &x_knots, &x_cps);

    let n2_ticks = 20u64;

    h.q_producer
        .enqueue(Segment {
            id: 1,
            x_handle: CurveHandle::UNUSED_SENTINEL,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle,
            t_start: 0,
            t_end: short_duration,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::Independent,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    h.q_producer
        .enqueue(Segment {
            id: 2,
            x_handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: short_duration,
            t_end: short_duration + n2_ticks * tc,
            kinematics: KinematicTag::CartesianXyzAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.04,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Tick through the full range.
    // First tick is at tc (past seg 1's t_end); seg 1 retires in boundary loop.
    for tick_idx in 1..=(n2_ticks + 1) {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed");
    }

    let mut out = [TraceSample::default(); 256];
    let n = h.drain_trace(&mut out);
    assert!(n > 0, "expected trace samples");

    // Find the last sample for segment 2.
    let last_seg2 = out[..n]
        .iter()
        .rev()
        .find(|s| s.segment_id == 2)
        .expect("expected samples for segment 2");

    // E should be approximately 7.5 + 0.04 * 10 = 7.9.
    // Without the fix, E would be ~0.04 * 10 = 0.4 (stale e_accumulator = 0).
    let expected_e = 7.5 + 0.04 * 10.0;
    let tolerance = 0.15; // generous for discretization
    assert!(
        (last_seg2.motor_e - expected_e as f32).abs() < tolerance,
        "E should be ~{expected_e} (Independent endpoint 7.5 + coupled 0.4), got {}",
        last_seg2.motor_e,
    );
}
