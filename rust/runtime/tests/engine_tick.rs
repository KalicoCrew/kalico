//! Integration tests for `Engine::tick`. Spec §4.2.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
)]

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{
    TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_SEGMENT_END, TRACE_RING_N, TraceSample,
};

// Default H723 Klipper Kconfig clock is 520 MHz (src/stm32/Kconfig). Keeping
// tests parametric here so a future bump to 550 MHz (or different alternate
// kconfig) doesn't invalidate the fixture math.
const CLOCK_FREQ: u32 = 520_000_000;

mod fixtures; // see Task 17a — shared step5_segments.json parser

/// Test scaffolding mirroring `RuntimeContext`'s SPSC split. Owns the
/// queue/trace backing storage so the `'static`-bound producer/consumer
/// halves stay alive across `engine.tick()` calls.
///
/// Step-6 Phase 1 Task 1.1 changed `Engine::tick`'s signature to take the
/// half-split `Consumer<Segment, Q_N>` + `Producer<TraceSample,
/// TRACE_RING_N>` directly. Tests own the backing `Queue`s on the heap and
/// leak them so the resulting halves are `'static` for the duration of the
/// test. Heap leaks are fine in unit tests; production paths use the
/// static-storage path inside `RuntimeContext::init`.
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
        // Box::leak the queues so the producer/consumer halves get a
        // `'static` lifetime. Each test-process leaks ~80 KB; cargo test
        // tears down the process per binary, so this never accumulates in
        // production.
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

/// Load fixture-by-name into the curve pool slot. Returns the freshly-issued
/// `CurveHandle` so the caller can use it to construct a `Segment`. Single
/// source of truth for "which curves the Step-5 tests use" — mirrored by
/// Surface C's host script.
fn load_fixture(pool: &CurvePool, slot_idx: u16, name: &str) -> CurveHandle {
    let set = fixtures::load();
    let f = set
        .fixtures
        .iter()
        .find(|f| f.name == name)
        .unwrap_or_else(|| panic!("fixture {name} missing from step5_segments.json"));
    let cps_flat: Vec<f32> = f
        .control_points
        .iter()
        .flat_map(|p| p.iter().copied())
        .collect();
    pool.validate_and_load(slot_idx, &cps_flat, &f.knots, &f.weights, f.degree)
        .unwrap()
}

// Helper: t_segment in u32 (engine widens internally). Since we start at
// raw_cyccnt = 0, no wrap concerns within these tests' tick budgets.
#[allow(clippy::cast_possible_truncation)]
fn raw_cyccnt(now: u64) -> u32 {
    now as u32
}

#[test]
fn tick_on_empty_queue_returns_idle() {
    let mut h = Harness::new();
    let r = h.tick(0);
    assert!(r.is_ok());
    assert_eq!(h.engine.status(), RuntimeStatus::Idle);
}

#[test]
fn tick_processes_one_segment_to_completion() {
    let mut h = Harness::new();
    let handle = load_fixture(&h.pool, 0, "straight_line_x");

    let tick_cycles = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n_ticks = 4u64;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            curve_handle: handle,
            t_start: 0,
            t_end: n_ticks * tick_cycles,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        })
        .unwrap();

    // Tick repeatedly through the segment.
    for tick_idx in 0..=n_ticks {
        let now = tick_idx * tick_cycles;
        h.tick(raw_cyccnt(now))
            .expect("tick should succeed in healthy run");
    }

    // Drain trace and verify samples emitted along the line.
    let mut out = [TraceSample::default(); 16];
    let n = h.drain_trace(&mut out);
    assert!(
        n >= 4,
        "expected at least 4 samples along the line, got {n}"
    );

    // Last sample at u≈1 → motors at endpoint, segment-end flag set.
    let last = &out[n - 1];
    assert_eq!(last.flags & TRACE_FLAG_SEGMENT_END, TRACE_FLAG_SEGMENT_END);
}

#[test]
fn sub_tick_boundary_carries_partial_into_next_segment() {
    let mut h = Harness::new();

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // Two distinct fixtures back-to-back — exercise sub-tick boundary carry.
    // straight_line_x ends at (10,0,0); rational_quadratic_arc starts at
    // (10,0,0) so the boundary is geometrically continuous and motor_a
    // increases monotonically across the seam.
    let h0 = load_fixture(&h.pool, 0, "straight_line_x");
    let h1 = load_fixture(&h.pool, 1, "rational_quadratic_arc");

    // Sized so that tick 1 lands near u≈1 of seg1 and tick 2 (post-boundary)
    // lands near u≈0 of seg2 — the only configuration that satisfies the
    // 0.05 mm seam tolerance with the 4-tick test loop. (D1 = tc + 1 cycle
    // → tick 1 at u = tc/(tc+1) ≈ 1; D2 = 1000·tc → tick 2 at u ≈ 0.001.)
    let d1 = tc + 1;
    let d2 = 1000 * tc;
    h.q_producer
        .enqueue(Segment {
            id: 1,
            curve_handle: h0,
            t_start: 0,
            t_end: d1,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        })
        .unwrap();
    h.q_producer
        .enqueue(Segment {
            id: 2,
            curve_handle: h1,
            t_start: d1,
            t_end: d1 + d2,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        })
        .unwrap();

    // Tick at t = 0, tc, 2tc, 3tc — third tick straddles the seg1→seg2 boundary.
    for tick_idx in 0..=3u64 {
        h.tick(raw_cyccnt(tick_idx * tc))
            .expect("tick should succeed in healthy run");
    }

    let mut out = [TraceSample::default(); 16];
    let n = h.drain_trace(&mut out);

    // Boundary correctness check: the LAST sample of segment 1 and the FIRST
    // sample of segment 2 must agree on (motor_a, motor_b, motor_e) to within
    // the sub-tick boundary tolerance. straight_line_x ends at (10, 0, 0);
    // rational_quadratic_arc starts at (10, 0, 0) — both yield motor = (10, 10, 0)
    // at the seam. Per-sample monotonicity over the whole trace is NOT asserted
    // (the arc's motor_a rises to ~14.14 mid-arc and falls back to 10 at u=1
    // because motor_a = X+Y and the arc's path through (10,10,0) increases X+Y).
    let mut last_seg1: Option<&TraceSample> = None;
    let mut first_seg2: Option<&TraceSample> = None;
    for s in out.iter().take(n) {
        if s.segment_id == 1 {
            last_seg1 = Some(s);
        }
        if s.segment_id == 2 && first_seg2.is_none() {
            first_seg2 = Some(s);
        }
    }
    let last1 = last_seg1.expect("expected at least one sample from segment 1");
    let first2 = first_seg2.expect("expected at least one sample from segment 2");

    // Seam tolerance: 25 µm × 2 (start + end of tick) = 50 µm = 0.05 mm.
    const SEAM_TOL_MM: f32 = 0.05;
    assert!(
        (first2.motor_a - last1.motor_a).abs() < SEAM_TOL_MM,
        "motor_a discontinuous at seam: {} → {}",
        last1.motor_a,
        first2.motor_a
    );
    assert!(
        (first2.motor_b - last1.motor_b).abs() < SEAM_TOL_MM,
        "motor_b discontinuous at seam: {} → {}",
        last1.motor_b,
        first2.motor_b
    );
    assert!(
        (first2.motor_e - last1.motor_e).abs() < SEAM_TOL_MM,
        "motor_e discontinuous at seam: {} → {}",
        last1.motor_e,
        first2.motor_e
    );
}

#[test]
fn invalid_curve_handle_latches_fault() {
    // Spec §5.5. Engine resolves an unloaded handle → InvalidHandle fault,
    // status latches to Fault, last_error code is set, fault marker emitted.
    let mut h = Harness::new();

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // Use a never-issued handle (gen=1) — the slot is empty (gen=0), so
    // lookup mismatches and the engine latches InvalidHandle.
    h.q_producer
        .enqueue(Segment {
            id: 1,
            curve_handle: CurveHandle::new(0, 1),
            t_start: 0,
            t_end: tc * 2,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
        })
        .unwrap();

    let r = h.tick(0);
    assert!(r.is_err());
    assert_eq!(h.engine.status(), RuntimeStatus::Fault);
    assert_eq!(
        h.engine.last_error(),
        runtime::error::KALICO_ERR_INVALID_HANDLE
    );

    // Trace has a fault-marker sample (last-known-good motors, segment_id=1).
    let mut out = [TraceSample::default(); 8];
    let n = h.drain_trace(&mut out);
    assert_eq!(n, 1, "exactly one fault-marker sample expected");
    assert_ne!(
        out[0].flags & TRACE_FLAG_FAULT_MARKER,
        0,
        "fault marker bit must be set"
    );
    assert_eq!(
        out[0].segment_id, 1,
        "fault marker carries the active segment id"
    );
}

// `boundary_loop_exhausted_latches_fault`: deferred to Step 6+.
//
// The plan (Task 17b, lines 2828–2829) calls for pushing 9+ short segments
// to drive the boundary loop past `MAX_BOUNDARY_ITERS = 8`. With the current
// `SegmentQueue` (heapless 0.8 `Queue<_, Q_N>` where Q_N=8 → effective
// capacity 7), the maximum number of segments accessible to a single tick is
// 1 (already in `engine.current` after the idle-pop) + 7 (queued) = 8. The
// boundary loop's 8th `try_pop` returns `None` and the engine takes the
// "drained" branch (status = Drained, no error) before the `iters > 8` check
// can fire on a 9th iteration. So `BoundaryLoopExhausted` is defense-in-depth
// code today, not reachable through the public API surface.
//
// TODO Step-6: when `Q_N` is bumped or the boundary-loop bound changes, add
// a real test here. Options surveyed:
//   (a) Bump `Q_N` to ≥10 to allow 9 successful pops in one tick.
//   (b) Lower `MAX_BOUNDARY_ITERS` (would change defense-in-depth headroom).
//   (c) Add a `#[cfg(test)] pub fn force_boundary_iters(...)` injection point.
// Decision deferred until Step 6's live producer protocol is in place — the
// final shape of `Q_N` and the bound is best resolved then, not now.

// `nan_or_inf_from_eval_latches_fault`: deferred to Step 6+.
//
// `CurvePool::load` performs producer-side validation (NaN/Inf rejection,
// non-monotone knots, non-positive weights, clamping, n_cp ≥ degree+1) so
// every curve that enters the pool is well-formed. With the Step-5 stub
// evaluator (`nurbs::vector_eval`) on a validated curve, NaN-from-eval is
// not reachable through the public API. The plan (Task 17b lines 2829–
// 2830) explicitly anticipates this case and instructs to skip it with a
// TODO when both upstream-rejection routes (degree-1 with `weights[0]=0`
// and `f32::NAN` in a control point) are blocked at load time:
//   - `weights[0] = 0.0` → rejected by `CurvePoolError::InvalidCurve`
//     (non-positive weight check).
//   - `f32::NAN` in any field → rejected by `CurvePoolError::NonFiniteData`.
//
// TODO Step-6: revisit when (a) the evaluator gains arc-length parameter
// inversion that can introduce floating-point ill-conditioning at runtime
// or (b) Step-9 tanh-PA introduces a runtime-evaluated transform that can
// produce NaN from finite inputs (tanh-of-large is fine, but division paths
// inside PA may not be). The fault path itself is exercised by
// `invalid_curve_handle_latches_fault` above (same code path: `latch_fault`
// + trace marker + last_error), so the *fault-handling machinery* is
// already covered; only the specific NaN-detection trigger is not.
