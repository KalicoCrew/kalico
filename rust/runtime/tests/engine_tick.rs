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
use runtime::config::EMode;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_FLAG_FAULT_MARKER, TRACE_RING_N, TraceSample};

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

#[test]
fn tick_on_empty_queue_returns_idle() {
    let mut h = Harness::new();
    let r = h.tick(0);
    assert!(r.is_ok());
    assert_eq!(h.engine.status(), RuntimeStatus::Idle);
}

// `tick_processes_one_segment_to_completion` and
// `sub_tick_boundary_carries_partial_into_next_segment`: retired
// 2026-05-14 (step-emission T12 cleanup pass). Both tests asserted that
// `Engine::tick` emits per-tick `TraceSample`s along the curve so the test
// could observe motor positions sample-by-sample. After the 2026-05-13
// trace-emit consolidation (commit 5fbd2c6 — `Engine::tick` only enqueues
// trace samples on `TRACE_FLAG_SEGMENT_END`), per-tick sample emission no
// longer exists; the boundary-loop trajectory is observed instead through
// the post-segment SEGMENT_END sample's `motor_*` fields and the
// `step_rings` ring counters. The new architecture (T7+T10) tests this
// path via `engine_modulated_tick.rs` + `engine_producer_integration.rs`.

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
            x_handle: CurveHandle::new(0, 1),
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: 0,
            t_end: tc * 2,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        })
        .unwrap();

    let r = h.tick(0);
    assert!(r.is_err());
    assert_eq!(h.engine.status(), RuntimeStatus::Fault);
    assert_eq!(
        h.engine.last_error(),
        runtime::error::KALICO_ERR_INVALID_HANDLE
    );

    // Closure-review fix #3: `SharedState.fault_detail` MUST carry the
    // §9.2-encoded payload `(slot << 16) | (observed XOR expected)`. Prior
    // to the fix, latch_fault never wrote the atomic and host always saw 0.
    use core::sync::atomic::Ordering as AtomicOrdering;
    let detail = h.shared.fault_detail.load(AtomicOrdering::Acquire);
    assert_ne!(
        detail, 0,
        "fault_detail must be non-zero after an InvalidHandle latch"
    );
    // Slot index 0, expected gen 1, observed gen 0 (slot is empty) ⇒
    // (0 << 16) | (0 XOR 1) = 1 with the encoder's current shape; what we
    // really want is `detail >> 16 == slot_idx`.
    assert_eq!(detail >> 16, 0, "slot index encoded in high 16 bits");

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
