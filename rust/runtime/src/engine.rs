//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use heapless::spsc::{Consumer, Producer};

use crate::clock::{TickCounter, WidenState, one_tick_cycles, publish_widened_now};
use crate::curve_pool::{CurveHandle, CurvePool, CurveView};
use crate::error::RuntimeError;
use crate::kinematics::{cartesian_xyz_with_e, corexy_with_e};
use crate::queue::Q_N;
use crate::segment::{KinematicTag, SEGMENT_FLAG_HOLD_SEGMENT, Segment};
use crate::slot::{IsSlot, PaSlot};
use crate::state::{SharedState, TickState};
use crate::trace::{
    TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_HOLD_SAMPLE, TRACE_FLAG_SEGMENT_END, TRACE_RING_N,
    TraceSample,
};

/// Bounded sub-tick boundary-loop iteration count.
///
/// Step-6 Phase 12.2: aligned to the queue's effective capacity (`Q_N - 1 = 7`)
/// so the bound matches what the public producer API can actually cram into
/// a single tick. With `Q_N = 8` the heapless SPSC effective cap is 7, plus
/// the engine's in-flight `current` makes 8 retire-able segments per tick;
/// `MAX_BOUNDARY_ITERS = 7` lets the boundary loop iterate 7 times (one per
/// retire) and fault on the 8th — which is reachable only via the
/// `#[cfg(test)] inject_iter_count` injection (the producer can never legally
/// stuff more than 7 zero-duration segments into the queue at once).
const MAX_BOUNDARY_ITERS: u32 = (Q_N - 1) as u32;

/// Throttle for the optional `TRACE_FLAG_HOLD_SAMPLE` trace event (§6.5).
/// At 40 kHz tick rate, ~10 ms = 400 ticks between hold-sample emissions.
const HOLD_SAMPLE_TICK_PERIOD: u32 = 400;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RuntimeStatus {
    Idle = 0,
    Running = 1,
    Drained = 2,
    Fault = 3,
}

impl RuntimeStatus {
    #[inline]
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Running,
            2 => Self::Drained,
            _ => Self::Fault,
        }
    }
}

#[allow(missing_debug_implementations)] // P, I are open trait bounds; ISR-internal struct.
pub struct Engine<P: PaSlot, I: IsSlot> {
    pub(crate) current: Option<Segment>,
    last_motors: [f32; 3], // last-known-good motor positions (used in FAULT marker)
    pa_slot: P,
    is_slot: I,
    one_tick_cycles_value: u64,
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
    /// §6.5 throttle: ticks since last `TRACE_FLAG_HOLD_SAMPLE` emission.
    /// Reset on segment activation; incremented per hold tick. At ~10 ms
    /// (`HOLD_SAMPLE_TICK_PERIOD = 400`) we drop one breadcrumb sample.
    hold_sample_ticks: u32,
    /// Phase 12.2 test-only injection — when non-zero, the boundary loop
    /// pretends it has already iterated this many times before the first
    /// carry, so the `n+1`-th carry trips the `MAX_BOUNDARY_ITERS` guard.
    /// Allows `cfg(any(test, feature = "test-injection"))` callers to reach
    /// the otherwise-defense-in-depth fault path without trying to overstuff
    /// the queue (which the public producer API rejects via
    /// `KALICO_ERR_QUEUE_FULL`). Gated so production builds don't carry the
    /// field at all.
    #[cfg(any(test, feature = "test-injection"))]
    pub(crate) injected_iter_start: u32,
}

impl<P: PaSlot + Default, I: IsSlot + Default> Engine<P, I> {
    pub fn new(clock_freq: u32) -> Self {
        Self {
            current: None,
            last_motors: [0.0; 3],
            pa_slot: P::default(),
            is_slot: I::default(),
            one_tick_cycles_value: u64::from(one_tick_cycles(clock_freq)),
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
            hold_sample_ticks: 0,
            #[cfg(any(test, feature = "test-injection"))]
            injected_iter_start: 0,
        }
    }

    /// Production-context constructor. Mirrors `::new(clock_freq)` but keeps
    /// the call site noise low (Step-6 spec §14): the C-side
    /// `kalico_clock_freq` static is read once at FFI init time and the value
    /// is threaded through here.
    pub fn new_production(clock_freq: u32) -> Self {
        Self::new(clock_freq)
    }
}

// Engine::Default impl for tests where slot types implement Default.
// Production callers must use ::new(clock_freq) — Default hardcodes 520 MHz.
#[cfg(test)]
impl<P: PaSlot + Default, I: IsSlot + Default> Default for Engine<P, I> {
    fn default() -> Self {
        // H723 Klipper Kconfig default is 520 MHz (src/stm32/Kconfig). Tests using
        // Default get this; tests requiring a specific value should call ::new() directly.
        Self::new(520_000_000)
    }
}

impl<P: PaSlot, I: IsSlot> Engine<P, I> {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    /// Round-2 fix B4: clear the current segment from outside the engine
    /// module. Used by Phase 7 §8.5 flush as defense-in-depth so foreground
    /// can drop the in-flight segment under disabled-IRQ before clearing
    /// `stream_open`. Phase 1 lands the accessor; the call site arrives in
    /// Phase 7.
    #[allow(dead_code)] // Wired in Phase 7.
    pub(crate) fn clear_current(&mut self) {
        self.current = None;
    }

    /// Phase 12.2 test-only helper: prime the boundary-loop iteration
    /// counter so the next tick that carries across one segment boundary
    /// trips the `MAX_BOUNDARY_ITERS` fault. The public producer API caps
    /// the queue at `Q_N - 1 = 7`, which combined with the in-flight segment
    /// puts the natural reachable max at 7 carries — exactly the bound.
    /// Without this injection the fault path is dead defense-in-depth.
    /// Gated on `cfg(any(test, feature = "test-injection"))` so production
    /// builds neither carry the field nor expose the setter.
    #[cfg(any(test, feature = "test-injection"))]
    pub fn inject_iter_count(&mut self, n: u32) {
        self.injected_iter_start = n;
    }

    /// Latch FAULT and emit one fault marker sample (last-known-good motors,
    /// not zero, so host plots show the fault in context). ISR self-disables
    /// the timer in the C wrapper after this returns.
    /// `segment_id` is passed explicitly by the call site (decoupled from
    /// `self.current`). Pass `0` only if no segment was active — producer-side
    /// segment ids start at 1, so `segment_id == 0` ⇒ fault before any segment
    /// was active.
    fn latch_fault(
        &mut self,
        code: RuntimeError,
        segment_id: u32,
        curve_handle: CurveHandle,
        now: u64,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
    ) {
        self.last_error.store(i32::from(code), Ordering::Release);
        self.status
            .store(RuntimeStatus::Fault as u8, Ordering::Release);
        let _ = trace.enqueue(TraceSample {
            tick: now,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_e: self.last_motors[2],
            segment_id,
            curve_handle,
            flags: TRACE_FLAG_FAULT_MARKER,
            _pad: [0; 3],
        });
    }

    /// Single 40 kHz tick. Spec §4.2 step ordering — must remain stable.
    ///
    /// Step-6 canonical signature (Phase 1 Task 1.2 + Round-4 verifier #5):
    /// the engine receives the raw CYCCNT u32, the `widen_state` (now lives
    /// in `IsrState`, not the engine), and the disjoint half-split borrows
    /// (queue consumer, trace producer, `SharedState`). Widening + the §11.4
    /// seqlock publish happen here so the foreground reader always sees a
    /// coherent widened `now`.
    ///
    /// Returns `Result<(), RuntimeError>` mainly for tests; the FFI shim
    /// drops the result because the fault is latched into `SharedState`
    /// regardless.
    pub fn tick(
        &mut self,
        raw_cyccnt: u32,
        widen_state: &mut WidenState,
        pool: &CurvePool,
        queue: &mut Consumer<'_, Segment, Q_N>,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) -> Result<(), RuntimeError> {
        // §8.5 step 2: force_idle short-circuit. BEFORE anything else —
        // BEFORE widen_state mutation, BEFORE queue dequeue, BEFORE
        // evaluation. Aborts current evaluation, sets acked_force_idle,
        // returns. Bounded ~25 µs at 40 kHz (single atomic load + branch).
        // The hold-segment short-circuit (Phase 9) lands AFTER this block.
        if shared.force_idle.load(Ordering::Acquire) {
            self.clear_current();
            shared.acked_force_idle.store(true, Ordering::Release);
            return Ok(());
        }

        let now = widen_state.widen(raw_cyccnt);
        // §11.4: republish the widened u64 to SharedState so foreground readers
        // (clock-sync responder, status frame) can fetch it without forming a
        // &mut on the IsrState.
        publish_widened_now(shared, now);

        if self.status() == RuntimeStatus::Fault {
            return Err(RuntimeError::FaultLatched);
        }

        // Step 1 + 2: queue + idle check, segment activation. See spec §4.2.
        // Idle/Drained path with §4.4 ISR-disable protocol.
        // (Producer protocol at runtime_ffi.rs re-enables TIM5 on either Idle
        // or Drained, so we keep the two distinct: Idle is the initial
        // post-init state set in Engine::new; Drained is set by the boundary
        // loop below when the queue is exhausted. We must NOT clobber Drained
        // back to Idle on subsequent empty-queue ticks — that would mask the
        // completed-segment state from the host's status query.)
        let Some(current) = self.current.take() else {
            // Re-check queue with Acquire — race against producer's enqueue.
            if !queue.ready() {
                // §8.2: queue empty + stream_open=true → KALICO_FAULT_UNDERRUN.
                // queue empty + stream_open=false → keep current status
                // (Idle pre-stream / Drained post-stream).
                if shared.stream_open.load(Ordering::Acquire) {
                    self.last_error
                        .store(crate::error::KALICO_ERR_UNDERRUN, Ordering::Release);
                    self.status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    return Err(RuntimeError::Underrun);
                }
                return Ok(());
            }
            self.current = queue.dequeue();
            if let Some(seg) = self.current {
                self.status
                    .store(RuntimeStatus::Running as u8, Ordering::Release);
                // Round-2 B14: ISR publishes the freshly activated segment id
                // so foreground status / Gate-B observers see it. Release so
                // the runtime_status update above is paired.
                shared
                    .current_segment_id
                    .store(seg.id, Ordering::Release);
                // Fall through with the freshly dequeued segment.
                return self.tick_with_current(seg, now, queue, pool, trace, shared);
            }
            return Ok(());
        };

        self.tick_with_current(current, now, queue, pool, trace, shared)
    }

    #[allow(clippy::too_many_lines)] // Spec §4.2 step 1-10 explicit pipeline — flatten on purpose.
    fn tick_with_current(
        &mut self,
        mut current: Segment,
        now: u64,
        queue: &mut Consumer<'_, Segment, Q_N>,
        pool: &CurvePool,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) -> Result<(), RuntimeError> {
        // Step 3: sub-tick boundary loop. Spec §4.2 step 3 — bounded by queue depth.
        // Phase 12.2 test injection: prime `iters` so the test can reach the
        // `MAX_BOUNDARY_ITERS` fault path without overstuffing the queue.
        #[cfg(any(test, feature = "test-injection"))]
        let mut iters: u32 = self.injected_iter_start;
        #[cfg(not(any(test, feature = "test-injection")))]
        let mut iters: u32 = 0;
        let mut t_segment = now.saturating_sub(current.t_start);
        while t_segment >= current.duration() {
            iters += 1;
            if iters > MAX_BOUNDARY_ITERS {
                let seg_id = current.id;
                let curve_handle = current.curve_handle;
                self.current = Some(current);
                self.latch_fault(
                    RuntimeError::BoundaryLoopExhausted,
                    seg_id,
                    curve_handle,
                    now,
                    trace,
                );
                return Err(RuntimeError::BoundaryLoopExhausted);
            }
            let delta_t = t_segment - current.duration();
            // Round-2 B14: a segment is finishing now. Publish its id as the
            // newest retired-through cursor before we advance.
            shared
                .retired_through_segment_id
                .store(current.id, Ordering::Release);
            // §8.3 terminal-segment hook: if foreground published a terminal
            // segment id and we're retiring it now, clear `stream_open` so
            // the next empty-queue observation goes to Drained, not Underrun.
            crate::stream::check_terminal_on_retire(shared, current.id);
            // §6.5: hold segments retire identically to motion segments —
            // emit SEGMENT_END so foreground reclaim sees the retirement,
            // even though the boundary-loop branch never evaluates the
            // curve. The sentinel handle's slot_idx is u16::MAX which
            // `confirm_retired` no-ops on (out of slots range), preserving
            // the "hold segments don't allocate slots" invariant.
            if current.flags & SEGMENT_FLAG_HOLD_SEGMENT != 0 {
                let _ = trace.enqueue(TraceSample {
                    tick: now,
                    motor_a: self.last_motors[0],
                    motor_b: self.last_motors[1],
                    motor_e: self.last_motors[2],
                    segment_id: current.id,
                    curve_handle: current.curve_handle,
                    flags: TRACE_FLAG_SEGMENT_END,
                    _pad: [0; 3],
                });
            }
            // Drop current; advance to next.
            let Some(next) = queue.dequeue() else {
                // No next segment — drained. §8.2: queue empty + stream_open=true
                // → KALICO_FAULT_UNDERRUN; queue empty + stream_open=false
                // → Drained (normal end-of-stream). The terminal hook above
                // may have just cleared stream_open if this was the terminal
                // segment, in which case we route to Drained.
                self.current = None;
                if shared.stream_open.load(Ordering::Acquire) {
                    self.last_error
                        .store(crate::error::KALICO_ERR_UNDERRUN, Ordering::Release);
                    self.status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    return Err(RuntimeError::Underrun);
                }
                self.status
                    .store(RuntimeStatus::Drained as u8, Ordering::Release);
                return Ok(());
            };
            current = next;
            current.t_start = now.saturating_sub(delta_t);
            t_segment = delta_t;
            // Reset hold-sample throttle on segment activation so a fresh
            // hold window emits its first breadcrumb early.
            self.hold_sample_ticks = 0;
            // Round-2 B14: new segment activated mid-boundary loop — publish
            // the current id so foreground sees the transition.
            shared
                .current_segment_id
                .store(current.id, Ordering::Release);
        }

        // §6.5 hold-segment short-circuit: AFTER force_idle (handled in
        // tick(), at the very top), AFTER the boundary loop (so retiring
        // a hold still emits SEGMENT_END), but BEFORE pool.resolve — hold
        // segments carry `CurveHandle::HOLD_SEGMENT_SENTINEL` (slot=u16::MAX,
        // gen=u16::MAX) which would fail lookup. ISR repeats the last
        // emitted motor positions; foreground sees the stream stay alive
        // across long Z-idle stretches without underrun.
        if current.flags & SEGMENT_FLAG_HOLD_SEGMENT != 0 {
            // Optional throttled HOLD_SAMPLE breadcrumb (§6.5). Emits at
            // most once per ~10 ms while the hold window is active.
            self.hold_sample_ticks = self.hold_sample_ticks.saturating_add(1);
            if self.hold_sample_ticks >= HOLD_SAMPLE_TICK_PERIOD {
                self.hold_sample_ticks = 0;
                let _ = trace.enqueue(TraceSample {
                    tick: now,
                    motor_a: self.last_motors[0],
                    motor_b: self.last_motors[1],
                    motor_e: self.last_motors[2],
                    segment_id: current.id,
                    curve_handle: current.curve_handle,
                    flags: TRACE_FLAG_HOLD_SAMPLE,
                    _pad: [0; 3],
                });
            }
            // SEGMENT_END at retire — same path as motion segments. The
            // boundary loop above already handled the case where now has
            // already crossed t_end; here we check whether the next tick
            // would do so, and pre-emit the SEGMENT_END flag now.
            let next_t_segment = t_segment.saturating_add(self.one_tick_cycles_value);
            if next_t_segment >= current.duration() {
                let _ = trace.enqueue(TraceSample {
                    tick: now,
                    motor_a: self.last_motors[0],
                    motor_b: self.last_motors[1],
                    motor_e: self.last_motors[2],
                    segment_id: current.id,
                    curve_handle: current.curve_handle,
                    flags: TRACE_FLAG_SEGMENT_END,
                    _pad: [0; 3],
                });
                shared
                    .retired_through_segment_id
                    .store(current.id, Ordering::Release);
                crate::stream::check_terminal_on_retire(shared, current.id);
            }
            self.current = Some(current);
            self.tick_counter.increment();
            self.status
                .store(RuntimeStatus::Running as u8, Ordering::Release);
            return Ok(());
        }

        // Step 4: curve evaluation. Spec invariant: segments are time-parameterized.
        let Some(curve_view) = pool.resolve(current.curve_handle) else {
            self.latch_fault(
                RuntimeError::InvalidHandle,
                current.id,
                current.curve_handle,
                now,
                trace,
            );
            return Err(RuntimeError::InvalidHandle);
        };
        let duration = current.duration().max(1) as f32; // saturating_sub avoids 0
        let u = (t_segment as f32 / duration).clamp(0.0, 1.0);
        let Ok(xyz_e) = nurbs_eval_3d(&curve_view, u) else {
            self.latch_fault(
                RuntimeError::InvalidCurve,
                current.id,
                current.curve_handle,
                now,
                trace,
            );
            return Err(RuntimeError::InvalidCurve);
        };

        // Step 5: NaN/Inf check. Spec §5.4 — necessary even with producer-side
        // validation (NaN can arise from finite inputs).
        if !xyz_e.iter().all(|x: &f32| x.is_finite()) {
            self.latch_fault(
                RuntimeError::NaNOrInfFromEval,
                current.id,
                current.curve_handle,
                now,
                trace,
            );
            return Err(RuntimeError::NaNOrInfFromEval);
        }

        // Step 6: kinematic transform. Pipeline order: kinematics BEFORE PA/IS.
        let motors = match current.kinematics {
            KinematicTag::CoreXyAndE => corexy_with_e(xyz_e),
            KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(xyz_e),
        };

        // Step 7: slot pipeline. Noop ZSTs at Step 5.
        let dt = 1.0 / (crate::clock::TICK_RATE_HZ as f32);
        let mut state = TickState { dt, xyz_e, motors };
        self.pa_slot.apply(&mut state);
        self.is_slot.apply(&mut state);

        // Step 8: trace emit.
        let next_t_segment = t_segment.saturating_add(self.one_tick_cycles_value);
        let segment_end_flag = if next_t_segment >= current.duration() {
            TRACE_FLAG_SEGMENT_END
        } else {
            0
        };
        // §13.1: trace-ring overflow → set `sample_drop_pending` so foreground
        // can latch `KALICO_FAULT_TRACE_OVERFLOW`. Unlike the Step-5 carry-bit
        // approach, the dropped sample is gone — foreground hard-faults
        // instead of trying to resynchronize a partial trace stream.
        if trace
            .enqueue(TraceSample {
                tick: now,
                motor_a: state.motors[0],
                motor_b: state.motors[1],
                motor_e: state.motors[2],
                segment_id: current.id,
                curve_handle: current.curve_handle,
                flags: segment_end_flag,
                _pad: [0; 3],
            })
            .is_err()
        {
            shared.sample_drop_pending.store(true, Ordering::Release);
        }
        // Round-2 B14: when the segment is about to retire (last sample in
        // its window emits SEGMENT_END), advance retired_through_segment_id
        // monotonically. The next-tick activation re-fires this update via
        // the boundary loop above — the duplicate write is a no-op against
        // the same id.
        if segment_end_flag != 0 {
            shared
                .retired_through_segment_id
                .store(current.id, Ordering::Release);
            // §8.3 terminal hook — see boundary-loop equivalent above.
            crate::stream::check_terminal_on_retire(shared, current.id);
        }
        self.last_motors = state.motors;
        self.current = Some(current);

        // Step 9: tick counter heartbeat.
        self.tick_counter.increment();

        // Step 10: status update.
        self.status
            .store(RuntimeStatus::Running as u8, Ordering::Release);
        Ok(())
    }
}

/// Wrapper around `nurbs::eval::vector_eval` for f32 3D rational NURBS.
///
/// Uses `nurbs::VectorNurbsRef` (the borrowed view type) per the actual
/// Layer-0 API at `rust/nurbs/src/vector.rs` (verified during plan review).
fn nurbs_eval_3d(curve: &CurveView<'_>, u: f32) -> Result<[f32; 3], ()> {
    use nurbs::VectorNurbsRef;

    // Actual API: try_new(degree: u8, knots: &[T], control_points: &[[T; N]],
    //                     weights: Option<&[T]>) -> Result<Self, ConstructError>.
    // Returns owning struct over the borrowed slices.
    let view = VectorNurbsRef::<f32, 3>::try_new(
        curve.degree,
        curve.knots,
        curve.control_points,
        Some(curve.weights),
    )
    .map_err(|_| ())?;
    // vector_eval returns [T; N] directly — no Result wrapper.
    Ok(nurbs::eval::vector_eval(&view, u))
}
