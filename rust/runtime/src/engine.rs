//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use heapless::spsc::{Consumer, Producer};
use nurbs::Float;

use crate::clock::{TickCounter, WidenState, one_tick_cycles, publish_widened_now};
use crate::curve_pool::{CurveHandle, CurvePool, CurveView};
use crate::endstop::{self, TripAction};
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

/// Per-stage diagnostic timing helpers. Cycle counter + accumulator is the
/// MCU build's path; host builds get inert stubs so the runtime crate's
/// host-side tests still link without the C-side BKPSRAM symbols.
#[inline(always)]
#[allow(unsafe_code)]
fn diag_cyccnt() -> u32 {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn runtime_cyccnt_read() -> u32;
        }
        // SAFETY: stable C ABI symbol provided by src/stm32/runtime_tick_h7.c
        // on the MCU; reads DWT->CYCCNT, no side effects, no preconditions.
        unsafe { runtime_cyccnt_read() }
    }
    #[cfg(not(target_os = "none"))]
    {
        0
    }
}

#[inline(always)]
#[allow(unsafe_code)]
fn diag_eval_record(cycles: u32) {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn diag_rt_eval_account(cycles: u32);
        }
        // SAFETY: stable C ABI symbol; takes one u32 by value, no aliasing.
        unsafe { diag_rt_eval_account(cycles) }
    }
    #[cfg(not(target_os = "none"))]
    {
        let _ = cycles;
    }
}

/// Emit `n_steps` step pulses on the motor at `motor_idx` (post-kinematic-
/// transform: `[A, B, Z, E]` for CoreXY, `[X, Y, Z, E]` for cartesian).
/// Sign carries direction. On hardware this calls into `src/stepper.c`'s
/// `runtime_emit_step_pulses`, which toggles the step/dir GPIOs configured
/// by the matching `command_config_runtime_stepper`. Host-sim is a no-op.
#[inline(always)]
#[allow(unsafe_code)]
fn emit_step_pulses(motor_idx: u8, n_steps: i32) {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn runtime_emit_step_pulses(motor_idx: u8, n_steps: i32);
        }
        // SAFETY: stable C ABI symbol provided by src/stepper.c. Two scalar
        // args by value, no aliasing. Bounds-checked motor_idx on the C side.
        unsafe { runtime_emit_step_pulses(motor_idx, n_steps) }
    }
    #[cfg(not(target_os = "none"))]
    {
        let _ = (motor_idx, n_steps);
    }
}

#[inline(always)]
#[allow(unsafe_code)]
fn diag_curve_meta_record(axis_idx: u32, degree: u32, cps_len: u32, knots_len: u32) {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn diag_rt_curve_meta(axis_idx: u32, degree: u32, cps_len: u32, knots_len: u32);
        }
        // SAFETY: stable C ABI symbol; takes four u32s by value, no aliasing.
        unsafe { diag_rt_curve_meta(axis_idx, degree, cps_len, knots_len) }
    }
    #[cfg(not(target_os = "none"))]
    {
        let _ = (axis_idx, degree, cps_len, knots_len);
    }
}

/// Snapshot the per-axis curve dimensions for the freshly activated
/// segment into the BKPSRAM diag struct. Called from the new-segment
/// branches of `tick` / `tick_with_current` boundary loop.
#[inline]
fn capture_segment_curve_meta(seg: &Segment, pool: &CurvePool) {
    let resolve_meta = |handle: CurveHandle| -> (u32, u32, u32) {
        if handle.is_unused_sentinel() {
            return (0, 0, 0);
        }
        if let Some(view) = pool.resolve(handle) {
            (
                u32::from(view.degree),
                view.control_points.len() as u32,
                view.knots.len() as u32,
            )
        } else {
            (0, 0, 0)
        }
    };
    let (xd, xc, xk) = resolve_meta(seg.x_handle);
    diag_curve_meta_record(0, xd, xc, xk);
    let (yd, yc, yk) = resolve_meta(seg.y_handle);
    diag_curve_meta_record(1, yd, yc, yk);
    let (zd, zc, zk) = resolve_meta(seg.z_handle);
    diag_curve_meta_record(2, zd, zc, zk);
}

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
    last_motors: [f32; 4], // last-known-good motor positions (used in FAULT marker)
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
    /// Previous X position for E-mode arc-length integration.
    prev_x: f32,
    /// Previous Y position for E-mode arc-length integration.
    prev_y: f32,
    /// E accumulator for CoupledToXy mode — f64 for sub-step accuracy over
    /// millions of ticks (H723 has hardware double-precision FPU).
    e_accumulator: f64,
    /// Set to `true` on init and after flush/clear so the first segment seeds
    /// `prev_x`/`prev_y` from X(0)/Y(0) rather than computing a spurious
    /// delta from (0,0).
    needs_xy_seed: bool,
    /// Diagnostic — last (now, t_start, duration) observed in tick_with_current.
    debug_last_now: u64,
    debug_last_tstart: u64,
    debug_last_duration: u64,
    /// Per-axis step accumulators. Indexed in motor space post-kinematics:
    /// CoreXY: [A=0, B=1, Z=2, E=3]. Step pulse emission deferred to 7-D;
    /// update() is called but results are logged/ignored for now.
    step_state: [crate::step::StepMotorState; 4],
    /// Per-MCU axis configuration. `None` until `configure()` is called;
    /// step generation is skipped when unconfigured.
    mcu_config: Option<crate::config::McuAxisConfig>,
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
            last_motors: [0.0; 4],
            pa_slot: P::default(),
            is_slot: I::default(),
            one_tick_cycles_value: u64::from(one_tick_cycles(clock_freq)),
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
            hold_sample_ticks: 0,
            prev_x: 0.0,
            prev_y: 0.0,
            e_accumulator: 0.0,
            needs_xy_seed: true,
            debug_last_now: 0,
            debug_last_tstart: 0,
            debug_last_duration: 0,
            step_state: [crate::step::StepMotorState::default(); 4],
            mcu_config: None,
            #[cfg(any(test, feature = "test-injection"))]
            injected_iter_start: 0,
        }
    }

    /// Production-context constructor. Mirrors `::new(clock_freq)` but keeps
    /// the call site noise low (Step-6 spec §14): the C-side
    /// `runtime_clock_freq` static is read once at FFI init time and the value
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

    /// Set the per-MCU axis configuration. Must be called before motion
    /// segments are pushed so step generation has valid steps-per-mm ratios.
    /// Diagnostic accessor: returns the configured steps_per_mm for axis `i`
    /// in motor space (CoreXY: A=0, B=1, Z=2, E=3). Returns 0.0 for
    /// out-of-range or unconfigured axes.
    pub fn debug_steps_per_mm(&self, i: usize) -> f32 {
        self.step_state.get(i).map(|s| s.debug_steps_per_mm()).unwrap_or(0.0)
    }

    pub fn debug_accumulator(&self, i: usize) -> f64 {
        self.step_state.get(i).map(|s| s.debug_accumulator()).unwrap_or(0.0)
    }

    /// Last observed motor position (post-PA/IS) for axis `i`.
    pub fn debug_last_motor(&self, i: usize) -> f32 {
        self.last_motors.get(i).copied().unwrap_or(0.0)
    }

    /// Last (now, t_start, duration) tuple recorded by the most recent tick.
    pub fn debug_last_timing(&self) -> (u64, u64, u64) {
        (self.debug_last_now, self.debug_last_tstart, self.debug_last_duration)
    }

    pub fn configure(&mut self, config: crate::config::McuAxisConfig) {
        // Seed step states from the motor config.
        for (i, motor_opt) in config.motors.iter().enumerate() {
            if let Some(motor) = motor_opt {
                if let Some(ss) = self.step_state.get_mut(i) {
                    *ss = crate::step::StepMotorState::new(motor.steps_per_mm);
                }
            }
        }
        self.mcu_config = Some(config);
    }

    /// Round-2 fix B4: clear the current segment from outside the engine
    /// module. Used by Phase 7 §8.5 flush as defense-in-depth so foreground
    /// can drop the in-flight segment under disabled-IRQ before clearing
    /// `stream_open`. Phase 1 lands the accessor; the call site arrives in
    /// Phase 7.
    ///
    /// Also resets E-mode and XY-seed state so the next stream starts clean.
    #[allow(dead_code)] // Wired in Phase 7.
    pub(crate) fn clear_current(&mut self) {
        self.current = None;
        self.needs_xy_seed = true;
        self.e_accumulator = 0.0;
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
    ///
    /// `detail` is the §9.2 `fault_detail` payload. Closure-review fix:
    /// `SharedState.fault_detail` was previously declared and exposed via
    /// FFI but never written, so the host always saw `0`. Call sites pass
    /// `Some(encode_*(...))` for fault types that carry per-event context
    /// (invalid handle, clock-sync quality, stream-state violation), and
    /// `None` for fault types that don't (`0` is the sentinel for "no
    /// detail").
    #[allow(clippy::too_many_arguments)] // Spec §9.2 fault-detail threading; refactor to a struct adds noise without clarity.
    fn latch_fault(
        &mut self,
        code: RuntimeError,
        segment_id: u32,
        curve_handle: CurveHandle,
        now: u64,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
        detail: Option<u32>,
    ) {
        self.last_error.store(i32::from(code), Ordering::Release);
        self.status
            .store(RuntimeStatus::Fault as u8, Ordering::Release);
        // Publish detail BEFORE the trace marker so any foreground reader
        // racing with the trace ring already sees the populated detail.
        // `None` → `0`, matching the doc-comment on `SharedState.fault_detail`
        // ("`0` when no fault has latched OR when the fault carries no detail").
        shared
            .fault_detail
            .store(detail.unwrap_or(0), Ordering::Release);
        let _ = trace.enqueue(TraceSample {
            tick: now,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_z: self.last_motors[2],
            motor_e: self.last_motors[3],
            segment_id,
            curve_handle,
            flags: TRACE_FLAG_FAULT_MARKER,
            _pad: [0; 7],
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
                if self.poll_endstop_trip(now, [0; 3], trace, shared) {
                    return Err(RuntimeError::HomingTrip);
                }
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
                shared.current_segment_id.store(seg.id, Ordering::Release);
                // Diagnostic: snapshot the per-axis curve dimensions so we
                // can characterize what shape post-shape curves take on
                // representative workloads.
                capture_segment_curve_meta(&seg, pool);
                // 2026-05-11: per-segment re-seed removed. The original
                // bring-up rationale (re-anchor accumulators to absorb
                // cross-segment discontinuities) caused silent motion
                // loss: when a segment activates with `t_segment > 0`
                // — e.g., the segment sat in queue for a while because
                // foreground was starved (BKPSRAM showed
                // `out_max_gap=7.6s` and `ring_overflow=39766`) —
                // re-seeding from `curve(u_initial)` declares the
                // skipped virtual position to be physical, and the
                // motion from `u=0` to `u_initial` never produces step
                // pulses. Bench-observed symptom: negative jogs sat in
                // queue while foreground stalled, then "creeped slowly"
                // in the queued direction once the engine caught up.
                //
                // Correct behaviour: TRUST cross-segment continuity as
                // a planner invariant. The planner emits curves where
                // `segment_N.end == segment_{N+1}.start` (split-at-s
                // shaping, commit `c03b3ed5e`); the engine should
                // therefore let the step accumulator and `prev_x/y`
                // carry over from one segment to the next without
                // re-anchoring. A genuine discontinuity is a planner
                // bug and should surface (StepBurstExceeded fault) so
                // it gets fixed, not silently absorbed.
                //
                // Seed remains in: (1) `Engine::new` initial state, (2)
                // `force_idle` (flush / homing recovery). The flag is
                // not touched on per-segment activation any more.
                // Fall through with the freshly dequeued segment.
                return self.tick_with_current(seg, now, queue, pool, trace, shared);
            }
            return Ok(());
        };

        self.tick_with_current(current, now, queue, pool, trace, shared)
    }

    fn poll_endstop_trip(
        &mut self,
        now: u64,
        v_per_axis_q16: [u32; 3],
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) -> bool {
        let mut stepper_counts = [0_i32; crate::state::MAX_STEPPER_OIDS];
        for (dst, src) in stepper_counts.iter_mut().zip(shared.stepper_counts.iter()) {
            *dst = src.load(Ordering::Acquire);
        }
        if endstop::tick(now, v_per_axis_q16, &stepper_counts) != TripAction::AbortNow {
            return false;
        }
        self.clear_current();
        self.latch_fault(
            RuntimeError::HomingTrip,
            0,
            CurveHandle::UNUSED_SENTINEL,
            now,
            trace,
            shared,
            None,
        );
        true
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
        self.debug_last_now = now;
        self.debug_last_tstart = current.t_start;
        self.debug_last_duration = current.duration();
        while t_segment >= current.duration() {
            iters += 1;
            if iters > MAX_BOUNDARY_ITERS {
                let seg_id = current.id;
                let curve_handle = current.x_handle;
                self.current = Some(current);
                self.latch_fault(
                    RuntimeError::BoundaryLoopExhausted,
                    seg_id,
                    curve_handle,
                    now,
                    trace,
                    shared,
                    None,
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
            // E-mode finalization: when retiring an Independent-E segment in
            // the boundary loop, sync e_accumulator to the segment's E
            // endpoint so a subsequent CoupledToXy segment resumes from
            // the correct E position.
            if current.e_mode == crate::config::EMode::Independent
                && !current.e_handle.is_unused_sentinel()
            {
                if let Some(e_view) = pool.resolve(current.e_handle) {
                    if let Ok(e_endpoint) = scalar_eval(&e_view, 1.0) {
                        self.e_accumulator = f64::from(e_endpoint);
                    }
                }
            }
            // Retire the curve-pool slots directly on the ISR, BEFORE
            // emitting the SEGMENT_END trace sample. Bench-observed
            // 2026-05-11: the previous design relied on the foreground
            // trace drain calling `pool.confirm_retired` when it
            // observed the SEGMENT_END flag on a trace sample. When
            // foreground stalled (BKPSRAM showed `out_max_gap=7.6s`),
            // the trace ring overflowed (~40k samples dropped) and
            // SEGMENT_END samples were lost on the floor. The cursor
            // `retired_through_segment_id` above advanced anyway, so
            // the host's `kalico_credit_freed` event fired (and the
            // host's slot_pool released the slot for reuse), but the
            // MCU's `last_retired_gen[slot]` never caught up — so the
            // next host-side `load_curve` into that slot was rejected
            // with `SlotAlreadyLoaded` (`current_gen != last_retired_gen`
            // in `curve_pool::try_alloc_and_load`). M84 surfaced the
            // divergence as an `InvalidHandle` crash. Fix: call
            // `confirm_retired` directly here so retirement is atomic
            // with the engine's logical retire and independent of the
            // trace transport's health. `confirm_retired` is a single
            // atomic store; safe from ISR.
            //
            // Sentinels (UNUSED, HOLD) have `slot_idx > CURVE_POOL_N`
            // and `confirm_retired` early-returns on them.
            pool.confirm_retired(current.x_handle);
            pool.confirm_retired(current.y_handle);
            pool.confirm_retired(current.z_handle);
            pool.confirm_retired(current.e_handle);

            // Emit SEGMENT_END unconditionally for all segment types (hold
            // AND motion). After 2026-05-11 the trace sample's role is
            // narrowed to HOST visibility (telemetry / step-pulse
            // accounting); MCU-internal slot retirement is handled by
            // the four `confirm_retired` calls above.
            let _ = trace.enqueue(TraceSample {
                tick: now,
                motor_a: self.last_motors[0],
                motor_b: self.last_motors[1],
                motor_z: self.last_motors[2],
                motor_e: self.last_motors[3],
                segment_id: current.id,
                curve_handle: current.x_handle,
                flags: TRACE_FLAG_SEGMENT_END,
                _pad: [0; 7],
            });
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
            // Diagnostic: snapshot per-axis curve dimensions for the
            // freshly activated segment.
            capture_segment_curve_meta(&current, pool);
            // 2026-05-11: per-boundary-loop re-seed removed alongside
            // the DRAINED→RUNNING re-seed above (engine.rs:~461). Same
            // rationale: silent absorption of `u=0..u_initial` motion
            // when a segment activates partway through (boundary loop
            // entered with `t_segment > duration` of a previous
            // segment, advancing into the next with delta_t > 0). The
            // planner's cross-segment-continuity invariant means the
            // step accumulator and `prev_x/y` are already aligned with
            // the new segment's `curve(0)`; no re-anchoring required.
        }

        // §6.5 hold-segment short-circuit: AFTER force_idle (handled in
        // tick(), at the very top), AFTER the boundary loop (so retiring
        // a hold still emits SEGMENT_END), but BEFORE pool.resolve — hold
        // segments carry `CurveHandle::HOLD_SEGMENT_SENTINEL` (slot=u16::MAX,
        // gen=u16::MAX) which would fail lookup. ISR repeats the last
        // emitted motor positions; foreground sees the stream stay alive
        // across long Z-idle stretches without underrun.
        if current.flags & SEGMENT_FLAG_HOLD_SEGMENT != 0 {
            if self.poll_endstop_trip(now, [0; 3], trace, shared) {
                return Err(RuntimeError::HomingTrip);
            }
            // Optional throttled HOLD_SAMPLE breadcrumb (§6.5). Emits at
            // most once per ~10 ms while the hold window is active.
            self.hold_sample_ticks = self.hold_sample_ticks.saturating_add(1);
            if self.hold_sample_ticks >= HOLD_SAMPLE_TICK_PERIOD {
                self.hold_sample_ticks = 0;
                let _ = trace.enqueue(TraceSample {
                    tick: now,
                    motor_a: self.last_motors[0],
                    motor_b: self.last_motors[1],
                    motor_z: self.last_motors[2],
                    motor_e: self.last_motors[3],
                    segment_id: current.id,
                    curve_handle: current.x_handle,
                    flags: TRACE_FLAG_HOLD_SAMPLE,
                    _pad: [0; 7],
                });
            }
            // SEGMENT_END at retire — same path as motion segments. The
            // boundary loop above already handled the case where now has
            // already crossed t_end; here we check whether the next tick
            // would do so, and pre-emit the SEGMENT_END flag now.
            let next_t_segment = t_segment.saturating_add(self.one_tick_cycles_value);
            if next_t_segment >= current.duration() {
                // Direct slot retirement (see boundary-loop branch above
                // for full rationale; bench session 2026-05-11). For
                // hold segments x/y/z handles are typically UNUSED, but
                // calling confirm_retired on a sentinel is a no-op.
                pool.confirm_retired(current.x_handle);
                pool.confirm_retired(current.y_handle);
                pool.confirm_retired(current.z_handle);
                pool.confirm_retired(current.e_handle);

                let _ = trace.enqueue(TraceSample {
                    tick: now,
                    motor_a: self.last_motors[0],
                    motor_b: self.last_motors[1],
                    motor_z: self.last_motors[2],
                    motor_e: self.last_motors[3],
                    segment_id: current.id,
                    curve_handle: current.x_handle,
                    flags: TRACE_FLAG_SEGMENT_END,
                    _pad: [0; 7],
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

        // Step 4: per-axis scalar curve evaluation. Spec invariant: segments
        // are time-parameterized; each axis has its own scalar NURBS.
        let duration = current.duration().max(1) as f32;
        let u = (t_segment as f32 / duration).clamp(0.0, 1.0);

        // -- X axis -- (combined position + dx/du from one de Boor walk)
        let (x, dx_du_x) = if current.x_handle.is_unused_sentinel() {
            (0.0, 0.0)
        } else {
            let Some(cv) = pool.resolve(current.x_handle) else {
                let detail = crate::error::encode_invalid_curve_handle(
                    current.x_handle.slot_idx,
                    0,
                    current.x_handle.generation,
                );
                self.latch_fault(
                    RuntimeError::InvalidHandle,
                    current.id,
                    current.x_handle,
                    now,
                    trace,
                    shared,
                    Some(detail),
                );
                return Err(RuntimeError::InvalidHandle);
            };
            match scalar_eval_with_derivative(&cv, u) {
                Ok(pair) => pair,
                Err(()) => {
                    self.latch_fault(
                        RuntimeError::InvalidCurve,
                        current.id,
                        current.x_handle,
                        now,
                        trace,
                        shared,
                        None,
                    );
                    return Err(RuntimeError::InvalidCurve);
                }
            }
        };

        // -- Y axis -- (combined position + dy/du)
        let (y, dx_du_y) = if current.y_handle.is_unused_sentinel() {
            (0.0, 0.0)
        } else {
            let Some(cv) = pool.resolve(current.y_handle) else {
                let detail = crate::error::encode_invalid_curve_handle(
                    current.y_handle.slot_idx,
                    0,
                    current.y_handle.generation,
                );
                self.latch_fault(
                    RuntimeError::InvalidHandle,
                    current.id,
                    current.y_handle,
                    now,
                    trace,
                    shared,
                    Some(detail),
                );
                return Err(RuntimeError::InvalidHandle);
            };
            match scalar_eval_with_derivative(&cv, u) {
                Ok(pair) => pair,
                Err(()) => {
                    self.latch_fault(
                        RuntimeError::InvalidCurve,
                        current.id,
                        current.y_handle,
                        now,
                        trace,
                        shared,
                        None,
                    );
                    return Err(RuntimeError::InvalidCurve);
                }
            }
        };

        // -- Z axis -- (combined position + dz/du)
        let (z, dx_du_z) = if current.z_handle.is_unused_sentinel() {
            (0.0, 0.0)
        } else {
            let Some(cv) = pool.resolve(current.z_handle) else {
                let detail = crate::error::encode_invalid_curve_handle(
                    current.z_handle.slot_idx,
                    0,
                    current.z_handle.generation,
                );
                self.latch_fault(
                    RuntimeError::InvalidHandle,
                    current.id,
                    current.z_handle,
                    now,
                    trace,
                    shared,
                    Some(detail),
                );
                return Err(RuntimeError::InvalidHandle);
            };
            match scalar_eval_with_derivative(&cv, u) {
                Ok(pair) => pair,
                Err(()) => {
                    self.latch_fault(
                        RuntimeError::InvalidCurve,
                        current.id,
                        current.z_handle,
                        now,
                        trace,
                        shared,
                        None,
                    );
                    return Err(RuntimeError::InvalidCurve);
                }
            }
        };

        // -- XY seed: first segment seeds prev_x/prev_y from the curve.
        //    Use the CURRENT evaluated x/y, not x(0)/y(0). Two reasons:
        //    (1) the toolhead is where the curve evaluates right now, not at
        //    its nominal start; (2) if the segment arrived late and the
        //    engine enters at u > 0 (e.g. host clock skew), seeding at
        //    u=0 makes the next tick see a multi-mm delta and trips
        //    StepBurstExceeded. --
        if self.needs_xy_seed {
            let seed_x = x;
            let seed_y = y;
            self.prev_x = seed_x;
            self.prev_y = seed_y;
            // Seed step accumulators from initial motor positions.
            let seed_positions = [seed_x, seed_y, z, self.e_accumulator as f32];
            let seed_motors = match current.kinematics {
                KinematicTag::CoreXyAndE => corexy_with_e(seed_positions),
                KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(seed_positions),
            };
            for i in 0..4 {
                if let Some(ss) = self.step_state.get_mut(i) {
                    if let Some(m) = seed_motors.get(i) {
                        ss.seed(*m);
                    }
                }
            }
            self.needs_xy_seed = false;
        }

        // -- E-mode dispatch --
        let e = match current.e_mode {
            crate::config::EMode::CoupledToXy => {
                let dx = x - self.prev_x;
                let dy = y - self.prev_y;
                let dist = (dx * dx + dy * dy).sqrt();
                self.e_accumulator += f64::from(current.extrusion_ratio) * f64::from(dist);
                self.prev_x = x;
                self.prev_y = y;
                self.e_accumulator as f32
            }
            crate::config::EMode::Independent => {
                let e_val = if current.e_handle.is_unused_sentinel() {
                    0.0
                } else {
                    let Some(cv) = pool.resolve(current.e_handle) else {
                        let detail = crate::error::encode_invalid_curve_handle(
                            current.e_handle.slot_idx,
                            0,
                            current.e_handle.generation,
                        );
                        self.latch_fault(
                            RuntimeError::InvalidHandle,
                            current.id,
                            current.e_handle,
                            now,
                            trace,
                            shared,
                            Some(detail),
                        );
                        return Err(RuntimeError::InvalidHandle);
                    };
                    match scalar_eval(&cv, u) {
                        Ok(v) => v,
                        Err(()) => {
                            self.latch_fault(
                                RuntimeError::InvalidCurve,
                                current.id,
                                current.e_handle,
                                now,
                                trace,
                                shared,
                                None,
                            );
                            return Err(RuntimeError::InvalidCurve);
                        }
                    }
                };
                // On segment end (last tick), sync e_accumulator so the next
                // CoupledToXy segment resumes correctly.
                let next_t_segment_check = t_segment.saturating_add(self.one_tick_cycles_value);
                if next_t_segment_check >= current.duration() {
                    self.e_accumulator = f64::from(e_val);
                }
                // Update prev_x/prev_y even in Independent mode so a
                // subsequent CoupledToXy segment doesn't see a stale position.
                self.prev_x = x;
                self.prev_y = y;
                e_val
            }
            crate::config::EMode::Travel => {
                // E unchanged — use current accumulator value.
                self.prev_x = x;
                self.prev_y = y;
                self.e_accumulator as f32
            }
        };

        // Step 5: NaN/Inf check. Spec §5.4 — necessary even with producer-side
        // validation (NaN can arise from finite inputs).
        let eval_result = [x, y, z, e];
        if !eval_result.iter().all(|v: &f32| v.is_finite()) {
            self.latch_fault(
                RuntimeError::NaNOrInfFromEval,
                current.id,
                current.x_handle,
                now,
                trace,
                shared,
                None,
            );
            return Err(RuntimeError::NaNOrInfFromEval);
        }

        // Endstop tick is after tick-N evaluation but before tick-N pulse
        // intent. The per-axis dx/du was computed alongside the position
        // value via `scalar_eval_with_derivative`'s combined de Boor walk
        // — no second pass needed. `velocity_q16_from_dx_du` just rescales
        // to the q16 endstop trip-checker units.
        let v_per_axis_q16 = [
            velocity_q16_from_dx_du(dx_du_x, duration, self.one_tick_cycles_value),
            velocity_q16_from_dx_du(dx_du_y, duration, self.one_tick_cycles_value),
            velocity_q16_from_dx_du(dx_du_z, duration, self.one_tick_cycles_value),
        ];
        if self.poll_endstop_trip(now, v_per_axis_q16, trace, shared) {
            return Err(RuntimeError::HomingTrip);
        }

        // Step 6: kinematic transform. Pipeline order: kinematics BEFORE PA/IS.
        let positions = [x, y, z, e];
        let motors = match current.kinematics {
            KinematicTag::CoreXyAndE => corexy_with_e(positions),
            KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(positions),
        };

        // Step 7: slot pipeline. Noop ZSTs at Step 5.
        let dt = 1.0 / (crate::clock::TICK_RATE_HZ as f32);
        let mut state = TickState {
            dt,
            positions,
            motors,
        };
        self.pa_slot.apply(&mut state);
        self.is_slot.apply(&mut state);

        // Step 7b: step generation. Update per-axis accumulators from the
        // post-PA/IS motor positions. On hardware (`target_os = "none"`)
        // also call into the C-side `runtime_emit_step_pulses` to actually
        // toggle the step/dir GPIOs for this motor inside the same ISR
        // tick. Host-sim builds skip the emit and only update the atomic
        // counter, which `runtime_status_drain`'s sim-only stderr taps
        // for progress observation.
        for i in 0..4 {
            if let (Some(ss), Some(&m)) = (self.step_state.get_mut(i), state.motors.get(i)) {
                let step_result = match ss.update(m) {
                    Ok(result) => result,
                    Err(()) => {
                        // Encode (axis_idx, attempted_step_delta) into
                        // fault_detail so the host log identifies the
                        // offending axis. Layout: low 8 bits = axis (0..3),
                        // upper 24 bits = signed step delta saturated.
                        let attempted = m.to_bits(); // f32 raw bits
                        let detail =
                            (attempted & 0xFFFF_FF00) | ((i as u32) & 0xFF);
                        self.latch_fault(
                            RuntimeError::StepBurstExceeded,
                            current.id,
                            current.x_handle,
                            now,
                            trace,
                            shared,
                            Some(detail),
                        );
                        return Err(RuntimeError::StepBurstExceeded);
                    }
                };
                if step_result.n_steps != 0 {
                    if let Some(counter) = shared.stepper_counts.get(i) {
                        counter.fetch_add(step_result.n_steps, Ordering::AcqRel);
                    }
                    emit_step_pulses(i as u8, step_result.n_steps);
                }
            }
        }

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
                motor_z: state.motors[2],
                motor_e: state.motors[3],
                segment_id: current.id,
                curve_handle: current.x_handle,
                flags: segment_end_flag,
                _pad: [0; 7],
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

/// Plain-eval form for the E-axis paths (segment-retire endpoint sync and
/// Independent-mode E position) — they don't need the derivative, and E
/// is not in the per-tick X/Y/Z hot path. Trips one `find_knot_span` +
/// one de Boor walk, no derivative pyramid.
fn scalar_eval(curve: &CurveView<'_>, u: f32) -> Result<f32, ()> {
    let p = usize::from(curve.degree);
    if p > nurbs::MAX_DEGREE
        || curve.knots.len() != curve.control_points.len() + p + 1
        || curve.control_points.is_empty()
    {
        return Err(());
    }
    Ok(nurbs::eval::eval_polynomial(
        curve.control_points,
        curve.knots,
        curve.degree,
        u,
    ))
}

/// Evaluate a scalar NURBS curve at `u`, returning `(P(u), dP/du)` from a
/// single combined de Boor walk. The eval and derivative recurrences run
/// in parallel, sharing `find_knot_span` + the d-array initialization +
/// most of the pyramid (~2× the work of plain eval, vs ~3× for two
/// separate passes).
///
/// X/Y/Z always need `dx/du` for the endstop trip-checker, so the combined
/// form is the universal hot path. Derivative-only callers would route
/// through `nurbs::eval::eval_derivative` (the public windowed form), but
/// none exist in the runtime crate today.
fn scalar_eval_with_derivative(curve: &CurveView<'_>, u: f32) -> Result<(f32, f32), ()> {
    let t0 = diag_cyccnt();
    let p = usize::from(curve.degree);
    if p > nurbs::MAX_DEGREE
        || curve.knots.len() != curve.control_points.len() + p + 1
        || curve.control_points.is_empty()
    {
        return Err(());
    }
    let result = nurbs::eval::eval_polynomial_with_derivative(
        curve.control_points,
        curve.knots,
        curve.degree,
        u,
    );
    let t1 = diag_cyccnt();
    diag_eval_record(t1.wrapping_sub(t0));
    Ok(result)
}

/// Scale a `dP/du` (in the segment's normalized `u ∈ [0,1]` parameterization)
/// to the q16 velocity units the endstop trip-checker expects (`steps/sec`
/// scaled by `2^16`). Pulled out of the old `axis_velocity_q16` so callers
/// can feed in a `dx_du` they already computed alongside the position.
#[inline]
fn velocity_q16_from_dx_du(dx_du: f32, duration_cycles: f32, one_tick_cycles_value: u64) -> u32 {
    if duration_cycles <= 0.0 {
        return 0;
    }
    let cps = one_tick_cycles_value as f32 * crate::clock::TICK_RATE_HZ as f32;
    let scaled = dx_du.abs() * cps / duration_cycles * 65_536.0;
    if !scaled.is_finite() || scaled <= 0.0 {
        0
    } else if scaled >= u32::MAX as f32 {
        u32::MAX
    } else {
        scaled as u32
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use core::sync::atomic::Ordering;
    use heapless::spsc::Queue;

    use super::*;
    use crate::config::{EMode, McuAxisConfig, MotorConfig};
    use crate::endstop::{ArmMsg, ArmPolicy, SourceConfig, SourceKind, VelocityAxis};
    use crate::queue::Q_N;
    use crate::slot::{NoopIs, NoopPa};

    #[test]
    fn endstop_abort_clears_current_and_latches_homing_trip() {
        let _guard = endstop::test_guard();
        let mut sources = [SourceConfig::EMPTY; endstop::MAX_SOURCES];
        sources[0] = SourceConfig {
            kind: SourceKind::Physical,
            gpio: 17,
            active_high: true,
            policy: ArmPolicy::TripImmediately,
            sample_n: 1,
            velocity_axis: VelocityAxis::X,
            v_min_q16: 0,
        };
        endstop::arm(ArmMsg {
            arm_id: 100,
            arm_clock: 0,
            source_count: 1,
            sources,
            stepper_count: 1,
            stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
        })
        .expect("arm endstop");

        let pool = CurvePool::new();
        let x_handle = pool
            .validate_and_load(0, 1, &[0.0, 0.0, 1.0, 1.0], &[0.0, 10.0])
            .expect("load x curve");
        let mut queue: Queue<Segment, Q_N> = Queue::new();
        let (mut producer, mut consumer) = queue.split();
        producer
            .enqueue(Segment {
                id: 1,
                x_handle,
                y_handle: CurveHandle::UNUSED_SENTINEL,
                z_handle: CurveHandle::UNUSED_SENTINEL,
                e_handle: CurveHandle::UNUSED_SENTINEL,
                t_start: 0,
                t_end: 52_000,
                kinematics: KinematicTag::CartesianXyzAndE,
                e_mode: EMode::Travel,
                extrusion_ratio: 0.0,
                flags: 0,
                _pad: [0; 1],
            })
            .expect("enqueue segment");
        let mut trace_queue: Queue<TraceSample, TRACE_RING_N> = Queue::new();
        let (mut trace_producer, _trace_consumer) = trace_queue.split();
        let shared = SharedState::new();
        let mut engine = Engine::<NoopPa, NoopIs>::new(520_000_000);
        engine.configure(McuAxisConfig {
            motors: [
                Some(MotorConfig {
                    steps_per_mm: 1.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                None,
                None,
                None,
            ],
            kinematics: KinematicTag::CartesianXyzAndE,
        });
        let mut widen = WidenState::default();
        let result = engine.tick(
            0,
            &mut widen,
            &pool,
            &mut consumer,
            &mut trace_producer,
            &shared,
        );
        assert!(result.is_ok());
        let result = engine.tick(
            13_000,
            &mut widen,
            &pool,
            &mut consumer,
            &mut trace_producer,
            &shared,
        );
        assert!(result.is_ok());
        assert!(
            shared.stepper_counts[0].load(Ordering::Acquire) > 0,
            "expected stepper counter to advance before trip"
        );
        endstop::set_pin_level(17, true);
        let result = engine.tick(
            26_000,
            &mut widen,
            &pool,
            &mut consumer,
            &mut trace_producer,
            &shared,
        );

        assert_eq!(result, Err(RuntimeError::HomingTrip));
        assert_eq!(engine.status(), RuntimeStatus::Fault);
        assert_eq!(engine.last_error(), crate::error::KALICO_ERR_HOMING_TRIP);
        assert!(engine.current.is_none());
        let trip = endstop::poll_trip().expect("trip event");
        assert_eq!(trip.stepper_count, 1);
        assert_eq!(trip.steppers[0].oid, 0);
        assert_eq!(
            trip.steppers[0].step_count,
            shared.stepper_counts[0].load(Ordering::Acquire)
        );
    }
}
