//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use heapless::spsc::{Consumer, Producer};
// `Float` is unused on `host`-feature builds (std provides f32::sqrt) but
// load-bearing in the MCU `no_std` profile, where it dispatches to libm.
#[cfg_attr(feature = "host", allow(unused_imports))]
use nurbs::Float;

use crate::clock::{TickCounter, WidenState, one_tick_cycles, publish_widened_now};
use crate::curve_pool::{CurveHandle, CurvePool, CurveView};
use crate::endstop::{self, TripAction};
use crate::error::RuntimeError;
use crate::kinematics::{cartesian_xyz_with_e, corexy_with_e};
use crate::queue::Q_N;
use crate::segment::{
    CONS_REMAINING_E_SHIFT, CONS_REMAINING_X_SHIFT, CONS_REMAINING_Y_SHIFT,
    CONS_REMAINING_Z_SHIFT, KinematicTag, SEGMENT_FLAG_HOLD_SEGMENT, Segment,
};
use crate::slot::{IsSlot, PaSlot};
use crate::state::{SharedState, StepMode, TickState};
use crate::step_producer::{ProducerState, ProducerTickResult};
use crate::step_ring::StepRing;
use crate::step_time::{StepTimeQuery, StepTimeResult, compute_next_step_time};
use crate::trace::{
    TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_HOLD_SAMPLE, TRACE_FLAG_SEGMENT_END, TRACE_RING_N,
    TraceSample,
};

/// Batch cap for one call to [`Engine::producer_step`] per motor. Sized for
/// the ~30 cycle/step Newton inner loop on H7 (520 MHz): 32 steps × 30
/// cycles ≈ 960 cycles ≈ 1.85 µs per motor; 4 motors ≈ 7.4 µs per call.
/// Bounded to keep producer-timer dispatch latency under control. Spec §3.4.
pub const PRODUCER_BATCH_CAP: u32 = 32;

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
    /// Previous X position. Used for E-mode arc-length integration AND as
    /// the "hold" value when the X handle of the current segment is
    /// `UNUSED_SENTINEL` (the bridge omits constant axis curves; the
    /// engine must hold the axis at its last known position rather than
    /// drop it to zero — bench session 2026-05-11, STEP_BURST_EXCEEDED
    /// fault on cross-segment Y handle transition).
    prev_x: f32,
    /// Previous Y position. Same dual role as `prev_x`.
    prev_y: f32,
    /// Previous Z position. Same dual role as `prev_x` / `prev_y`. Before
    /// 2026-05-11 the engine had no `prev_z` field because Z was never
    /// E-arc-length-integrated and the bridge always sent a Z curve
    /// (no `UNUSED` case). Added when `UNUSED → hold prev value` became
    /// the engine's semantic for all kinematic axes.
    prev_z: f32,
    /// E accumulator for CoupledToXy mode — f64 for sub-step accuracy over
    /// millions of ticks (H723 has hardware double-precision FPU).
    e_accumulator: f64,
    /// Set to `true` on init and after flush/clear so the first segment seeds
    /// `prev_x`/`prev_y`/`prev_z` from X(0)/Y(0)/Z(0) rather than computing
    /// a spurious delta from (0,0,0).
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

    // ─── Step-emission rewrite (Task 5) ──────────────────────────────────
    // Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md
    /// Per-motor append-only step-pulse rings (spec §3.3). The producer
    /// (`Engine::producer_step`) pushes `(cycles_abs_lo, dir)` entries
    /// derived by Newton iteration on the curve; the per-stepper C-side
    /// timer consumer (Task 7) reads and fires them. Indexed in motor
    /// space: CoreXY `[A, B, Z, E]`; Cartesian `[X, Y, Z, E]`.
    ///
    /// Field is `pub` so the FFI (Task 6) can hand the C consumer a
    /// pointer to a specific motor's ring without going through a Rust
    /// accessor each tick.
    pub step_rings: [StepRing; 4],
    /// Per-motor Newton-fill resume state (spec §3.4). Tracks the active
    /// curve's `(step_distance, t_resume, step_at_curve_start,
    /// steps_pushed_this_curve)` between batch-capped producer_step
    /// invocations.
    pub producer_states: [ProducerState; 4],
    /// Per-motor monotonic counter — how many segments this motor has
    /// finished consuming. Currently advances in lockstep with the engine's
    /// shared `producer_current` slot (see `producer_step` for the
    /// simplification rationale). Reserved for true per-motor cursor
    /// independence post-MVP.
    pub motor_curve_cursor: [u32; 4],
    /// Per-motor "currently-filling-this-segment-id" stash. `Some(seg.id)`
    /// while the motor's producer state is Newton-filling a curve;
    /// cleared back to `None` when Newton returns `SegmentExhausted` so
    /// `producer_step` can clear the motor's bit in the segment's
    /// `consumers_remaining` mask.
    pub motor_current_segment_id: [Option<u32>; 4],
    /// Shared "segment currently being filled by the StepTime producer
    /// path." Lockstep simplification: all four motors operate on this
    /// same segment until every motor's `consumers_remaining` bit is
    /// clear, then the segment retires and we dequeue the next. Distinct
    /// from `current` (which the legacy `tick`/TIM5 path uses) so the
    /// two paths can coexist during the T5→T11 transition.
    pub producer_current: Option<Segment>,

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
            prev_z: 0.0,
            e_accumulator: 0.0,
            needs_xy_seed: true,
            debug_last_now: 0,
            debug_last_tstart: 0,
            debug_last_duration: 0,
            step_state: [crate::step::StepMotorState::default(); 4],
            mcu_config: None,
            step_rings: [
                StepRing::new(),
                StepRing::new(),
                StepRing::new(),
                StepRing::new(),
            ],
            producer_states: [
                ProducerState::new(0.0),
                ProducerState::new(0.0),
                ProducerState::new(0.0),
                ProducerState::new(0.0),
            ],
            motor_curve_cursor: [0; 4],
            motor_current_segment_id: [None; 4],
            producer_current: None,
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
                if let Some(ps) = self.producer_states.get_mut(i) {
                    // Seed the producer's step_distance so Newton's target math
                    // (step n × step_distance) lines up with the configured
                    // steps_per_mm. The producer recomputes step_distance only
                    // on configure() — there's no per-segment override.
                    let step_distance = if motor.steps_per_mm > 0.0 {
                        1.0_f64 / f64::from(motor.steps_per_mm)
                    } else {
                        0.0
                    };
                    *ps = ProducerState::new(step_distance);
                }
            }
        }
        self.mcu_config = Some(config);
    }

    /// Read-only accessor for a motor's step ring. Used by the new
    /// integration tests and by Task 7's C-side consumer (which gets its
    /// pointer via FFI in Task 6, not through this Rust accessor).
    pub fn step_ring(&self, motor_idx: usize) -> Option<&StepRing> {
        self.step_rings.get(motor_idx)
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

    /// Synchronous foreground flush. Spec §3.10 (Task 11).
    ///
    /// Drains every in-flight state container the new step-emission
    /// architecture holds (`producer_current`, the segment queue, the
    /// step rings, per-motor `ProducerState`, per-motor `StepAccumulator`,
    /// `clear_current`'s legacy boundary-loop state) and retires every
    /// curve-pool slot the dropped segments referenced. After this returns,
    /// the engine is in the "fresh, no work" state: subsequent
    /// `producer_step` / `runtime_modulated_tick` invocations return
    /// immediately because both the queue and `producer_current` are
    /// empty.
    ///
    /// **Caller contract:** must guarantee no concurrent `producer_step`,
    /// `runtime_modulated_tick`, `Engine::tick`, or per-motor consumer
    /// access. The host-side flush path serialises through the bridge
    /// command channel before invoking this. The FFI wrapper
    /// (`kalico_runtime_force_idle`) inherits the same contract and is
    /// the single legitimate caller.
    ///
    /// Returns `()` because the operation is total — there is no failure
    /// mode beyond preconditions, which are caller-responsibility.
    pub fn runtime_force_idle(
        &mut self,
        pool: &CurvePool,
        queue: &mut Consumer<'_, Segment, Q_N>,
        shared: &SharedState,
    ) {
        // 1. Disarm producer kicks. Any kick that lands mid-flush
        //    re-CASes; producer_pending stays false until the next
        //    legitimate push or low-water hook. Producer-pending is
        //    consulted only by the kicker (CAS-false→true) and the
        //    producer entry point (clear-on-entry); the synchronous
        //    flush path mirrors the "clear-on-entry" half.
        shared.producer_pending.store(false, Ordering::Release);

        // 2. Drain the segment queue. Each dequeued segment's four pool
        //    handles are retired now — the host's lookahead is being
        //    torn down, so we discharge their slot ownership eagerly.
        //    `confirm_retired` is a no-op on `UNUSED_SENTINEL` (the
        //    sentinel slot_idx is out-of-range vs `CURVE_POOL_N`).
        while let Some(seg) = queue.dequeue() {
            pool.confirm_retired(seg.x_handle);
            pool.confirm_retired(seg.y_handle);
            pool.confirm_retired(seg.z_handle);
            pool.confirm_retired(seg.e_handle);
        }

        // 3. Reset every step ring. `StepRing::reset` documents its
        //    caller-side quiescence requirement; the §3.10 contract on
        //    this method satisfies it.
        for ring in &mut self.step_rings {
            ring.reset();
        }

        // 4. Clear every motor's Newton-fill resume state.
        for ps in &mut self.producer_states {
            ps.clear();
        }

        // 5. Retire the producer's in-flight wall-clock segment if any
        //    (T7's per-motor path) and clear the slot.
        if let Some(seg) = self.producer_current.take() {
            pool.confirm_retired(seg.x_handle);
            pool.confirm_retired(seg.y_handle);
            pool.confirm_retired(seg.z_handle);
            pool.confirm_retired(seg.e_handle);
        }

        // 6. Per-motor curve cursor + current-segment-id reset. All
        //    motors restart from "no segment in flight" — the lockstep
        //    simplification (see `motor_curve_cursor` field doc) makes
        //    this safe: there is no "this motor is behind that motor"
        //    state to preserve.
        for cur in &mut self.motor_curve_cursor {
            *cur = 0;
        }
        for slot in &mut self.motor_current_segment_id {
            *slot = None;
        }

        // 7. Reset per-motor `StepAccumulator` residual. The
        //    cross-segment accumulator memory carries pre-flush
        //    sub-step position; on the next segment push the host
        //    re-anchors motor position via `SET_KINEMATIC_POSITION`
        //    (or via a planner-emitted re-seed) so the residual is
        //    meaningless. We must NOT `*ss = Default::default()` —
        //    that would zero the configured `steps_per_mm`, which the
        //    host doesn't re-emit after a flush.
        for ss in &mut self.step_state {
            ss.reset_accumulator();
        }

        // 8. Clear the legacy boundary-loop / tick-path state. T12 will
        //    delete `Engine::tick` and `current`/`needs_xy_seed`/
        //    `e_accumulator` will move out; until then keep the clear
        //    here so a same-process re-stream after force_idle is
        //    clean even on the legacy path.
        self.clear_current();
        self.last_motors = [0.0; 4];
        self.prev_x = 0.0;
        self.prev_y = 0.0;
        self.prev_z = 0.0;

        // 9. Re-publish settled engine status. After force_idle the
        //    host either re-streams (transitions Idle → Running on
        //    next segment activation) or stays idle, so `Idle` is the
        //    most accurate post-flush state. Fault status is NOT
        //    cleared — the host explicitly inspects the fault before
        //    issuing flush; clearing it would mask the failure history.
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }

        // 10. Transition gesture for the legacy `acked_force_idle`
        //     polling path (e.g., `runtime::stream::flush`'s Plan-
        //     decision A spin loop). Setting the ack here lets the
        //     legacy code path observe "ISR ack" after the synchronous
        //     flush returns; the legacy atomic deletion is T12's
        //     cleanup-pass scope.
        shared.acked_force_idle.store(true, Ordering::Release);
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
        // T11 (spec §3.10): the legacy ISR-side `force_idle` short-circuit
        // is gone. Force-idle is now a synchronous foreground call
        // (`Engine::runtime_force_idle` / `kalico_runtime_force_idle`)
        // that drains queue + step rings + producer state directly under
        // the caller's quiescence contract — no ISR ack required.
        // `shared.force_idle` / `shared.acked_force_idle` survive in
        // SharedState as a transition-period courtesy for the legacy
        // `runtime::stream::flush` polling code (set by
        // `runtime_force_idle`'s tail); T12's cleanup pass deletes both
        // atomics once no caller polls them.
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
        //
        // **UNUSED-handle semantic (2026-05-11 fix).** When the host bridge
        // omits a curve for this axis (`is_trivially_constant` skip in
        // `dispatch.rs::build_push_params`), `x_handle` is the unused
        // sentinel. The engine MUST hold the axis at its last known
        // position — returning `(0.0, 0.0)` here declares the axis "is at
        // zero" and, combined with the per-axis kinematic transform
        // (CoreXY `motor_a = x + y`), produces a phantom 100 mm position
        // jump that trips STEP_BURST_EXCEEDED on the first tick of the
        // affected segment. See engine_tests::unused_handle_holds_prev.
        let (x, dx_du_x) = if current.x_handle.is_unused_sentinel() {
            (self.prev_x, 0.0)
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
        // UNUSED-handle semantic: see X-axis branch above.
        let (y, dx_du_y) = if current.y_handle.is_unused_sentinel() {
            (self.prev_y, 0.0)
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
        // UNUSED-handle semantic: see X-axis branch above. `prev_z` was
        // added in the same 2026-05-11 fix — pre-fix the engine had no
        // Z hold state because Z was never E-arc-length-integrated.
        let (z, dx_du_z) = if current.z_handle.is_unused_sentinel() {
            (self.prev_z, 0.0)
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
            let seed_z = z;
            self.prev_x = seed_x;
            self.prev_y = seed_y;
            self.prev_z = seed_z;
            // Seed step accumulators from initial motor positions.
            let seed_positions = [seed_x, seed_y, seed_z, self.e_accumulator as f32];
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
                self.prev_z = z;
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
                // Update prev_x/prev_y/prev_z even in Independent mode so a
                // subsequent CoupledToXy segment doesn't see a stale position,
                // and so the UNUSED-handle hold semantic has the right value.
                self.prev_x = x;
                self.prev_y = y;
                self.prev_z = z;
                e_val
            }
            crate::config::EMode::Travel => {
                // E unchanged — use current accumulator value.
                self.prev_x = x;
                self.prev_y = y;
                self.prev_z = z;
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
            // 2026-05-13 MVP revert: the StepTime gate (per-stepper struct timer
            // handles output) is disabled. The per-stepper-timer path was not
            // catching live segments fast enough on real hardware (bench 2026-05-13:
            // 88k step_time_event polls, 8 emit_calls; segments retired within
            // 1 tick of activation, before step_time_event could compute step
            // pulses). Reverting to the polled-tick StepAccumulator path that
            // worked pre-refactor: Engine::tick emits all axes every tick, the
            // accumulator's multi-step-per-tick semantics handle step rates
            // higher than the tick rate. Per-stepper timer code remains and
            // step_time_event will continue polling at 1 kHz NO_STEP, but the
            // double-emit risk is negligible at the 0.01% success rate it was
            // showing. Re-enable the gate once per-stepper-timer is fixed
            // properly (spec §6 follow-up).
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
        // §13.1: trace-ring usage. The production `runtime_drain` path's only
        // load-bearing consumer of the trace stream is `drain_and_reclaim`,
        // which acts ONLY on `TRACE_FLAG_SEGMENT_END` samples — per-tick
        // samples are drained and discarded. The streamed `kalico_trace
        // count=N data=*` output to the host is wider than the USB-CDC 320 B
        // `transmit_buf` so `console_sendf` silently drops it in production
        // anyway. Per-tick samples are observably unused.
        //
        // Therefore: enqueue ONLY when SEGMENT_END is set. This makes the
        // 1199-deep ring effectively unbounded in steady state (one entry
        // per ~10 Hz segment retirement vs the 40 kHz ISR previously hammering
        // every tick), eliminating the F446 trace-overflow fault entirely
        // (180 MHz soft-float foreground previously couldn't drain a fully-
        // fed ring fast enough to stay ahead, latching `TraceOverflow` mid-
        // motion). H7 retains identical observable behaviour — its slower
        // 64 k/s vs 40 k/s drain-vs-fill margin was already comfortable, and
        // the runtime_drain path still receives every SEGMENT_END.
        //
        // If a future cycle wants per-tick host visibility (e.g. live phase
        // current logging) the path is: (a) widen `transmit_buf`, (b) re-
        // introduce per-tick enqueue gated on a new `runtime_trace_verbose`
        // shared flag, (c) keep the SEGMENT_END-drop-only fault filter.
        if segment_end_flag != 0 {
            let enqueue_failed = trace
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
                .is_err();
            if enqueue_failed {
                // SEGMENT_END drop is still fatal — losing the marker would
                // leak a CurvePool slot since reclaim is driven by it.
                shared.sample_drop_pending.store(true, Ordering::Release);
            }
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

    /// Compute the absolute MCU clock cycle at which the next step pulse should
    /// fire for stepper `stepper_idx`, given the currently active segment.
    ///
    /// `stepper_idx` is in motor space (same indexing as `step_state` /
    /// `stepper_counts`). For Cartesian kinematics: 0=X, 1=Y, 2=Z, 3=E. For
    /// CoreXY: 0=A(=X+Y), 1=B(=X−Y), 2=Z, 3=E. CoreXY motors 0 and 1 require
    /// two curves each and are **not yet supported** — they return `None`.
    ///
    /// Returns `None` when:
    /// - No active segment (`engine.current` is `None`)
    /// - `stepper_idx` is out of range
    /// - The axis curve handle is the UNUSED sentinel
    /// - The curve pool lookup fails (stale handle)
    /// - The Newton solver reports `SegmentExhausted` (no more steps in this
    ///   segment in the current direction)
    ///
    /// Returns `(cycles_abs, dir)` where `dir` is `+1` (positive / forward) or
    /// `-1` (negative / reverse) — the direction the stepper moves at the
    /// moment of the next step, derived from `sign(velocity(t_curr))`.
    pub fn arm_step_timer(
        &self,
        pool: &CurvePool,
        stepper_idx: u8,
        now_cycles: u64,
        current_step: i32,
    ) -> Option<(u64, i8)> {
        let current = self.current.as_ref()?;

        // Bounds-check: only 4 motors supported (indices 0–3).
        let motor_idx = stepper_idx as usize;
        if motor_idx >= 4 {
            return None;
        }

        // Map motor index → curve handle(s) and "is this the CoreXY A or B
        // combined motor" flag. CoreXY motors 0 (A = X + Y) and 1 (B = X − Y)
        // need BOTH the X and Y curves combined; the rest are single-axis.
        //
        // 2026-05-13: this is the "X/Y don't move on CoreXY" bug from Codex's
        // step-gen audit. Previously this match returned None for motors 0/1
        // on CoreXY, so step_time_event for A/B steppers always saw NO_STEP
        // and never emitted a pulse. The Trident bench uses CoreXY (kin=0 in
        // motion_toolhead.configure_axes log), which was a 100% repro of the
        // "stepper energized but no XY motion" bench symptom.
        let (handle, second_handle_for_corexy, corexy_sign) = match current.kinematics {
            KinematicTag::CoreXyAndE => match motor_idx {
                0 => (current.x_handle, Some(current.y_handle), 1.0_f64), // A = X + Y
                1 => (current.x_handle, Some(current.y_handle), -1.0_f64), // B = X − Y
                2 => (current.z_handle, None, 0.0_f64),
                3 => (current.e_handle, None, 0.0_f64),
                _ => return None,
            },
            KinematicTag::CartesianXyzAndE => match motor_idx {
                0 => (current.x_handle, None, 0.0_f64),
                1 => (current.y_handle, None, 0.0_f64),
                2 => (current.z_handle, None, 0.0_f64),
                3 => (current.e_handle, None, 0.0_f64),
                _ => return None,
            },
        };

        // For CoreXY A/B, the combined position is `x + sign·y`. If both
        // handles are UNUSED, the segment doesn't move this motor (e.g.
        // a pure-Z segment on a CoreXY MCU's A motor) — return None.
        let primary_unused = handle.is_unused_sentinel();
        let secondary_unused = second_handle_for_corexy
            .map(|h| h.is_unused_sentinel())
            .unwrap_or(true);
        if primary_unused && secondary_unused {
            return None;
        }

        // Resolve curves. For CoreXY each may be absent (treat as constant 0).
        // CurveView doesn't impl Copy/Clone, so resolve once and consume into
        // the closure by move. Use raw pointers to side-step the borrow
        // checker (the pool reference outlives this scope, so the underlying
        // slices stay valid for the closure's lifetime).
        let is_corexy = second_handle_for_corexy.is_some();
        let cv_primary = if primary_unused { None } else { pool.resolve(handle) };
        let cv_secondary = second_handle_for_corexy.and_then(|h| {
            if h.is_unused_sentinel() {
                None
            } else {
                pool.resolve(h)
            }
        });
        if !is_corexy && cv_primary.is_none() {
            return None;
        }

        // Segment time domain.
        let t_start = current.t_start;
        let t_end = current.t_end;

        // steps_per_mm for this motor axis.
        let spm = self
            .step_state
            .get(motor_idx)
            .map(|s| s.debug_steps_per_mm())
            .unwrap_or(0.0);
        if spm <= 0.0 {
            // Axis not configured; can't compute step timing.
            return None;
        }
        let step_distance_mm = 1.0_f64 / f64::from(spm);

        // Convert to normalized segment domain BEFORE entering float to avoid
        // catastrophic cancellation when t_start is a large absolute cycle count.
        // The u64 subtraction (now_cycles - t_start) is done in integer space;
        // we then normalize once into a small `[0, 1)` range where f64 precision
        // is essentially exact for our purposes.
        let duration = t_end.saturating_sub(t_start);
        if duration == 0 {
            return None;
        }
        let t_curr_norm = (now_cycles.saturating_sub(t_start) as f64) / duration as f64;
        let t_end_norm = 1.0_f64;

        if t_curr_norm < 0.0 || t_curr_norm >= 1.0 {
            return None;
        }

        // Build per-motor cubic Bézier coefficients. The planner emits
        // uniform cubic Béziers (knots [0,0,0,0,1,1,1,1]); we extract the
        // 4 control points per axis and compose them under the motor's
        // kinematic transform (CoreXY A = X+Y, B = X−Y; Cartesian = single
        // axis). Missing/UNUSED axes default to all-zero coefficients so
        // the Cardano solver naturally reports no root crossing and the
        // caller treats the motor as motionless on this segment.
        let coeffs_primary = cv_primary
            .as_ref()
            .and_then(extract_uniform_cubic_bezier_coeffs)
            .unwrap_or_else(|| crate::cardano::CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 0.0));
        let coeffs_secondary = cv_secondary
            .as_ref()
            .and_then(extract_uniform_cubic_bezier_coeffs)
            .unwrap_or_else(|| crate::cardano::CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 0.0));
        let coeffs = if is_corexy {
            if corexy_sign > 0.0 {
                coeffs_primary.add(&coeffs_secondary)
            } else {
                coeffs_primary.sub(&coeffs_secondary)
            }
        } else {
            coeffs_primary
        };

        let q = crate::step_time::StepTimeQuery {
            coeffs: &coeffs,
            step_distance: step_distance_mm,
            current_step,
            t_curr: t_curr_norm,
            t_segment_end: t_end_norm,
        };

        match crate::step_time::compute_next_step_time(&q) {
            crate::step_time::StepTimeResult::NextAt { t: t_norm_next, dir } => {
                // Convert back to absolute cycles in integer space.
                // dt_norm is the fractional step over the segment; multiplying
                // by duration (u64) gives the cycle delta without f32
                // precision loss from large absolute offsets.
                let dt_norm = t_norm_next - t_curr_norm;
                let dt_cycles = (dt_norm * duration as f64) as u64;
                Some((now_cycles + dt_cycles, dir))
            }
            crate::step_time::StepTimeResult::SegmentExhausted => None,
        }
    }

    // ─── Step-emission rewrite (Task 5) ──────────────────────────────────
    // Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md

    /// Enqueue a segment into the runtime's segment queue and kick the
    /// StepTime producer.
    ///
    /// The segment's `consumers_remaining` mask is recomputed here so the
    /// caller (test harness or bridge) doesn't have to thread the
    /// kinematics-aware bitmask logic — any bits already set in `seg.consumers_remaining`
    /// are ignored.
    ///
    /// Returns `Err(seg)` if the queue is full (mirrors `heapless::spsc::Producer::enqueue`).
    ///
    /// Spec §3.4 wake source #1.
    pub fn push_segment<'q>(
        &mut self,
        mut seg: Segment,
        queue_producer: &mut Producer<'q, Segment, Q_N>,
        shared: &SharedState,
    ) -> Result<(), Segment> {
        seg.consumers_remaining = Segment::compute_consumers_remaining(
            seg.kinematics,
            seg.x_handle,
            seg.y_handle,
            seg.z_handle,
            seg.e_handle,
        );
        queue_producer.enqueue(seg)?;
        // CAS-set the kick flag false→true. If we win, the caller's wake-
        // scheduling path (typically `sched_add_timer(producer_timer, now)`
        // on the C side, Task 8) fires next; if we lose, a pending kick is
        // already queued — the producer will see our new segment on its
        // next run regardless.
        let _ = shared.producer_pending.compare_exchange(
            false,
            true,
            Ordering::Release,
            Ordering::Acquire,
        );
        Ok(())
    }

    /// Pull the next un-consumed segment for motor `motor_idx`.
    ///
    /// **Lockstep simplification.** Today the StepTime producer operates
    /// on a single shared `producer_current` segment — when it's `None`
    /// we dequeue the next from the queue. The per-motor cursor
    /// (`motor_curve_cursor[i]`) tracks how many segments each motor has
    /// finished, but in the lockstep regime all four cursors advance
    /// together. Returns the (possibly newly-dequeued) shared segment and
    /// its effective per-motor curve handle.
    fn fetch_segment_for_motor(
        &mut self,
        motor_idx: usize,
        queue: &mut Consumer<'_, Segment, Q_N>,
    ) -> Option<(Segment, CurveHandle, Option<CurveHandle>, f64)> {
        if self.producer_current.is_none() {
            self.producer_current = queue.dequeue();
        }
        let seg = self.producer_current?;

        // Map motor index → curve handle(s) under the segment's kinematics.
        // For CoreXY A (motor 0) and B (motor 1) we get TWO handles
        // (x_handle + y_handle); for the cartesian / Z / E single-axis
        // case we get one handle and the optional second is None.
        let (primary, secondary, sign) = match seg.kinematics {
            KinematicTag::CoreXyAndE => match motor_idx {
                0 => (seg.x_handle, Some(seg.y_handle), 1.0_f64),
                1 => (seg.x_handle, Some(seg.y_handle), -1.0_f64),
                2 => (seg.z_handle, None, 0.0_f64),
                3 => (seg.e_handle, None, 0.0_f64),
                _ => return None,
            },
            KinematicTag::CartesianXyzAndE => match motor_idx {
                0 => (seg.x_handle, None, 0.0_f64),
                1 => (seg.y_handle, None, 0.0_f64),
                2 => (seg.z_handle, None, 0.0_f64),
                3 => (seg.e_handle, None, 0.0_f64),
                _ => return None,
            },
        };

        // If the motor consumes neither handle (e.g. CoreXY motor 2 on a
        // segment whose Z handle is UNUSED, or CoreXY motor 0 on a segment
        // where BOTH X and Y are UNUSED), this motor has no work for this
        // segment.
        let primary_unused = primary.is_unused_sentinel();
        let secondary_unused = secondary.map(|h| h.is_unused_sentinel()).unwrap_or(true);
        if primary_unused && secondary_unused {
            return None;
        }

        Some((seg, primary, secondary, sign))
    }

    /// Clear motor `motor_idx`'s consumer bit across the segment's
    /// `consumers_remaining` mask (one bit per UNUSED-aware axis curve).
    /// Called after Newton returns `SegmentExhausted` for that motor.
    ///
    /// Lockstep note: in the MVP regime where all four motors finish a
    /// segment within the same producer_step call, this drains the mask
    /// for every contributing motor; once all bits are clear the segment
    /// retires (curve handles → `pool.confirm_retired`) and is dropped
    /// from `producer_current`.
    fn clear_motor_bits_in_mask(seg: &mut Segment, motor_idx: u8) {
        // Compute which axis-nibbles this motor reads under the segment's
        // kinematics — only those nibbles get the motor's bit cleared.
        let motor_bit = 1_u16 << motor_idx;
        let consumes_x = match seg.kinematics {
            KinematicTag::CartesianXyzAndE => motor_idx == 0,
            KinematicTag::CoreXyAndE => motor_idx == 0 || motor_idx == 1,
        };
        let consumes_y = match seg.kinematics {
            KinematicTag::CartesianXyzAndE => motor_idx == 1,
            KinematicTag::CoreXyAndE => motor_idx == 0 || motor_idx == 1,
        };
        let consumes_z = motor_idx == 2;
        let consumes_e = motor_idx == 3;
        if consumes_x && !seg.x_handle.is_unused_sentinel() {
            seg.consumers_remaining &= !(motor_bit << CONS_REMAINING_X_SHIFT);
        }
        if consumes_y && !seg.y_handle.is_unused_sentinel() {
            seg.consumers_remaining &= !(motor_bit << CONS_REMAINING_Y_SHIFT);
        }
        if consumes_z && !seg.z_handle.is_unused_sentinel() {
            seg.consumers_remaining &= !(motor_bit << CONS_REMAINING_Z_SHIFT);
        }
        if consumes_e && !seg.e_handle.is_unused_sentinel() {
            seg.consumers_remaining &= !(motor_bit << CONS_REMAINING_E_SHIFT);
        }
    }

    /// Mainline producer entry point. Called from the C-side producer
    /// Klipper struct timer (Task 8) and from integration tests directly.
    ///
    /// Pipeline:
    /// 1. Clear `producer_pending`. Kicks landing after this point re-set
    ///    it and trigger a follow-on call.
    /// 2. Increment `producer_runs_total` (heartbeat).
    /// 3. For each StepTime motor whose `ProducerState` is idle, look up
    ///    its next curve from the segment queue (via `fetch_segment_for_motor`,
    ///    which dequeues into `producer_current` on demand). If no curve
    ///    is available, that motor stays idle this round.
    /// 4. For each StepTime motor with an active curve, Newton-fill its
    ///    ring up to `PRODUCER_BATCH_CAP` or `ring.space()`, whichever
    ///    comes first. The per-motor eval closure applies the kinematic
    ///    transform inline (CoreXY mixes x + y / x − y for motors 0 / 1;
    ///    cartesian / Z / E is identity on the single handle).
    /// 5. When Newton returns `SegmentExhausted` for a motor, that
    ///    motor's bit in the segment's `consumers_remaining` mask is
    ///    cleared and `motor_curve_cursor[i]` advances by one. Once the
    ///    mask reaches zero, the segment retires: every curve handle
    ///    runs through `pool.confirm_retired`, and `producer_current` is
    ///    cleared so the next call dequeues the next segment.
    ///
    /// Returns `ProducerTickResult::WorkPending` iff at least one motor
    /// still has an active curve at exit (more work for the caller to
    /// reschedule via `sched_add_timer`); `AllIdle` iff every motor is
    /// idle (caller waits for a kick).
    ///
    /// Spec §3.4 (`producer_step` pseudocode) + §3.8 (retirement).
    pub fn producer_step(
        &mut self,
        pool: &CurvePool,
        queue: &mut Consumer<'_, Segment, Q_N>,
        shared: &SharedState,
    ) -> ProducerTickResult {
        // (1) Clear kick-pending flag at start; (2) heartbeat.
        shared.producer_pending.store(false, Ordering::Release);
        shared.producer_runs_total.fetch_add(1, Ordering::AcqRel);

        // We return `WorkPending` only when this call actually made
        // progress (filled at least one ring entry OR finished a curve).
        // Returning WorkPending when we did nothing — e.g., because every
        // motor's ring was full and we filled zero entries — causes the
        // C-side producer timer to self-reschedule at `now + 1µs`, which
        // pegs Klipper's timer-dispatch loop at ~333 kHz busy-spinning,
        // starves other timers, and eventually trips Klipper's
        // "Rescheduled timer in the past" panic (armcm_timer.c:152
        // fires when any timer's waketime is >1ms behind `now` while the
        // dispatch loop has been running tight). The correct contract:
        // - WorkPending = "I did work, please run me again ASAP."
        // - AllIdle    = "I'm blocked or done, wait for an external kick
        //                (push_segment or consumer low-water)."
        // When the ring is full, the consumer's low-water hook will kick
        // us back to life after it drains below STEP_RING_LOW_WATER.
        let mut made_progress = false;

        // Single-pass per-motor fill. Spec §3.4 budget: ~30 cycles per
        // Newton solve × PRODUCER_BATCH_CAP=32 entries × 4 motors ≈
        // 3.8k cycles ≈ 7.4 µs at 520 MHz (H7). Earlier revisions ran
        // up to 8 outer iterations here, which inflated worst-case ISR
        // duration to ~59 µs (1024 step times per call) and starved
        // other ISRs. The producer is event-driven: any motor with
        // remaining work returns WorkPending, the caller reschedules
        // promptly, and the next call resumes. Spec §3.4 + §3.8.
        {
            // (3) Per-motor work pass.
            for motor_idx in 0..4_usize {
                let mode = shared
                    .step_modes
                    .get(motor_idx)
                    .map(|m| m.load(Ordering::Acquire))
                    .unwrap_or(StepMode::Modulated as u8);
                if mode != StepMode::StepTime as u8 {
                    continue;
                }
                // Skip motors with no configured step_distance — they
                // can't produce step times.
                let step_distance = self
                    .producer_states
                    .get(motor_idx)
                    .map(|s| s.step_distance())
                    .unwrap_or(0.0);
                if step_distance <= 0.0 {
                    continue;
                }

                // Ensure a segment is available for this motor before we
                // build the eval closure. The actual `start_curve` call
                // happens AFTER the closure is constructed so we can seed
                // `initial_step` from `eval(0.0) / step_distance` — see
                // the comment block at `start_curve` below.
                let is_idle = self
                    .producer_states
                    .get(motor_idx)
                    .map(|s| s.is_idle())
                    .unwrap_or(true);
                if is_idle {
                    if self.fetch_segment_for_motor(motor_idx, queue).is_none() {
                        continue;
                    }
                }

                // Fill ring while space permits, capped at PRODUCER_BATCH_CAP.
                // The motor's "current curve" is whatever resolves out of
                // producer_current under the segment's kinematics —
                // re-fetch here to pick up freshly-started curves.
                let Some((seg_for_fill, primary, secondary, sign)) =
                    self.fetch_segment_for_motor(motor_idx, queue)
                else {
                    continue;
                };
                let cv_primary = if primary.is_unused_sentinel() {
                    None
                } else {
                    pool.resolve(primary)
                };
                let cv_secondary = secondary.and_then(|h| {
                    if h.is_unused_sentinel() {
                        None
                    } else {
                        pool.resolve(h)
                    }
                });

                let is_corexy = matches!(seg_for_fill.kinematics, KinematicTag::CoreXyAndE)
                    && (motor_idx == 0 || motor_idx == 1);

                let t_start_cycles = seg_for_fill.t_start;
                let duration_cycles = seg_for_fill.duration();
                if duration_cycles == 0 {
                    // Zero-duration segment — can't produce step times.
                    // Retire the motor's contribution and move on.
                    Self::clear_motor_bits_in_mask(
                        self.producer_current
                            .as_mut()
                            .expect("producer_current set"),
                        motor_idx as u8,
                    );
                    if let Some(ps) = self.producer_states.get_mut(motor_idx) {
                        ps.clear();
                    }
                    if let Some(slot) = self.motor_current_segment_id.get_mut(motor_idx) {
                        *slot = None;
                    }
                    if let Some(cur) = self.motor_curve_cursor.get_mut(motor_idx) {
                        *cur = cur.wrapping_add(1);
                    }
                    continue;
                }
                let duration_f64 = duration_cycles as f64;

                // Build per-motor cubic Bézier coefficients up-front (see
                // `arm_step_timer` for the rationale). The planner emits
                // uniform cubic Béziers; missing/UNUSED axes default to
                // zero coefficients so the Cardano solver reports no
                // root and the producer retires the motor's contribution.
                let coeffs_primary = cv_primary
                    .as_ref()
                    .and_then(extract_uniform_cubic_bezier_coeffs)
                    .unwrap_or_else(|| {
                        crate::cardano::CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 0.0)
                    });
                let coeffs_secondary = cv_secondary
                    .as_ref()
                    .and_then(extract_uniform_cubic_bezier_coeffs)
                    .unwrap_or_else(|| {
                        crate::cardano::CubicCoeffs::from_bezier(0.0, 0.0, 0.0, 0.0)
                    });
                let coeffs = if is_corexy {
                    if sign > 0.0 {
                        coeffs_primary.add(&coeffs_secondary)
                    } else {
                        coeffs_primary.sub(&coeffs_secondary)
                    }
                } else {
                    coeffs_primary
                };

                // Seed producer state if idle, using the curve's position at
                // u=0 as the integer-step baseline. Curves are in absolute
                // motor-frame mm; Newton's target is `(initial_step + dir) *
                // step_distance`, which must land within the curve's value
                // range or Newton diverges out of `[0, 1]` on the first
                // iteration and the segment exhausts with zero pulses.
                //
                // `stepper_counts[i]` cannot be used here: it's a counter,
                // not an absolute step position. It only grows when
                // `runtime_emit_step_pulses` fires — so at boot it is 0
                // regardless of where the toolhead physically is. Anchoring
                // `initial_step` to `eval(0.0) / step_distance` keeps the
                // Newton target in the same coordinate frame as the curve.
                //
                // We also publish this value to `shared.stepper_counts[i]`
                // so host queries (`kalico_runtime_get_stepper_count`) and
                // Klippy's position tracking remain coherent with the
                // motor's logical step position.
                if is_idle {
                    // `coeffs.eval(0.0) == c0 == p0` for a uniform cubic
                    // Bézier — the curve's position at u=0 in motor-frame
                    // millimetres. Same anchoring as the old eval(0.0).
                    let pos0 = coeffs.eval(0.0);
                    let initial_step = if step_distance > 0.0 {
                        (pos0 / step_distance) as i32
                    } else {
                        0
                    };
                    if let Some(ps) = self.producer_states.get_mut(motor_idx) {
                        ps.start_curve(initial_step);
                    }
                    if let Some(slot) = self.motor_current_segment_id.get_mut(motor_idx) {
                        *slot = Some(seg_for_fill.id);
                    }
                    if let Some(c) = shared.stepper_counts.get(motor_idx) {
                        c.store(initial_step, Ordering::Release);
                    }
                }

                // Inline Cardano step-fill — equivalent to step_producer::producer_step
                // for one motor, but here we have the engine-context loop so
                // we can short-circuit on ring-full and reuse the per-motor
                // `coeffs` value (composed once above from cv_primary /
                // cv_secondary under the motor's kinematic transform)
                // without slicing four parallel arrays across motors.
                let mut filled = 0_u32;
                let ring = match self.step_rings.get_mut(motor_idx) {
                    Some(r) => r,
                    None => continue,
                };
                let ps = match self.producer_states.get_mut(motor_idx) {
                    Some(p) => p,
                    None => continue,
                };
                let mut motor_finished_curve = false;
                while filled < PRODUCER_BATCH_CAP && ring.space() > 0 {
                    let q = StepTimeQuery {
                        coeffs: &coeffs,
                        step_distance: ps.step_distance(),
                        current_step: ps
                            .step_at_curve_start()
                            .wrapping_add(ps.steps_pushed_this_curve()),
                        t_curr: ps.t_resume().unwrap_or(0.0),
                        t_segment_end: 1.0,
                    };
                    match compute_next_step_time(&q) {
                        StepTimeResult::NextAt { t, dir } => {
                            // Convert normalized u → absolute cycles low-32.
                            let dt_cycles = (t * duration_f64) as u64;
                            let abs_cycles = t_start_cycles.saturating_add(dt_cycles);
                            ring.push(abs_cycles as u32, dir);
                            ps.set_t_resume(Some(t));
                            ps.bump_steps_pushed(i32::from(dir));
                            filled += 1;
                        }
                        StepTimeResult::SegmentExhausted => {
                            ps.clear();
                            motor_finished_curve = true;
                            break;
                        }
                    }
                }

                // Update ring high-water mark.
                let avail = ring.available();
                if let Some(hw) = shared.ring_high_water.get(motor_idx) {
                    let mut prev = hw.load(Ordering::Relaxed);
                    while avail > prev {
                        match hw.compare_exchange_weak(
                            prev,
                            avail,
                            Ordering::Release,
                            Ordering::Relaxed,
                        ) {
                            Ok(_) => break,
                            Err(observed) => prev = observed,
                        }
                    }
                }

                if motor_finished_curve {
                    if let Some(seg_mut) = self.producer_current.as_mut() {
                        Self::clear_motor_bits_in_mask(seg_mut, motor_idx as u8);
                    }
                    if let Some(slot) = self.motor_current_segment_id.get_mut(motor_idx) {
                        *slot = None;
                    }
                    if let Some(cur) = self.motor_curve_cursor.get_mut(motor_idx) {
                        *cur = cur.wrapping_add(1);
                    }
                }

                // Track whether this motor made progress this call. Filling
                // any ring entries OR finishing a curve both count — both
                // change the system's state in a way that warrants
                // self-rescheduling for another batch. Filling zero entries
                // (because the ring was full or batch budget already
                // consumed by other motors) is NOT progress.
                if filled > 0 || motor_finished_curve {
                    made_progress = true;
                }
            }

            // (5) Retire the producer-current segment if every consumer bit
            // is clear. The Modulated path (TIM5) writes its own bits in
            // future Task 10; today (StepTime-only) bits clear exclusively
            // through the loop above.
            if let Some(seg) = self.producer_current {
                if seg.consumers_done() {
                    // Curve-handle retirement. Sentinels are no-ops in
                    // `confirm_retired`, so this is safe regardless of
                    // which handles were UNUSED.
                    pool.confirm_retired(seg.x_handle);
                    pool.confirm_retired(seg.y_handle);
                    pool.confirm_retired(seg.z_handle);
                    pool.confirm_retired(seg.e_handle);
                    // Publish the retired-through cursor so the host's
                    // `kalico_credit_freed` plumbing (still wired through
                    // the existing trace-drain path on the C side) sees
                    // the segment as retired without a SEGMENT_END trace
                    // sample. The trace path itself stays alive for the
                    // legacy `tick` callers during the T5→T11 transition.
                    shared
                        .retired_through_segment_id
                        .store(seg.id, Ordering::Release);
                    crate::stream::check_terminal_on_retire(shared, seg.id);
                    self.producer_current = None;
                }
            }
        }

        // Result: WorkPending iff we made progress this call (filled at
        // least one entry OR finished a curve). See the comment at the
        // declaration of `made_progress` above for why this is correct
        // and why the prior `any_pending = (!state.is_idle())` was a
        // busy-spin bug.
        if made_progress {
            ProducerTickResult::WorkPending
        } else {
            ProducerTickResult::AllIdle
        }
    }

    /// TIM5 callback for Modulated motors (spec §3.2, T10).
    ///
    /// Runs only when at least one motor is in `StepMode::Modulated` (the
    /// FFI's caller — the TIM5 ISR — is itself gated on
    /// `count_modulated_steppers > 0`; `runtime_tick_enable` leaves TIM5
    /// disabled otherwise).
    ///
    /// Per-tick body:
    ///   1. Determine `u = (now - t_start) / duration` for the currently-
    ///      playing wall-clock segment (`producer_current` under the
    ///      lockstep simplification — see §7 question 2 of the spec).
    ///   2. For each Modulated motor: evaluate position from the segment's
    ///      axis curves at `u`, apply the kinematic transform, call
    ///      `StepMotorState::update`, and emit step pulses on non-zero
    ///      deltas.
    ///   3. When `now ≥ t_end`, clear every Modulated motor's bits in the
    ///      segment's `consumers_remaining` mask. Once that mask reaches
    ///      zero, retire the four curve-pool slots, publish the
    ///      `retired_through_segment_id` cursor, and clear `producer_current`.
    ///
    /// Does NOT touch the segment queue, the producer state, the step
    /// rings, or the trace ring. The StepTime path (`producer_step` +
    /// `step_time_event`) is responsible for those.
    ///
    /// **Simplification (T10 scope)**: when `StepMotorState::update` returns
    /// `Err(StepBurstExceeded)` we set `last_error` / `status` directly via
    /// the atomics on `SharedState` instead of going through `latch_fault`.
    /// `latch_fault` requires the trace `Producer`, which `IsrState` holds
    /// behind the field-disjoint borrow already used by `Engine::tick`; the
    /// modulated tick's FFI shim is built around the producer_current
    /// segment and doesn't currently thread the trace ring in. Tightening
    /// this up to mirror the `tick`-path fault path is wired in T11 once
    /// the force_idle / trace-emit topology settles.
    pub fn runtime_modulated_tick(&mut self, now: u64, pool: &CurvePool, shared: &SharedState) {
        // Pull the wall-clock segment. The producer's segment cursor is the
        // shared cursor under the MVP lockstep regime.
        let Some(mut seg) = self.producer_current else {
            return;
        };

        let elapsed = now.saturating_sub(seg.t_start);
        let duration = seg.duration().max(1);

        if elapsed >= duration {
            // Wall-clock crossed t_end — clear every Modulated motor's
            // bits in the segment's mask.
            for motor_idx in 0..4_u8 {
                let mode = shared
                    .step_modes
                    .get(motor_idx as usize)
                    .map(|m| m.load(Ordering::Acquire))
                    .unwrap_or(StepMode::StepTime as u8);
                if mode != StepMode::Modulated as u8 {
                    continue;
                }
                Self::clear_motor_bits_in_mask(&mut seg, motor_idx);
            }
            self.producer_current = Some(seg);

            if seg.consumers_done() {
                // Curve-handle retirement — sentinels are no-ops in
                // `confirm_retired`, mirroring `producer_step`'s path.
                pool.confirm_retired(seg.x_handle);
                pool.confirm_retired(seg.y_handle);
                pool.confirm_retired(seg.z_handle);
                pool.confirm_retired(seg.e_handle);
                shared
                    .retired_through_segment_id
                    .store(seg.id, Ordering::Release);
                crate::stream::check_terminal_on_retire(shared, seg.id);
                self.producer_current = None;
            }
            return;
        }

        // Wall-clock inside the segment — evaluate per-motor position,
        // run StepAccumulator, emit pulses.
        let u = ((elapsed as f32) / (duration as f32)).clamp(0.0, 1.0);

        // Resolve per-axis curve views once. `None` = UNUSED or unresolvable;
        // the corresponding motor holds its previous position.
        let cv_x = if seg.x_handle.is_unused_sentinel() {
            None
        } else {
            pool.resolve(seg.x_handle)
        };
        let cv_y = if seg.y_handle.is_unused_sentinel() {
            None
        } else {
            pool.resolve(seg.y_handle)
        };
        let cv_z = if seg.z_handle.is_unused_sentinel() {
            None
        } else {
            pool.resolve(seg.z_handle)
        };

        let x = cv_x
            .as_ref()
            .and_then(|c| scalar_eval(c, u).ok())
            .unwrap_or(self.prev_x);
        let y = cv_y
            .as_ref()
            .and_then(|c| scalar_eval(c, u).ok())
            .unwrap_or(self.prev_y);
        let z = cv_z
            .as_ref()
            .and_then(|c| scalar_eval(c, u).ok())
            .unwrap_or(self.prev_z);

        // E follows the existing tick-path semantics for Modulated; this
        // path is reached only when a Modulated motor is present, but the
        // per-motor StepAccumulator update needs a per-motor target. For
        // the T10 structural extraction we mirror `tick_with_current`'s
        // CoupledToXy / Independent / Travel dispatch but skip the
        // boundary-loop / seed-on-activation bookkeeping (those belong to
        // the producer side).
        let e: f32 = match seg.e_mode {
            crate::config::EMode::CoupledToXy => {
                let dx = x - self.prev_x;
                let dy = y - self.prev_y;
                let dist = (dx * dx + dy * dy).sqrt();
                self.e_accumulator += f64::from(seg.extrusion_ratio) * f64::from(dist);
                self.e_accumulator as f32
            }
            crate::config::EMode::Independent => {
                let cv_e = if seg.e_handle.is_unused_sentinel() {
                    None
                } else {
                    pool.resolve(seg.e_handle)
                };
                cv_e.as_ref()
                    .and_then(|c| scalar_eval(c, u).ok())
                    .unwrap_or(self.e_accumulator as f32)
            }
            crate::config::EMode::Travel => self.e_accumulator as f32,
        };

        self.prev_x = x;
        self.prev_y = y;
        self.prev_z = z;

        let positions = [x, y, z, e];
        let motors = match seg.kinematics {
            KinematicTag::CoreXyAndE => corexy_with_e(positions),
            KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(positions),
        };

        for motor_idx in 0..4_usize {
            let mode = shared
                .step_modes
                .get(motor_idx)
                .map(|m| m.load(Ordering::Acquire))
                .unwrap_or(StepMode::StepTime as u8);
            if mode != StepMode::Modulated as u8 {
                continue;
            }
            let Some(ss) = self.step_state.get_mut(motor_idx) else {
                continue;
            };
            let Some(&m) = motors.get(motor_idx) else {
                continue;
            };
            match ss.update(m) {
                Ok(step_result) => {
                    if step_result.n_steps != 0 {
                        if let Some(counter) = shared.stepper_counts.get(motor_idx) {
                            counter.fetch_add(step_result.n_steps, Ordering::AcqRel);
                        }
                        emit_step_pulses(motor_idx as u8, step_result.n_steps);
                    }
                }
                Err(()) => {
                    // T10 simplification: latch the fault via atomics only
                    // (no trace marker — see method-level doc comment).
                    shared
                        .last_error
                        .store(crate::error::KALICO_ERR_STEP_BURST_EXCEEDED, Ordering::Release);
                    shared
                        .runtime_status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    self.last_error
                        .store(crate::error::KALICO_ERR_STEP_BURST_EXCEEDED, Ordering::Release);
                    self.status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    return;
                }
            }
        }

        self.last_motors = motors;
    }
}

/// Compute the next step-pulse cycle for stepper `stepper_idx` using the
/// `RuntimeContext`'s currently active segment.
///
/// Called from the per-stepper step-time `struct timer` ISR (Task D1's
/// `step_time_event` in C) and from the foreground configure/segment-load
/// path. Forms `&IsrState` via `IsrState::raw_ref_from_ctx`; safety
/// requires no concurrent `&mut IsrState` writer at the call site.
///
/// The invariant: TIM5 is the only writer that forms `&mut IsrState`
/// (in its handler). Task D2 guarantees TIM5 is disabled whenever no
/// stepper is in `Modulated` mode, so on F4 (no PHASE_STEPPING capability)
/// TIM5 never enables and `&IsrState` from this path is exclusive. On
/// H7 with mixed Modulated/StepTime steppers, callers must ensure their
/// invocation does not interleave with a TIM5 ISR fire — either by
/// preempting TIM5 (higher NVIC priority for the step-time timer) or
/// by gating the call on `Modulated` count.
///
/// Returns `None` on the same conditions as `Engine::arm_step_timer`.
/// On success, returns `Some((cycles_abs, dir))` — see `Engine::arm_step_timer`.
#[allow(unsafe_code)]
pub fn arm_step_timer_for_stepper(
    ctx: &crate::state::RuntimeContext,
    stepper_idx: u8,
    now_cycles: u64,
) -> Option<(u64, i8)> {
    // SAFETY: We form a shared (non-exclusive) reference to IsrState.
    // The caller guarantees this is invoked from the ISR context where no
    // concurrent &mut IsrState is held. `UnsafeCell::raw_get` is the
    // canonical pattern for this crate (see runtime_ffi.rs).
    let isr: &crate::state::IsrState =
        // SAFETY: isr field is valid, initialized at RuntimeContext::init time,
        // and we hold only a shared reference here.
        unsafe { &*crate::state::IsrState::raw_ref_from_ctx(ctx) };

    let current_step = ctx
        .shared
        .stepper_counts
        .get(stepper_idx as usize)?
        .load(core::sync::atomic::Ordering::Acquire);

    isr.engine.arm_step_timer(&ctx.curve_pool, stepper_idx, now_cycles, current_step)
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

/// Extract uniform-cubic-Bézier monomial coefficients from a `CurveView`.
///
/// Returns `None` when the curve isn't a degree-3 with 4 control points;
/// such a curve can't be inverted in closed form by the Cardano solver,
/// and the producer should treat the affected motor as motionless for
/// this segment. The planner-emit invariant is uniform cubic Bézier with
/// knots `[0,0,0,0,1,1,1,1]`, so the conversion is exact via the
/// standard Bernstein → monomial expansion in `CubicCoeffs::from_bezier`.
///
/// We trust the knot vector without re-checking — if a non-uniform-knot
/// cubic ever reaches the producer, that's a planner/bridge contract
/// violation upstream, not a producer bug. The hard length/degree guard
/// remains because a malformed curve in a release MCU build must not
/// silently produce garbage coefficients.
fn extract_uniform_cubic_bezier_coeffs(
    cv: &CurveView<'_>,
) -> Option<crate::cardano::CubicCoeffs> {
    if cv.degree != 3 {
        return None;
    }
    // Pattern match the slice — proves to clippy that we won't panic
    // on indexing and replaces the `len() == 4` + four index-into-slice
    // accesses that tripped `clippy::indexing_slicing` (deny-level).
    let [c0, c1, c2, c3] = *<&[f32; 4]>::try_from(cv.control_points).ok()?;
    Some(crate::cardano::CubicCoeffs::from_bezier(
        f64::from(c0),
        f64::from(c1),
        f64::from(c2),
        f64::from(c3),
    ))
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
                consumers_remaining: 0,
            })
            .expect("enqueue segment");
        let mut trace_queue: Queue<TraceSample, TRACE_RING_N> = Queue::new();
        let (mut trace_producer, _trace_consumer) = trace_queue.split();
        let shared = SharedState::new();
        // Test exercises the polled-tick step-accumulator path, which is
        // gated to `Modulated` mode. Default is `StepTime` (per-stepper
        // timer ISR handles output instead). Flip motor 0 to Modulated
        // so `Engine::tick` actually emits the step pulses this test asserts.
        shared.step_modes[0].store(
            crate::state::StepMode::Modulated as u8,
            Ordering::Release,
        );
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

    /// **Regression for 2026-05-11 STEP_BURST_EXCEEDED on cross-segment
    /// UNUSED-handle transitions.** Two CoreXY-style segments back to
    /// back, where the bridge sent Y and Z curves on segment 1 (anchoring
    /// `prev_y = 100`, `prev_z = 10`) and then UNUSED Y and Z on segment
    /// 2 (because refit produced exact constants and the now-removed
    /// `is_trivially_constant` skip would have fired). With the
    /// pre-2026-05-11 engine, the segment-2 first tick evaluated Y and Z
    /// as `0.0`, motor A delta = (x + 0) − (x + 100) = −100 mm = −8000
    /// steps ≫ MAX_STEPS_PER_TICK, faulting STEP_BURST_EXCEEDED.
    ///
    /// Post-fix the engine returns `(prev_y, 0.0)` for UNUSED Y (same
    /// for Z), so segment 2 sees `delta_y = 0` on its first tick — no
    /// burst, motor A's accumulator tracks just X motion.
    #[test]
    fn unused_handle_holds_prev_value_across_segment_boundary() {
        use crate::queue::Q_N;
        let pool = CurvePool::new();

        // Segment 1: X 0→25, Y constant 100, Z constant 10 (all three
        // curves loaded into the pool).
        let s1_x = pool
            .validate_and_load(0, 1, &[0.0, 0.0, 1.0, 1.0], &[0.0, 25.0])
            .expect("s1 x curve");
        let s1_y = pool
            .validate_and_load(1, 1, &[0.0, 0.0, 1.0, 1.0], &[100.0, 100.0])
            .expect("s1 y curve");
        let s1_z = pool
            .validate_and_load(2, 1, &[0.0, 0.0, 1.0, 1.0], &[10.0, 10.0])
            .expect("s1 z curve");

        // Segment 2: X 25→50, Y and Z UNUSED (simulates the bridge having
        // detected the constant Y/Z curves as trivially-constant on this
        // segment, even though it sent them on segment 1 — the exact
        // brittle behaviour that triggered the bug).
        let s2_x = pool
            .validate_and_load(3, 1, &[0.0, 0.0, 1.0, 1.0], &[25.0, 50.0])
            .expect("s2 x curve");

        let mut queue: Queue<Segment, Q_N> = Queue::new();
        let (mut producer, mut consumer) = queue.split();
        // Sized so per-tick motion stays well under MAX_STEPS_PER_TICK
        // (= 16 steps = 0.2 mm at 80 steps/mm). 25 mm over 1000 ticks =
        // 0.025 mm / tick = 2 steps / tick, comfortably under the cap.
        const TICKS_PER_SEG: u32 = 1000;
        const CYC_PER_TICK: u32 = 13_000;
        const SEG_DURATION: u64 = (TICKS_PER_SEG as u64) * (CYC_PER_TICK as u64);

        producer
            .enqueue(Segment {
                id: 1,
                x_handle: s1_x,
                y_handle: s1_y,
                z_handle: s1_z,
                e_handle: CurveHandle::UNUSED_SENTINEL,
                t_start: 0,
                t_end: SEG_DURATION,
                kinematics: KinematicTag::CoreXyAndE,
                e_mode: EMode::Travel,
                extrusion_ratio: 0.0,
                flags: 0,
                _pad: [0; 1],
                consumers_remaining: 0,
            })
            .expect("enqueue s1");
        producer
            .enqueue(Segment {
                id: 2,
                x_handle: s2_x,
                y_handle: CurveHandle::UNUSED_SENTINEL,
                z_handle: CurveHandle::UNUSED_SENTINEL,
                e_handle: CurveHandle::UNUSED_SENTINEL,
                t_start: SEG_DURATION,
                t_end: SEG_DURATION * 2,
                kinematics: KinematicTag::CoreXyAndE,
                e_mode: EMode::Travel,
                extrusion_ratio: 0.0,
                flags: 0,
                _pad: [0; 1],
                consumers_remaining: 0,
            })
            .expect("enqueue s2");

        let mut trace_queue: Queue<TraceSample, TRACE_RING_N> = Queue::new();
        let (mut trace_producer, _trace_consumer) = trace_queue.split();
        let shared = SharedState::new();
        let mut engine = Engine::<NoopPa, NoopIs>::new(520_000_000);
        // CoreXY MCU: A (motor 0) + B (motor 1) drive X/Y, Z on motor 2.
        engine.configure(McuAxisConfig {
            motors: [
                Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
                Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
                Some(MotorConfig { steps_per_mm: 400.0, is_awd: false, invert_dir: false }),
                None,
            ],
            kinematics: KinematicTag::CoreXyAndE,
        });
        let mut widen = WidenState::default();

        // Drive segment 1 to completion.
        for i in 0..TICKS_PER_SEG {
            let cyc = i.wrapping_mul(CYC_PER_TICK);
            let result = engine.tick(
                cyc,
                &mut widen,
                &pool,
                &mut consumer,
                &mut trace_producer,
                &shared,
            );
            assert!(
                result.is_ok(),
                "segment 1 tick {i} should not fault: {result:?}"
            );
        }

        // Critical: first tick of segment 2 (Y/Z handles UNUSED).
        // Pre-fix: y eval = 0.0 → motor A delta = (x + 0) - (prev x +
        // 100) ≈ -100 mm → STEP_BURST_EXCEEDED.
        // Post-fix: y = prev_y = 100 → delta tracks only X motion.
        let cyc_at_s2_start = TICKS_PER_SEG.wrapping_mul(CYC_PER_TICK);
        let result = engine.tick(
            cyc_at_s2_start,
            &mut widen,
            &pool,
            &mut consumer,
            &mut trace_producer,
            &shared,
        );
        assert!(
            result.is_ok(),
            "segment 2 first tick must not fault on UNUSED Y/Z handles: {result:?}"
        );
        assert_eq!(
            engine.last_error(),
            0,
            "engine should not have latched any fault"
        );
        assert_eq!(
            engine.status(),
            RuntimeStatus::Running,
            "engine should still be running on segment 2"
        );
    }
}
