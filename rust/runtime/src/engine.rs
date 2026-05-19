//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

// Segment queue is C-backed (see crate::c_segment_queue); the trace ring
// still uses heapless::spsc.
use crate::c_segment_queue::{Consumer as SegConsumer, Producer as SegProducer};
use heapless::spsc::Producer;
// `Consumer` is no longer needed from heapless — the segment queue is the
// only spsc consumer in this crate. (Trace ring uses Producer only.)
#[allow(unused_imports)]
use heapless::spsc::Consumer;

// 2026-05-18 wedge fix: C-side volatile gate flag for
// `engine.producer_current`. Rust's `AtomicBool` was observed to give
// inconsistent reads across the &mut Engine borrow boundary (bench:
// producer_step's load = true while modulated_tick had written false).
// The C global + volatile read/write pattern is the most ironclad way to
// defeat any Rust / LTO compile-time optimization.
#[cfg(target_os = "none")]
#[allow(unsafe_code)]
unsafe extern "C" {
    pub static mut kalico_producer_current_present: u8;
    pub static mut kalico_producer_current_set_count: u32;
    pub static mut kalico_producer_current_cleared_count: u32;
    // 2026-05-18 wedge fix: go through C FFI functions for the gate
    // read/write instead of touching the volatile global directly from
    // Rust. The function call is opaque to LLVM — it can't inline or
    // reorder across it — so the volatile semantics live entirely
    // inside the C translation unit and reach the Rust caller as
    // "definitely-happened" memory ops.
    fn kalico_producer_current_is_present() -> i32;
    fn kalico_producer_current_set_present(present: i32);
}

/// Read the C-side gate (MCU build) or the AtomicBool (host build).
/// Returns true if `engine.producer_current` is Some.
#[inline(never)]
fn read_producer_current_present(shared: &SharedState) -> bool {
    #[cfg(target_os = "none")]
    {
        #[allow(unsafe_code)]
        // SAFETY: pure C function call, no preconditions, no aliasing.
        unsafe { kalico_producer_current_is_present() != 0 }
    }
    #[cfg(not(target_os = "none"))]
    {
        shared.producer_current_present.load(Ordering::Acquire)
    }
}

/// Write the C-side gate (MCU build) and mirror into the AtomicBool (host
/// build). Single source of truth for `engine.producer_current.is_some()`
/// visibility across foreground / ISR boundaries.
#[inline(never)]
fn write_producer_current_present(shared: &SharedState, present: bool) {
    #[cfg(target_os = "none")]
    {
        #[allow(unsafe_code)]
        // SAFETY: pure C function call; the C function performs a volatile
        // store + counter increment. Opaque to LLVM.
        unsafe { kalico_producer_current_set_present(if present { 1 } else { 0 }) };
    }
    shared
        .producer_current_present
        .store(present, Ordering::Release);
}
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

/// Host-only step-pulse observer. Host builds invoke this hook on each
/// `emit_step_pulses` call instead of the FFI extern. Production
/// (`target_os = "none"`) bypasses this entirely — the cfg branches in
/// `emit_step_pulses` keep `runtime_emit_step_pulses` as the only emission
/// path on the MCU.
///
/// Intended for integration tests + offline simulators: install a closure
/// that records each `(motor_idx, n_steps)` call (typically with a wall-clock
/// or simulated timestamp captured by the caller) so test code can diff
/// step traces between Modulated and StepTime modes for the same input.
#[cfg(not(target_os = "none"))]
pub mod step_sink {
    use std::cell::RefCell;

    thread_local! {
        pub(crate) static SINK: RefCell<Option<Box<dyn FnMut(u8, i32)>>> =
            const { RefCell::new(None) };
    }

    /// Install a step-pulse observer for the current thread. Returns the
    /// previously-installed observer (if any), so callers can implement a
    /// scoped/RAII pattern by re-installing the prior sink on drop.
    pub fn install<F: FnMut(u8, i32) + 'static>(
        f: F,
    ) -> Option<Box<dyn FnMut(u8, i32)>> {
        SINK.with(|s| s.borrow_mut().replace(Box::new(f)))
    }

    /// Remove and return the current observer.
    pub fn uninstall() -> Option<Box<dyn FnMut(u8, i32)>> {
        SINK.with(|s| s.borrow_mut().take())
    }
}

/// Emit `n_steps` step pulses on the motor at `motor_idx` (post-kinematic-
/// transform: `[A, B, Z, E]` for CoreXY, `[X, Y, Z, E]` for cartesian).
/// Sign carries direction. On hardware this calls into `src/stepper.c`'s
/// `runtime_emit_step_pulses`, which toggles the step/dir GPIOs configured
/// by the matching `command_config_runtime_stepper`. Host builds invoke the
/// `step_sink` observer if installed (defaults to no-op).
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
        step_sink::SINK.with(|cell| {
            if let Some(sink) = cell.borrow_mut().as_mut() {
                sink(motor_idx, n_steps);
            }
        });
    }
}

/// Issue a TMC5160 XDIRECT SPI write for the phase-stepping output path
/// (Task 6 of the 2026-05-18 phase-stepping plan). On target, this calls
/// into `src/stm32/phase_stepping_spi.c`'s blocking helper (Task 3); on
/// host-test builds, the call is recorded into `test_xdirect_capture` so
/// integration tests can assert on the SPI traffic without a real bus.
#[inline]
#[allow(unsafe_code)]
fn write_xdirect(bus_id: u8, cs_pin: u8, coil_a: i16, coil_b: i16) {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn phase_stepping_write_xdirect(
                bus_id: u8,
                cs_pin: u8,
                coil_a: i16,
                coil_b: i16,
            );
        }
        // SAFETY: stable C ABI symbol provided by src/stm32/phase_stepping_spi.c
        // (Task 3). Four scalar args by value, no aliasing. Bus / CS validity
        // is the C side's responsibility.
        unsafe { phase_stepping_write_xdirect(bus_id, cs_pin, coil_a, coil_b) }
    }
    #[cfg(not(target_os = "none"))]
    {
        crate::test_xdirect_capture::record(bus_id, cs_pin, coil_a, coil_b);
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
    /// Per-motor phase-stepping state. Populated lazily on the first phase
    /// tick when `shared.phase_config[motor_idx]` reports a phase config;
    /// cleared by `runtime_force_idle` so a re-stream after flush re-seeds
    /// from scratch. Sized `MAX_STEPPER_OIDS` so the per-motor walk (which
    /// allows up to 16 phase-stepped motors per MCU; AWD partners + N-per-
    /// slot industrial configs) can index it directly by `motor_idx`.
    phase_modulators:
        [Option<crate::modulator::PhaseDirectModulator>; crate::state::MAX_STEPPER_OIDS],
    /// Monotonic tick counter for phase-stepping round-robin SPI scheduling.
    /// Wraps after 2^32 ticks; mod-by-N indexing handles the wrap.
    phase_tick_counter: u32,
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
            phase_modulators: [const { None }; crate::state::MAX_STEPPER_OIDS],
            phase_tick_counter: 0,
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

    /// 2026-05-18: diagnostic accessor for the wedge investigation.
    /// `producer_current` is a non-atomic `Option<Segment>` mutated by
    /// both `producer_step` (foreground) and `runtime_modulated_tick` (ISR);
    /// reading it through this accessor gives the C-side rotation a way to
    /// snapshot whether the foreground sees the ISR's retire-time clear or
    /// not. Not Ordering-correct for cross-thread sync — just diagnostic.
    pub fn producer_current_is_some_diag(&self) -> bool {
        self.producer_current.is_some()
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

    /// Read-only accessor for a motor's configured `step_distance`
    /// (mm per microstep). Returns `None` for out-of-range indices,
    /// `Some(0.0)` for unconfigured motors, `Some(1/steps_per_mm)` for
    /// configured ones. Used by the C-side `init_step_time_timers` to
    /// skip enabling consumer Klipper timers for unconfigured motors
    /// (the default `StepMode::StepTime` would otherwise trip every
    /// motor's timer even when only a few are actually wired up).
    pub fn producer_step_distance(&self, motor_idx: usize) -> Option<f64> {
        self.producer_states.get(motor_idx).map(|s| s.step_distance())
    }

    /// Seed the engine's logical toolhead position. Used by tests + by
    /// the bridge's `kalico_stream_open` / `SET_KINEMATIC_POSITION` paths
    /// to anchor `prev_x`/`prev_y`/`prev_z` and each motor's
    /// `StepMotorState` accumulator before the first segment runs.
    /// Without this, `runtime_modulated_tick` on a non-origin segment
    /// computes a spurious motor-delta = (segment_start - 0) on its first
    /// tick and emits thousands of catch-up step pulses.
    ///
    /// `xyz` is in trajectory frame (mm), pre-kinematic-transform. The
    /// per-motor accumulator is seeded from `xyz` through the configured
    /// kinematic (CoreXY A=X+Y / B=X−Y, or Cartesian X/Y/Z).
    pub fn seed_position(&mut self, xyz: [f32; 3]) {
        self.prev_x = xyz[0];
        self.prev_y = xyz[1];
        self.prev_z = xyz[2];
        self.needs_xy_seed = false;
        let motors = match self.mcu_config.as_ref().map(|c| c.kinematics) {
            Some(KinematicTag::CoreXyAndE) => {
                corexy_with_e([xyz[0], xyz[1], xyz[2], 0.0])
            }
            _ => cartesian_xyz_with_e([xyz[0], xyz[1], xyz[2], 0.0]),
        };
        for i in 0..4 {
            if let Some(ss) = self.step_state.get_mut(i) {
                if let Some(&m) = motors.get(i) {
                    ss.seed(m);
                }
            }
        }
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
        queue: &mut SegConsumer<Segment>,
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

        // 7b. Drop the per-motor phase-stepping state (Task 6 of the
        //     2026-05-18 phase-stepping plan). The first phase tick after
        //     a re-stream re-seeds the modulator from the freshly-anchored
        //     motor position — same contract as `StepMotorState`'s
        //     `reset_accumulator` above. `phase_tick_counter` resets so
        //     the round-robin SPI schedule restarts at ordinal 0; for a
        //     deterministic re-stream that's the right behaviour.
        for slot in &mut self.phase_modulators {
            *slot = None;
        }
        self.phase_tick_counter = 0;

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
        queue: &mut SegConsumer<Segment>,
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
            // 2026-05-18: queue is C-backed; use len()>0 instead of the
            // heapless-specific `ready()`. Equivalent semantics.
            if queue.is_empty() {
                if self.poll_endstop_trip(now, [0; 3], pool, None, trace, shared) {
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

    /// Retire every in-flight segment's curve-pool slots and transition
    /// the engine to the `Drained` state in response to an endstop trip.
    ///
    /// **`HomingTrip` is not a runtime fault.** The trip is an expected,
    /// host-coordinated abort: the runtime publishes the `EndstopTripped`
    /// event on the events channel, and the host responds by submitting a
    /// back-off segment (homing.py `home_rails` → `toolhead.move`). Latching
    /// `RuntimeStatus::Fault` on trip would block that back-off via the
    /// `FaultLatched` gate at the top of `tick`, forcing a `force_idle`
    /// recovery round-trip the homing protocol does not perform.
    /// Transitioning to `Drained` instead leaves the engine ready to accept
    /// the next dispatched segment without losing the trip in observability:
    /// `last_error` is set to `KALICO_ERR_HOMING_TRIP` so a status query
    /// distinguishes "freshly tripped" from "fresh idle," and a
    /// `TRACE_FLAG_FAULT_MARKER` trace sample marks the trip point for
    /// plot-side diagnostics. `stream_open` is cleared so the next
    /// empty-queue tick observed before the host dispatches the back-off
    /// stays in `Drained` rather than tripping the §8.2 underrun fault.
    ///
    /// `active_segment` is the caller's stack-owned segment, if any —
    /// `tick_with_current` borrows the live segment out of `self.current`
    /// before calling, so without this parameter the handles would be
    /// dropped on the Err-return path. `self.producer_current` covers the
    /// Modulated / per-motor producer pipeline, which holds its own
    /// segment in a persistent field. Together these are the only places
    /// a segment can be in flight at trip time; retiring both keeps the
    /// curve-pool accounting consistent without relying on a downstream
    /// `runtime_force_idle` to recover the slots.
    ///
    /// All `confirm_retired` calls are no-ops on `UNUSED_SENTINEL`
    /// (hold-segment branches) and on `HOLD_SEGMENT_SENTINEL`, so callers
    /// from any trip site can share the same path.
    ///
    /// Shared between the Modulated trip path (`poll_endstop_trip`, driven
    /// off `Engine::tick`'s per-tick endstop sample) and the StepTime trip
    /// path (`abort_for_step_time_trip`, driven off the per-step ISR
    /// `kalico_endstop_tick_step_time`). Both paths need the same
    /// retire-cursor + Drained-transition semantics; the only difference is
    /// the caller's segment ownership.
    pub(crate) fn abort_for_homing_trip(
        &mut self,
        now: u64,
        pool: &CurvePool,
        active_segment: Option<&Segment>,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) {
        let mut retired_max: Option<u32> = None;
        let mut retire = |seg: &Segment| {
            pool.confirm_retired(seg.x_handle);
            pool.confirm_retired(seg.y_handle);
            pool.confirm_retired(seg.z_handle);
            pool.confirm_retired(seg.e_handle);
            retired_max = Some(match retired_max {
                Some(prev) if prev >= seg.id => prev,
                _ => seg.id,
            });
        };
        if let Some(seg) = active_segment {
            retire(seg);
        }
        if let Some(seg) = self.producer_current.take() {
            retire(&seg);
        }
        // Advance the retire cursor + retirement counter that the C-side
        // drain loop (`runtime_tick.c` §10.4) watches to emit
        // `kalico_credit_freed`. Without these stores, the host's
        // CreditCounter never observes the trip-released slots — the
        // host pool deadlocks on the second G28 once an active homing
        // session's retirements add up to capacity.
        if let Some(id) = retired_max {
            shared
                .retired_through_segment_id
                .store(id, Ordering::Release);
            shared
                .producer_segment_retired_total
                .fetch_add(1, Ordering::AcqRel);
            crate::stream::check_terminal_on_retire(shared, id);
        }
        self.clear_current();
        self.last_error.store(
            i32::from(RuntimeError::HomingTrip),
            Ordering::Release,
        );
        self.status
            .store(RuntimeStatus::Drained as u8, Ordering::Release);
        shared.stream_open.store(false, Ordering::Release);
        let _ = trace.enqueue(TraceSample {
            tick: now,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_z: self.last_motors[2],
            motor_e: self.last_motors[3],
            segment_id: 0,
            curve_handle: CurveHandle::UNUSED_SENTINEL,
            flags: TRACE_FLAG_FAULT_MARKER,
            _pad: [0; 7],
        });
    }

    /// Modulated-path trip sampler. Called from `Engine::tick` /
    /// `tick_with_current` — runs the endstop's per-modulation-period
    /// check and, on `AbortNow`, hands off to `abort_for_homing_trip`.
    fn poll_endstop_trip(
        &mut self,
        now: u64,
        v_per_axis_q16: [u32; 3],
        pool: &CurvePool,
        active_segment: Option<&Segment>,
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
        self.abort_for_homing_trip(now, pool, active_segment, trace, shared);
        true
    }

    /// StepTime-path trip evaluator. Called from
    /// `kalico_endstop_tick_step_time` after each per-step GPIO sample —
    /// the StepTime ISR has no current segment on the stack (the engine
    /// owns it via `producer_current`), so `active_segment` is always
    /// `None`. Returns `true` if a trip fired this call.
    ///
    /// Velocity-gated policies (`IgnoreUntilMoving`, `WaitForClear`)
    /// receive `[u32::MAX; 3]` from this entry: the per-step ISR firing
    /// itself signals "in motion," and the MVP has no precise per-axis
    /// velocity hook at step resolution. A precise velocity hook is
    /// Step 8 work.
    pub fn abort_for_step_time_trip(
        &mut self,
        now: u64,
        pool: &CurvePool,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) -> bool {
        let mut stepper_counts = [0_i32; crate::state::MAX_STEPPER_OIDS];
        for (dst, src) in stepper_counts.iter_mut().zip(shared.stepper_counts.iter()) {
            *dst = src.load(Ordering::Acquire);
        }
        if endstop::tick(now, [u32::MAX; 3], &stepper_counts) != TripAction::AbortNow {
            return false;
        }
        self.abort_for_homing_trip(now, pool, None, trace, shared);
        true
    }

    #[allow(clippy::too_many_lines)] // Spec §4.2 step 1-10 explicit pipeline — flatten on purpose.
    fn tick_with_current(
        &mut self,
        mut current: Segment,
        now: u64,
        queue: &mut SegConsumer<Segment>,
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
            if self.poll_endstop_trip(
                now,
                [0; 3],
                pool,
                Some(&current),
                trace,
                shared,
            ) {
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
        if self.poll_endstop_trip(
            now,
            v_per_axis_q16,
            pool,
            Some(&current),
            trace,
            shared,
        ) {
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

        // Build per-motor cubic Bézier CPs. The planner emits uniform
        // cubic Béziers (knots [0,0,0,0,1,1,1,1]) for `arm_step_timer`'s
        // single-piece path; we extract the 4 control points per axis
        // and compose them under the motor's kinematic transform (CoreXY
        // A = X+Y, B = X−Y; Cartesian = single axis). Missing/UNUSED
        // axes default to all-zero CPs so `solve_monotone_cubic_root`
        // reports no root and the caller treats the motor as motionless.
        // f32 storage matches the solver's working precision. Cubic
        // piece CPs come from `cv.control_points: &[f32]`, so no cast.
        let cps_primary: [f32; 4] = cv_primary
            .as_ref()
            .and_then(|cv| cubic_piece(cv, 0).map(|(_, _, cps)| cps))
            .unwrap_or([0.0; 4]);
        let cps_secondary: [f32; 4] = cv_secondary
            .as_ref()
            .and_then(|cv| cubic_piece(cv, 0).map(|(_, _, cps)| cps))
            .unwrap_or([0.0; 4]);
        let cps: [f32; 4] = if is_corexy {
            if corexy_sign > 0.0 {
                [
                    cps_primary[0] + cps_secondary[0],
                    cps_primary[1] + cps_secondary[1],
                    cps_primary[2] + cps_secondary[2],
                    cps_primary[3] + cps_secondary[3],
                ]
            } else {
                [
                    cps_primary[0] - cps_secondary[0],
                    cps_primary[1] - cps_secondary[1],
                    cps_primary[2] - cps_secondary[2],
                    cps_primary[3] - cps_secondary[3],
                ]
            }
        } else {
            cps_primary
        };

        let q = crate::step_time::StepTimeQuery {
            cps,
            step_distance: step_distance_mm as f32,
            current_step,
            t_curr: t_curr_norm as f32,
            t_segment_end: t_end_norm as f32,
        };

        match crate::step_time::compute_next_step_time(&q) {
            crate::step_time::StepTimeResult::NextAt { t: t_norm_next, dir } => {
                // Convert back to absolute cycles in integer space.
                // dt_norm is the fractional step over the segment; promote
                // to f64 for the multiply with duration so the cycle math
                // stays exact even for multi-second segments at MHz clocks.
                let dt_norm = f64::from(t_norm_next) - t_curr_norm;
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
        queue_producer: &mut SegProducer<Segment>,
        _shared: &SharedState,
    ) -> Result<(), Segment> {
        seg.consumers_remaining = Segment::compute_consumers_remaining(
            seg.kinematics,
            seg.x_handle,
            seg.y_handle,
            seg.z_handle,
            seg.e_handle,
        );
        queue_producer.enqueue(seg)?;
        // 2026-05-17: do NOT CAS-set producer_pending here. The C-side FFI
        // wrapper (`handle_push_segment` in src/kalico_dispatch.c) calls
        // `arm_producer_timer_if_kicked` immediately after this returns,
        // which calls `kalico_runtime_kick_producer` and is the single
        // owner of the kick→arm transition. If Engine::push_segment also
        // CASes false→true here, it preempts the C-side: C-side's CAS
        // fails (false→true on an already-true atomic), early-returns
        // without scheduling the producer timer, and a pure-Modulated MCU
        // (F4 with phase-stepped Z, no StepTime motors to drive
        // step_time_event re-kicks) deadlocks at SlotPoolExhausted because
        // producer_step never runs → producer_current never populated →
        // modulated_tick no-ops → segments never retire → no
        // kalico_credit_freed. Live bench repro: klippy.log L1617041 on
        // 2026-05-17; sim repro:
        // tools/sim_klippy/tests/test_bridge_stall_repro.py::test_same_direction_jogs_reproduce_slot_pool_exhaustion.
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
        queue: &mut SegConsumer<Segment>,
        shared: &SharedState,
    ) -> Option<(Segment, CurveHandle, Option<CurveHandle>, f64)> {
        shared
            .producer_fetch_attempts_total
            .fetch_add(1, Ordering::AcqRel);
        // 2026-05-18: gate via the C-side volatile flag (see helper docs).
        if !read_producer_current_present(shared) {
            self.producer_current = queue.dequeue();
            if let Some(seg) = self.producer_current {
                write_producer_current_present(shared, true);
                shared
                    .producer_segment_dequeued_total
                    .fetch_add(1, Ordering::AcqRel);
                // Publish the active segment id for host-side status frames
                // and retire-cursor consumers. `Engine::tick` does the same
                // store at the modulated-tick equivalent dequeue point
                // (line ~759); StepTime motors don't run tick(), so without
                // this store `shared.current_segment_id` would stay at 0
                // forever and the host's `wait_for_segment_id` polling
                // (kalico_status_v6 `current_segment_id` field) would never
                // observe the engine advancing through queued segments.
                shared
                    .current_segment_id
                    .store(seg.id, Ordering::Release);
            }
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

        // If this motor has already cleared every consumer bit it owns in
        // `seg.consumers_remaining` (motor previously returned
        // `SegmentExhausted` for this segment and `clear_motor_bits_in_mask`
        // ran), do not return the segment for it again. Without this guard the
        // motor's `ProducerState::is_idle()` -> `start_curve` -> Cardano ->
        // SegmentExhausted -> `motor_finished_curve=true` cycle re-fires on
        // every `producer_step` call, manufacturing fake `made_progress=true`
        // and pegging `runtime_producer_event` at the
        // `SF_RESCHEDULE_FLOOR=100 µs` cadence indefinitely. Reproduces as IWDG
        // reset on any G1 jog that has constant-curve axes (Y and E on an
        // X-only jog with Cartesian kinematics, all four motors in lockstep
        // on segments after motor 0 also finishes its real work).
        if !seg.motor_has_remaining_work(motor_idx as u8) {
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
    // 2026-05-18: #[inline(never)] to prevent LTO from inlining producer_step
    // into the Klipper timer callback. The volatile gate read at the top of
    // this function was being optimized to a stale value, suggesting the
    // compiler was caching across loop iterations of the scheduler.
    #[inline(never)]
    pub fn producer_step(
        &mut self,
        pool: &CurvePool,
        queue: &mut SegConsumer<Segment>,
        shared: &SharedState,
    ) -> ProducerTickResult {
        // 2026-05-18 wedge fix: prevent the compiler from caching
        // `self.producer_current` across calls. Bench evidence (tag 0xCC,
        // commit 3573b30f8): producer_step's view of
        // `producer_current.is_some()` returned 1 while status_drain's view
        // (read via a different `&IsrState` borrow path) returned 0 for the
        // same field at the same moment. The non-atomic `Option<Segment>`
        // field is being optimised across function boundaries despite
        // modulated_tick writing `None` from the ISR borrow path.
        //
        // SeqCst fence at the head of producer_step forces all prior memory
        // operations to globally serialise — the compiler can't reorder a
        // read of `self.producer_current` to before this fence, and any
        // write that happened-before this fence (including modulated_tick's
        // `None` write that ran before producer_step was scheduled) is
        // visible to subsequent reads.
        core::sync::atomic::fence(Ordering::SeqCst);
        // (1) Clear kick-pending flag at start; (2) heartbeat.
        shared.producer_pending.store(false, Ordering::Release);
        shared.producer_runs_total.fetch_add(1, Ordering::AcqRel);

        // 2026-05-17: foreground-owned dequeue for pure-Modulated configs.
        // The per-motor loop below `continue`s on every non-StepTime motor,
        // so for an MCU with only Modulated motors (F446 with phase-stepped
        // Z and no StepTime stepper) `fetch_segment_for_motor` is never
        // called, `producer_current` stays `None`, and `runtime_modulated_tick`
        // returns immediately without advancing the queue. Segments queue
        // forever, retired_through_segment_id never advances, the host's
        // `kalico_credit_freed` accounting deadlocks at SlotPoolExhausted
        // (live-bench repro: klippy.log L1617041, 2026-05-17).
        //
        // The earlier lazy-dequeue in `runtime_modulated_tick` (introduced
        // 081ab4a3b, reverted bb88c5d8) raced this same Consumer from the
        // TIM5 ISR vs. the foreground. Restoring the dequeue here keeps it
        // foreground-only (single-consumer-site invariant) while still
        // advancing Modulated-only segment streams: producer_step is the
        // single foreground task driving the segment cursor.
        //
        // Idempotent with the existing lazy dequeue inside
        // `fetch_segment_for_motor`: when producer_current is already set,
        // this is a no-op; when not set, this dequeue runs first and
        // fetch_segment_for_motor's `is_none()` check sees the populated
        // state.
        // 2026-05-18 wedge fix: gate via the volatile C-side flag (helper).
        let cur_is_some_view = read_producer_current_present(shared);
        shared
            .producer_step_current_is_some_snapshot
            .store(u8::from(cur_is_some_view), Ordering::Release);
        // 2026-05-18 wedge diag: snapshot queue.len() on EVERY producer_step
        // call (not just inside the !is_some branch) so we can compare
        // producer_step's view of the queue with status_drain's view
        // even when producer_step is "done" with the current segment.
        let qlen_here = queue.len() as u32;
        shared
            .producer_step_last_len_snapshot
            .store(qlen_here, Ordering::Release);
        if !cur_is_some_view {
            shared
                .producer_observed_none_total
                .fetch_add(1, Ordering::AcqRel);
            if let Some(seg) = queue.dequeue() {
                self.producer_current = Some(seg);
                // Set the volatile gate so the next producer_step call
                // sees the dequeue.
                write_producer_current_present(shared, true);
                shared
                    .producer_segment_dequeued_total
                    .fetch_add(1, Ordering::AcqRel);
                shared
                    .current_segment_id
                    .store(seg.id, Ordering::Release);
            }
        }

        // Contract: `WorkPending` iff this call pushed step times to a
        // ring. `AllIdle` otherwise — including state-only changes like
        // retiring an empty segment. Violating this with "WorkPending on
        // state change" causes the C-side producer timer to self-
        // reschedule at SF_RESCHEDULE_FLOOR (100 µs) for what is in
        // effect a no-op, and over many segments saturates Klipper's
        // timer-dispatch loop until one timer drifts >1 ms behind
        // `timer_read_time()` → `try_shutdown("Rescheduled timer in the
        // past")` at `src/generic/armcm_timer.c:152`.
        //   - WorkPending = "I pushed ring entries; expect more work."
        //   - AllIdle     = "I'm blocked or done; wait for an external
        //                    kick (push_segment or consumer low-water)."
        //
        // Per-motor fill budget is per-call (NOT per-segment). On
        // segment boundaries we retire the current segment AND continue
        // filling from the next queued segment within the same call —
        // see the `'segment_loop` below. Without this, every
        // segment-to-segment transition cost a zero-fill `WorkPending`
        // call, and multi-segment jogs (≥ 2.5 mm after shaper
        // convolution) accumulated enough no-op reschedules to trip the
        // "Rescheduled timer in the past" shutdown.
        let mut motor_filled_this_call: [u32; 4] = [0; 4];
        let mut any_filled = false;

        // Single-pass per-motor fill. Spec §3.4 budget: ~30 cycles per
        // Newton solve × PRODUCER_BATCH_CAP=32 entries × 4 motors ≈
        // 3.8k cycles ≈ 7.4 µs at 520 MHz (H7). The per-call budget
        // (`motor_filled_this_call`) preserves this bound across the
        // segment-loop iterations: a motor that fills 32 entries in
        // segment N has budget 0 for segments N+1, N+2 in the same
        // call — capping aggregate Newton work at the original
        // single-segment number. Earlier revisions ran up to 8 outer
        // iterations *without* the per-call cap, which inflated
        // worst-case ISR duration to ~59 µs (1024 step times per call)
        // and starved other ISRs. Spec §3.4 + §3.8.
        'segment_loop: loop {
            let mut iter_any_filled = false;
            let mut iter_any_progress = false;

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

                // Per-call budget cap. If this motor has already filled
                // its share via earlier segments in this call, skip it
                // until the next call. Bounds aggregate Newton work
                // even when we span multiple segments per call.
                if motor_filled_this_call[motor_idx] >= PRODUCER_BATCH_CAP {
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
                    if self.fetch_segment_for_motor(motor_idx, queue, shared).is_none() {
                        continue;
                    }
                }

                // Fill ring while space permits, capped at PRODUCER_BATCH_CAP.
                // The motor's "current curve" is whatever resolves out of
                // producer_current under the segment's kinematics —
                // re-fetch here to pick up freshly-started curves.
                let Some((seg_for_fill, primary, secondary, sign)) =
                    self.fetch_segment_for_motor(motor_idx, queue, shared)
                else {
                    continue;
                };
                let cv_primary = if primary.is_unused_sentinel() {
                    shared
                        .producer_primary_unused_total
                        .fetch_add(1, Ordering::AcqRel);
                    None
                } else {
                    let r = pool.resolve(primary);
                    if r.is_some() {
                        shared
                            .producer_primary_resolved_total
                            .fetch_add(1, Ordering::AcqRel);
                    } else {
                        shared
                            .producer_primary_stale_total
                            .fetch_add(1, Ordering::AcqRel);
                    }
                    // 2026-05-15 live diagnosis (CP capture). Only sample
                    // motor 0 so motor 1's CPs don't overwrite motor 0's
                    // between status emits. Captures the very-first
                    // control point (cps[0], curve value at u=0) and the
                    // very-last (cps[3], curve value at u=1) of piece 0
                    // of the resolved primary X curve. For a 0.5mm pure-X
                    // jog from X=125, expect cps_0 = 125.0
                    // (0x42FA0000) and cps_3 ≈ 125.5 (0x42FB0000).
                    // Constant-on-constant or identical values indicate
                    // wire-level curve corruption — the host runtime
                    // bench reproduces the live inputs successfully, so
                    // any divergence here pins the bug to the wire.
                    if motor_idx == 0 {
                        if let Some(ref cv) = r {
                            if let Some((_, _, cps0)) = cubic_piece(cv, 0) {
                                shared.last_resolved_primary_cps_0.store(
                                    cps0[0].to_bits(),
                                    Ordering::Release,
                                );
                                shared.last_resolved_primary_cps_3.store(
                                    cps0[3].to_bits(),
                                    Ordering::Release,
                                );
                            }
                        }
                    }
                    r
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
                    iter_any_progress = true;
                    continue;
                }
                let duration_f64 = duration_cycles as f64;

                // Multi-piece cubic Bézier walker. Planner emits piecewise-
                // Bézier degree-3 NURBS (see `trajectory/src/refit.rs::refit_to_cubic`
                // and `nurbs/src/bezier.rs::bezier_pieces_to_nurbs`): each
                // interior knot has multiplicity 3, control points span
                // `3N + 1` for `N` pieces. A 1 mm X jog typically arrives as
                // 2–10 pieces depending on the shaper kernel.
                //
                // The pre-2026-05-14 producer assumed a single 4-CP cubic
                // (extract_uniform_cubic_bezier_coeffs, since deleted) and
                // fell back to zero-coeffs for anything else, which made
                // every motor immediately exhaust on the first
                // `compute_next_step_time` call — zero steps emitted, host
                // slot pool drained segment-after-segment, no toolhead motion.
                //
                // Now: walk pieces sequentially per `compute_next_step_time`
                // call, mapping local `t ∈ [0,1]` of each piece back to
                // global `u ∈ [u_start_piece, u_end_piece]` of the segment.
                let n_pieces_primary = cv_primary.as_ref().map_or(0, cubic_n_pieces);
                let n_pieces_secondary = cv_secondary.as_ref().map_or(0, cubic_n_pieces);

                // UNUSED handle (no primary curve) → motor has no work
                // this segment. Clear its mask bit so retirement can fire,
                // mark progress, move on.
                if n_pieces_primary == 0 {
                    if let Some(seg_mut) = self.producer_current.as_mut() {
                        Self::clear_motor_bits_in_mask(seg_mut, motor_idx as u8);
                    }
                    if let Some(ps) = self.producer_states.get_mut(motor_idx) {
                        ps.clear();
                    }
                    if let Some(slot) = self.motor_current_segment_id.get_mut(motor_idx) {
                        *slot = None;
                    }
                    if let Some(cur) = self.motor_curve_cursor.get_mut(motor_idx) {
                        *cur = cur.wrapping_add(1);
                    }
                    iter_any_progress = true;
                    continue;
                }

                // CoreXY combines x + y / x − y per motor. If the
                // secondary curve is UNUSED (Y missing while X is real),
                // treat it as a zero-curve — motor 0 = X + 0 = X, motor 1
                // = X − 0 = X.
                //
                // **Piece-count handling (2026-05-15 fix).** If both
                // primary and secondary are present, piece counts may
                // differ when one axis is a single-piece constant curve
                // (host sends it to anchor `prev_value` per the 2026-05-11
                // dispatch fix) while the other is multi-piece shaped
                // (the shaper produced many pieces for the moving axis).
                // In that case, iterate by the SHAPED side's pieces and
                // treat the constant side as a per-piece constant offset.
                // If both are shaped with mismatched piece counts (not
                // reachable from any current host path, since the host
                // refits both axes together), skip the motor — the runtime
                // does not perform piece-merge.
                let secondary_is_zero = n_pieces_secondary == 0;
                let cv_primary_ref = cv_primary.as_ref();
                let cv_secondary_ref = cv_secondary.as_ref();
                let primary_constant_value =
                    cv_primary_ref.and_then(curve_constant_value);
                let secondary_constant_value =
                    cv_secondary_ref.and_then(curve_constant_value);

                // Both-shaped-mismatched detection (CoreXY only). When both
                // axis curves are shaped but their piece counts differ
                // (e.g., smooth_mzv with X@186 Hz and Y@122 Hz produces
                // different kernel-driven piece subdivisions), the piece
                // walker can't just pick one as driver — it must visit
                // the SORTED UNION of both curves' piece boundaries and
                // slice each curve's piece to the merged sub-range before
                // combining (see `cubic_subsegment` for the De Casteljau
                // slicer). Prior to this fix the runtime defensively
                // skipped the motor in this case, the result was zero
                // step pulses on every CoreXY jog where the per-axis
                // shaper frequencies differed — i.e., the entire bench
                // configuration, since X and Y use different mechanical
                // resonance frequencies.
                let both_shaped_mismatched = is_corexy
                    && !secondary_is_zero
                    && n_pieces_primary != n_pieces_secondary
                    && primary_constant_value.is_none()
                    && secondary_constant_value.is_none();

                // Build the merged-piece breakpoint list ONLY for the
                // both-shaped-mismatched case. Other paths use the existing
                // single-driver logic without this allocation. The merged
                // breakpoints are the SORTED UNION of primary's and
                // secondary's piece boundaries.
                //
                // Stack-allocated heapless::Vec — MAX_MERGED_BREAKPOINTS sized
                // to comfortably cover the bench's worst-case shaper output
                // (~30 pieces per axis on a 50mm jog → ~60 merged breakpoints).
                const MAX_MERGED_BREAKPOINTS: usize = 256;
                let mut merged_u: heapless::Vec<f32, MAX_MERGED_BREAKPOINTS> =
                    heapless::Vec::new();
                if both_shaped_mismatched {
                    // Both checked is_some via curve_constant_value.is_none()
                    // guards above (a None curve_constant_value implies the
                    // curve resolved to Some(CurveView)).
                    if let (Some(prim), Some(sec)) = (cv_primary_ref, cv_secondary_ref) {
                        // Append primary's piece-boundary u-values: each
                        // piece contributes its u_lo; the last piece also
                        // contributes u_hi.
                        for pi in 0..n_pieces_primary {
                            if let Some((u_lo, u_hi, _)) = cubic_piece(prim, pi) {
                                let _ = merged_u.push(u_lo);
                                if pi + 1 == n_pieces_primary {
                                    let _ = merged_u.push(u_hi);
                                }
                            }
                        }
                        // Append secondary's piece-boundary u-values.
                        for pi in 0..n_pieces_secondary {
                            if let Some((u_lo, u_hi, _)) = cubic_piece(sec, pi) {
                                let _ = merged_u.push(u_lo);
                                if pi + 1 == n_pieces_secondary {
                                    let _ = merged_u.push(u_hi);
                                }
                            }
                        }
                    }
                    // Sort. partial_cmp is safe because all u-values are
                    // finite reals in [0, 1]. Use the slice's
                    // sort_unstable_by (works in no_std, no alloc).
                    merged_u.as_mut_slice().sort_unstable_by(|a, b| {
                        a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
                    });
                    // In-place dedup with epsilon: keep merged piece widths
                    // above 1e-7 so we don't generate zero-width pieces from
                    // coincident boundaries (start-clamped knot multiplicity
                    // means u=0 appears in both curves' breakpoint sets).
                    let mut write_idx = 0_usize;
                    let read_len = merged_u.len();
                    for read_idx in 0..read_len {
                        let v_opt = merged_u.get(read_idx).copied();
                        let prev_opt = if write_idx == 0 {
                            None
                        } else {
                            merged_u.get(write_idx - 1).copied()
                        };
                        if let Some(v) = v_opt {
                            let keep = match prev_opt {
                                None => true,
                                Some(p) => (v - p).abs() > 1e-7,
                            };
                            if keep {
                                if let Some(slot) = merged_u.get_mut(write_idx) {
                                    *slot = v;
                                }
                                write_idx += 1;
                            }
                        }
                    }
                    merged_u.truncate(write_idx);
                }

                // Determine which curve drives iteration (defines u-spans).
                //   driver_is_primary=true  → iterate primary's pieces
                //   driver_is_primary=false → iterate secondary's pieces
                //                              (used when primary is constant
                //                              and secondary is shaped).
                //   both_shaped_mismatched   → iterate merged_u sub-pieces
                let (driver_is_primary, iter_n_pieces) = if both_shaped_mismatched {
                    (true, merged_u.len().saturating_sub(1))
                } else if is_corexy
                    && !secondary_is_zero
                    && n_pieces_primary != n_pieces_secondary
                {
                    match (primary_constant_value, secondary_constant_value) {
                        (None, Some(_)) => (true, n_pieces_primary),
                        (Some(_), None) => (false, n_pieces_secondary),
                        _ => {
                            // Defensive: both_shaped_mismatched should have
                            // covered the (None, None) case; (Some, Some) with
                            // mismatched piece counts is impossible because
                            // curve_constant_value requires n_pieces == 1.
                            // If we ever land here, skip the motor cleanly.
                            if let Some(seg_mut) = self.producer_current.as_mut() {
                                Self::clear_motor_bits_in_mask(seg_mut, motor_idx as u8);
                            }
                            if let Some(ps) = self.producer_states.get_mut(motor_idx) {
                                ps.clear();
                            }
                            iter_any_progress = true;
                            continue;
                        }
                    }
                } else {
                    (true, n_pieces_primary)
                };

                // Build the kinematic-transformed coeffs for piece `pi`.
                // Cartesian: just the primary's piece-coeffs. CoreXY: add/sub
                // primary and secondary pieces, or pass through primary alone
                // when secondary is UNUSED, or combine with a constant
                // follower when one side is a single-piece constant curve.
                // Both-shaped-mismatched (CoreXY): for each merged sub-piece
                // (union of breakpoints), De Casteljau-slice each curve to
                // the sub-range, then combine.
                let piece_coeffs = |pi: usize| -> Option<(f32, f32, [f32; 4])> {
                    if both_shaped_mismatched {
                        let u_lo = *merged_u.get(pi)?;
                        let u_hi = *merged_u.get(pi + 1)?;
                        if !(u_hi > u_lo) {
                            return None;
                        }
                        let prim = cv_primary_ref?;
                        let sec = cv_secondary_ref?;
                        // Find each curve's piece containing `u_lo` (the
                        // merged sub-piece sits entirely within one
                        // primary piece AND one secondary piece by
                        // construction — boundaries are the union).
                        // Linear scan is fine: piece counts are small (~30).
                        let mut p_piece: Option<(f32, f32, [f32; 4])> = None;
                        for ppi in 0..n_pieces_primary {
                            if let Some((pul, puh, pc)) = cubic_piece(prim, ppi) {
                                if pul <= u_lo + 1e-9 && u_hi <= puh + 1e-9 {
                                    p_piece = Some((pul, puh, pc));
                                    break;
                                }
                            }
                        }
                        let mut s_piece: Option<(f32, f32, [f32; 4])> = None;
                        for spi in 0..n_pieces_secondary {
                            if let Some((sul, suh, sc)) = cubic_piece(sec, spi) {
                                if sul <= u_lo + 1e-9 && u_hi <= suh + 1e-9 {
                                    s_piece = Some((sul, suh, sc));
                                    break;
                                }
                            }
                        }
                        let (p_ul, p_uh, p_cps) = p_piece?;
                        let (s_ul, s_uh, s_cps) = s_piece?;
                        // Slice each curve's piece to [u_lo, u_hi] sub-range.
                        let p_span = p_uh - p_ul;
                        let s_span = s_uh - s_ul;
                        if p_span <= 0.0 || s_span <= 0.0 {
                            return None;
                        }
                        let p_local_s = ((u_lo - p_ul) / p_span).clamp(0.0, 1.0);
                        let p_local_t = ((u_hi - p_ul) / p_span).clamp(0.0, 1.0);
                        let s_local_s = ((u_lo - s_ul) / s_span).clamp(0.0, 1.0);
                        let s_local_t = ((u_hi - s_ul) / s_span).clamp(0.0, 1.0);
                        let p_sub = cubic_subsegment(p_cps, p_local_s, p_local_t);
                        let s_sub = cubic_subsegment(s_cps, s_local_s, s_local_t);
                        // CoreXY combine: motor 0 = primary + secondary,
                        // motor 1 = primary - secondary (sign captured at
                        // fetch_segment_for_motor).
                        let combined = if sign > 0.0 {
                            [
                                p_sub[0] + s_sub[0],
                                p_sub[1] + s_sub[1],
                                p_sub[2] + s_sub[2],
                                p_sub[3] + s_sub[3],
                            ]
                        } else {
                            [
                                p_sub[0] - s_sub[0],
                                p_sub[1] - s_sub[1],
                                p_sub[2] - s_sub[2],
                                p_sub[3] - s_sub[3],
                            ]
                        };
                        return Some((u_lo, u_hi, combined));
                    }
                    if is_corexy && !secondary_is_zero {
                        // Read driver piece (u-span + CPs) and synthesize follower CPs
                        let (u_lo, u_hi, cps_driver) = if driver_is_primary {
                            cubic_piece(cv_primary_ref?, pi)?
                        } else {
                            cubic_piece(cv_secondary_ref?, pi)?
                        };
                        let cps_follower: [f32; 4] = if driver_is_primary {
                            match secondary_constant_value {
                                Some(c) => [c, c, c, c],
                                None => {
                                    let (_, _, cps_s) = cubic_piece(cv_secondary_ref?, pi)?;
                                    cps_s
                                }
                            }
                        } else {
                            // Follower is primary, which is necessarily constant
                            // (we only set driver_is_primary=false in that case).
                            let c = primary_constant_value?;
                            [c, c, c, c]
                        };
                        // Map driver/follower back to (primary, secondary) so
                        // the kinematic sign convention `primary ± secondary`
                        // stays explicit.
                        let (cps_p, cps_s) = if driver_is_primary {
                            (cps_driver, cps_follower)
                        } else {
                            (cps_follower, cps_driver)
                        };
                        let combined = if sign > 0.0 {
                            [
                                cps_p[0] + cps_s[0],
                                cps_p[1] + cps_s[1],
                                cps_p[2] + cps_s[2],
                                cps_p[3] + cps_s[3],
                            ]
                        } else {
                            [
                                cps_p[0] - cps_s[0],
                                cps_p[1] - cps_s[1],
                                cps_p[2] - cps_s[2],
                                cps_p[3] - cps_s[3],
                            ]
                        };
                        Some((u_lo, u_hi, combined))
                    } else {
                        let (u_lo, u_hi, cps_p) = cubic_piece(cv_primary_ref?, pi)?;
                        Some((u_lo, u_hi, cps_p))
                    }
                };

                // Seed producer state on first call: anchor `initial_step`
                // to the curve's position at u=0 in motor-frame mm. Same
                // rationale as the pre-piecewise path — `stepper_counts`
                // is a counter, not an absolute position, so we cannot
                // seed from it. Piece 0 evaluated at local t=0 equals
                // `curve(u_start_of_segment)`, which IS the absolute
                // motor-frame position klippy planned.
                if is_idle {
                    // Determine direction from piece 0's cps end-to-end so we
                    // can seed `initial_step` consistently with the
                    // compute_next_step_time contract `target = (cs + dir) * sd`.
                    //
                    // The formula assumes `cs` is the integer step the motor is
                    // physically at — i.e., `cs * sd == motor_position`. When
                    // `pos0` falls strictly between two step boundaries, the
                    // motor's discrete physical position depends on which side
                    // it was last commanded to. For a fresh-curve seed we don't
                    // have that history, so we seed direction-aware:
                    //   * Motion going +: physical motor at floor(pos0/sd). Next
                    //     +step boundary is (floor + 1) * sd = (cs + dir) * sd. ✓
                    //   * Motion going -: physical motor at ceil(pos0/sd). Next
                    //     -step boundary is (ceil - 1) * sd = (cs + dir) * sd. ✓
                    //
                    // Truncation alone (the prior behaviour) is `floor` for
                    // positive pos0, which is right for + motion but off-by-one
                    // for - motion: target = (floor - 1) * sd lands one step
                    // BELOW the actual next -step boundary, falling outside the
                    // first piece's value range and returning SegmentExhausted
                    // on the very first call. That's the cause of the
                    // "energizes-but-no-motion" bug on every negative-direction
                    // jog from a non-step-aligned starting position.
                    let (pos0, dir): (f32, i8) = match piece_coeffs(0) {
                        Some((_, _, cps0)) => {
                            let p0 = cps0[0];
                            let p3 = cps0[3];
                            // 2026-05-15 live diagnosis (CP capture). For
                            // motor 0, snapshot the COMBINED cps[0] and
                            // cps[3] returned by piece_coeffs — this is
                            // what compute_next_step_time actually sees as
                            // its boundary values. Compare with the raw
                            // primary captured above: the difference
                            // reflects how kine.combine mixes X+Y (CoreXY).
                            // If raw primary cps_0 and cps_3 differ but
                            // combined cps_0 and cps_3 are equal, the
                            // kinematic combination is cancelling the
                            // displacement (e.g. an X jog with Y curve
                            // accidentally containing the opposite sign).
                            if motor_idx == 0 {
                                shared.last_combined_motor_a_cps_0.store(
                                    p0.to_bits(),
                                    Ordering::Release,
                                );
                                shared.last_combined_motor_a_cps_3.store(
                                    p3.to_bits(),
                                    Ordering::Release,
                                );
                            }
                            let d: i8 = if p3 > p0 { 1 } else if p3 < p0 { -1 } else { 0 };
                            (p0, d)
                        }
                        None => (0.0_f32, 0),
                    };
                    let initial_step = if step_distance > 0.0 {
                        let raw = f64::from(pos0) / step_distance;
                        if dir < 0 {
                            // Ceiling for negative motion. `as i32` truncates
                            // toward zero, so for negative pos0 ceil is also
                            // toward zero; libm::ceil handles both signs.
                            libm::ceil(raw) as i32
                        } else {
                            // Floor for positive / zero motion. `as i32`
                            // truncates toward zero, which is floor for
                            // non-negative values; for negative pos0 with +
                            // motion the truncation differs from floor — but
                            // a positive curve starting at negative pos0 means
                            // the toolhead is in negative-X territory which is
                            // outside the printer's working range, so the
                            // truncation behaviour matches floor here in
                            // practice.
                            raw as i32
                        }
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

                // Inline bezier_root-per-piece step-fill. For each step
                // pulse:
                //   1. Find the piece containing `t_resume` (or the first
                //      piece if `t_resume == 0`).
                //   2. Solve `solve_monotone_cubic_root` in local
                //      `t ∈ [t_low_local, 1]` for that piece.
                //   3. If NextAt: map t back to global u, push step, advance.
                //   4. If SegmentExhausted on this piece: advance to next
                //      piece and retry.
                //   5. If exhausted all pieces past current: real
                //      SegmentExhausted, break.
                let starting_filled = motor_filled_this_call[motor_idx];
                let mut filled = starting_filled;
                let ring = match self.step_rings.get_mut(motor_idx) {
                    Some(r) => r,
                    None => continue,
                };
                let ps = match self.producer_states.get_mut(motor_idx) {
                    Some(p) => p,
                    None => continue,
                };
                let mut motor_finished_curve = false;
                // f32 in u-domain throughout — matches the solver's
                // working precision and the curve pool's storage type.
                // `ps.t_resume()` returns f64 (preserves the existing
                // per-curve cursor storage); we down-cast at entry. f32
                // ulp at u=1 is ~1.2e-7, well below any meaningful step
                // boundary resolution.
                let step_distance_f32 = ps.step_distance() as f32;
                while filled < PRODUCER_BATCH_CAP && ring.space() > 0 {
                    let t_curr = ps.t_resume().unwrap_or(0.0) as f32;
                    let mut found_root: Option<(f32, i8)> = None;
                    for pi in 0..iter_n_pieces {
                        let Some((u_lo, u_hi, cps)) = piece_coeffs(pi) else {
                            continue;
                        };
                        if u_hi <= t_curr {
                            continue; // already past this piece
                        }
                        let span = u_hi - u_lo;
                        // Local t-bounds: clamp current u into [0, 1] of
                        // this piece. `solve_monotone_cubic_root` uses
                        // `(t_low, t_high]` (exclusive low, inclusive high)
                        // so a `t_curr` exactly at a piece boundary picks
                        // up roots in the next piece.
                        let t_low_local =
                            if t_curr > u_lo { (t_curr - u_lo) / span } else { 0.0_f32 };
                        let q = StepTimeQuery {
                            cps,
                            step_distance: step_distance_f32,
                            current_step: ps
                                .step_at_curve_start()
                                .wrapping_add(ps.steps_pushed_this_curve()),
                            t_curr: t_low_local,
                            t_segment_end: 1.0_f32,
                        };
                        match compute_next_step_time(&q) {
                            StepTimeResult::NextAt { t, dir } => {
                                let u_global = u_lo + t * span;
                                found_root = Some((u_global, dir));
                                break;
                            }
                            StepTimeResult::SegmentExhausted => continue,
                        }
                    }
                    match found_root {
                        Some((u_global, dir)) => {
                            // u_global is f32 in [0, 1]; promote to f64
                            // before multiplying by duration_f64 so the
                            // cycle math stays exact for multi-second
                            // segments at MHz clocks.
                            let dt_cycles = (f64::from(u_global) * duration_f64) as u64;
                            let abs_cycles = t_start_cycles.saturating_add(dt_cycles);
                            ring.push(abs_cycles as u32, dir);
                            shared
                                .producer_steps_pushed_total
                                .fetch_add(1, Ordering::AcqRel);
                            ps.set_t_resume(Some(f64::from(u_global)));
                            ps.bump_steps_pushed(i32::from(dir));
                            filled += 1;
                        }
                        None => {
                            ps.clear();
                            shared
                                .producer_motor_finished_curve_total
                                .fetch_add(1, Ordering::AcqRel);
                            motor_finished_curve = true;
                            break;
                        }
                    }
                }

                // Commit this motor's per-call fill total.
                motor_filled_this_call[motor_idx] = filled;

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
                    iter_any_progress = true;
                }

                // Pushing ring entries is the only thing that earns
                // `WorkPending` (see `any_filled` → return value below).
                // Finishing a curve is structural progress that earns
                // another segment-loop iteration but NOT a WorkPending
                // reschedule by itself — that's the contract fix.
                if filled > starting_filled {
                    iter_any_filled = true;
                    any_filled = true;
                    iter_any_progress = true;
                }
            }

            // (5) Retire the producer-current segment if every consumer bit
            // is clear. The Modulated path (TIM5) writes its own bits in
            // future Task 10; today (StepTime-only) bits clear exclusively
            // through the loop above.
            let mut retired_this_iter = false;
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
                    shared
                        .producer_segment_retired_total
                        .fetch_add(1, Ordering::AcqRel);
                    crate::stream::check_terminal_on_retire(shared, seg.id);
                    self.producer_current = None;
                    write_producer_current_present(shared, false);
                    retired_this_iter = true;
                    iter_any_progress = true;
                }
            }

            // Segment-loop continuation. Three break conditions:
            //   1. Nothing changed this iteration (no fills, no bits
            //      cleared, no retire) — we're blocked (ring full or
            //      no segments to process). Return whatever any_filled
            //      tells us.
            //   2. The current segment is still in flight (didn't
            //      retire). Yield so the consumer can drain — kicks
            //      will re-arm us when work is available.
            //   3. All motors have hit the per-call PRODUCER_BATCH_CAP.
            //      No more Newton work this call regardless of how many
            //      segments wait in the queue.
            // Otherwise: producer_current is None and we have budget —
            // loop back so the per-motor pass fetches the next segment.
            if !iter_any_filled && !iter_any_progress {
                break 'segment_loop;
            }
            if self.producer_current.is_some() {
                break 'segment_loop;
            }
            let all_motors_at_cap = (0..4_usize)
                .all(|i| motor_filled_this_call[i] >= PRODUCER_BATCH_CAP);
            if all_motors_at_cap {
                break 'segment_loop;
            }
            // Silence unused-variable lint when the loop continues.
            let _ = retired_this_iter;
        }

        // Contract: WorkPending iff at least one ring entry was pushed
        // this call. Retiring/state-change without a fill is NOT enough
        // to earn a 100-µs reschedule (see the design comment at the
        // top of this function).
        if any_filled {
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
    pub fn runtime_modulated_tick(
        &mut self,
        now: u64,
        queue: &mut SegConsumer<Segment>,
        pool: &CurvePool,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        shared: &SharedState,
    ) {
        // Pull the wall-clock segment. The producer's segment cursor is the
        // shared cursor under the MVP lockstep regime — under the
        // co-resident StepTime+Modulated configuration, `producer_step`
        // populates `producer_current` via `fetch_segment_for_motor`.
        //
        // 2026-05-18: lazy-dequeue from ISR. The C-side segment queue
        // (src/kalico_segment_queue.c) uses proper _Atomic uint head/tail
        // with Acquire/Release memory ordering, so SPSC concurrent
        // access from foreground (push_segment) and ISR (this function)
        // is sound. This replaces the foreground-only dequeue pattern
        // that was added 2026-05-17 to fix the heapless::spsc race —
        // but that fix introduced a different wedge where the
        // non-atomic `producer_current` field was being miscompiled
        // across the &mut Engine borrow boundary (bench 2026-05-18:
        // producer_step's view of producer_current.is_some() persistently
        // disagreed with status_drain's view despite atomic gate +
        // volatile reads + C FFI accessors). With ISR-side dequeue, the
        // ISR is the single producer AND consumer of producer_current,
        // so the cross-function visibility issue can't exist.
        if self.producer_current.is_none() {
            if let Some(seg) = queue.dequeue() {
                self.producer_current = Some(seg);
                write_producer_current_present(shared, true);
                shared
                    .producer_segment_dequeued_total
                    .fetch_add(1, Ordering::AcqRel);
                shared
                    .current_segment_id
                    .store(seg.id, Ordering::Release);
            }
        }
        let Some(mut seg) = self.producer_current else {
            return;
        };


        let elapsed = now.saturating_sub(seg.t_start);
        let duration = seg.duration().max(1);

        // 2026-05-17 F4 retire-stall diagnostic: publish elapsed and
        // duration to SharedState so the host's fault_detail rotation can
        // expose them. Lets bench debugging confirm whether the engine's
        // `now` is reaching `seg.t_start + duration` (retirement should
        // fire) or staying behind (clock-skew bug).
        shared
            .last_modulated_elapsed_lo
            .store(elapsed as u32, Ordering::Release);
        shared
            .last_modulated_duration_lo
            .store(duration as u32, Ordering::Release);

        if elapsed >= duration {
            // 2026-05-17 diag: record that we reached the retirement branch.
            shared
                .modulated_retire_attempts
                .fetch_add(1, Ordering::AcqRel);
            // Wall-clock crossed t_end — the segment is over. Clear bits
            // for EVERY motor in the consumers mask, not just Modulated
            // ones.
            for motor_idx in 0..4_u8 {
                Self::clear_motor_bits_in_mask(&mut seg, motor_idx);
            }
            // 2026-05-17 diag: snapshot consumers_remaining after the clear
            // loop so the host can see which bits the per-motor loop
            // didn't reach.
            shared
                .last_retire_consumers_after_clear
                .store(seg.consumers_remaining as u32, Ordering::Release);
            self.producer_current = Some(seg);

            if seg.consumers_done() {
                shared
                    .modulated_retire_successes
                    .fetch_add(1, Ordering::AcqRel);
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
                // 2026-05-18 wedge fix: clear the volatile gate so
                // producer_step sees the retirement and re-dequeues.
                write_producer_current_present(shared, false);
                // 2026-05-18 wedge fix: pure-Modulated configs (F4 Z-only,
                // or H7 X/Y when E's step_time_event isn't polling) have no
                // path that rearms the producer Klipper timer after the ISR
                // retires a segment. Without this kick, the next queued
                // segment sits unfetched forever — TIM5 modulated_tick reads
                // producer_current = None and early-returns, queue_depth
                // stays at N-1 (live bench 2026-05-18: H7 + F4 both stall
                // at queue_depth=6, retired_through=1 after a 4-jog burst).
                //
                // Setting producer_pending true from ISR is half the fix —
                // the C-side foreground task (runtime_drain) needs to
                // observe it and call arm_producer_timer_if_kicked so the
                // Klipper scheduler actually fires producer_step. See the
                // paired change in src/runtime_tick.c::runtime_drain.
                shared.producer_pending.store(true, Ordering::Release);
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

        // 2026-05-18: drive the endstop ARM state machine from the modulated
        // path. Without this, sensorless homing on Modulated motors arms the
        // ARM (host sends `runtime_arm_endstop`, firmware samples the DIAG
        // pin every TIM5 fire into `PIN_LEVELS`), but the trip-check
        // (`endstop::tick`) is never invoked — bench 2026-05-18 G28 X moves
        // ran the full homing distance and skipped against the rail while
        // PG9 was correctly asserting stallguard. The legacy
        // `Engine::tick` path called `poll_endstop_trip` after position
        // eval; the modulated path is now responsible for the same call.
        //
        // `v_per_axis_q16 = [u32::MAX; 3]` matches the StepTime path's
        // convention (`abort_for_step_time_trip`): the IgnoreUntilMoving
        // policy uses velocity purely to gate the `moved_above_v` latch,
        // and TIM5 firing already implies the engine is processing
        // motion. A precise per-axis velocity hook from `dx_du` is a
        // follow-up once the modulated tick is ready to expose curve
        // derivatives.
        if self.poll_endstop_trip(now, [u32::MAX; 3], pool, None, trace, shared) {
            // `abort_for_homing_trip` already cleared `producer_current`,
            // retired curve slots, marked `last_error = HOMING_TRIP`, and
            // transitioned status to `Drained`. Skip step emission for
            // this tick so no further pulses fire past the trip point.
            return;
        }

        // Phase-stepping round-robin scheduling (variable-length per-motor
        // table). The phase_motor_count and per-motor (config, slot_idx)
        // entries are populated by configure_axes_blob's variable-length
        // branch. Each Modulated motor with a phase config computes its
        // `(mscount, i_a, i_b)` every tick, but only ONE phase motor
        // writes SPI per tick to keep the bus bandwidth bounded — the
        // schedule is `phase_tick_counter % phase_motor_count`. With up
        // to 16 phase motors the loop bound stays small; cost is
        // negligible.
        let count = shared.phase_motor_count.load(Ordering::Acquire) as usize;
        let mut phase_motor_ordinals: [Option<usize>; crate::state::MAX_STEPPER_OIDS] =
            [None; crate::state::MAX_STEPPER_OIDS];
        let mut active_phase_motors: u32 = 0;
        for motor_idx in 0..count {
            // Each phase entry is keyed by its slot_idx into step_modes.
            let slot_idx = shared
                .phase_slot_idx
                .get(motor_idx)
                .map(|s| s.load(Ordering::Acquire))
                .unwrap_or(0xFF) as usize;
            if slot_idx >= 4 {
                continue;
            }
            let mode_i = shared
                .step_modes
                .get(slot_idx)
                .map(|m| m.load(Ordering::Acquire))
                .unwrap_or(StepMode::StepTime as u8);
            if mode_i != StepMode::Modulated as u8 {
                continue;
            }
            let has_phase_cfg = shared
                .phase_config
                .get(motor_idx)
                .and_then(|s| crate::phase_config::load(s))
                .is_some();
            if has_phase_cfg {
                if let Some(slot) = phase_motor_ordinals
                    .get_mut(active_phase_motors as usize)
                {
                    *slot = Some(motor_idx);
                }
                active_phase_motors = active_phase_motors.saturating_add(1);
            }
        }
        let phase_motor_due = if active_phase_motors > 0 {
            let idx = (self.phase_tick_counter % active_phase_motors) as usize;
            phase_motor_ordinals.get(idx).copied().flatten()
        } else {
            None
        };
        let trace_enabled = shared.phase_trace_enabled.load(Ordering::Acquire);

        // Walk the per-motor phase table (motor_idx → motors[slot_idx]).
        // Each motor writes its own TMC chip's XDIRECT register; multiple
        // motors may share a slot (AWD partners) and consume identical
        // commanded positions but emit to distinct CS pins.
        for motor_idx in 0..count {
            let phase_cfg = shared
                .phase_config
                .get(motor_idx)
                .and_then(|s| crate::phase_config::load(s));
            let Some(cfg) = phase_cfg else {
                continue;
            };
            let slot_idx = shared
                .phase_slot_idx
                .get(motor_idx)
                .map(|s| s.load(Ordering::Acquire))
                .unwrap_or(0xFF) as usize;
            if slot_idx >= 4 {
                continue;
            }
            let mode = shared
                .step_modes
                .get(slot_idx)
                .map(|m| m.load(Ordering::Acquire))
                .unwrap_or(StepMode::StepTime as u8);
            if mode != StepMode::Modulated as u8 {
                continue;
            }
            let Some(&m) = motors.get(slot_idx) else {
                continue;
            };

            // Phase-stepping output path. Seed the per-motor modulator
            // from the engine's configured steps_per_mm (looked up from
            // the kinematic slot's `step_state`) on first use
            // post-configure / post-flush. The accumulator inside the
            // modulator carries the sub-microstep residual across ticks
            // the same way `StepMotorState` does.
            //
            // Note: when 2+ motors share a slot (AWD pair), each motor
            // has its own modulator instance. They consume the same
            // motors[slot_idx] commanded position each tick and produce
            // identical (mscount, i_a, i_b) — i.e. their state stays in
            // lockstep, by construction. The only divergence is the SPI
            // round-robin: only one motor writes XDIRECT per tick, while
            // the others still advance their own accumulators.
            let steps_per_mm = self
                .step_state
                .get(slot_idx)
                .map(|s| s.debug_steps_per_mm())
                .unwrap_or(0.0);
            let modulator = self
                .phase_modulators
                .get_mut(motor_idx)
                .and_then(|slot| {
                    Some(slot.get_or_insert_with(|| {
                        crate::modulator::PhaseDirectModulator::new(steps_per_mm)
                    }))
                });
            let Some(modulator) = modulator else {
                continue;
            };

            match modulator.compute(m) {
                Ok(r) => {
                    // Maintain `stepper_counts` so homing snapshots and
                    // host position queries keep working for phase-stepped
                    // axes (spec §3.1 step 1). Indexed by motor_idx so
                    // each physical motor has its own counter.
                    if r.steps_delta != 0 {
                        if let Some(counter) = shared.stepper_counts.get(motor_idx) {
                            counter.fetch_add(r.steps_delta, Ordering::AcqRel);
                        }
                    }

                    // SPI write: only the round-robin-due motor writes
                    // its XDIRECT register this tick. Non-due phase
                    // motors still trace their computed values with
                    // `wrote_spi=false` so the host can reconstruct the
                    // full per-tick state even though the bus only
                    // carries one write.
                    let wrote_spi = phase_motor_due == Some(motor_idx);
                    if wrote_spi {
                        write_xdirect(cfg.spi_bus_id, cfg.cs_pin_id, r.i_a, r.i_b);
                    }

                    if trace_enabled {
                        let sample = TraceSample::phase_step(
                            self.phase_tick_counter,
                            motor_idx as u8,
                            r.mscount,
                            r.i_a,
                            r.i_b,
                            wrote_spi,
                        );
                        // Match the existing pattern: trace pushes are
                        // best-effort, and the `sample_drop_pending`
                        // latch carries the overflow signal forward.
                        let _ = trace.enqueue(sample);
                    }
                }
                Err(()) => {
                    shared.last_error.store(
                        crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                        Ordering::Release,
                    );
                    shared.runtime_status.store(
                        RuntimeStatus::Fault as u8,
                        Ordering::Release,
                    );
                    self.last_error.store(
                        crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                        Ordering::Release,
                    );
                    self.status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    return;
                }
            }
        }

        // StepPulse output path for non-phase-stepped slots (independent
        // of the per-motor phase table). Indexed by kinematic slot —
        // motors[slot_idx] drives stepper_counts[slot_idx] via
        // StepAccumulator + emit_step_pulses. Slots that are Modulated
        // (i.e. phase-stepped) skip this branch.
        for motor_idx in 0..4_usize {
            let mode = shared
                .step_modes
                .get(motor_idx)
                .map(|m| m.load(Ordering::Acquire))
                .unwrap_or(StepMode::StepTime as u8);
            if mode == StepMode::Modulated as u8 {
                // Phase-stepped slot — the per-motor walk above already
                // updated XDIRECT + stepper_counts for every motor on
                // this slot.
                continue;
            }
            let Some(&m) = motors.get(motor_idx) else {
                continue;
            };

            // Existing StepPulse output path — unchanged.
            let Some(ss) = self.step_state.get_mut(motor_idx) else {
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
                    shared.last_error.store(
                        crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                        Ordering::Release,
                    );
                    shared.runtime_status.store(
                        RuntimeStatus::Fault as u8,
                        Ordering::Release,
                    );
                    self.last_error.store(
                        crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                        Ordering::Release,
                    );
                    self.status
                        .store(RuntimeStatus::Fault as u8, Ordering::Release);
                    return;
                }
            }
        }

        // Advance the round-robin counter once per tick, not per motor —
        // otherwise the SPI schedule rotates too fast and a single motor
        // would write every tick when only one phase motor is present.
        self.phase_tick_counter = self.phase_tick_counter.wrapping_add(1);

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

/// Number of cubic Bézier pieces in a degree-3 NURBS expressed in piecewise-
/// Bézier form (each interior knot has multiplicity 3). Returns 0 for any
/// other shape (non-cubic degree, too few CPs, n_cps - 1 not divisible by 3).
///
/// The host's `bezier_pieces_to_nurbs` (rust/nurbs/src/bezier.rs:532) produces
/// exactly this layout: `n_cps = 3N + 1` for `N` pieces, knot vector
/// `[u₀;p+1, u₁;p, u₂;p, …, u_N;p+1]` (clamped ends, multiplicity-p interior).
/// `trajectory/src/refit.rs::refit_to_cubic` always emits this form before
/// the bridge serializes via `from_scalar_nurbs_normalized`.
/// Detect a curve that holds a single constant value across its entire
/// domain — i.e. a degree-3 piecewise-Bezier with exactly one piece whose
/// four control points are all equal within an f32 ulp budget.
///
/// Returns the constant value if the curve is trivially constant, otherwise
/// `None`. Used by the producer's CoreXY combine path to combine a constant
/// axis curve (sent by the bridge as a real handle to anchor `prev_value`)
/// with a multi-piece shaped axis curve without requiring matched piece
/// counts.
///
/// The f32 epsilon (1e-5) is sized for typical printer-coordinate ranges
/// (0..400 mm). The host bridge emits constants by truncating an f64
/// `start_pos` to f32 four times, so they're exactly equal in f32 — the
/// epsilon tolerates any future bridge that emits "approximately constant"
/// without breaking detection.
fn curve_constant_value(cv: &CurveView<'_>) -> Option<f32> {
    if cubic_n_pieces(cv) != 1 {
        return None;
    }
    let cps = cv.control_points;
    if cps.is_empty() {
        return None;
    }
    let first = cps[0];
    if cps.iter().all(|&v| libm::fabsf(v - first) < 1e-5) {
        Some(first)
    } else {
        None
    }
}

fn cubic_n_pieces(cv: &CurveView<'_>) -> usize {
    if cv.degree != 3 {
        return 0;
    }
    let n_cps = cv.control_points.len();
    if n_cps < 4 {
        return 0;
    }
    // Piecewise-Bezier layout: n_cps = 3N + 1.
    if (n_cps - 1) % 3 != 0 {
        return 0;
    }
    // Sanity: knots must satisfy n_knots = n_cps + degree + 1 = 3N + 5.
    if cv.knots.len() != n_cps + 4 {
        return 0;
    }
    (n_cps - 1) / 3
}

/// Extract the `piece_idx`-th cubic Bézier piece of a degree-3 NURBS in
/// piecewise-Bézier form. Returns `(u_start, u_end, cps)` where
/// `u_start`/`u_end` are the piece's domain in the curve's global parameter
/// space, and `cps` are the four Bernstein control points of the piece on
/// local `t ∈ [0, 1]` where `t = (u - u_start) / (u_end - u_start)`.
///
/// Caller is responsible for mapping `t` back to `u` after solving:
/// `u_global = u_start + t * (u_end - u_start)`.
///
/// Returns `None` for out-of-range `piece_idx` or for non-cubic/malformed
/// curves (same gate as `cubic_n_pieces`).
fn cubic_piece(
    cv: &CurveView<'_>,
    piece_idx: usize,
) -> Option<(f32, f32, [f32; 4])> {
    let n_pieces = cubic_n_pieces(cv);
    if piece_idx >= n_pieces {
        return None;
    }
    let p = 3_usize;
    let cp_base = p * piece_idx;
    // Bounds-checked indexing — n_pieces already proved n_cps >= 3N + 1 so
    // cp_base + 3 < n_cps. Use explicit `get` to avoid clippy::indexing_slicing.
    // f32 throughout: storage is f32, solver is f32, no precision loss.
    let p0 = *cv.control_points.get(cp_base)?;
    let p1 = *cv.control_points.get(cp_base + 1)?;
    let p2 = *cv.control_points.get(cp_base + 2)?;
    let p3 = *cv.control_points.get(cp_base + 3)?;
    // Knot indexing: `u_start = knots[p + p*i]`, `u_end = knots[p + p*(i+1)]`.
    // Validated against the host's emission layout in
    // `rust/nurbs/src/bezier.rs::bezier_pieces_to_nurbs`.
    let u_start_idx = p + p * piece_idx;
    let u_end_idx = p + p * (piece_idx + 1);
    let u_start = *cv.knots.get(u_start_idx)?;
    let u_end = *cv.knots.get(u_end_idx)?;
    if !(u_end > u_start) {
        return None;
    }
    Some((u_start, u_end, [p0, p1, p2, p3]))
}

/// De Casteljau split of a cubic Bezier (cps on local parameter `[0, 1]`)
/// at parameter `r`. Returns `(left, right)` cps, each parameterized on
/// their own local `[0, 1]`. The two halves together represent the
/// original curve: `left` covers original-`[0, r]`, `right` covers
/// original-`[r, 1]`.
#[inline]
fn cubic_split_at(cps: [f32; 4], r: f32) -> ([f32; 4], [f32; 4]) {
    let one_minus_r = 1.0 - r;
    let p01 = one_minus_r * cps[0] + r * cps[1];
    let p12 = one_minus_r * cps[1] + r * cps[2];
    let p23 = one_minus_r * cps[2] + r * cps[3];
    let p012 = one_minus_r * p01 + r * p12;
    let p123 = one_minus_r * p12 + r * p23;
    let p0123 = one_minus_r * p012 + r * p123;
    ([cps[0], p01, p012, p0123], [p0123, p123, p23, cps[3]])
}

/// Slice a cubic Bezier (cps on local parameter `[0, 1]`) to a sub-range
/// `[s, t]`. Returns 4 new cps representing the sub-curve as a fresh
/// cubic Bezier on its own local `[0, 1]`. Used by the producer's
/// piece-merge walker to combine two CoreXY axis curves whose piece
/// boundaries don't align (e.g., shaper at 186 Hz on X + 122 Hz on Y
/// emits 10 X-pieces and 11 Y-pieces with different breakpoints).
///
/// Preconditions: `0 ≤ s ≤ t ≤ 1`. `s = 0` and/or `t = 1` are allowed
/// (degenerate / pass-through cases).
fn cubic_subsegment(cps: [f32; 4], s: f32, t: f32) -> [f32; 4] {
    // Fast path: full piece (no slicing needed).
    if s <= 0.0 && t >= 1.0 {
        return cps;
    }
    // Step 1: split at `s`. The right half is the sub-curve on
    // original-`[s, 1]`, parameterized on local-`[0, 1]`.
    let (_, right) = cubic_split_at(cps, s);
    // Step 2: in the right half, find the local parameter that maps to
    // original `t`. The right half parameterizes original `s + r*(1-s)`,
    // so `original_t = s + r*(1-s)` ⇒ `r = (t - s) / (1 - s)`. Guard
    // against `s ≈ 1` (degenerate slice at the curve's end).
    let span = 1.0 - s;
    if span < 1e-9 {
        // s is essentially 1; return a degenerate point at curve(1).
        return [cps[3]; 4];
    }
    let r = (t - s) / span;
    let r = r.clamp(0.0, 1.0);
    let (left, _) = cubic_split_at(right, r);
    left
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
        // HomingTrip is an expected, host-coordinated abort — engine
        // transitions to Drained, not Fault. The host receives the
        // EndstopTripped event via the runtime-events channel and
        // submits a back-off segment, which would be blocked by the
        // FaultLatched gate if status were Fault. `last_error` still
        // records the trip so a status query distinguishes "freshly
        // tripped" from "fresh idle".
        assert_eq!(engine.status(), RuntimeStatus::Drained);
        assert_eq!(engine.last_error(), crate::error::KALICO_ERR_HOMING_TRIP);
        assert!(engine.current.is_none());
        // Trip must clear stream_open so a back-off segment dispatched
        // by the host activates without the engine first underrun-faulting
        // on the empty queue.
        assert!(!shared.stream_open.load(Ordering::Acquire));
        // Trip must return the active segment's curve-pool slots.
        // Without this, every G28 trip leaked a slot, exhausting the
        // pool after ~CURVE_POOL_N / 2 trips and breaking subsequent
        // motion with SlotPoolExhausted. The x_handle returned slot 0;
        // assert that slot is free again post-trip.
        assert!(
            pool.is_slot_free(0),
            "trip leaked x_handle slot 0 — pool retirement regressed",
        );
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
