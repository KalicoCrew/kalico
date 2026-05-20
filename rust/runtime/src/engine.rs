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
//
// Access patterns (2026-05-19 A3 boundary audit clarifications):
//
//   - The `_present` flag (read+write from Rust) is accessed EXCLUSIVELY
//     through the C accessor functions kalico_producer_current_is_present
//     / _set_present. The function-call boundary makes the volatile
//     semantics opaque to LLVM. The bare static mut import for `_present`
//     was deleted in A3 because it carried the same borrow-projection
//     miscompilation risk that motivated the queue migration.
//
//   - The `_set_count` / `_cleared_count` diagnostic counters are read-only
//     from Rust via `core::ptr::read_volatile(addr_of!(...))` in
//     kalico_runtime_producer_current_gate_counters_diag (see
//     kalico-c-api/src/runtime_ffi.rs:2698). The read_volatile path
//     emits a real LDR with no borrow projection, so the LLVM
//     miscompilation that motivated A3 does NOT apply here. These bare
//     static mut imports stay.
#[cfg(target_os = "none")]
#[allow(unsafe_code)]
unsafe extern "C" {
    pub static mut kalico_producer_current_set_count: u32;
    pub static mut kalico_producer_current_cleared_count: u32;
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
use crate::curve_pool::{CurveHandle, CurvePool};
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
///
/// 2026-05-19 — keyed by `motor_idx` instead of `(bus_id, cs_pin)` to fix
/// the multi-TMC5160-on-one-SPI-bus CS-aliasing bug; the C side now
/// looks up both bus cfg and CS handle from a per-motor table. See
/// docs/superpowers/specs/2026-05-19-phase-stepping-per-motor-cs-design.md.
#[inline]
#[allow(unsafe_code)]
fn write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16) {
    #[cfg(target_os = "none")]
    {
        unsafe extern "C" {
            fn phase_stepping_write_xdirect(
                motor_idx: u8,
                coil_a: i16,
                coil_b: i16,
            );
        }
        // SAFETY: stable C ABI symbol provided by src/stm32/phase_stepping_spi.c.
        // Three scalar args by value, no aliasing. Motor / bus validity is the
        // C side's responsibility (no-op on out-of-range or unregistered).
        unsafe { phase_stepping_write_xdirect(motor_idx, coil_a, coil_b) }
    }
    #[cfg(not(target_os = "none"))]
    {
        crate::test_xdirect_capture::record(motor_idx, coil_a, coil_b);
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
fn capture_segment_curve_meta(_seg: &Segment, _pool: &CurvePool) {
    // Legacy NURBS diagnostic: reported (degree, n_cps, n_knots) which have no
    // meaning under the cubic-piece representation. Removed in
    // stepping-redesign-finish Task 12 along with `tick` / `tick_with_current`.
    unimplemented!("removed in stepping-redesign-finish Task 12");
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
    /// Bitmask: bits 0-3 are axes A/B/Z/E. Set at `arm_segment` (Task 8)
    /// for each axis whose curve handle is non-sentinel AND which
    /// participates in retire (E in CoupledToXy mode is non-participating).
    /// Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §4.5.
    pub participating_mask: u8,

    /// Bitmask: starts equal to `participating_mask` at arm; bits clear
    /// as each axis's curve exhausts during evaluation. Retire fires
    /// when `pending_mask == 0`.
    pub pending_mask: u8,

    /// Snapshot of `e_accumulator` (truncated to f32) at segment-arm.
    /// Phase-3 evaluator returns `segment_base_e + segment_local_e` to
    /// produce absolute E position (§4.6).
    pub segment_base_e: f32,

    /// XY arc-length accumulator, in mm, segment-scoped. Reset at arm,
    /// updated each sample in Phase 2 of `runtime_tick_sample`.
    pub ds_xy_segment: f32,
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

    // ─── Stepping-redesign Task 11 — unified per-axis configuration ──────
    //
    // Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
    //
    // Populated by `kalico_configure_axis` / `kalico_configure_kinematics` /
    // `kalico_configure_pressure_advance` (Task 11 FFI). Consumed by the
    // unified sample-ISR tick path (Task 8) that replaces the legacy
    // `tick()` / `producer_step()` split.
    //
    // These fields coexist with the legacy step-emission state above
    // (`step_rings`, `producer_states`, `producer_current`) until Task 16
    // deletes the legacy path. The unified tick reads only the fields below;
    // the legacy tick reads only the legacy fields. No overlap or
    // synchronization required.

    /// Per-logical-axis configuration: active Bezier piece, cached scalars,
    /// stepper bindings. Indexed `[X=0, Y=1, Z=2, E=3]` in logical-axis
    /// space (the unified engine performs the kinematic transform at piece
    /// activation time, not per-sample).
    pub stepping_axes:
        [crate::stepping_state::AxisConfig; crate::stepping_state::N_AXES],

    /// Kinematic scale factor relating logical-XY velocity to physical
    /// motor-coordinate velocity magnitude. `1.0` for Cartesian (XY motor
    /// positions equal logical XY); `1.0 / sqrt(2)` for CoreXY (each motor
    /// moves at √2 times the per-axis logical speed at 45° diagonals).
    /// Consumed by the XY-arc-length integrator that feeds the E-follows-XY
    /// and pressure-advance paths. Spec §3.4.
    pub k_xy: f32,

    /// Linear pressure-advance coefficient during the toolhead's accelerating
    /// phase (s). The unified tick adds `+ advance_accel * ratio_per_xy_mm
    /// * |v_xy|` to the integrated extrusion while `v̇_xy > 0`. `0.0`
    /// disables PA on acceleration. Spec §3.5.
    pub advance_accel: f32,

    /// Linear pressure-advance coefficient during the toolhead's decelerating
    /// phase (s). Mirror of `advance_accel`; allows asymmetric K_accel /
    /// K_decel (Kalico bleeding-edge Step 9). `0.0` disables PA on
    /// deceleration. Spec §3.5.
    pub advance_decel: f32,

    /// Sample-rate period in seconds. Equal to `1.0 / sample_rate_hz`
    /// (typically 25 µs at 40 kHz). Consumed by sub-sample timing for
    /// secant-slope velocity recovery. Published once at
    /// `configure_kinematics` time; recomputed if the timer cadence changes.
    pub sample_period_sec: f32,

    /// Sample-rate period in MCU clock cycles. Equal to
    /// `cycles_per_second * sample_period_sec` rounded to the nearest u32.
    /// Mirrors the host-published `SharedState::sample_period_cycles`
    /// scheduler tunable but lives on the engine so the hot path doesn't
    /// reload from `SharedState` every tick.
    pub sample_period_cycles: u32,

    /// MCU clock frequency (Hz). Cached from the C-side
    /// `runtime_clock_freq` constant at `Engine::new` time so the unified
    /// tick can convert cycles ↔ seconds without a foreign read per sample.
    pub cycles_per_second: f32,

    /// ISR-local scratch carried across consecutive sample ticks. Never
    /// observed by anything outside the sample ISR — plain values, no
    /// atomics. Used by secant-slope sub-sample timing and the
    /// E-follows-XY arc-length accumulator.
    pub tick_caches: crate::stepping_state::TickCaches,
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
            participating_mask: 0,
            pending_mask: 0,
            segment_base_e: 0.0,
            ds_xy_segment: 0.0,
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
            // Task 11 unified per-axis configuration. All fields default
            // to "unconfigured" until configure_axis / configure_kinematics
            // / configure_pressure_advance publish real values.
            stepping_axes: [
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
            ],
            // Default to Cartesian k_xy=1.0 so the unified-tick XY
            // arc-length integration produces sane numbers even if the
            // host never sends configure_kinematics (a misconfiguration,
            // but better than NaN propagation). CoreXY hosts overwrite
            // this with 1.0/sqrt(2).
            k_xy: 1.0,
            advance_accel: 0.0,
            advance_decel: 0.0,
            sample_period_sec: 0.0,
            sample_period_cycles: 0,
            cycles_per_second: clock_freq as f32,
            tick_caches: crate::stepping_state::TickCaches::new(),
        }
    }

    /// Production-context constructor. Mirrors `::new(clock_freq)` but keeps
    /// the call site noise low (Step-6 spec §14): the C-side
    /// `runtime_clock_freq` static is read once at FFI init time and the value
    /// is threaded through here.
    pub fn new_production(clock_freq: u32) -> Self {
        Self::new(clock_freq)
    }

    /// In-place initialization. Writes every field of `*ptr` via raw
    /// pointer projections without ever materializing an `Engine` on the
    /// stack — `step_rings` alone is ~20 KB and the H7 stack is only 8 KB,
    /// so any path that returns `Engine` by value risks blowing the stack
    /// during construction unless RVO succeeds. RVO is best-effort, not
    /// guaranteed; the stepping-redesign growth (Tasks 6/8/11) tipped the
    /// compiler past whatever heuristic was load-bearing in the prior
    /// build and the MCU started crashing during `runtime_handle_create`
    /// before USB enumeration.
    ///
    /// # Safety
    /// `ptr` must be valid for writes of `size_of::<Engine<P, I>>()` bytes
    /// and properly aligned. Caller must guarantee no concurrent reads.
    /// Used by [`crate::state::RuntimeContext::init`] to construct the
    /// engine field directly inside the C-owned `rt_storage` buffer.
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place(ptr: *mut Self, clock_freq: u32) {
        use core::ptr::addr_of_mut;
        unsafe {
            addr_of_mut!((*ptr).current).write(None);
            addr_of_mut!((*ptr).last_motors).write([0.0; 4]);
            addr_of_mut!((*ptr).pa_slot).write(P::default());
            addr_of_mut!((*ptr).is_slot).write(I::default());
            addr_of_mut!((*ptr).one_tick_cycles_value)
                .write(u64::from(one_tick_cycles(clock_freq)));
            addr_of_mut!((*ptr).status)
                .write(AtomicU8::new(RuntimeStatus::Idle as u8));
            addr_of_mut!((*ptr).last_error).write(AtomicI32::new(0));
            addr_of_mut!((*ptr).tick_counter).write(TickCounter::new());
            addr_of_mut!((*ptr).hold_sample_ticks).write(0);
            addr_of_mut!((*ptr).prev_x).write(0.0);
            addr_of_mut!((*ptr).prev_y).write(0.0);
            addr_of_mut!((*ptr).prev_z).write(0.0);
            addr_of_mut!((*ptr).e_accumulator).write(0.0);
            addr_of_mut!((*ptr).participating_mask).write(0);
            addr_of_mut!((*ptr).pending_mask).write(0);
            addr_of_mut!((*ptr).segment_base_e).write(0.0);
            addr_of_mut!((*ptr).ds_xy_segment).write(0.0);
            addr_of_mut!((*ptr).needs_xy_seed).write(true);
            addr_of_mut!((*ptr).debug_last_now).write(0);
            addr_of_mut!((*ptr).debug_last_tstart).write(0);
            addr_of_mut!((*ptr).debug_last_duration).write(0);
            addr_of_mut!((*ptr).step_state)
                .write([crate::step::StepMotorState::default(); 4]);
            addr_of_mut!((*ptr).phase_modulators)
                .write([const { None }; crate::state::MAX_STEPPER_OIDS]);
            addr_of_mut!((*ptr).phase_tick_counter).write(0);
            addr_of_mut!((*ptr).mcu_config).write(None);
            // step_rings is the largest field (~20 KB); initialize each
            // slot individually so even the array literal doesn't live on
            // the stack as a temporary.
            let rings_ptr = addr_of_mut!((*ptr).step_rings).cast::<StepRing>();
            for i in 0..4 {
                rings_ptr.add(i).write(StepRing::new());
            }
            let states_ptr = addr_of_mut!((*ptr).producer_states)
                .cast::<ProducerState>();
            for i in 0..4 {
                states_ptr.add(i).write(ProducerState::new(0.0));
            }
            addr_of_mut!((*ptr).motor_curve_cursor).write([0; 4]);
            addr_of_mut!((*ptr).motor_current_segment_id).write([None; 4]);
            addr_of_mut!((*ptr).producer_current).write(None);
            #[cfg(any(test, feature = "test-injection"))]
            addr_of_mut!((*ptr).injected_iter_start).write(0);
            // Task 11 unified per-axis state. Same in-place pattern —
            // `AxisConfig` contains `heapless::Vec<StepperRef, 4>` which
            // is itself non-trivially-sized.
            let axes_ptr = addr_of_mut!((*ptr).stepping_axes)
                .cast::<crate::stepping_state::AxisConfig>();
            for i in 0..crate::stepping_state::N_AXES {
                axes_ptr.add(i)
                    .write(crate::stepping_state::AxisConfig::new_unconfigured());
            }
            addr_of_mut!((*ptr).k_xy).write(1.0);
            addr_of_mut!((*ptr).advance_accel).write(0.0);
            addr_of_mut!((*ptr).advance_decel).write(0.0);
            addr_of_mut!((*ptr).sample_period_sec).write(0.0);
            addr_of_mut!((*ptr).sample_period_cycles).write(0);
            addr_of_mut!((*ptr).cycles_per_second).write(clock_freq as f32);
            addr_of_mut!((*ptr).tick_caches)
                .write(crate::stepping_state::TickCaches::new());
        }
    }

    /// Production-context in-place init. Same contract as
    /// [`init_in_place`] with the production allocator/slot types.
    ///
    /// # Safety
    /// See [`init_in_place`].
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place_production(ptr: *mut Self, clock_freq: u32) {
        unsafe { Self::init_in_place(ptr, clock_freq) }
    }
}

// ─── Stepping-redesign Task 11 — configuration entry points ───────────────
//
// Foreground command handlers reach these through the FFI shims in
// `kalico-c-api::runtime_ffi` (`kalico_runtime_configure_axis`,
// `kalico_runtime_configure_kinematics`,
// `kalico_runtime_configure_pressure_advance`). All three are
// foreground-only and never racing with the ISR — Klipper sends these
// commands from the single-threaded command dispatcher before / between
// segments, not while the TIM5 ISR is mid-tick on the same axis state.
//
// Return convention: `0` = success, negative = host-visible error. The
// negative values are kept abstract here (`-1`) rather than reaching for
// the KALICO_ERR_* constants in `runtime::error` because the C handlers
// merely log "rejected" — host-side surfacing of the precise error code
// is not part of Task 11's surface area.
impl<P: PaSlot, I: IsSlot> Engine<P, I> {
    /// Publish new per-axis configuration for a single logical axis.
    ///
    /// `axis_idx` is `0..N_AXES` (X=0, Y=1, Z=2, E=3). `mode` selects the
    /// per-stepper output path (Pulse or Phase). `microstep_distance` is
    /// the per-step distance in mm-equivalent units (must be finite,
    /// positive). `extrusion_per_xy_mm` is accepted for ABI
    /// compatibility but ignored — per-segment `Segment::extrusion_ratio`
    /// is now authoritative (Task 6 + Task 11). Validated finite so a
    /// NaN/inf slip-through from a slicer still surfaces here as an
    /// error.
    ///
    /// `stepper_count` is accepted for ABI compatibility but currently
    /// unused — physical stepper bindings are still wired by
    /// `config_runtime_stepper` until Task 16 unifies that path.
    ///
    /// On success the new configuration is published with `Release`
    /// ordering so the ISR's next `mode.load(Acquire)` observes the new
    /// mode together with the new scalar fields (the ISR re-reads
    /// `microstep_distance` whenever it samples a fresh piece, so
    /// plain-store ordering relative to the atomic mode-publish is
    /// sufficient).
    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: crate::stepping_state::StepMode,
        microstep_distance: f32,
        extrusion_per_xy_mm: f32,
        stepper_count: u8,
    ) -> i32 {
        if (axis_idx as usize) >= crate::stepping_state::N_AXES {
            return -1;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return -1;
        }
        if !extrusion_per_xy_mm.is_finite() {
            return -1;
        }
        // stepper_count and extrusion_per_xy_mm are accepted but ignored —
        // physical stepper bindings remain on `config_runtime_stepper`
        // until Task 16, and per-segment `Segment::extrusion_ratio` is
        // the authoritative source for the E-follows-XY ratio (Task 11).
        let _ = stepper_count;
        let _ = extrusion_per_xy_mm;
        let axis = &mut self.stepping_axes[axis_idx as usize];
        axis.microstep_distance = microstep_distance;
        // Clear any prior piece so the next segment-arrival path re-seeds
        // from scratch with the new microstep_distance / mode.
        axis.piece = None;
        axis.piece_start_time_cycles = 0;
        axis.last_step_count = 0;
        // Atomic publish of the mode last — ISR's Acquire load on `mode`
        // synchronizes against the plain stores above.
        axis.mode
            .store(mode as u8, core::sync::atomic::Ordering::Release);
        0
    }

    /// Publish kinematic scale factor relating logical-XY velocity to
    /// physical motor-coordinate velocity. `1.0` for Cartesian, `1/√2`
    /// (≈0.7071) for CoreXY. Validated finite + strictly positive — a
    /// zero / negative `k_xy` would silently zero out the XY arc-length
    /// integrator that feeds E-follows-XY and pressure-advance.
    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 {
            return -1;
        }
        self.k_xy = k_xy;
        0
    }

    /// Publish asymmetric pressure-advance coefficients. `advance_accel`
    /// applies while `v̇_xy > 0`, `advance_decel` while `v̇_xy < 0`. Both
    /// in seconds; `0.0` on either side disables PA in that phase. Reject
    /// non-finite or negative values — negative PA would invert the
    /// filament-pressure-correction sense, which is never physical.
    pub fn configure_pressure_advance(
        &mut self,
        advance_accel: f32,
        advance_decel: f32,
    ) -> i32 {
        if !advance_accel.is_finite() || !advance_decel.is_finite() {
            return -1;
        }
        if advance_accel < 0.0 || advance_decel < 0.0 {
            return -1;
        }
        self.advance_accel = advance_accel;
        self.advance_decel = advance_decel;
        0
    }

    /// ISR-side segment arm. Called by `runtime_tick_sample` after dequeueing
    /// a `Segment` from the SPSC queue. Populates per-axis state and the
    /// engine-level retire-bookkeeping masks. NEVER called from foreground —
    /// the §11.1 half-split discipline reserves `AxisConfig` mutation for the
    /// ISR.
    ///
    /// Logic (spec §3.3 + §4.5):
    /// 1. Per-axis: decode the segment's handle. If `UNUSED_SENTINEL`, leave
    ///    axis idle. Else resolve via `curve_pool.lookup_active` and populate
    ///    `curve_handle / piece_cursor / piece / piece_start_time_cycles`.
    /// 2. `participating_mask`: bits A/B/Z (0..2) follow handle validity;
    ///    bit E (3) ALSO requires `e_mode == EMode::Independent`
    ///    (`CoupledToXy` E is non-participating; `Travel` E excluded).
    /// 3. `pending_mask = participating_mask`.
    /// 4. `segment_base_e = e_accumulator as f32`.
    /// 5. `ds_xy_segment = 0.0`.
    /// 6. `current = Some(seg)`.
    ///
    /// Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §3.3 + §4.5.
    #[allow(unsafe_code)]
    pub fn arm_segment(
        &mut self,
        seg: crate::segment::Segment,
        curve_pool: &crate::curve_pool::CurvePool,
    ) {
        let handles = [seg.x_handle, seg.y_handle, seg.z_handle, seg.e_handle];

        // Per-axis arm.
        for (axis_idx, handle) in handles.iter().enumerate() {
            let axis = &mut self.stepping_axes[axis_idx];
            if *handle == crate::curve_pool::CurveHandle::UNUSED_SENTINEL {
                axis.curve_handle = None;
                axis.piece = None;
                axis.piece_cursor = 0;
            } else if let Some(curve_ptr) = curve_pool.lookup_active(*handle) {
                // SAFETY: curve_pool's generation guard published the slot;
                // ISR is sole reader for the duration of the segment.
                let curve = unsafe { &*curve_ptr };
                if curve.piece_count == 0 {
                    // Defensive: `populate_from_wire` rejects empty wire so
                    // this should be unreachable, but treat as idle.
                    axis.curve_handle = None;
                    axis.piece = None;
                    axis.piece_cursor = 0;
                } else {
                    axis.curve_handle = Some(*handle);
                    axis.piece_cursor = 0;
                    axis.piece = Some(curve.pieces[0]);
                    axis.piece_start_time_cycles = seg.t_start;
                }
            } else {
                // Slot generation mismatch (should be impossible — foreground
                // validated at push). Treat as idle for this axis.
                axis.curve_handle = None;
                axis.piece = None;
                axis.piece_cursor = 0;
            }
        }

        // Compute participating_mask. Bits A/B/Z (0..2) follow handle
        // validity; bit E (3) ALSO requires e_mode == Independent.
        let mut participating: u8 = 0;
        for axis_idx in 0..3 {
            if self.stepping_axes[axis_idx].curve_handle.is_some() {
                participating |= 1u8 << axis_idx;
            }
        }
        if seg.e_mode == crate::config::EMode::Independent
            && self.stepping_axes[3].curve_handle.is_some()
        {
            participating |= 1u8 << 3;
        }
        self.participating_mask = participating;
        self.pending_mask = participating;

        // E-accumulator base for absolute-position math (spec §4.6).
        self.segment_base_e = self.e_accumulator as f32;
        self.ds_xy_segment = 0.0;

        self.current = Some(seg);
    }

    /// Per-sample post-pass: after every per-axis `advance_piece_if_needed`
    /// has run for the current sample, update `pending_mask` and fault on
    /// early exhaustion. Spec §4.4 + §4.5.
    ///
    /// Must be called exactly once per `runtime_tick_sample`, AFTER all
    /// per-axis advances and BEFORE [`Engine::retire_if_complete`].
    ///
    /// Logic:
    /// - `exhausted_now`: bits where the axis WAS participating but its
    ///   `curve_handle` is now `None` (cleared by `advance_piece_if_needed`
    ///   on curve exhaustion).
    /// - `pending_mask = participating_mask & !exhausted_now`.
    /// - `raise_piece_advance_underflow` fires ONLY when at least one axis
    ///   exhausted THIS sample (was pending coming in, now cleared) AND
    ///   the post-pass `pending_mask` is still non-zero (i.e., other
    ///   participating axes still owe samples). Simultaneous exhaustion
    ///   of every pending axis drops `pending_mask` to zero in the same
    ///   sample → no fault, retire fires instead. Order-independent.
    pub fn post_pass_exhaustion(&mut self, shared: &SharedState) {
        if self.current.is_none() {
            return;
        }
        let mut exhausted_now: u8 = 0;
        for axis_idx in 0..crate::stepping_state::N_AXES {
            let bit = 1u8 << axis_idx;
            if self.participating_mask & bit != 0
                && self.stepping_axes[axis_idx].curve_handle.is_none()
            {
                exhausted_now |= bit;
            }
        }
        let prev_pending = self.pending_mask;
        self.pending_mask = self.participating_mask & !exhausted_now;

        // Early-exhaustion fault: bit cleared THIS sample (was pending,
        // now exhausted) AND post-pass pending_mask still non-zero.
        let exhausted_this_sample = prev_pending & exhausted_now;
        if exhausted_this_sample != 0 && self.pending_mask != 0 {
            let axis_idx = exhausted_this_sample.trailing_zeros() as usize;
            crate::fault_helpers::raise_piece_advance_underflow(shared, axis_idx);
        }
    }

    /// Phase-5 retire: when `pending_mask == 0` for the active segment,
    /// publish retirement bookkeeping and clear segment state. Spec §4.5.
    ///
    /// Side effects (atomic-publishing in `Release` order):
    /// 1. `shared.retired_through_segment_id ← current.id` — host slot
    ///    pool's `release_through(retired_through)` watches this cursor.
    ///    Matches the legacy contract (mirrored by `abort_for_homing_trip`
    ///    at engine.rs:1683).
    /// 2. `shared.producer_segment_retired_total += 1` — drained-segments
    ///    counter consumed by `kalico_credit_freed` foreground emission.
    /// 3. `stream::check_terminal_on_retire(shared, seg_id)` — terminal-
    ///    segment + stream-machine bookkeeping (mirrors the legacy
    ///    `abort_for_homing_trip` site).
    /// 4. Enqueue `TRACE_FLAG_SEGMENT_END` sample — foreground
    ///    `drain_and_reclaim` (`rust/runtime/src/reclaim.rs`) reads this
    ///    and calls `pool.confirm_retired` for each handle in the
    ///    `RetirementTable` entry keyed by `seg_id`.
    /// 5. Roll forward `e_accumulator` by the segment's CoupledToXy
    ///    contribution (`extrusion_ratio * ds_xy_segment`). For
    ///    `Independent` / `Travel` the intrinsic E NURBS already drove
    ///    the accumulator forward via Phase 3; Task 11 refines this.
    /// 6. Clear `current`, `participating_mask`, `pending_mask`,
    ///    `segment_base_e`, `ds_xy_segment`.
    ///
    /// Returns `true` iff retirement fired.
    pub fn retire_if_complete(
        &mut self,
        shared: &SharedState,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
    ) -> bool {
        if self.current.is_none() || self.pending_mask != 0 {
            return false;
        }
        // `current` is Some here.
        #[allow(clippy::unwrap_used)] // checked immediately above
        let seg = self.current.as_ref().unwrap();
        let seg_id = seg.id;
        let e_mode = seg.e_mode;
        let extrusion_ratio = seg.extrusion_ratio;

        // 1. Publish retirement cursor.
        shared
            .retired_through_segment_id
            .store(seg_id, Ordering::Release);
        // 2. Drained-segments counter.
        shared
            .producer_segment_retired_total
            .fetch_add(1, Ordering::AcqRel);
        // 3. Stream-state terminal hook.
        crate::stream::check_terminal_on_retire(shared, seg_id);
        // 4. SEGMENT_END trace marker. `tick`/`curve_handle` use the same
        // sentinel pattern the existing `abort_for_homing_trip` emission
        // uses for non-fault retire markers — reclaim only needs
        // `segment_id` + the `TRACE_FLAG_SEGMENT_END` bit.
        let _ = trace.enqueue(TraceSample {
            tick: 0,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_z: self.last_motors[2],
            motor_e: self.last_motors[3],
            segment_id: seg_id,
            curve_handle: CurveHandle::UNUSED_SENTINEL,
            flags: TRACE_FLAG_SEGMENT_END,
            _pad: [0; 7],
        });
        // 5. Roll forward e_accumulator (CoupledToXy follower portion).
        // Task 11's absolute-E evaluator will refine; for Task 10 the
        // segment's final XY-arc × extrusion_ratio captures the follower
        // contribution. Independent / Travel modes have their E delta
        // already integrated into the Phase-3 step accumulator.
        if e_mode == crate::config::EMode::CoupledToXy {
            self.e_accumulator += f64::from(extrusion_ratio * self.ds_xy_segment);
        }
        // 6. Clear segment-scoped state.
        self.current = None;
        self.participating_mask = 0;
        self.pending_mask = 0;
        self.segment_base_e = 0.0;
        self.ds_xy_segment = 0.0;
        true
    }

    /// Test-only accessor to seed `e_accumulator` before invoking
    /// `arm_segment`. The field stays private so production callers can't
    /// race the f64 mutation; the test integration crate would otherwise
    /// have no path to set up the snapshot-from-accumulator invariant.
    #[cfg(any(test, feature = "host"))]
    pub fn debug_set_e_accumulator(&mut self, value: f64) {
        self.e_accumulator = value;
    }

    /// Test-only accessor: returns `true` iff the engine currently holds
    /// an armed segment (i.e., `arm_segment` was called and no retire has
    /// fired). The `current` field is `pub(crate)` to keep external
    /// callers from peeking at the ISR-owned segment manifest; this
    /// accessor lets the post-pass tests verify that retire cleared it.
    #[cfg(any(test, feature = "host"))]
    pub fn debug_current_is_some(&self) -> bool {
        self.current.is_some()
    }

    /// Test-only accessor: returns the active segment's id, or `None` if
    /// no segment is currently armed.
    #[cfg(any(test, feature = "host"))]
    pub fn debug_current_segment_id(&self) -> Option<u32> {
        self.current.as_ref().map(|s| s.id)
    }

    /// Test-only accessor to seed `ds_xy_segment` before invoking
    /// `arm_segment`, so the reset-to-zero invariant is observable from a
    /// non-zero starting value.
    #[cfg(any(test, feature = "host"))]
    pub fn debug_set_ds_xy_segment(&mut self, value: f32) {
        self.ds_xy_segment = value;
    }

    // ─── Stepping-redesign Task 12 — axis-mode + stepper-offset commands ─
    //
    // Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
    //
    // `set_axis_mode` flips a single axis between Pulse and Phase output
    // modes. The spec sequence (§6.5) requires four ordered side-effects
    // before the mode-swap is published:
    //
    //   1. Motion-active gate. The flip is only legal between segments
    //      because the sample ISR's per-axis state assumes one mode for
    //      the lifetime of an active Bezier piece. A non-`None`
    //      `axis.piece` is the runtime's "segment is in flight" signal.
    //   2. Per-axis step-queue flush. The ISR-side StepQueue feeds the
    //      Pulse-path per-axis timer (Task 7) and is meaningless under
    //      Phase mode; flushing it on every flip prevents stale entries
    //      from firing once we switch back.
    //   3. SPI queue flush (Task 14). Stubbed here as a no-op; the
    //      placeholder marks the spec step so the Task-14 patch can
    //      hook in without re-locating the call site.
    //   4. Counter resync on `Pulse → Phase`. The Phase ISR consumes
    //      `last_phase_target` deltas; if we enter Phase mode without
    //      seeding it from `last_step_count + phase_offset`, the first
    //      Phase tick computes a multi-microstep delta and slams the
    //      coil-current target which the TMC5160 cannot track.
    //
    // Finally the mode atomic is published with `Release` ordering — the
    // ISR's `Acquire` load synchronises against the queue resets and
    // counter-resync stores above.
    //
    // `set_stepper_offset` adds `delta_microsteps` to a single stepper's
    // `phase_offset_target`. The TIM5 ramp helper (Task 13) walks
    // `phase_offset_microsteps` toward this target at no more than
    // `max_microsteps_per_sample` per sample. Task 12 only owns parameter
    // validation + target publish; Task 13 owns the ramp itself, so the
    // ramp-rate argument is validated and stashed on `SharedState` for the
    // ramp helper to read. Invalid arguments latch
    // `FaultCode::JogParametersInvalid` so the host sees the rejection.
    //
    // Return convention: `0` = success; `-1` = bad argument; `-2` =
    // `set_axis_mode` rejected because a segment is in flight. The C
    // handler treats any non-zero return as a shutdown trigger.
    pub fn set_axis_mode(&mut self, axis_idx: u8, new_mode_byte: u8) -> i32 {
        const ERR_MOTION_IN_PROGRESS: i32 = -2;
        const KALICO_OK: i32 = 0;

        if (axis_idx as usize) >= crate::stepping_state::N_AXES {
            return -1;
        }
        let new_mode = match new_mode_byte {
            0 => crate::stepping_state::StepMode::Pulse,
            1 => crate::stepping_state::StepMode::Phase,
            _ => return -1,
        };

        // Step 1: motion-active gate. Any axis with an active piece means
        // the engine is mid-segment; reject the flip rather than tear
        // state out from under the ISR.
        let motion_active = self.stepping_axes.iter().any(|a| a.piece.is_some());
        if motion_active {
            return ERR_MOTION_IN_PROGRESS;
        }

        // Step 2: flush per-axis step queue. The C-declared `step_queues`
        // symbol owns the storage (one queue per axis, ring-buffer with
        // `head == tail` indicating empty). Reset both counters to zero
        // with volatile stores — the matching ISR-side reads also use
        // volatile, so the ordering relative to the mode-publish below is
        // sufficient without an explicit fence.
        #[cfg(not(any(test, feature = "host")))]
        {
            #[allow(unsafe_code)]
            {
                use crate::step_queue::{step_queues, StepQueue};
                // SAFETY: `step_queues` is a C-owned static of size
                // `N_AXIS_STEP_QUEUES` (asserted at compile time to be ≥
                // `N_AXES`); `axis_idx < N_AXES` was verified above so the
                // pointer arithmetic stays in-bounds. The volatile writes are
                // race-safe against the ISR's volatile reads — both sides
                // share the same ordering model that the ring uses normally.
                unsafe {
                    let q = step_queues
                        .get()
                        .cast::<StepQueue>()
                        .add(axis_idx as usize);
                    core::ptr::write_volatile(&mut (*q).head, 0);
                    core::ptr::write_volatile(&mut (*q).tail, 0);
                }
            }
        }

        // Step 3: SPI queue flush — Task 14 wires the actual reset.
        // Intentional no-op here so the spec sequence is preserved at
        // this call site.

        // Step 4: counter resync. Only `Pulse → Phase` needs it; the
        // Phase tick maintains `last_step_count` on the Pulse-bound
        // `axis.last_step_count` field as a side-effect, so going the
        // other direction is already in sync.
        let axis = &mut self.stepping_axes[axis_idx as usize];
        match new_mode {
            crate::stepping_state::StepMode::Phase => {
                use core::sync::atomic::Ordering;
                for stepper in &axis.steppers {
                    let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
                    let target = axis.last_step_count.wrapping_add(offset);
                    stepper.last_phase_target.store(target, Ordering::Release);
                }
            }
            crate::stepping_state::StepMode::Pulse => {
                // `last_step_count` stays valid — Phase samples maintain it.
            }
        }

        // Step 5: publish the new mode atomically.
        axis.mode
            .store(new_mode as u8, core::sync::atomic::Ordering::Release);
        KALICO_OK
    }

    /// Apply an additive target-phase offset to a single stepper.
    ///
    /// `stepper_idx` is the global index across all configured axes (sum
    /// of `axis.steppers.len()` for each axis in `0..N_AXES`).
    /// `delta_microsteps` is added to `phase_offset_target` so callers can
    /// chain incremental nudges without first reading the current target.
    /// `max_microsteps_per_sample` bounds the ramp rate the Task-13 helper
    /// applies; validated `1..=256` so a runaway value can't slam the
    /// coils between samples.
    ///
    /// `shared` is the runtime's [`SharedState`]; the method uses it only
    /// on rejection paths to latch `FaultCode::JogParametersInvalid`. The
    /// FFI projects it from `RuntimeContext::shared` (same pattern as the
    /// existing `kalico_runtime_set_step_mode` entry point).
    pub fn set_stepper_offset(
        &mut self,
        shared: &SharedState,
        stepper_idx: u8,
        delta_microsteps: i32,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        use core::sync::atomic::Ordering;

        // Zero-delta is a no-op; reject only on truly invalid parameters
        // so callers can issue repeated `set_stepper_offset` commands
        // without first reading state to detect "nothing to do."
        if delta_microsteps == 0 {
            return 0;
        }
        if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
            crate::fault_helpers::raise_jog_parameters_invalid(shared);
            return -1;
        }

        // Walk axes accumulating per-axis stepper counts until we hit the
        // requested global index. `stepping_axes.iter_mut()` so the
        // borrow-checker permits the `let stepper = &axis.steppers[..]`
        // projection without a re-borrow gymnastic.
        let mut remaining = stepper_idx as usize;
        for axis in &mut self.stepping_axes {
            if remaining < axis.steppers.len() {
                let stepper = &axis.steppers[remaining];
                let new_target = stepper
                    .phase_offset_target
                    .load(Ordering::Acquire)
                    .wrapping_add(delta_microsteps);
                stepper
                    .phase_offset_target
                    .store(new_target, Ordering::Release);
                // Task 13 owns the ramp itself; for Task 12 the only
                // contract is that `max_microsteps_per_sample` is in
                // range. Stash on `SharedState` so the ramp helper has
                // a single source of truth.
                shared
                    .max_phase_offset_ramp_per_sample
                    .store(max_microsteps_per_sample, Ordering::Release);
                return 0;
            }
            remaining -= axis.steppers.len();
        }
        // Out of range across every configured axis.
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        -1
    }

    /// Stepping-redesign Task 17 — TIM5 ISR body wrapper.
    ///
    /// Constructs a [`crate::tick::TickContext`] from engine state +
    /// `shared` and dispatches to [`crate::tick::runtime_tick_sample`],
    /// which evaluates the active per-axis Bezier piece(s), runs Newton
    /// iteration for step waketimes, and pushes step entries into the
    /// per-axis SPSC `step_queues`.
    ///
    /// Caller contract (matches `runtime_modulated_tick`): the TIM5 ISR
    /// is the sole writer of engine state under the §11 half-split
    /// borrow discipline; the foreground only mutates `axis.mode`
    /// atomically and pushes pieces via the producer's exclusive-access
    /// configure path.
    ///
    /// Returns immediately if the engine isn't yet configured
    /// (`cycles_per_second` / `sample_period_sec` not yet published by
    /// `configure_kinematics`). Without the guard, NaN sample times
    /// would propagate into the per-axis piece evaluator and latch a
    /// `MathNonFinite` fault on boot before any host segment lands.
    #[allow(clippy::cast_precision_loss)]
    pub fn tick_sample(
        &mut self,
        shared: &SharedState,
        curve_pool: &crate::curve_pool::CurvePool,
        trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
    ) {
        if self.cycles_per_second <= 0.0 {
            return;
        }
        if self.sample_period_sec <= 0.0 {
            return;
        }

        // Resolve per-axis queue pointers. On MCU builds the storage is
        // the C-declared `step_queues` symbol (one queue per axis); on
        // host/test builds, null pointers — `dispatch_axis` short-
        // circuits before any push.
        #[cfg(not(any(test, feature = "host")))]
        #[allow(unsafe_code)]
        let queue_ptrs: [*mut crate::step_queue::StepQueue;
            crate::stepping_state::N_AXES] = {
            use crate::step_queue::{StepQueue, step_queues};
            // SAFETY: `step_queues` is a C-owned static of size
            // `N_AXIS_STEP_QUEUES` (compile-time asserted ≥ N_AXES);
            // pointer arithmetic stays in-bounds for the four axis
            // indices we form here. The pointers are handed to
            // `runtime_tick_sample` which threads them through to the
            // single-producer SPSC `step_queue_push` — the TIM5 ISR is
            // the sole producer for every axis, satisfying the SPSC
            // contract documented on `step_queue::push`.
            unsafe {
                let base = step_queues.get().cast::<StepQueue>();
                [base, base.add(1), base.add(2), base.add(3)]
            }
        };
        #[cfg(any(test, feature = "host"))]
        let queue_ptrs: [*mut crate::step_queue::StepQueue;
            crate::stepping_state::N_AXES] =
            [core::ptr::null_mut(); crate::stepping_state::N_AXES];

        // Read the widened-clock low half from `SharedState`. On MCU
        // this seqlock cell is published by the producer Klipper timer
        // (`runtime_widened_host_clock` in `src/runtime_tick.c`) every
        // tick; on host/test the cell stays zero and the tick body's
        // dispatchers absorb that gracefully (zero `t_sample_end_global`
        // simply means no piece has advanced yet).
        let now_cycles = shared
            .widened_now_lo
            .load(core::sync::atomic::Ordering::Acquire);
        let cycles_per_second = self.cycles_per_second;
        let t_sample_end_global = (now_cycles as f32) / cycles_per_second;

        // Spec §4.6: Phase 3 needs the active segment (for `e_mode` /
        // `extrusion_ratio`) and the f32 snapshot of `e_accumulator`
        // taken at `arm_segment`. Snapshot them here — `tick_sample`
        // holds the exclusive borrow on `self`, and `runtime_tick_sample`
        // does not mutate `current` / `segment_base_e`, so threading the
        // reference + scalar is sound.
        let engine_segment_base_e = self.segment_base_e;
        let current_segment = self.current.as_ref();
        let mut ctx = crate::tick::TickContext {
            axes: &mut self.stepping_axes,
            queues: queue_ptrs,
            shared,
            caches: &mut self.tick_caches,
            curve_pool,
            ds_xy_segment: &mut self.ds_xy_segment,
            current_segment,
            engine_segment_base_e,
            sample_period_sec: self.sample_period_sec,
            sample_period_cycles: self.sample_period_cycles,
            cycles_per_second,
            k_xy: self.k_xy,
            advance_accel: self.advance_accel,
            advance_decel: self.advance_decel,
            now_cycles,
            t_sample_end_global,
        };
        crate::tick::runtime_tick_sample(&mut ctx);

        // Per-sample post-pass exhaustion (§4.4 + §4.5): runs after every
        // per-axis advance for this sample has happened inside
        // `runtime_tick_sample`. Updates `pending_mask`, raises
        // `PieceAdvanceUnderflow` on order-independent early exhaustion.
        self.post_pass_exhaustion(shared);
        // Phase-5 retire: fires when `pending_mask == 0` for the active
        // segment. Publishes the retirement cursor, drained-segments
        // counter, terminal-segment hook, and a `TRACE_FLAG_SEGMENT_END`
        // sample that foreground `drain_and_reclaim` consumes to retire
        // the curve-pool slots tracked in the `RetirementTable`.
        self.retire_if_complete(shared, trace);
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
        // Legacy NURBS evaluation path. Removed in stepping-redesign-finish
        // Task 12 — replaced by the cubic-piece `tick_sample` chain.
        let _ = (current, now, queue, pool, trace, shared);
        unimplemented!("removed in stepping-redesign-finish Task 12");
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
        _pool: &CurvePool,
        _stepper_idx: u8,
        _now_cycles: u64,
        _current_step: i32,
    ) -> Option<(u64, i8)> {
        // Legacy NURBS evaluation path.
        // Removed in stepping-redesign-finish Task 12 — replaced by the cubic-piece step-time path.
        unimplemented!("removed in stepping-redesign-finish Task 12");
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
        _pool: &CurvePool,
        _queue: &mut SegConsumer<Segment>,
        _shared: &SharedState,
    ) -> ProducerTickResult {
        // Legacy NURBS evaluation path.
        // Removed in stepping-redesign-finish Task 12 — replaced by per-stepper consumer body.
        unimplemented!("removed in stepping-redesign-finish Task 12");
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
        _now: u64,
        _queue: &mut SegConsumer<Segment>,
        _pool: &CurvePool,
        _trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
        _shared: &SharedState,
    ) {
        // Legacy NURBS evaluation path.
        // Removed in stepping-redesign-finish Task 12 — phase stepping now sourced from tick_sample.
        unimplemented!("removed in stepping-redesign-finish Task 12");
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

// Legacy NURBS scalar-eval helpers (scalar_eval, scalar_eval_with_derivative,
// curve_constant_value, cubic_n_pieces, cubic_piece, cubic_split_at,
// cubic_subsegment, velocity_q16_from_dx_du) deleted in stepping-redesign-
// finish Task 4. They evaluated `CurveView` (the now-removed NURBS slot
// projection) and were only consumed from `tick_with_current` / `arm_step_timer`
// / `producer_step` / `runtime_modulated_tick`, all of which are stubbed in
// this same task and deleted entirely in Task 12.
