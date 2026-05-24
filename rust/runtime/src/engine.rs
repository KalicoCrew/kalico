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

use crate::clock::{TickCounter, one_tick_cycles};
use crate::curve_pool::{CurveHandle, CurvePool};
use crate::endstop::{self, TripAction};
use crate::error::RuntimeError;
use crate::segment::Segment;
use crate::slot::{IsSlot, PaSlot};
use crate::state::SharedState;
use crate::trace::{TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_SEGMENT_END, TRACE_RING_N, TraceSample};

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
    /// Previous motor-A position (motor frame). For CoreXY MCUs this is
    /// A = X+Y (host-combined); for Cartesian MCUs it is logical X. Used
    /// for E-mode arc-length integration AND as the "hold" value when the
    /// X handle of the current segment is `UNUSED_SENTINEL`.
    prev_x: f32,
    /// Previous motor-B position (motor frame). For CoreXY MCUs this is
    /// B = X−Y (host-combined); for Cartesian MCUs it is logical Y. Same
    /// dual role as `prev_x`.
    prev_y: f32,
    /// Previous Z position (motor frame; identical to logical Z on all
    /// current MCU configurations). Same dual role as `prev_x` / `prev_y`.
    prev_z: f32,
    /// E accumulator for CoupledToXy mode — f64 for sub-step accuracy over
    /// millions of ticks (H723 has hardware double-precision FPU).
    e_accumulator: f64,
    /// Bitmask: bits 0-3 are axes A/B/Z/E. Set at `arm_segment` (Task 8)
    /// for each axis whose curve handle is non-sentinel AND which
    /// participates in retire (E in `CoupledToXy` mode is non-participating).
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

    // ─── Stepping-redesign — unified per-axis configuration ──────
    //
    // Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
    //
    // Populated by `kalico_configure_axis` / `kalico_configure_kinematics` /
    // `kalico_configure_pressure_advance` (Task 11 FFI). Consumed by the
    // unified sample-ISR tick path (Task 8).
    /// Per-logical-axis configuration: active Bezier piece, cached scalars,
    /// stepper bindings. Indexed `[X=0, Y=1, Z=2, E=3]` in logical-axis
    /// space (the unified engine performs the kinematic transform at piece
    /// activation time, not per-sample).
    pub stepping_axes: [crate::stepping_state::AxisConfig; crate::stepping_state::N_AXES],

    /// Kinematic scale factor relating logical-XY velocity to physical
    /// motor-coordinate velocity magnitude. `1.0` for Cartesian (XY motor
    /// positions equal logical XY); `1.0 / sqrt(2)` for `CoreXY` (each motor
    /// moves at `√2` times the per-axis logical speed at 45° diagonals).
    /// Consumed by the XY-arc-length integrator that feeds the E-follows-XY
    /// and pressure-advance paths. Spec §3.4.
    pub k_xy: f32,

    /// Linear pressure-advance coefficient during the toolhead's accelerating
    /// phase (s). The unified tick adds
    /// `+ advance_accel * ratio_per_xy_mm * |v_xy|` to the integrated
    /// extrusion while `v̇_xy > 0`. `0.0` disables PA on acceleration.
    /// Spec §3.5.
    pub advance_accel: f32,

    /// Linear pressure-advance coefficient during the toolhead's decelerating
    /// phase (s). Mirror of `advance_accel`; allows asymmetric `K_accel` /
    /// `K_decel` (Kalico bleeding-edge Step 9). `0.0` disables PA on
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

    /// Host/test only: per-axis StepQueue pointers installed by
    /// `Engine::test_install_step_queues` so `tick_sample`'s host branch
    /// has somewhere to push step entries. Stays at `[null; N_AXES]` on
    /// the MCU build (the field exists unconditionally to keep
    /// `init_in_place` field-writes identical on host vs. MCU).
    #[cfg(any(test, feature = "host"))]
    test_queue_ptrs: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES],
}

impl<P: PaSlot + Default, I: IsSlot + Default> Engine<P, I> {
    /// Construct a new engine for the given MCU clock frequency and TIM5 sample
    /// rate. Both values are read from C-side constants at production init time
    /// (`runtime_clock_freq` / `runtime_sample_rate_hz` in `src/runtime_tick.c`);
    /// tests supply them directly. `sample_period_sec` and `sample_period_cycles`
    /// are derived here so `tick_sample`'s `sample_period_sec <= 0.0` guard
    /// never fires in production (Codex 2026-05-20 gap #2 fix).
    pub fn new(clock_freq: u32, sample_rate_hz: u32) -> Self {
        let (sample_period_sec, sample_period_cycles) =
            Self::compute_sample_period(clock_freq, sample_rate_hz);
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
            sample_period_sec,
            sample_period_cycles,
            cycles_per_second: clock_freq as f32,
            tick_caches: crate::stepping_state::TickCaches::new(),
            #[cfg(any(test, feature = "host"))]
            test_queue_ptrs: [core::ptr::null_mut(); crate::stepping_state::N_AXES],
        }
    }

    /// Compute `(sample_period_sec, sample_period_cycles)` from raw clock and
    /// sample-rate values. Extracted so both `new` and `init_in_place` share
    /// the same arithmetic without duplicating it.
    ///
    /// `sample_rate_hz == 0` is treated as "unconfigured" and returns `(0.0, 0)`,
    /// preserving the `tick_sample` guard semantics for the zero-init edge case.
    #[inline]
    fn compute_sample_period(clock_freq: u32, sample_rate_hz: u32) -> (f32, u32) {
        if sample_rate_hz == 0 {
            return (0.0, 0);
        }
        let sec = 1.0_f32 / (sample_rate_hz as f32);
        // Integer round-nearest: (a + b/2) / b avoids f32::round() which is
        // not available on no_std MCU targets (thumbv7em-none-eabihf).
        // f32::round() compiles to a `roundf` call that requires libm or a
        // C runtime not present on bare-metal. Integer arithmetic is exact
        // for u32 inputs and has no precision loss.
        #[allow(clippy::integer_division)]
        let cycles = (clock_freq + sample_rate_hz / 2) / sample_rate_hz;
        (sec, cycles)
    }

    /// Production-context constructor. Mirrors `::new(clock_freq, sample_rate_hz)`
    /// but keeps the call site noise low (Step-6 spec §14): both C-side statics
    /// (`runtime_clock_freq`, `runtime_sample_rate_hz`) are read once at FFI init
    /// time and threaded through here.
    pub fn new_production(clock_freq: u32, sample_rate_hz: u32) -> Self {
        Self::new(clock_freq, sample_rate_hz)
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
    /// `sample_rate_hz` is the TIM5 ISR fire rate; both it and `clock_freq`
    /// come from C-side constants (`runtime_clock_freq` /
    /// `runtime_sample_rate_hz`) at production init time. Passing them here
    /// ensures `sample_period_sec` is non-zero from the first tick
    /// (Codex 2026-05-20 gap #2 fix).
    ///
    /// # Safety
    /// `ptr` must be valid for writes of `size_of::<Engine<P, I>>()` bytes
    /// and properly aligned. Caller must guarantee no concurrent reads.
    /// Used by [`crate::state::RuntimeContext::init`] to construct the
    /// engine field directly inside the C-owned `rt_storage` buffer.
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        use core::ptr::addr_of_mut;
        let (sample_period_sec, sample_period_cycles) =
            Self::compute_sample_period(clock_freq, sample_rate_hz);
        unsafe {
            addr_of_mut!((*ptr).current).write(None);
            addr_of_mut!((*ptr).last_motors).write([0.0; 4]);
            addr_of_mut!((*ptr).pa_slot).write(P::default());
            addr_of_mut!((*ptr).is_slot).write(I::default());
            addr_of_mut!((*ptr).one_tick_cycles_value)
                .write(u64::from(one_tick_cycles(clock_freq)));
            addr_of_mut!((*ptr).status).write(AtomicU8::new(RuntimeStatus::Idle as u8));
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
            addr_of_mut!((*ptr).step_state).write([crate::step::StepMotorState::default(); 4]);
            addr_of_mut!((*ptr).phase_modulators)
                .write([const { None }; crate::state::MAX_STEPPER_OIDS]);
            addr_of_mut!((*ptr).phase_tick_counter).write(0);
            addr_of_mut!((*ptr).mcu_config).write(None);
            // Task 11 unified per-axis state. Same in-place pattern —
            // `AxisConfig` contains `heapless::Vec<StepperRef, 4>` which
            // is itself non-trivially-sized.
            let axes_ptr =
                addr_of_mut!((*ptr).stepping_axes).cast::<crate::stepping_state::AxisConfig>();
            for i in 0..crate::stepping_state::N_AXES {
                axes_ptr
                    .add(i)
                    .write(crate::stepping_state::AxisConfig::new_unconfigured());
            }
            addr_of_mut!((*ptr).k_xy).write(1.0);
            addr_of_mut!((*ptr).advance_accel).write(0.0);
            addr_of_mut!((*ptr).advance_decel).write(0.0);
            addr_of_mut!((*ptr).sample_period_sec).write(sample_period_sec);
            addr_of_mut!((*ptr).sample_period_cycles).write(sample_period_cycles);
            addr_of_mut!((*ptr).cycles_per_second).write(clock_freq as f32);
            #[cfg(any(test, feature = "host"))]
            addr_of_mut!((*ptr).test_queue_ptrs)
                .write([core::ptr::null_mut(); crate::stepping_state::N_AXES]);
            addr_of_mut!((*ptr).tick_caches).write(crate::stepping_state::TickCaches::new());
        }
    }

    /// Production-context in-place init. Same contract as
    /// [`init_in_place`] with the production allocator/slot types.
    ///
    /// # Safety
    /// See [`init_in_place`].
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place_production(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        unsafe { Self::init_in_place(ptr, clock_freq, sample_rate_hz) }
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
    /// per-stepper output path (`Pulse` for GPIO STEP/DIR; `Phase` for
    /// TMC5160 XDIRECT SPI coil-current modulation). `microstep_distance`
    /// is the per-step distance in mm-equivalent units (must be finite,
    /// positive). `bindings` maps each physical stepper's TMC chip-select
    /// OID to its logical axis slot; use `TMC_CS_OID_NONE` (0xFF) for
    /// Pulse-only steppers without a TMC driver.
    ///
    /// Rejected with `KALICO_ERR_MOTION_IN_PROGRESS` when a segment is
    /// currently armed (i.e. `current.is_some()`). The configure path must
    /// only be called between segments, from the single-threaded foreground
    /// command dispatcher.
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
        bindings: &[crate::stepping_state::StepperBindingRust],
    ) -> i32 {
        use crate::error::{KALICO_ERR_INVALID_ARG, KALICO_ERR_MOTION_IN_PROGRESS, KALICO_OK};

        if (axis_idx as usize) >= crate::stepping_state::N_AXES {
            return KALICO_ERR_INVALID_ARG;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return KALICO_ERR_INVALID_ARG;
        }
        // Configuration is only legal between segments — the ISR borrows
        // `axis.steppers` and `axis.microstep_distance` for the lifetime
        // of each armed piece; mutating them mid-flight would corrupt
        // in-progress step generation.
        if self.current.is_some() {
            return KALICO_ERR_MOTION_IN_PROGRESS;
        }

        // `axis_idx as usize < N_AXES` was checked above; the indexing is
        // in-range for `stepping_axes: [_; N_AXES]`.
        #[allow(clippy::indexing_slicing)]
        let axis = &mut self.stepping_axes[axis_idx as usize];
        axis.microstep_distance = microstep_distance;
        // Clear any prior piece so the next segment-arrival path re-seeds
        // from scratch with the new microstep_distance / mode.
        axis.piece = None;
        axis.piece_start_time_cycles = 0;
        axis.last_step_count = 0;
        // Repopulate stepper bindings from the caller-supplied slice.
        // `heapless::Vec::clear` drops all existing elements and resets
        // len to 0; subsequent `push` calls fill in the new entries up
        // to `MAX_STEPPERS_PER_AXIS`. Extra bindings beyond capacity are
        // silently truncated (the hardware limit is 4 steppers per axis;
        // the host should never exceed it).
        axis.steppers.clear();
        for b in bindings {
            let tmc_cs_oid = if b.tmc_cs_oid == crate::stepping_state::TMC_CS_OID_NONE {
                None
            } else {
                Some(b.tmc_cs_oid)
            };
            let stepper = crate::stepping_state::StepperRef::new(b.stepper_oid, tmc_cs_oid);
            // `push` returns `Err` only when the Vec is full (capacity ==
            // MAX_STEPPERS_PER_AXIS); silently drop the excess rather than
            // faulting — the C host is responsible for sending a
            // well-bounded count.
            let _ = axis.steppers.push(stepper);
        }
        // Atomic publish of the mode last — ISR's Acquire load on `mode`
        // synchronizes against the plain stores above.
        axis.mode
            .store(mode as u8, core::sync::atomic::Ordering::Release);
        KALICO_OK
    }

    /// Publish kinematic scale factor relating logical-XY velocity to
    /// physical motor-coordinate velocity. `1.0` for Cartesian, `1/√2`
    /// (≈ 0.7071) for `CoreXY`. Validated finite + strictly positive — a
    /// zero / negative `k_xy` would silently zero out the XY arc-length
    /// integrator that feeds E-follows-XY and pressure-advance.
    /// Test-only: set `sample_period_sec` + `sample_period_cycles` to mirror
    /// what production would receive once Engine::init wires
    /// `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ` through. Production currently
    /// leaves both fields zero (Codex 2026-05-20 analysis: configure_kinematics
    /// only sets k_xy; nothing publishes the sample period), so `tick_sample`
    /// returns at the `sample_period_sec <= 0.0` guard before evaluating any
    /// loaded piece. Tests must call this to exercise the producer-side step
    /// pipeline on host without re-tripping that gate.
    #[cfg(any(test, feature = "host"))]
    pub fn test_set_sample_period(&mut self, sample_rate_hz: u32) {
        let sec = 1.0_f32 / (sample_rate_hz as f32);
        let cycles = (self.cycles_per_second / (sample_rate_hz as f32)).round() as u32;
        self.sample_period_sec = sec;
        self.sample_period_cycles = cycles;
    }

    /// Test-only host queue installer. Production `tick_sample` resolves
    /// `step_queues` from the C-declared array on `target_os = "none"`; on
    /// host the resolver returns `[null; N_AXES]`, which makes `dispatch_axis`
    /// short-circuit before any step entry is pushed. Tests install owned
    /// `StepQueue` instances so the producer-side pipeline becomes observable.
    /// The companion `test_queue_ptr` getter exposes them to the per-axis
    /// timer body (`kalico_per_axis_step_event`) for end-to-end drives.
    #[cfg(any(test, feature = "host"))]
    pub fn test_install_step_queues(
        &mut self,
        queues: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES],
    ) {
        self.test_queue_ptrs = queues;
    }

    #[cfg(any(test, feature = "host"))]
    pub fn test_queue_ptr(&self, axis_idx: usize) -> *mut crate::step_queue::StepQueue {
        self.test_queue_ptrs
            .get(axis_idx)
            .copied()
            .unwrap_or(core::ptr::null_mut())
    }

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
    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
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
        self.arm_segment_inner(seg, curve_pool, None)
    }

    /// 2026-05-21 production entry point that also publishes per-arm diag
    /// atomics to `shared`. Tests use the 2-arg `arm_segment`.
    pub fn arm_segment_with_diag(
        &mut self,
        seg: crate::segment::Segment,
        curve_pool: &crate::curve_pool::CurvePool,
        shared: &crate::state::SharedState,
    ) {
        self.arm_segment_inner(seg, curve_pool, Some(shared))
    }

    #[allow(unsafe_code)]
    fn arm_segment_inner(
        &mut self,
        seg: crate::segment::Segment,
        curve_pool: &crate::curve_pool::CurvePool,
        shared: Option<&crate::state::SharedState>,
    ) {
        let handles = [seg.x_handle, seg.y_handle, seg.z_handle, seg.e_handle];

        // 2026-05-21 arm diag: snapshot the X-axis input handle so the host
        // can see whether bridge sent UNUSED, a real handle, or something else.
        if let Some(shared) = shared {
            shared
                .isr_last_arm_x_handle
                .store(seg.x_handle.pack(), core::sync::atomic::Ordering::Relaxed);
        }

        // Per-axis arm. X-axis (idx 0) is also instrumented per branch.
        let mut x_outcome: u32 = 0;
        let mut x_piece_count: u32 = 0;
        for (axis_idx, handle) in handles.iter().enumerate() {
            // `handles` has 4 entries and `stepping_axes` is `[_; N_AXES]`
            // (N_AXES == 4); the index range matches by construction.
            #[allow(clippy::indexing_slicing)]
            let axis = &mut self.stepping_axes[axis_idx];
            if *handle == crate::curve_pool::CurveHandle::UNUSED_SENTINEL {
                axis.curve_handle = None;
                axis.piece = None;
                axis.piece_cursor = 0;
                if axis_idx == 0 {
                    x_outcome = 1; // UNUSED
                }
            } else if let Some(curve_ptr) = curve_pool.lookup_active(*handle) {
                // SAFETY: curve_pool's generation guard published the slot;
                // ISR is sole reader for the duration of the segment.
                let curve = unsafe { &*curve_ptr };
                if axis_idx == 0 {
                    x_piece_count = u32::from(curve.piece_count);
                    // 2026-05-21 capture pieces[0].duration f32-bits for diag.
                    // duration=0 → bernstein_to_monomial divides coeffs by
                    // 0 → inf/NaN coeffs → all-zero signed_steps downstream.
                    if curve.piece_count > 0 {
                        if let Some(shared_ref) = shared {
                            #[allow(clippy::indexing_slicing)]
                            let dbits = curve.pieces[0].duration.to_bits();
                            shared_ref
                                .isr_last_arm_x_piece0_duration_bits
                                .store(dbits, core::sync::atomic::Ordering::Relaxed);
                        }
                    }
                }
                if curve.piece_count == 0 {
                    // Defensive: `populate_from_wire` rejects empty wire so
                    // this should be unreachable, but treat as idle.
                    axis.curve_handle = None;
                    axis.piece = None;
                    axis.piece_cursor = 0;
                    if axis_idx == 0 {
                        x_outcome = 3; // piece_count == 0
                    }
                } else {
                    axis.curve_handle = Some(*handle);
                    axis.piece_cursor = 0;
                    axis.piece = Some(curve.pieces[0]);
                    axis.piece_start_time_cycles = seg.t_start;
                    if axis_idx == 0 {
                        x_outcome = 4; // OK
                    }
                }
            } else {
                // Slot generation mismatch (should be impossible — foreground
                // validated at push). Treat as idle for this axis.
                axis.curve_handle = None;
                axis.piece = None;
                axis.piece_cursor = 0;
                if axis_idx == 0 {
                    x_outcome = 2; // lookup_active miss
                }
            }
        }
        if let Some(shared) = shared {
            shared
                .isr_last_arm_x_outcome
                .store(x_outcome, core::sync::atomic::Ordering::Relaxed);
            shared
                .isr_last_arm_x_piece_count
                .store(x_piece_count, core::sync::atomic::Ordering::Relaxed);
        }

        // Compute participating_mask. Bits A/B/Z (0..2) follow handle
        // validity; bit E (3) ALSO requires e_mode == Independent.
        // `stepping_axes` is `[_; N_AXES]` with N_AXES == 4; the literal
        // indices 0..3 and 3 are in-range by construction.
        #[allow(clippy::indexing_slicing)]
        let mut participating: u8 = 0;
        #[allow(clippy::indexing_slicing)]
        for axis_idx in 0..3 {
            if self.stepping_axes[axis_idx].curve_handle.is_some() {
                participating |= 1u8 << axis_idx;
            }
        }
        #[allow(clippy::indexing_slicing)]
        if seg.e_mode == crate::config::EMode::Independent
            && self.stepping_axes[3].curve_handle.is_some()
        {
            participating |= 1u8 << 3;
        }
        self.participating_mask = participating;
        self.pending_mask = participating;

        // 2026-05-21 arm diag: snapshot final participating mask.
        if let Some(shared) = shared {
            shared.isr_last_arm_participating.store(
                u32::from(participating),
                core::sync::atomic::Ordering::Relaxed,
            );
        }

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
            // `axis_idx < N_AXES` by construction; `stepping_axes` has
            // length `N_AXES`.
            #[allow(clippy::indexing_slicing)]
            let exhausted = self.stepping_axes[axis_idx].curve_handle.is_none();
            if self.participating_mask & bit != 0 && exhausted {
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
    /// 5. Roll forward `e_accumulator` by the segment's `CoupledToXy`
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
                use crate::step_queue::{StepQueue, step_queues};
                // SAFETY: `step_queues` is a C-owned static of size
                // `N_AXIS_STEP_QUEUES` (asserted at compile time to be ≥
                // `N_AXES`); `axis_idx < N_AXES` was verified above so the
                // pointer arithmetic stays in-bounds. The volatile writes are
                // race-safe against the ISR's volatile reads — both sides
                // share the same ordering model that the ring uses normally.
                unsafe {
                    let q = step_queues.get().cast::<StepQueue>().add(axis_idx as usize);
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
        // `axis_idx as usize < N_AXES` was validated by the caller's
        // dispatcher (and re-asserted by the per-axis bounds check in
        // `configure_axis`'s sibling callers).
        #[allow(clippy::indexing_slicing)]
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
                // `remaining < axis.steppers.len()` was just checked.
                #[allow(clippy::indexing_slicing)]
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
        let queue_ptrs: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES] = {
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
        let queue_ptrs: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES] =
            self.test_queue_ptrs;

        // Read the widened-clock low half from `SharedState`. On MCU
        // this seqlock cell is published by the producer Klipper timer
        // (`runtime_widened_host_clock` in `src/runtime_tick.c`) every
        // tick; on host/test the cell stays zero and the tick body's
        // dispatchers absorb that gracefully (zero `t_sample_end_global`
        // simply means no piece has advanced yet).
        // 2026-05-21 fix: read the FULL u64 widened-now via the §11.4
        // seqlock, not just the low 32 bits. At 520 MHz on H7, u32 wraps
        // every ~8.26 s of uptime — after which `now_cycles` (treated as
        // u32) is `actual_widened_u64 % 2^32`, while `piece_start_time_cycles`
        // is a true u64. The mismatched-domain subtraction sends `t_local`
        // hugely negative, the Bezier eval extrapolates wildly, p_end
        // saturates target_step_count to i32::MAX, signed_steps explodes,
        // compute_step_times spins for ~4 seconds, IWDG fires. This single
        // u32→u64 widening is the entire missing piece. Bench debug
        // 2026-05-21: CD tag decoded t_local ≈ -92 to -130 sec (matches
        // wrap arithmetic perfectly for an MCU that has been up ~9 s).
        let now_cycles_u64 = crate::clock::read_widened_now(shared);
        let cycles_per_second = self.cycles_per_second;
        // Keep the f32 conversion for downstream consumers that compare to
        // piece_start_sec (also derived from a u64). Both are now in the
        // full-precision wall-clock domain.
        let t_sample_end_global = (now_cycles_u64 as f32) / cycles_per_second;
        // Retain `now_cycles` as the u32 low half for downstream code that
        // expects a u32 (`sample_start_cycles` in dispatch_pulse — used
        // only as a relative reference for step times within one sample
        // window, so low-32 truncation is fine there).
        #[allow(clippy::cast_possible_truncation)]
        let now_cycles = now_cycles_u64 as u32;

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
            now_cycles_u64,
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
// Production callers must use ::new(clock_freq, sample_rate_hz) — Default
// hardcodes H723 Kconfig defaults (520 MHz clock, 40 kHz sample rate).
#[cfg(test)]
impl<P: PaSlot + Default, I: IsSlot + Default> Default for Engine<P, I> {
    fn default() -> Self {
        // H723 Klipper Kconfig defaults: 520 MHz clock, 40 kHz sample rate.
        // Tests requiring specific values should call ::new() directly.
        Self::new(520_000_000, crate::clock::TICK_RATE_HZ)
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
        self.step_state
            .get(i)
            .map(|s| s.debug_steps_per_mm())
            .unwrap_or(0.0)
    }

    pub fn debug_accumulator(&self, i: usize) -> f64 {
        self.step_state
            .get(i)
            .map(|s| s.debug_accumulator())
            .unwrap_or(0.0)
    }

    /// Last observed motor position (post-PA/IS) for axis `i`.
    pub fn debug_last_motor(&self, i: usize) -> f32 {
        self.last_motors.get(i).copied().unwrap_or(0.0)
    }

    /// Last (now, t_start, duration) tuple recorded by the most recent tick.
    pub fn debug_last_timing(&self) -> (u64, u64, u64) {
        (
            self.debug_last_now,
            self.debug_last_tstart,
            self.debug_last_duration,
        )
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

    /// Seed the engine's motor-frame position. Used by tests + by
    /// the bridge's `kalico_stream_open` / `SET_KINEMATIC_POSITION` paths
    /// to anchor each motor's `StepMotorState` accumulator before the first
    /// segment runs. The unified TIM5 step path also has per-axis integer
    /// step caches (`stepping_axes[*].last_step_count`) and secant-slope
    /// position caches (`tick_caches.p_prev`); those must be seeded to the
    /// same motor-frame origin or the first absolute-position segment after
    /// SET_KINEMATIC_POSITION is interpreted as a huge catch-up move.
    /// Without this, `runtime_modulated_tick` on a non-origin segment
    /// computes a spurious motor-delta = (segment_start - 0) on its first
    /// tick and emits thousands of catch-up step pulses.
    ///
    /// **`xyz` is in motor frame** — the kinematic transform (CoreXY
    /// A=X+Y / B=X−Y) is applied by the bridge before calling this
    /// function. The MCU engine is motor-frame end-to-end; there is no
    /// kinematic transform in the hot path or in this seed path.
    pub fn seed_position(&mut self, xyz: [f32; 3]) {
        self.prev_x = xyz[0];
        self.prev_y = xyz[1];
        self.prev_z = xyz[2];
        self.needs_xy_seed = false;
        // Motor-frame positions directly — no kinematic transform on the MCU.
        let motor_positions = [xyz[0], xyz[1], xyz[2], 0.0_f32];
        for i in 0..4 {
            if let Some(ss) = self.step_state.get_mut(i) {
                ss.seed(motor_positions[i]);
            }
        }
        self.last_motors = motor_positions;
        self.tick_caches.p_prev = motor_positions;
        self.tick_caches.v_prev = [0.0; 4];
        self.tick_caches.v_xy_prev = 0.0;
        self.tick_caches.v_xy_this = 0.0;
        self.tick_caches.vdot_xy_accelerating = false;

        for (axis, &axis_pos_mm) in self.stepping_axes.iter_mut().zip(motor_positions.iter()) {
            let microstep_distance = axis.microstep_distance;
            if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let seed_steps = libm::roundf(axis_pos_mm / microstep_distance) as i32;
            axis.last_step_count = seed_steps;
            for stepper in &axis.steppers {
                stepper.position_count.store(seed_steps, Ordering::Release);
                stepper
                    .last_phase_target
                    .store(seed_steps, Ordering::Release);
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

    /// Synchronous foreground flush. Spec §3.10.
    ///
    /// Drains the segment queue and retires every curve-pool slot the
    /// dropped segments referenced. After this returns, the engine is in
    /// the "fresh, no work" state.
    ///
    /// **Caller contract:** must guarantee no concurrent `Engine::tick_sample`
    /// or per-axis consumer access. The host-side flush path serialises
    /// through the bridge command channel before invoking this. The FFI
    /// wrapper (`kalico_runtime_force_idle`) inherits the same contract and
    /// is the single legitimate caller.
    pub fn runtime_force_idle(
        &mut self,
        pool: &CurvePool,
        queue: &mut SegConsumer<Segment>,
        shared: &SharedState,
    ) {
        // 1. Disarm producer kicks. Any kick that lands mid-flush
        //    re-CASes; producer_pending stays false until the next
        //    legitimate push or low-water hook.
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

        // 3. Reset per-motor `StepAccumulator` residual. The
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

        // 4. Drop the per-motor phase-stepping state. The first phase
        //    tick after a re-stream re-seeds the modulator from the
        //    freshly-anchored motor position. `phase_tick_counter`
        //    resets so the round-robin SPI schedule restarts at
        //    ordinal 0; for a deterministic re-stream that's the right
        //    behaviour.
        for slot in &mut self.phase_modulators {
            *slot = None;
        }
        self.phase_tick_counter = 0;

        // 5. Clear current-segment / position-seed state.
        self.clear_current();
        self.last_motors = [0.0; 4];
        self.prev_x = 0.0;
        self.prev_y = 0.0;
        self.prev_z = 0.0;

        // 6. Re-publish settled engine status. After force_idle the
        //    host either re-streams (transitions Idle → Running on
        //    next segment activation) or stays idle, so `Idle` is the
        //    most accurate post-flush state. Fault status is NOT
        //    cleared — the host explicitly inspects the fault before
        //    issuing flush; clearing it would mask the failure history.
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }

        // 7. Transition gesture for the `acked_force_idle` polling
        //    path (e.g., `runtime::stream::flush`'s spin loop).
        shared.acked_force_idle.store(true, Ordering::Release);
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
    /// `active_segment` is the caller's stack-owned segment, if any.
    /// Together with `self.current` it covers every place a segment can be
    /// in flight at trip time.
    ///
    /// All `confirm_retired` calls are no-ops on `UNUSED_SENTINEL`
    /// (hold-segment branches) and on `HOLD_SEGMENT_SENTINEL`, so callers
    /// from any trip site can share the same path.
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
        if let Some(seg) = self.current.take() {
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
        self.last_error
            .store(i32::from(RuntimeError::HomingTrip), Ordering::Release);
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

    /// Endstop trip evaluator. Called per-sample from the unified tick
    /// path; on `AbortNow`, hands off to `abort_for_homing_trip`.
    /// Velocity-gated policies receive `[u32::MAX; 3]` here: precise
    /// per-axis velocity hooks at step resolution are a follow-up.
    pub fn poll_endstop_trip(
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

    /// Enqueue a segment into the runtime's segment queue.
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
}
