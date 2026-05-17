//! Half-split runtime state per Step-6 spec §11.
//!
//! `FgState` is touched only from foreground command-dispatch.
//! `IsrState` is touched only from the TIM5 ISR.
//! `SharedState` is touched concurrently from both via atomics only.
//!
//! Discipline contract: code-review-enforced. No compiler check. The TIM5
//! ISR is the SOLE writer of `IsrState`; any other interrupt that needs MCU
//! state goes through `SharedState` atomics.
//!
//! `TickState` is the per-tick struct shared with the PA/IS slot pipeline
//! and predates the half-split — it stays in this module.

// The half-split FFI projection (see `init` below) needs raw-pointer writes
// through `MaybeUninit` and an `unsafe impl Sync` for `RuntimeContext`; the
// foreign symbol declarations for `runtime_clock_freq` / `irq_save` /
// `irq_restore` also require `unsafe extern "C"`. Workspace lints deny
// `unsafe_code` globally — this module is one of two places in `runtime`
// (alongside `curve_pool::load_unchecked`) where we opt out, with the
// rationale documented inline.
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32};
// `AtomicU64` comes from `portable-atomic` because thumbv7em-none-eabi[hf]
// (Cortex-M7) lacks native 64-bit CAS — `core::sync::atomic::AtomicU64` is
// not provided on that target. `portable-atomic`'s `fallback` feature
// implements u64 atomics via a critical section, which is correct for our
// usage (counters bumped from the producer/consumer paths and read by the
// foreground). API is drop-in compatible with `core::sync::atomic::AtomicU64`.
use portable_atomic::AtomicU64;

use heapless::spsc::{Consumer, Producer, Queue};

use crate::clock::WidenState;
use crate::curve_pool::CurvePool;
use crate::engine::Engine;
use crate::queue::Q_N;
use crate::segment::Segment;
use crate::slot::{NoopIs, NoopPa};
use crate::trace::{TRACE_RING_N, TraceSample};

/// Per-stepper stepping-output strategy. Stored as `AtomicU8` in
/// `SharedState::step_modes`; runtime-mutable via `runtime_set_step_mode`.
///
/// Spec: docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md §3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepMode {
    /// Driven by TIM5 ISR at the MCU's modulation rate. Current behavior
    /// (polled curve eval + `StepAccumulator`). Future: grows to include
    /// sin/cos commutation per build-order Step 10.
    Modulated = 0,
    /// Driven by per-stepper Klipper `struct timer`. Engine computes each
    /// step's firing time via Newton iteration on the position polynomial.
    /// Default for all steppers; mandatory on MCUs that don't advertise
    /// the `PHASE_STEPPING` capability bit.
    StepTime = 1,
}

impl StepMode {
    pub fn from_u8(raw: u8) -> Option<StepMode> {
        match raw {
            0 => Some(StepMode::Modulated),
            1 => Some(StepMode::StepTime),
            _ => None,
        }
    }
}

/// Per-MCU stepper oid counter slots for homing snapshots.
///
/// The MVP firmware owns at most four motors per MCU today; eight slots keep
/// room for AWD pairs and match `endstop::MAX_STEPPERS` without growing the
/// ISR hot-path scan beyond a small fixed array.
pub const MAX_STEPPER_OIDS: usize = 8;

/// Per-tick state shared with PA/IS slots. Spec §3.1.
#[derive(Debug, Clone, Copy)]
pub struct TickState {
    pub dt: f32,
    pub positions: [f32; 4], // [x, y, z, e] logical
    pub motors: [f32; 4],    // [a, b, z, e] or [x, y, z, e] post-kinematic
}

/// Production `Engine` instantiation: ZST PA/IS slots per Step-5 spec §3.1.
///
/// Step 8 (smooth shapers) and Step 9 (tanh PA) replace these with real impls.
/// Round-2 fix B1: Engine is generic, so the FFI shim refers to this typedef
/// rather than spelling out the slot types at every call site.
pub type EngineImpl = Engine<NoopPa, NoopIs>;

// Phase 7 §8.5 force_idle handshake calls Klipper's `irq_save`/`irq_restore`
// to gate the queue-drain step. We import via thin C wrappers
// `runtime_irq_save` / `runtime_irq_restore` defined in `src/runtime_tick.c`,
// not the bare functions directly: Klipper's MCU build uses
// `-flto=auto -fwhole-program`, which empirically lets the LTO/whole-program
// inliner DCE the standalone `irq_save` / `irq_restore` symbols (their
// bodies get inlined at every Klipper callsite) — leaving the kalico
// staticlib's references unresolvable at the final link. The wrappers are
// marked `used + externally_visible` on the C side so LTO keeps them.
#[allow(dead_code)]
unsafe extern "C" {
    pub fn runtime_irq_save() -> u32;
    pub fn runtime_irq_restore(flags: u32);
}

/// Foreground-only state. Touched exclusively by the command-dispatch task
/// (`kalico_runtime_push_segment`, `kalico_runtime_load_curve`,
/// `kalico_runtime_drain_trace`, …).
#[allow(missing_debug_implementations)] // Producer/Consumer don't implement Debug.
pub struct FgState {
    pub queue_producer: Producer<'static, Segment, Q_N>,
    pub trace_consumer: Consumer<'static, TraceSample, TRACE_RING_N>,
    pub stream_state_machine: crate::stream::FgStreamState,
    /// Stream-open identity tracking for §8.5 idempotency (same-`stream_id` rule).
    pub current_stream_id: Option<u32>,
    /// Arm-time idempotency (§8.5: arm with same `t_start_t0` returns OK).
    pub armed_t_start_t0: Option<u64>,
    /// Round-2 fix B6: foreground tracks the FIRST priming segment's
    /// `t_start` at push-acceptance time. `arm()` reads from here (not from
    /// the ISR-owned queue) per §6.3 + §11.1 SPSC ownership discipline.
    pub first_priming_segment_t_start: Option<u64>,
    /// Set by §8.3 `kalico_stream_terminal` handler; consumed by the ISR
    /// retire path (cross-half via `SharedState` atomics).
    pub terminal_segment_id: Option<u32>,
    /// Used by §8.5 flush spin-wait deadline computation.
    pub flush_start_tick: Option<u64>,
    /// Multi-handle retirement table: maps `segment_id → [CurveHandle; 4]`
    /// so the trace-drain pipeline can retire all 4 per-axis curve slots on
    /// a single `SEGMENT_END` observation. Populated at push time.
    pub retirement_table: crate::reclaim::RetirementTable,
}

/// ISR-only state. Touched exclusively by the TIM5 ISR.
#[allow(missing_debug_implementations)] // Producer/Consumer don't implement Debug.
pub struct IsrState {
    pub queue_consumer: Consumer<'static, Segment, Q_N>,
    pub trace_producer: Producer<'static, TraceSample, TRACE_RING_N>,
    pub engine: EngineImpl,
    /// CYCCNT widening lives here from Phase 1 onward (Round-3 fix B-R3-4):
    /// the ownership move out of `Engine` is what lets the foreground read
    /// the widened `now` via the §11.4 seqlock instead of reaching into the
    /// engine. The ISR is the sole writer; foreground must not call
    /// `widen()` directly.
    pub widen_state: WidenState,
}

impl IsrState {
    /// Return a raw const pointer to the `IsrState` inside `ctx`.
    ///
    /// Used by `engine::arm_step_timer_for_stepper` to form a shared reference
    /// to the ISR-owned engine and curve-pool state without taking `&mut
    /// IsrState`. The caller is responsible for the aliasing discipline (see
    /// the function's SAFETY doc).
    pub fn raw_ref_from_ctx(ctx: &RuntimeContext) -> *const Self {
        // `addr_of!` does not form a reference; `raw_get` returns a raw
        // pointer from the UnsafeCell without any unsafe operation.
        // The caller is responsible for upholding the non-exclusive borrow
        // invariant when it dereferences the returned pointer.
        core::cell::UnsafeCell::raw_get(core::ptr::addr_of!(ctx.isr))
    }
}

/// Cross-half shared state. Atomics only; no `&mut` reaches this struct.
#[derive(Debug)]
pub struct SharedState {
    // Step-5 carryover.
    pub last_error: AtomicI32,
    pub runtime_status: AtomicU8,
    // Step-6: stream lifecycle (§8).
    pub stream_open: AtomicBool,
    // Step-6: flush handshake (Plan-decision A — foreground sets force_idle
    // FIRST, ack-waits, THEN clears stream_open).
    pub force_idle: AtomicBool,
    pub acked_force_idle: AtomicBool,
    // Step-6: §11.4 widened-clock seqlock — foreground reads, ISR writes.
    pub widened_now_lo: AtomicU32,
    pub widened_now_hi: AtomicU32,
    pub widened_now_seq: AtomicU32,
    // Step-6: §13.1 trace-overflow latch (ISR sets, foreground latches fault).
    pub sample_drop_pending: AtomicBool,
    // Step-6: cross-half cursors (foreground reads ISR-published values).
    pub current_segment_id: AtomicU32,
    pub credit_epoch: AtomicU32,
    pub accepted_segment_id: AtomicU32,
    pub retired_through_segment_id: AtomicU32,
    /// 2026-05-17 F4-retire-stall diagnostic: low 32 bits of the most
    /// recent `now - seg.t_start` value computed inside
    /// `runtime_modulated_tick`. Exposed via fault_detail tag 0xFB. If
    /// this stays at 0 while a segment is queued, the engine's clock
    /// (`now` from `runtime_widened_host_clock`) is stuck or behind
    /// `seg.t_start`, so the `elapsed >= duration` retirement check
    /// can never fire.
    pub last_modulated_elapsed_lo: AtomicU32,
    /// Companion to `last_modulated_elapsed_lo`: low 32 bits of the
    /// active segment's `duration()`. Exposed via fault_detail tag 0xFC.
    /// Comparing tag 0xFB ≥ tag 0xFC tells us whether the retirement
    /// branch should fire on the next tick.
    pub last_modulated_duration_lo: AtomicU32,
    /// 2026-05-17 F4 retire-stall diagnostic: increments on every entry
    /// into `runtime_modulated_tick`'s `elapsed >= duration` branch.
    /// If this stays 0, retirement isn't being reached (clock not
    /// advancing past t_start + duration). If > 0 but
    /// `retired_through_segment_id` stays 0, the retirement branch
    /// enters but consumers_done returns false (motor bits not cleared).
    pub modulated_retire_attempts: AtomicU32,
    /// Increments on every successful retirement (consumers_done == true
    /// path). Should equal `modulated_retire_attempts` if the consumers
    /// mask is being cleared correctly.
    pub modulated_retire_successes: AtomicU32,
    /// 2026-05-17: snapshot of `seg.consumers_remaining` AFTER the
    /// clear-all-motors loop in modulated_tick's retirement branch.
    /// If non-zero, the clear loop missed bits that compute_consumers_remaining
    /// set — the remaining bits tell us which positions need investigation.
    pub last_retire_consumers_after_clear: AtomicU32,
    /// §9.2 + §5.3 — last latched fault's encoded `fault_detail` payload.
    /// Set in lockstep with `last_error` whenever a fault latches; read by
    /// the periodic 10 Hz `kalico_status_v6` frame and the async
    /// `kalico_fault` event so the host can decode the fault context
    /// (slot index, observed/expected generation, etc.) per spec §9.2.
    /// `0` when no fault has latched OR when the fault carries no
    /// per-event detail.
    pub fault_detail: AtomicU32,
    // Step-6: terminal-segment communication foreground → ISR (§8.3).
    // Foreground sets `terminal_segment_id_set` true + `terminal_segment_id_value`
    // to the segment id from `kalico_stream_terminal`; the ISR retire path
    // checks the flag + value and clears `stream_open` when matched. Both are
    // cleared on flush / new stream_open.
    pub terminal_segment_id_set: AtomicBool,
    pub terminal_segment_id_value: AtomicU32,
    // Step-6: foreground sees the most-recently-accepted `segment_id` (in
    // addition to `accepted_segment_id` which is the cumulative accept
    // cursor) so duplicate-id rejection can short-circuit on the hot path.
    pub accepted_segment_id_seen: AtomicBool,
    // Step 7-D: signed per-stepper pulse counters, indexed by stepper oid.
    pub stepper_counts: [AtomicI32; MAX_STEPPER_OIDS],
    /// Per-stepper `StepMode` (spec §5). Atomic so the host can flip a
    /// stepper between Modulated and StepTime at runtime (needed for future
    /// sensorless homing on phase-stepped axes — TMC StallGuard requires
    /// the driver's internal sequencer, which the direct/phase-stepping
    /// path bypasses). Default `StepTime` (enum value 1).
    pub step_modes: [AtomicU8; MAX_STEPPER_OIDS],

    // ─── Step 7-emission (Task 5) diagnostics ─────────────────────────────
    // Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md §6.
    /// Dedupe flag for producer-timer kicks. Kickers (push_segment + the
    /// per-motor consumer low-water hook) CAS-set this `false→true`; the
    /// CAS-winner schedules the producer struct timer. The producer clears
    /// this on entry. A spurious double-kick is benign (producer runs once
    /// more with little work to do).
    pub producer_pending: AtomicBool,
    /// Number of times `Engine::producer_step` has entered. Replaces the
    /// pre-emission-rewrite `Engine::tick_counter` diagnostic; host
    /// telemetry plots this as the producer heartbeat.
    pub producer_runs_total: AtomicU64,
    /// Per-motor consumer step-pulse counter. Matches `stepper_counts`
    /// cumulatively; surfaced as a sanity-check that the consumer is
    /// firing the entries the producer pushed.
    pub consumer_pulses_total: [AtomicU64; 4],
    /// Per-motor consumer underrun count. Bumped when the per-stepper
    /// consumer timer wakes up to find its ring empty (the poll-cadence
    /// fallback in spec §3.5). A non-zero value means the producer is
    /// falling behind for that motor — either insufficient ring depth
    /// at the natural step rate or a foreground stall hiding kicks.
    pub consumer_underrun_total: [AtomicU64; 4],
    /// Per-motor peak `available` ever observed in the StepRing. Surfaces
    /// how close we are to ring-full back-pressure during steady state.
    pub ring_high_water: [AtomicU32; 4],
    /// Total successful `ring.push` calls across all motors in
    /// `Engine::producer_step`. If `producer_runs_total` is growing but
    /// this stays at 0, the producer is running but every motor either
    /// hits SegmentExhausted on first Cardano call OR `fetch_segment_for_motor`
    /// returns None.
    pub producer_steps_pushed_total: AtomicU64,
    /// Total times `compute_next_step_time` returned `SegmentExhausted` in
    /// `Engine::producer_step` (per motor, summed across motors). Tells us
    /// whether Cardano is finding zero roots immediately.
    pub producer_motor_finished_curve_total: AtomicU64,
    /// Total times a full segment was retired (all consumers_remaining
    /// bits cleared) by `Engine::producer_step`. Increments once per
    /// dequeued+retired segment.
    pub producer_segment_retired_total: AtomicU64,
    /// Total times `Engine::producer_step`'s queue.dequeue returned Some
    /// (a segment was pulled into `producer_current`). Cross-check against
    /// host-side PushSegment count to detect dropped frames between
    /// `runtime_handle_push_segment` and `queue.enqueue`.
    pub producer_segment_dequeued_total: AtomicU64,
    /// Total times `Engine::fetch_segment_for_motor` was called. Bumps
    /// unconditionally at function entry — distinguishes "producer loop
    /// is filtering out all motors at the gates" from "fetch is called
    /// but queue.dequeue always returns None".
    pub producer_fetch_attempts_total: AtomicU64,
    /// Total times `push_segment_impl` reached its successful enqueue.
    /// Bumps AFTER `fg.queue_producer.enqueue(seg)` returns Ok. If this
    /// is non-zero while `producer_segment_dequeued_total` is 0, the
    /// queue's enqueue/dequeue ends aren't sharing the backing buffer.
    pub producer_enqueue_success_total: AtomicU64,
    /// Last result code returned from `push_segment_impl`. 0 = KALICO_OK,
    /// negative = an error path (see error.rs constants). Set on every
    /// call so the C-side diag can show which rejection path is firing.
    pub last_push_segment_result: AtomicI32,
    /// Count of times `pool.resolve(primary)` returned `Some` in
    /// `Engine::producer_step` (i.e. the primary curve handle was valid
    /// and the slot's generation matched). Cross-check against
    /// `producer_fetch_attempts_total`: if the latter is non-zero but
    /// this is 0, every primary handle is either UNUSED or the slot's
    /// generation has been retired without a matching reload.
    pub producer_primary_resolved_total: AtomicU64,
    /// Count of times `pool.resolve(primary)` returned `None` AND the
    /// handle was NOT the UNUSED sentinel. Distinguishes "host sent
    /// a real handle but the pool no longer has it" from "host sent
    /// UNUSED on purpose."
    pub producer_primary_stale_total: AtomicU64,
    /// Count of times `primary.is_unused_sentinel()` was true. The
    /// natural case for a stationary-axis segment.
    pub producer_primary_unused_total: AtomicU64,
    /// 2026-05-15 live diagnosis: count of `push_segment_impl` calls
    /// where the computed `consumers_remaining` mask is zero (i.e. every
    /// handle was UNUSED). Such segments retire on the producer's very
    /// first dequeue without ever invoking the motor processing path.
    /// If this counter advances during a jog, the bridge is sending
    /// no-handle segments to the MCU.
    pub push_segment_all_unused_total: AtomicU64,
    /// Last `x_handle.pack()` value observed by `push_segment_impl`. Used
    /// only for live diagnosis; not part of any production invariant.
    pub last_push_x_handle_packed: AtomicU32,
    /// Last `y_handle.pack()` value observed by `push_segment_impl`.
    pub last_push_y_handle_packed: AtomicU32,
    /// Last `consumers_remaining` mask computed by `push_segment_impl`.
    pub last_push_consumers_remaining: AtomicU32,
    /// 2026-05-15 live diagnosis (CP capture): cps[0] (start control
    /// point, mm) of the last resolved primary X curve, raw f32 bits.
    /// Captured in producer_step right after `pool.resolve(primary)`
    /// returns Some. For a 0.5 mm X jog starting at X=125.0, this should
    /// be 125.0 (= 0x42FA0000). If the bits look corrupted or constant,
    /// the curve_pool's slot contents have been corrupted on the wire.
    pub last_resolved_primary_cps_0: AtomicU32,
    /// Last resolved primary X curve's cps[3] (end control point, mm),
    /// raw f32 bits. For a 0.5 mm X jog starting at X=125.0, this should
    /// be 125.5 (= 0x42FB0000). Compare with cps_0 — if they match, the
    /// curve has zero displacement and the producer correctly returns
    /// `SegmentExhausted` (which would indicate a planner-side bug, not
    /// MCU corruption).
    pub last_resolved_primary_cps_3: AtomicU32,
    /// CoreXY-combined cps[0] after `kine.combine` for motor A
    /// (A = X + Y). f32 bits. Compare with last_resolved_primary_cps_0
    /// to detect kinematic-mixing bugs (e.g. Y curve resolves to a
    /// non-constant when it should be constant for a pure-X jog).
    pub last_combined_motor_a_cps_0: AtomicU32,
    /// CoreXY-combined cps[3] after `kine.combine` for motor A. f32 bits.
    pub last_combined_motor_a_cps_3: AtomicU32,
}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            last_error: AtomicI32::new(0),
            runtime_status: AtomicU8::new(crate::engine::RuntimeStatus::Idle as u8),
            stream_open: AtomicBool::new(false),
            force_idle: AtomicBool::new(false),
            acked_force_idle: AtomicBool::new(false),
            widened_now_lo: AtomicU32::new(0),
            widened_now_hi: AtomicU32::new(0),
            widened_now_seq: AtomicU32::new(0),
            sample_drop_pending: AtomicBool::new(false),
            current_segment_id: AtomicU32::new(0),
            last_modulated_elapsed_lo: AtomicU32::new(0),
            last_modulated_duration_lo: AtomicU32::new(0),
            modulated_retire_attempts: AtomicU32::new(0),
            modulated_retire_successes: AtomicU32::new(0),
            last_retire_consumers_after_clear: AtomicU32::new(0),
            credit_epoch: AtomicU32::new(0),
            accepted_segment_id: AtomicU32::new(0),
            retired_through_segment_id: AtomicU32::new(0),
            fault_detail: AtomicU32::new(0),
            terminal_segment_id_set: AtomicBool::new(false),
            terminal_segment_id_value: AtomicU32::new(0),
            accepted_segment_id_seen: AtomicBool::new(false),
            stepper_counts: [
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
            ],
            step_modes: [
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
            ],
            producer_pending: AtomicBool::new(false),
            producer_runs_total: AtomicU64::new(0),
            consumer_pulses_total: [
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
            ],
            consumer_underrun_total: [
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
            ],
            ring_high_water: [
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
            ],
            producer_steps_pushed_total: AtomicU64::new(0),
            producer_motor_finished_curve_total: AtomicU64::new(0),
            producer_segment_retired_total: AtomicU64::new(0),
            producer_segment_dequeued_total: AtomicU64::new(0),
            producer_fetch_attempts_total: AtomicU64::new(0),
            producer_enqueue_success_total: AtomicU64::new(0),
            last_push_segment_result: AtomicI32::new(0),
            producer_primary_resolved_total: AtomicU64::new(0),
            producer_primary_stale_total: AtomicU64::new(0),
            producer_primary_unused_total: AtomicU64::new(0),
            push_segment_all_unused_total: AtomicU64::new(0),
            last_push_x_handle_packed: AtomicU32::new(0),
            last_push_y_handle_packed: AtomicU32::new(0),
            last_push_consumers_remaining: AtomicU32::new(0),
            last_resolved_primary_cps_0: AtomicU32::new(0),
            last_resolved_primary_cps_3: AtomicU32::new(0),
            last_combined_motor_a_cps_0: AtomicU32::new(0),
            last_combined_motor_a_cps_3: AtomicU32::new(0),
        }
    }
}

impl Default for SharedState {
    fn default() -> Self {
        Self::new()
    }
}

/// Step-6 half-split runtime context. Replaces Step-5's monolithic struct.
///
/// Layout invariants:
/// - `fg` and `isr` are `UnsafeCell<…>` so the FFI shim can project to either
///   half-state via `core::ptr::addr_of!` + `UnsafeCell::raw_get` without
///   ever materializing `&mut RuntimeContext`. Spec §11.2 closes the Step-5
///   latent UB by ensuring at most ONE `&mut FgState` OR `&mut IsrState`
///   (disjoint memory) exists at a time.
/// - `shared` is plain (no `UnsafeCell`); all writes go through atomics.
/// - `curve_pool` is at the top level: foreground writes via
///   `try_alloc_and_load`; the ISR reads via `lookup`. Per-slot atomics
///   guard concurrent access (Phase 2 §10.2 + Round-1 Codex #4 — see
///   `curve_pool::PoolSlot`). Spec §10.5.
/// - `queue_storage` / `trace_storage` are wrapped in `UnsafeCell` so that
///   `init` can split them into `Producer`/`Consumer` halves and store the
///   halves into `FgState`/`IsrState` while keeping the backing storage
///   alive at `'static`.
#[allow(missing_debug_implementations)] // Inner half-states wrap non-Debug types.
pub struct RuntimeContext {
    /// Foreground-only half. Reach via `addr_of!((*ctx).fg)` →
    /// `UnsafeCell::raw_get` → `&mut FgState`. Spec §11.2.
    pub fg: UnsafeCell<FgState>,
    /// ISR-only half. Same projection pattern as `fg`. Spec §11.2.
    pub isr: UnsafeCell<IsrState>,
    /// Cross-half atomics. Read/written through `&SharedState` (atomics
    /// supply the synchronization). Spec §11.3.
    pub shared: SharedState,
    /// Top-level curve slab. Foreground writer / ISR reader; per-slot
    /// generation atomics arrive in Phase 2. Spec §10.5.
    pub curve_pool: CurvePool,
    /// Backing storage for the segment SPSC. Split into `Producer` /
    /// `Consumer` halves at `init` time and stored on `FgState` /
    /// `IsrState` respectively.
    pub queue_storage: UnsafeCell<Queue<Segment, Q_N>>,
    /// Backing storage for the trace SPSC. Same split pattern as
    /// `queue_storage`.
    pub trace_storage: UnsafeCell<Queue<TraceSample, TRACE_RING_N>>,
}

// SAFETY: see discipline contract above. `CurvePool` carries its own
// per-slot atomics from Phase 2 onward; for Phase 1 the foreground is the
// only writer and the ISR's read access is gated by the §11 ownership
// discipline. `UnsafeCell` is `!Sync` by default — we provide `Sync` for
// `RuntimeContext` because the FFI shim only ever forms shared `&` and
// projects to disjoint `&mut FgState` / `&mut IsrState` via raw pointers,
// never `&mut RuntimeContext`.
unsafe impl Sync for RuntimeContext {}

unsafe extern "C" {
    /// C-side static, set at `runtime_init` time in `src/runtime_tick.c`.
    static runtime_clock_freq: u32;
}

impl RuntimeContext {
    /// Initializes the runtime context in place at `rt_ptr`.
    ///
    /// SAFETY: the caller must guarantee single-threaded init before any FFI
    /// call hits the runtime (the FFI shim enforces this via a one-shot
    /// `AtomicBool` guard on `kalico_runtime_init`). This function writes
    /// through raw-pointer projections; it never materializes `&mut
    /// RuntimeContext` and is sound to call against `MaybeUninit::as_mut_ptr()`.
    pub unsafe fn init(rt_ptr: *mut RuntimeContext) {
        // SAFETY: caller guarantees `rt_ptr` is valid for writes and
        // exclusively-owned for the duration of init. We only form raw
        // pointers to fields and never a `&mut RuntimeContext`.
        unsafe {
            // Initialize queue storage and split into producer + consumer.
            let queue_storage_ptr = core::ptr::addr_of_mut!((*rt_ptr).queue_storage);
            queue_storage_ptr.write(UnsafeCell::new(Queue::new()));
            let queue_ref: &'static mut Queue<Segment, Q_N> = &mut *(*queue_storage_ptr).get();
            let (q_producer, q_consumer) = queue_ref.split();

            // Initialize trace storage and split.
            let trace_storage_ptr = core::ptr::addr_of_mut!((*rt_ptr).trace_storage);
            trace_storage_ptr.write(UnsafeCell::new(Queue::new()));
            let trace_ref: &'static mut Queue<TraceSample, TRACE_RING_N> =
                &mut *(*trace_storage_ptr).get();
            let (t_producer, t_consumer) = trace_ref.split();

            // Initialize SharedState.
            let shared_ptr = core::ptr::addr_of_mut!((*rt_ptr).shared);
            shared_ptr.write(SharedState::new());

            // Initialize CurvePool at top level.
            let pool_ptr = core::ptr::addr_of_mut!((*rt_ptr).curve_pool);
            pool_ptr.write(CurvePool::new());

            // Read C-side clock frequency once so the ISR's WidenState +
            // Engine::new_production both see the same value. `runtime_clock_freq`
            // is set at static-init time before the runtime ever runs.
            let freq = core::ptr::read_volatile(core::ptr::addr_of!(runtime_clock_freq));

            // Initialize FgState.
            let fg_ptr = core::ptr::addr_of_mut!((*rt_ptr).fg);
            fg_ptr.write(UnsafeCell::new(FgState {
                queue_producer: q_producer,
                trace_consumer: t_consumer,
                stream_state_machine: crate::stream::FgStreamState::Idle,
                current_stream_id: None,
                armed_t_start_t0: None,
                first_priming_segment_t_start: None,
                terminal_segment_id: None,
                flush_start_tick: None,
                retirement_table: crate::reclaim::RetirementTable::new(),
            }));

            // Initialize IsrState.
            let isr_ptr = core::ptr::addr_of_mut!((*rt_ptr).isr);
            isr_ptr.write(UnsafeCell::new(IsrState {
                queue_consumer: q_consumer,
                trace_producer: t_producer,
                engine: EngineImpl::new_production(freq),
                widen_state: WidenState::default(),
            }));
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetStepModeError {
    /// Requested `StepMode::Modulated` on an MCU whose capability bitmap
    /// does not advertise `PHASE_STEPPING`. Spec §4.
    CapabilityMissing,
    /// `stepper_idx >= MAX_STEPPER_OIDS`.
    OutOfRange,
}

/// Atomically flip a stepper's `StepMode`. Enforces the capability
/// ceiling: `Modulated` is rejected if the MCU doesn't advertise the
/// phase-stepping bit. Spec §10.
///
/// `Release` ordering on the store pairs with `Acquire` loads in the
/// engine ISR and `count_modulated_steppers` foreground reads.
pub fn set_step_mode(
    shared: &SharedState,
    stepper_idx: u8,
    mode: StepMode,
    mcu_supports_phase: bool,
) -> Result<(), SetStepModeError> {
    if (stepper_idx as usize) >= MAX_STEPPER_OIDS {
        return Err(SetStepModeError::OutOfRange);
    }
    if mode == StepMode::Modulated && !mcu_supports_phase {
        return Err(SetStepModeError::CapabilityMissing);
    }
    shared.step_modes[stepper_idx as usize].store(mode as u8, core::sync::atomic::Ordering::Release);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_state_default_is_idle() {
        let s = SharedState::new();
        assert_eq!(
            s.runtime_status.load(core::sync::atomic::Ordering::Relaxed),
            crate::engine::RuntimeStatus::Idle as u8
        );
        assert!(!s.stream_open.load(core::sync::atomic::Ordering::Relaxed));
        assert!(!s.force_idle.load(core::sync::atomic::Ordering::Relaxed));
    }

    #[test]
    fn shared_state_default_widened_now_zero() {
        let s = SharedState::new();
        assert_eq!(
            s.widened_now_lo.load(core::sync::atomic::Ordering::Relaxed),
            0
        );
        assert_eq!(
            s.widened_now_hi.load(core::sync::atomic::Ordering::Relaxed),
            0
        );
        assert_eq!(
            s.widened_now_seq
                .load(core::sync::atomic::Ordering::Relaxed),
            0
        );
    }
}
