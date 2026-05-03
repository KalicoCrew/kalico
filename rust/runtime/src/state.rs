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
// foreign symbol declarations for `kalico_clock_freq` / `irq_save` /
// `irq_restore` also require `unsafe extern "C"`. Workspace lints deny
// `unsafe_code` globally — this module is one of two places in `runtime`
// (alongside `curve_pool::load_unchecked`) where we opt out, with the
// rationale documented inline.
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32};

use heapless::spsc::{Consumer, Producer, Queue};

use crate::clock::WidenState;
use crate::curve_pool::CurvePool;
use crate::engine::Engine;
use crate::queue::Q_N;
use crate::segment::Segment;
use crate::slot::{NoopIs, NoopPa};
use crate::trace::{TRACE_RING_N, TraceSample};

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
// `kalico_irq_save` / `kalico_irq_restore` defined in `src/runtime_tick.c`,
// not the bare functions directly: Klipper's MCU build uses
// `-flto=auto -fwhole-program`, which empirically lets the LTO/whole-program
// inliner DCE the standalone `irq_save` / `irq_restore` symbols (their
// bodies get inlined at every Klipper callsite) — leaving the kalico
// staticlib's references unresolvable at the final link. The wrappers are
// marked `used + externally_visible` on the C side so LTO keeps them.
#[allow(dead_code)]
unsafe extern "C" {
    pub fn kalico_irq_save() -> u32;
    pub fn kalico_irq_restore(flags: u32);
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
    // Step 7-B: homing gate — ISR checks this before accepting motion segments.
    pub homed: AtomicBool,
    // Step 7-D: signed per-stepper pulse counters, indexed by stepper oid.
    pub stepper_counts: [AtomicI32; MAX_STEPPER_OIDS],
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
            credit_epoch: AtomicU32::new(0),
            accepted_segment_id: AtomicU32::new(0),
            retired_through_segment_id: AtomicU32::new(0),
            fault_detail: AtomicU32::new(0),
            terminal_segment_id_set: AtomicBool::new(false),
            terminal_segment_id_value: AtomicU32::new(0),
            accepted_segment_id_seen: AtomicBool::new(false),
            homed: AtomicBool::new(false),
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
    static kalico_clock_freq: u32;
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
            // Engine::new_production both see the same value. `kalico_clock_freq`
            // is set at static-init time before the runtime ever runs.
            let freq = core::ptr::read_volatile(core::ptr::addr_of!(kalico_clock_freq));

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
