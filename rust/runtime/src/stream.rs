//! Stream lifecycle state machine (host + MCU side). Spec §8.
//!
//! Phase 1 introduced the `FgStreamState` enum so `FgState::stream_state_machine`
//! has a type to point at. Phase 3.2 stubbed the FFI handlers; Phase 6 fleshed
//! out the transition rules; Phase 7 lands the §8.5 `force_idle` handshake +
//! flush sequence.
//!
//! The `flush` entry point takes `*mut RuntimeContext` rather than `&mut
//! FgState` because under the disabled-IRQ window it transiently projects to
//! the ISR-owned queue consumer and clears the engine's in-flight segment as
//! defense-in-depth. The discipline contract is preserved: at most one of
//! foreground (`FgState`) or ISR (`IsrState`) projection is live at any moment.

#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::Ordering;

use crate::error::{
    KALICO_ERR_ARM_REJECTED, KALICO_ERR_NULL_PTR, KALICO_ERR_STREAM_STATE_VIOLATION, KALICO_OK,
    encode_stream_state_violation,
};
use crate::state::{FgState, IsrState, RuntimeContext, SharedState};

/// Closure-review fix: publish a §9.2 stream-state-violation `fault_detail`
/// payload so the host's `kalico_runtime_fault_detail` accessor (and the
/// periodic `kalico_status_v6` frame's `fault_detail` column) can carry
/// diagnostic context for the rejection. Stream-state violations are
/// *soft* rejections (no `RuntimeStatus::Fault` transition), so we only
/// touch `fault_detail` here — `last_error` and `runtime_status` are
/// left to engine-side latching.
fn publish_stream_state_violation_detail(
    shared: &SharedState,
    observed: FgStreamState,
    expected: FgStreamState,
) {
    let detail = encode_stream_state_violation(observed as u8, expected as u8);
    shared.fault_detail.store(detail, Ordering::Release);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FgStreamState {
    Idle = 0,
    StreamOpening = 1,
    StreamOpenPriming = 2,
    Arming = 3,
    Armed = 4,
    Running = 5,
    Draining = 6,
    Drained = 7,
    Fault = 8,
}

// Foreign symbols — only present in MCU and integration-test builds. Test
// crates that link against `runtime` provide `#[no_mangle]` stubs (mirror
// of `runtime_clock_freq` stubbing in `kalico-c-api/tests/init_once.rs`).
unsafe extern "C" {
    /// Klipper-side host-clock helper (foreground only). Returns wall-clock
    /// µs since boot, derived from `timer_read_time()` divided by
    /// `runtime_clock_freq / 1_000_000`. Not ISR-safe in spirit (the
    /// underlying `timer_read_time` may wrap), but the §8.5 flush window is
    /// bounded to ≤1 ms so a single wrap is the worst case.
    fn runtime_host_now_us() -> u64;
}

// `runtime_widened_host_clock` (C-side helper in `src/runtime_tick.c`) and
// the local `read_widened_host_clock` wrapper that used to wire
// `clock_sync_respond` into stats-based widening were removed during the
// step-emission T12 cleanup pass (2026-05-14). The 2026-05-13 re-fix
// (see `clock_sync_respond`'s doc comment) reverted clock-sync back to
// the §11.4 seqlock, so the helper had no caller. The
// `kalico_runtime_modulated_tick` FFI path declares its own local extern
// of `runtime_widened_host_clock` for its widening read; that path is
// independent. The pending architectural cutover for clock-sync (spec
// §3.9 — stats-based widening, independent of TIM5 state) needs more
// than a simple symbol re-wire — it has to address the
// stats_send_time_high wrap lag bench-observed on 2026-05-13 — and is
// out of T12 scope.

// `runtime_irq_save` / `runtime_irq_restore` are declared in `state.rs` —
// thin C wrappers around Klipper's `irq_save` / `irq_restore` that survive
// the MCU build's `-flto=auto -fwhole-program` DCE (see state.rs comment).
use crate::state::{runtime_irq_restore, runtime_irq_save};

// ────────────────────────────── open ─────────────────────────────────────

/// `kalico_stream_open` handler. §8.3 / §8.5.
///
/// Idempotent on same `stream_id` while in `StreamOpening` /
/// `StreamOpenPriming` (Plan §8.5 defensive idempotency).
pub fn open(fg: &mut FgState, shared: &SharedState, stream_id: u32) -> i32 {
    if shared.stream_open.load(Ordering::Acquire) {
        // Idempotent only for SAME stream_id while still pre-arm.
        if fg.current_stream_id == Some(stream_id)
            && (fg.stream_state_machine == FgStreamState::StreamOpening
                || fg.stream_state_machine == FgStreamState::StreamOpenPriming)
        {
            return KALICO_OK;
        }
        publish_stream_state_violation_detail(shared, fg.stream_state_machine, FgStreamState::Idle);
        return KALICO_ERR_STREAM_STATE_VIOLATION;
    }
    // Round-1 B14: ensure terminal_segment_id is cleared on stream_open.
    fg.terminal_segment_id = None;
    shared
        .terminal_segment_id_set
        .store(false, Ordering::Release);
    shared.terminal_segment_id_value.store(0, Ordering::Release);
    // Round-3 B-R3-8: reset accepted-id-seen so the new stream's first push
    // starts a fresh monotonicity sequence.
    shared
        .accepted_segment_id_seen
        .store(false, Ordering::Release);
    shared.accepted_segment_id.store(0, Ordering::Release);

    fg.stream_state_machine = FgStreamState::StreamOpening;
    fg.current_stream_id = Some(stream_id);
    fg.armed_t_start_t0 = None;
    fg.first_priming_segment_t_start = None;
    shared.stream_open.store(true, Ordering::Release);
    KALICO_OK
}

// ────────────────────────────── arm ──────────────────────────────────────

/// `kalico_stream_arm` handler. §6.3 / §6.4 / §8.3 / §8.5.
///
/// Validates the FIRST priming segment's `t_start` (tracked in `FgState` by
/// `push_segment_impl` per Round-2 B6) is at least `arm_lead_cycles` ahead
/// of the current widened `now`. Idempotent on same `t_start_t0`.
pub fn arm(
    fg: &mut FgState,
    shared: &SharedState,
    t_start_t0: u64,
    arm_lead_cycles: u32,
) -> (i32, u64) {
    // Idempotency: arm in Armed state with the same t_start_t0 is OK.
    if fg.stream_state_machine == FgStreamState::Armed {
        if fg.armed_t_start_t0 == Some(t_start_t0) {
            return (KALICO_OK, t_start_t0);
        }
        publish_stream_state_violation_detail(
            shared,
            fg.stream_state_machine,
            FgStreamState::StreamOpenPriming,
        );
        return (KALICO_ERR_STREAM_STATE_VIOLATION, 0);
    }
    if fg.stream_state_machine != FgStreamState::StreamOpenPriming {
        publish_stream_state_violation_detail(
            shared,
            fg.stream_state_machine,
            FgStreamState::StreamOpenPriming,
        );
        return (KALICO_ERR_STREAM_STATE_VIOLATION, 0);
    }
    // Per spec §6.3: at least 1 priming segment.
    let Some(first_t_start) = fg.first_priming_segment_t_start else {
        return (KALICO_ERR_ARM_REJECTED, 0);
    };
    let now = crate::clock::read_widened_now(shared);
    if first_t_start < now.saturating_add(u64::from(arm_lead_cycles)) {
        return (KALICO_ERR_ARM_REJECTED, 0);
    }
    fg.stream_state_machine = FgStreamState::Armed;
    fg.armed_t_start_t0 = Some(t_start_t0);
    (KALICO_OK, t_start_t0)
}

// ────────────────────────────── terminal ─────────────────────────────────

/// `kalico_stream_terminal` handler. §8.3 / §8.5.
///
/// Publishes terminal-segment id to `SharedState` so the ISR retire path
/// can clear `stream_open` once that segment finishes. Idempotent on same
/// segment id.
pub fn terminal(fg: &mut FgState, shared: &SharedState, segment_id: u32) -> i32 {
    // Allow Running/StreamOpenPriming/Armed; reject everything else.
    let st = fg.stream_state_machine;
    if st != FgStreamState::Running
        && st != FgStreamState::StreamOpenPriming
        && st != FgStreamState::Armed
    {
        // Allow same-segment-id idempotency in Draining state.
        if st == FgStreamState::Draining && fg.terminal_segment_id == Some(segment_id) {
            return KALICO_OK;
        }
        publish_stream_state_violation_detail(shared, st, FgStreamState::Running);
        return KALICO_ERR_STREAM_STATE_VIOLATION;
    }
    if let Some(existing) = fg.terminal_segment_id {
        if existing == segment_id {
            return KALICO_OK;
        }
        publish_stream_state_violation_detail(shared, st, FgStreamState::Running);
        return KALICO_ERR_STREAM_STATE_VIOLATION;
    }
    fg.terminal_segment_id = Some(segment_id);
    shared
        .terminal_segment_id_value
        .store(segment_id, Ordering::Release);
    shared
        .terminal_segment_id_set
        .store(true, Ordering::Release);
    fg.stream_state_machine = FgStreamState::Draining;
    KALICO_OK
}

/// Engine-side helper (called from `Engine::tick` retire path):
/// if `shared.terminal_segment_id_set` is true and the just-retired
/// segment's id matches the published value, clear `stream_open`.
/// Subsequent boundary-loop on empty queue → Drained, not Underrun.
///
/// Does NOT clear `terminal_segment_id_set`/`terminal_segment_id_value`;
/// foreground (next `stream_open` / flush) owns clearing per Round-2 B14.
pub fn check_terminal_on_retire(shared: &SharedState, retired_seg_id: u32) {
    if !shared.terminal_segment_id_set.load(Ordering::Acquire) {
        return;
    }
    if shared.terminal_segment_id_value.load(Ordering::Acquire) != retired_seg_id {
        return;
    }
    shared.stream_open.store(false, Ordering::Release);
}

// ────────────────────────────── clock_sync ───────────────────────────────

/// `kalico_clock_sync_request` handler. §12.1.
///
/// Phase-6 returns the §11.4 widened-now snapshot. Phase 8's clock-sync
/// machinery uses the `request_id` / `host_send_time` to form a complete
/// round-trip estimate; here we just sample MCU clock and let the host do
/// the math.
///
/// **2026-05-13 re-fix.** Returns to `read_widened_now(shared)` — the
/// seqlock published by Engine::tick — now that the "always-on TIM5"
/// fix (commit 0512e962d, 2026-05-12) guarantees Engine::tick runs
/// continuously regardless of count_modulated. The 2026-05-11
/// stats-based workaround was correct ONLY while TIM5 could go silent
/// on Drained / Fault; with TIM5 always armed, the engine-side seqlock
/// is fresh every 25 µs (H7) / 100 µs (F4).
///
/// The stats-based widening (Klipper's stats_send_time_high, 5 s cadence)
/// caused a new failure mode after the always-on-TIM5 fix: WidenState
/// observes 2^32 wraps via TIM5 ISR (40 kHz / 10 kHz) MUCH faster than
/// stats_send_time_high which only updates on the stats_update task at
/// ~0.2 Hz. When a wrap occurred, the engine's WidenState bumped `.high`
/// immediately, but stats_send_time_high lagged up to 5 seconds. During
/// that window, the host's `t_start_clock` was stamped using stale
/// stats-based clock → engine's `now` >> `t_start` → segment retired
/// instantly → no step pulses fired. Bench symptom 2026-05-13: jog
/// retires 6 segments to Drained in milliseconds, emit_calls=0,
/// step_time_event fires at 1 kHz poll cadence but always hits NO_STEP
/// because by the time it polls, engine.current is already None.
///
/// Engine-side and host-side now share a SINGLE widening source (the
/// engine's WidenState, seeded at runtime_tick_enable from
/// stats_send_time_high but evolved independently via TIM5 ISR
/// observations) — eliminating the divergence between the frame the
/// host stamps t_start in and the frame the engine evaluates `now` in.
pub fn clock_sync_respond(
    _fg: &mut FgState,
    shared: &SharedState,
    _request_id: u32,
    _host_send_time_lo: u32,
    _host_send_time_hi: u32,
) -> (i32, u64) {
    let mcu_clock = crate::clock::read_widened_now(shared);
    (KALICO_OK, mcu_clock)
}

// ────────────────────────────── flush ────────────────────────────────────

/// §8.5 flush sequence — T11 rewrite.
///
/// New ordering (post-T11; the legacy spin-wait handshake is removed):
///   1. `stream_open` = false (queue-empty ticks now go Drained, not
///      Underrun, before the in-flight ISR is even touched).
///   2. `irq_save` + synchronous `Engine::runtime_force_idle` (drains
///      queue, retires every in-flight pool slot, resets step rings +
///      producer states + per-motor cursors + step accumulators, clears
///      `producer_current` / legacy `current`).
///   3. `pool.reset_all_retired_to_current()` (defense-in-depth: the
///      synchronous flush already retired the *queued* + *in-flight*
///      handles, but slots holding pre-arm-rejected loads may still
///      have a generation gap).
///   4. bump `credit_epoch`.
///   5. clear flags + reset stream-machine + clear `terminal_segment_id`.
///   6. clear `force_idle` / `acked_force_idle` atomics (transition-period
///      cleanup; T12 deletes them).
///
/// Takes `*mut RuntimeContext` because step 2 transiently projects to
/// `IsrState` under disabled IRQ. The half-split discipline holds: with
/// IRQs disabled, the foreground is the sole context running → no
/// concurrent ISR / producer-timer / consumer-timer access window exists
/// → forming `&mut IsrState` briefly is sound.
///
/// SAFETY: caller must guarantee single-threaded foreground entry (the
/// command-dispatch task is single-threaded by Klipper's design) and
/// `rt` is the published `RuntimeContext` pointer from
/// `kalico_runtime_init`.
pub unsafe fn flush(rt: *mut RuntimeContext, out_credit_epoch: *mut u32) -> i32 {
    if rt.is_null() {
        return KALICO_ERR_NULL_PTR;
    }
    let ctx = rt;

    // Project FgState (foreground exclusive) and SharedState (atomics).
    // SAFETY: caller-supplied valid pointer; we form one `&mut FgState`
    // via the half-split projection. The ISR path is held off in step 4
    // by `irq_save`; until then we touch only foreground + atomic state.
    let (fg, shared, pool) = unsafe {
        let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
        let shared_ptr: *const SharedState = core::ptr::addr_of!((*ctx).shared);
        let pool_ptr: *const crate::curve_pool::CurvePool = core::ptr::addr_of!((*ctx).curve_pool);
        (&mut *fg_ptr, &*shared_ptr, &*pool_ptr)
    };

    // ─── T11: synchronous foreground flush, no ISR handshake ────────────
    //
    // The legacy Plan-decision A spin-wait on `acked_force_idle` is gone.
    // The new step-emission architecture (spec §3.10) makes flush a
    // foreground-only operation: the caller already serialises through
    // the bridge command channel, and under `irq_save` the producer +
    // every per-motor consumer Klipper timer is held off, so we form
    // `&mut IsrState` and drive `Engine::runtime_force_idle` directly.
    // The ISR-side short-circuit branch in `Engine::tick` was removed
    // in the same task; `runtime_force_idle` itself sets
    // `acked_force_idle = true` at its tail as a transition courtesy
    // so any external observer polling that bit still sees the ack.
    fg.flush_start_tick = Some(unsafe { runtime_host_now_us() });

    // Step 1: clear stream_open BEFORE the synchronous flush. Without
    // an ISR ack to gate this on, the ordering simplifies — there is
    // no in-flight ISR mid-tick to race with the host's drain (irq_save
    // below holds it off). Subsequent ticks on empty queue see
    // stream_open=false → Drained, not Underrun.
    shared.stream_open.store(false, Ordering::Release);

    // Step 2: IRQ-disable + synchronous flush via raw-pointer projection
    // to IsrState. SAFETY: under irq_save, no ISR / producer-timer /
    // consumer-timer can run, so we transiently hold exclusive access
    // to IsrState — the discipline contract holds because there's no
    // concurrent access window.
    let irq_flags = unsafe { runtime_irq_save() };
    {
        // SAFETY: runtime_irq_save() above pins the ISR + producer +
        // consumer timers off; we transiently form `&mut IsrState` via
        // the UnsafeCell projection. No concurrent context can race the
        // engine / queue / step-ring / producer-state writes below.
        let isr: &mut IsrState = unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            &mut *isr_ptr
        };
        // Drive the synchronous flush. Drains the queue, retires every
        // in-flight pool slot, resets step rings + producer states +
        // per-motor curve cursors + step accumulators, and clears the
        // engine's legacy `current` / `producer_current` slots.
        let IsrState {
            engine,
            queue_consumer,
            pending_segment,
            ..
        } = isr;
        // Retire the deferred segment's handles too — Codex M1 fix
        // (2026-05-20). `isr_sample_tick` may have dequeued a segment whose
        // `t_start` lies in the future and parked it here; a flush tears
        // the lookahead down, so the parked handles must surrender their
        // pool ownership in lockstep with the queued segments
        // `runtime_force_idle` retires.
        if let Some(seg) = pending_segment.take() {
            pool.confirm_retired(seg.x_handle);
            pool.confirm_retired(seg.y_handle);
            pool.confirm_retired(seg.z_handle);
            pool.confirm_retired(seg.e_handle);
        }
        engine.runtime_force_idle(pool, queue_consumer, shared);
    }
    unsafe { runtime_irq_restore(irq_flags) };

    // Step 5: reset per-slot last_retired_gen = current_gen for all slots.
    pool.reset_all_retired_to_current();

    // Step 6: increment credit_epoch (any pending credit events from
    // pre-flush are now stale by epoch comparison).
    let new_epoch = shared
        .credit_epoch
        .fetch_add(1, Ordering::AcqRel)
        .wrapping_add(1);

    // Step 7: clear stream-machine + terminal-segment + monotonicity flags.
    fg.stream_state_machine = FgStreamState::Idle;
    fg.current_stream_id = None;
    fg.armed_t_start_t0 = None;
    fg.first_priming_segment_t_start = None;
    fg.terminal_segment_id = None;
    fg.flush_start_tick = None;
    shared
        .terminal_segment_id_set
        .store(false, Ordering::Release);
    shared.terminal_segment_id_value.store(0, Ordering::Release);
    shared
        .accepted_segment_id_seen
        .store(false, Ordering::Release);
    shared.accepted_segment_id.store(0, Ordering::Release);
    shared.current_segment_id.store(0, Ordering::Release);
    shared
        .retired_through_segment_id
        .store(0, Ordering::Release);
    // Step 5 also implicitly clears any leftover sample_drop_pending if
    // the host plans to re-open after a trace-overflow fault. The host
    // discipline is: post-fault, query state; if it decides to flush + retry,
    // the trace ring is empty by the time we get here (foreground drained
    // it). Clear the latch so the post-flush state is clean.
    shared.sample_drop_pending.store(false, Ordering::Release);
    // last_error and runtime_status: do NOT clear. The host explicitly
    // observes the fault before issuing flush; clearing would mask the
    // failure history. The host may issue a separate "reset fault state"
    // command in the future.

    // Step 8: clear force_idle + acked_force_idle. ISR resumes normal
    // operation on next tick.
    shared.acked_force_idle.store(false, Ordering::Release);
    shared.force_idle.store(false, Ordering::Release);

    // Out-param: caller (FFI shim) may want the new credit_epoch.
    if !out_credit_epoch.is_null() {
        // SAFETY: caller-provided pointer documented to be writable when
        // non-null.
        unsafe {
            *out_credit_epoch = new_epoch;
        }
    }
    KALICO_OK
}
