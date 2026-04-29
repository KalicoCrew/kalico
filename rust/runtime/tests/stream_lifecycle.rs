//! Step-6 §8.3 + §8.5 stream-lifecycle handler tests.
//!
//! Covers `open()` / `arm()` / `terminal()` / `clock_sync_respond()` and the
//! `check_terminal_on_retire` ISR-side helper. flush() is exercised in a
//! separate test file (flush_basic.rs / flush_timeout.rs) because it requires
//! more elaborate fixture setup (raw `RuntimeContext` projection).
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::doc_markdown,
    unsafe_code
)]

use core::sync::atomic::Ordering;

use heapless::spsc::Queue;

use runtime::error::{
    KALICO_ERR_ARM_REJECTED, KALICO_ERR_STREAM_STATE_VIOLATION, KALICO_OK,
};
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::state::{FgState, SharedState};
use runtime::stream::{self, FgStreamState};
use runtime::trace::{TRACE_RING_N, TraceSample};

// stream::flush imports `kalico_host_now_us` (foreign symbol from
// `src/runtime_tick.c`). Stream-lifecycle tests don't call flush(), but the
// linker still needs to resolve the symbol. Same goes for `irq_save`/
// `irq_restore` — declared in `runtime::state` for Phase 7's flush. Provide
// no-op host stubs at file scope.
#[unsafe(no_mangle)]
pub extern "C" fn kalico_host_now_us() -> u64 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_irq_save() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_irq_restore(_flags: u32) {}

/// Minimal `FgState` constructor for tests. Owns a `Queue<Segment, Q_N>`
/// and a `Queue<TraceSample, TRACE_RING_N>` on the leaked-Box pattern so
/// the producer/consumer halves get `'static` lifetimes.
fn fg_state_for_test() -> FgState {
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (q_producer, _q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (_t_producer, t_consumer) = trace.split();
    FgState {
        queue_producer: q_producer,
        trace_consumer: t_consumer,
        stream_state_machine: FgStreamState::Idle,
        current_stream_id: None,
        armed_t_start_t0: None,
        first_priming_segment_t_start: None,
        terminal_segment_id: None,
        flush_start_tick: None,
    }
}

// ───────────────────────── open ─────────────────────────────

#[test]
fn open_from_idle_succeeds() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    let r = stream::open(&mut fg, &shared, 42);
    assert_eq!(r, KALICO_OK);
    assert!(shared.stream_open.load(Ordering::Acquire));
    assert_eq!(fg.current_stream_id, Some(42));
    assert_eq!(fg.stream_state_machine, FgStreamState::StreamOpening);
}

#[test]
fn open_idempotent_same_stream_id() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    assert_eq!(stream::open(&mut fg, &shared, 7), KALICO_OK);
    // Second open with the same stream_id while still in StreamOpening → OK.
    assert_eq!(stream::open(&mut fg, &shared, 7), KALICO_OK);
}

#[test]
fn open_with_different_stream_id_violates_state() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    assert_eq!(stream::open(&mut fg, &shared, 1), KALICO_OK);
    let r = stream::open(&mut fg, &shared, 2);
    assert_eq!(r, KALICO_ERR_STREAM_STATE_VIOLATION);
}

#[test]
fn open_clears_stale_terminal_state() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    // Pretend a previous stream left terminal state set.
    shared
        .terminal_segment_id_set
        .store(true, Ordering::Release);
    shared
        .terminal_segment_id_value
        .store(99, Ordering::Release);
    fg.terminal_segment_id = Some(99);
    let r = stream::open(&mut fg, &shared, 1);
    assert_eq!(r, KALICO_OK);
    assert!(!shared.terminal_segment_id_set.load(Ordering::Acquire));
    assert_eq!(fg.terminal_segment_id, None);
}

// ───────────────────────── arm ──────────────────────────────

#[test]
fn arm_without_open_violates_state() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    let (r, t) = stream::arm(&mut fg, &shared, 1_000_000, 100);
    assert_eq!(r, KALICO_ERR_STREAM_STATE_VIOLATION);
    assert_eq!(t, 0);
}

#[test]
fn arm_without_priming_segment_rejected() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    stream::open(&mut fg, &shared, 1);
    // Move into priming-state without recording a first segment.
    fg.stream_state_machine = FgStreamState::StreamOpenPriming;
    let (r, _) = stream::arm(&mut fg, &shared, 1_000_000, 100);
    assert_eq!(r, KALICO_ERR_ARM_REJECTED);
}

#[test]
fn arm_with_first_segment_too_close_rejected() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    stream::open(&mut fg, &shared, 1);
    fg.stream_state_machine = FgStreamState::StreamOpenPriming;
    fg.first_priming_segment_t_start = Some(50);
    // widened_now = 0 (default), arm_lead_cycles = 1000 → first_t_start (50)
    // < 0 + 1000 → rejected.
    let (r, _) = stream::arm(&mut fg, &shared, 50, 1000);
    assert_eq!(r, KALICO_ERR_ARM_REJECTED);
}

#[test]
fn arm_succeeds_with_adequate_lead() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    stream::open(&mut fg, &shared, 1);
    fg.stream_state_machine = FgStreamState::StreamOpenPriming;
    fg.first_priming_segment_t_start = Some(1_000_000);
    // widened_now = 0, lead = 1000 → 1_000_000 >= 1000. OK.
    let (r, t) = stream::arm(&mut fg, &shared, 1_000_000, 1000);
    assert_eq!(r, KALICO_OK);
    assert_eq!(t, 1_000_000);
    assert_eq!(fg.stream_state_machine, FgStreamState::Armed);
    assert_eq!(fg.armed_t_start_t0, Some(1_000_000));
}

#[test]
fn arm_idempotent_same_t_start_t0() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    stream::open(&mut fg, &shared, 1);
    fg.stream_state_machine = FgStreamState::StreamOpenPriming;
    fg.first_priming_segment_t_start = Some(1_000_000);
    assert_eq!(stream::arm(&mut fg, &shared, 1_000_000, 1000).0, KALICO_OK);
    // Re-arm with same value → OK.
    let (r, t) = stream::arm(&mut fg, &shared, 1_000_000, 1000);
    assert_eq!(r, KALICO_OK);
    assert_eq!(t, 1_000_000);
}

#[test]
fn arm_different_t_start_after_armed_violates() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    stream::open(&mut fg, &shared, 1);
    fg.stream_state_machine = FgStreamState::StreamOpenPriming;
    fg.first_priming_segment_t_start = Some(1_000_000);
    assert_eq!(stream::arm(&mut fg, &shared, 1_000_000, 1000).0, KALICO_OK);
    let (r, _) = stream::arm(&mut fg, &shared, 2_000_000, 1000);
    assert_eq!(r, KALICO_ERR_STREAM_STATE_VIOLATION);
}

// ───────────────────── terminal ─────────────────────────────

#[test]
fn terminal_in_running_publishes_atomics_and_transitions() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    fg.stream_state_machine = FgStreamState::Running;
    let r = stream::terminal(&mut fg, &shared, 17);
    assert_eq!(r, KALICO_OK);
    assert_eq!(fg.terminal_segment_id, Some(17));
    assert!(shared.terminal_segment_id_set.load(Ordering::Acquire));
    assert_eq!(
        shared.terminal_segment_id_value.load(Ordering::Acquire),
        17
    );
    assert_eq!(fg.stream_state_machine, FgStreamState::Draining);
}

#[test]
fn terminal_idempotent_same_segment_id() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    fg.stream_state_machine = FgStreamState::Running;
    assert_eq!(stream::terminal(&mut fg, &shared, 42), KALICO_OK);
    assert_eq!(stream::terminal(&mut fg, &shared, 42), KALICO_OK);
}

#[test]
fn terminal_different_segment_id_violates() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    fg.stream_state_machine = FgStreamState::Running;
    assert_eq!(stream::terminal(&mut fg, &shared, 42), KALICO_OK);
    let r = stream::terminal(&mut fg, &shared, 43);
    assert_eq!(r, KALICO_ERR_STREAM_STATE_VIOLATION);
}

#[test]
fn terminal_in_idle_violates() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    let r = stream::terminal(&mut fg, &shared, 1);
    assert_eq!(r, KALICO_ERR_STREAM_STATE_VIOLATION);
}

// ─────────────── check_terminal_on_retire ───────────────────

#[test]
fn check_terminal_on_retire_clears_stream_open_on_match() {
    let shared = SharedState::new();
    shared.stream_open.store(true, Ordering::Release);
    shared
        .terminal_segment_id_set
        .store(true, Ordering::Release);
    shared
        .terminal_segment_id_value
        .store(7, Ordering::Release);
    stream::check_terminal_on_retire(&shared, 7);
    assert!(!shared.stream_open.load(Ordering::Acquire));
    // Per Round-2 B14 the helper does NOT clear the published terminal
    // atomics; foreground (next stream_open / flush) owns clearing.
    assert!(shared.terminal_segment_id_set.load(Ordering::Acquire));
}

#[test]
fn check_terminal_on_retire_no_match_keeps_stream_open() {
    let shared = SharedState::new();
    shared.stream_open.store(true, Ordering::Release);
    shared
        .terminal_segment_id_set
        .store(true, Ordering::Release);
    shared
        .terminal_segment_id_value
        .store(7, Ordering::Release);
    stream::check_terminal_on_retire(&shared, 6);
    assert!(shared.stream_open.load(Ordering::Acquire));
}

#[test]
fn check_terminal_on_retire_no_terminal_set_is_noop() {
    let shared = SharedState::new();
    shared.stream_open.store(true, Ordering::Release);
    stream::check_terminal_on_retire(&shared, 99);
    assert!(shared.stream_open.load(Ordering::Acquire));
}

// ─────────────── clock_sync_respond ─────────────────────────

#[test]
fn clock_sync_returns_widened_now_snapshot() {
    let mut fg = fg_state_for_test();
    let shared = SharedState::new();
    runtime::clock::publish_widened_now(&shared, 0xDEAD_BEEF_1234_5678);
    let (r, mcu) = stream::clock_sync_respond(&mut fg, &shared, 1, 0, 0);
    assert_eq!(r, KALICO_OK);
    assert_eq!(mcu, 0xDEAD_BEEF_1234_5678);
}

// Suppress warnings for the unused `Segment` / `KinematicTag` imports —
// they're available for future tests that exercise enqueue paths.
#[allow(dead_code)]
fn _unused_segment_marker(_s: Segment, _k: KinematicTag) {}
