//! Regression tests for the force_idle ring-drain fix.
//!
//! # The bug (now fixed)
//!
//! `runtime_force_idle` iterated every configured axis and called
//! `axis.reset_isr_cache()`, which set `axis.armed = None` and zeroed the
//! step/position counters.  It did NOT call `axis.ring.drain()`.
//!
//! When a piece was mid-play at force_idle time (`armed.is_some()`, piece window
//! still open), the ring descriptor's `tail` and `retired` fields were NOT
//! advanced.  The ring slot of the currently-playing piece therefore remained
//! occupied from the descriptor's perspective (`retired < head`, `tail ==
//! retired % ring_depth`).
//!
//! On the first post-force_idle ISR tick, `get_position_and_velocity`
//! (engine.rs:680-718) took the following path:
//!
//!   (a) `axis.armed` is `None` — branch 1 (`now < piece_end`) was skipped.
//!   (b) `armed.take()` returned `None` — `is_some() == false` — the guard at
//!       line 704 did NOT call `advance_counter`.
//!   (c) `arm_next` peeked `axis.ring` at `tail` — returned the STALE mid-play
//!       piece that was playing before force_idle.
//!   (d) Fault check: `now.saturating_sub(stale.piece_start_cycles) > 2 *
//!       sample_period_cycles`.  The stale piece started many ticks ago (the
//!       piece was already playing for at least 2 ticks when force_idle fired),
//!       so the check fired `-308 PieceStartInPast` even though the piece's
//!       `piece_end_cycles` extended far into the future.
//!
//! The fault was SPURIOUS — the engine rejected a piece whose window was still
//! open because force_idle left the ring cursor stranded on the un-retired slot.
//!
//! # The fix
//!
//! `runtime_force_idle` now calls `axis.ring.drain()` immediately after
//! `axis.reset_isr_cache()`.  `drain()` advances `retired` and `tail` to
//! `head`, discarding all committed-but-unretired entries so the consumer
//! cannot re-arm an aborted timeline.  Only consumer-owned cursors (`retired`,
//! `tail`) are touched — `head` is never modified — preserving the C/Rust
//! ownership boundary.
//!
//! # Placement-invariance (hardware fact #3)
//!
//! The fault fired identically whether jog2 arrived 1.8 s or 127 ms after
//! jog1, because the trigger was the stale ring cursor left by force_idle, not
//! the timing of jog2's pieces.  Even if jog2's pieces had `start_time` far
//! in the future, `arm_next` peeked `tail`, which pointed at jog1's un-retired
//! slot — not at jog2's slot — and the fault check fired on jog1's stale
//! `start_time`.
//!
//! # Tests in this file
//!
//! These are regression tests asserting that force_idle correctly drains stale
//! pieces so no spurious -308 fires.  Tests 1-3 were previously documented as
//! bug-reproducers; they now assert the fixed (correct) behavior.
//!
//! 1. `force_idle_mid_piece_moving_axis_no_spurious_fault`
//!    Single moving axis.  force_idle fires while a 10 ms piece is mid-window.
//!    Asserts no fault fires after the fix.
//!
//! 2. `force_idle_mid_piece_held_z_axis_no_spurious_fault`
//!    Held Z axis (exact hardware geometry: axis 2, 250 ms constant hold piece,
//!    520 MHz clock).  Asserts no fault fires on the F446 fault path.
//!
//! 3. `force_idle_then_jog2_future_start_no_spurious_fault`
//!    force_idle mid-piece, push jog2 with start_time 250 ms in the future,
//!    tick once.  Asserts no fault fires — drain eliminates the stale jog1 slot
//!    before arm_next can peek it.
//!
//! 4. `natural_drain_no_force_idle_no_fault` (control)
//!    Same geometry but without force_idle: jog1 drains naturally, jog2 is
//!    pushed with a future start_time, no fault.  Establishes that the normal
//!    path is correct and the fix did not regress it.
//!
//! 5. `force_idle_drains_stale_piece_retires_cursor`
//!    Regression test: force_idle drains the stale jog1 slot, so `retired == 1`
//!    after the drain and jog2 arms without fault.

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

// Hardware constants matching the H723/F446 bench.
const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64; // 13_000 cycles
const FAULT_TOL: u64 = TICK * 2; // 26_000 cycles (the strict-greater-than threshold)
// 250 ms expressed in cycles: matches the host LEAD used for both jog pieces.
const LEAD_CYCLES: u64 = 130_000_000; // 0.250 s * 520 MHz

fn make_engine() -> Engine {
    Engine::new(CLOCK_FREQ, SAMPLE_RATE)
}

fn make_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ]
}

fn pulse_binding(stepper_oid: u8) -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn const_piece(start_time: u64, duration_s: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0; 4],
        duration: duration_s,
        _reserved: 0,
    }
}

/// Compute piece end in cycles for assertions.
#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
fn piece_end(start: u64, duration_s: f32) -> u64 {
    start + (duration_s * CLOCK_FREQ as f32) as u64
}

fn configure_axis(engine: &mut Engine, axis_idx: u8, ring_depth: usize) {
    let rc = engine.configure_axis(
        axis_idx,
        StepMode::Pulse,
        0.0125,
        ring_depth,
        &[pulse_binding(axis_idx)],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis({axis_idx}) failed");
}

/// Install step queues for all axes and return a SharedState.
fn setup_queues(engine: &mut Engine) -> ([StepQueue; MAX_AXES], SharedState) {
    let mut qs = core::array::from_fn::<StepQueue, MAX_AXES, _>(|_| StepQueue::new());
    let mut ptrs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    for (i, q) in qs.iter_mut().enumerate() {
        ptrs[i] = q as *mut StepQueue;
    }
    engine.test_install_step_queues(ptrs);
    let shared = SharedState::new();
    (qs, shared)
}

// ── Test 1: moving axis, force_idle mid-piece ─────────────────────────────────

/// Regression test: axis 0 (moving).  Push a 10 ms piece.  Arm it.  Tick 2
/// more times while mid-window (well before piece_end).  Issue force_idle.
/// Tick once more.
///
/// After the fix, `force_idle` calls `ring.drain()` which advances `retired`
/// to `head`, so `arm_next` finds an empty ring on the next tick and no
/// spurious `-308 PieceStartInPast` fires.
#[test]
fn force_idle_mid_piece_moving_axis_no_spurious_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let piece_start = TICK;
    let piece_dur = 0.010_f32;
    let piece_end_cyc = piece_end(piece_start, piece_dur);

    let rc = engine.push_pieces(0, &[const_piece(piece_start, piece_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog1 piece");

    // Arm the piece.
    engine.tick(piece_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault at arm"
    );
    assert_eq!(engine.retired_counts()[0], 0, "piece armed, not yet retired");

    // Two more ticks mid-window (the piece window extends to ~5.2M cycles;
    // we are at TICK + 2*TICK = 3*TICK = 39_000 << piece_end).
    engine.tick(piece_start + TICK, &shared, &mut storage);
    engine.tick(piece_start + 2 * TICK, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault mid-window"
    );
    assert!(
        piece_start + 2 * TICK < piece_end_cyc,
        "precondition: still mid-window when force_idle fires"
    );

    // force_idle fires while the piece is still live.
    engine.runtime_force_idle(&shared);

    // The ring cursor must have been advanced (or the ring emptied) by
    // force_idle.
    let retired_after_fi = engine.retired_counts()[0];

    let now_post_fi = piece_start + 2 * TICK + 1;
    let lateness = now_post_fi.saturating_sub(piece_start);
    assert!(
        lateness > FAULT_TOL,
        "precondition: if the ring cursor was NOT advanced, \
         the stale piece's lateness ({lateness}) exceeds FAULT_TOL ({FAULT_TOL})"
    );

    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);

    assert_eq!(
        err,
        0,
        "REGRESSION: engine raised {err} (PieceStartInPast=-308) on a \
         piece whose window was still open at force_idle time \
         (piece_end={piece_end_cyc}, now_post_fi={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}). \
         retired_after_force_idle={retired_after_fi}. \
         FIX: runtime_force_idle must call axis.ring.drain() after \
         axis.reset_isr_cache() to advance the ring cursor past stale slots."
    );
}

// ── Test 2: held Z axis, long 250ms hold piece ────────────────────────────────

/// Regression test: axis 2 (held Z, F446 path).  250 ms constant piece
/// starting at `TICK`.  This is the exact geometry of the Z-axis hold piece
/// in the hardware two-jog repro.  force_idle fires at `piece_start + 2*TICK`
/// (the piece has been live for 2 ticks, window extends 130M more cycles).
///
/// After the fix, `ring.drain()` in force_idle advances `retired` to `head`,
/// so no spurious `-308 PieceStartInPast` fires on the next tick.
#[test]
fn force_idle_mid_piece_held_z_axis_no_spurious_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 2, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let piece_start = TICK;
    let piece_dur = 0.250_f32; // 250 ms hold piece (Z axis during an X jog)
    let piece_end_cyc = piece_end(piece_start, piece_dur);

    // Sanity: the piece window extends far beyond our force_idle point.
    assert!(
        piece_end_cyc > piece_start + 1_000 * TICK,
        "precondition: hold piece must extend >> 1000 ticks past start"
    );

    let rc = engine.push_pieces(2, &[const_piece(piece_start, piece_dur)], &mut storage);
    assert_eq!(rc, 0, "push Z hold piece");

    engine.tick(piece_start, &shared, &mut storage);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "no fault at arm");

    engine.tick(piece_start + TICK, &shared, &mut storage);
    engine.tick(piece_start + 2 * TICK, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault mid-window"
    );

    // force_idle fires while the Z hold is still live.
    engine.runtime_force_idle(&shared);

    let now_post_fi = piece_start + 2 * TICK + 1;
    let lateness = now_post_fi.saturating_sub(piece_start);
    let remaining_window = piece_end_cyc - now_post_fi;

    assert!(lateness > FAULT_TOL, "precondition: lateness={lateness} > fault_tol={FAULT_TOL}");

    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        err,
        0,
        "REGRESSION on Z-axis hold piece: engine raised {err} \
         (PieceStartInPast=-308). \
         piece_end={piece_end_cyc}, now={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}, \
         remaining_window={remaining_window} cycles. \
         FIX: force_idle must call ring.drain() to advance the cursor past \
         the un-retired Z hold piece slot."
    );
}

// ── Test 3: placement-invariance — jog2 future start does not prevent fault ───

/// Regression test: arm jog1 piece, tick mid-window, force_idle, push jog2
/// with start_time = now + LEAD_CYCLES (250 ms in the future), tick once.
///
/// After the fix, `ring.drain()` in force_idle retires jog1's slot before
/// `arm_next` can peek it, so the next peek sees jog2 (future, lateness=0)
/// and no fault fires.
///
/// This test also validates placement-invariance: with the fix, the fault does
/// NOT fire regardless of jog2's start_time, because the stale jog1 slot is
/// gone before arm_next runs.
#[test]
fn force_idle_then_jog2_future_start_no_spurious_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let jog1_start = TICK;
    let jog1_dur = 0.010_f32;
    let jog1_end_cyc = piece_end(jog1_start, jog1_dur);

    // Push and arm jog1.
    let rc = engine.push_pieces(0, &[const_piece(jog1_start, jog1_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog1");

    engine.tick(jog1_start, &shared, &mut storage);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "arm jog1 ok");
    assert_eq!(engine.retired_counts()[0], 0, "jog1 armed, not retired");

    engine.tick(jog1_start + TICK, &shared, &mut storage);
    engine.tick(jog1_start + 2 * TICK, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "mid-window ok"
    );
    assert!(
        jog1_start + 2 * TICK < jog1_end_cyc,
        "precondition: jog1 still mid-window at force_idle time"
    );

    let now_at_fi = jog1_start + 2 * TICK;

    // force_idle: drains ring (advances retired to head), clears armed.
    engine.runtime_force_idle(&shared);

    // Push jog2 well into the future — 250 ms LEAD so lateness=0 at arm time.
    let jog2_start = now_at_fi + LEAD_CYCLES;
    let jog2_dur = 0.010_f32;
    let rc = engine.push_pieces(0, &[const_piece(jog2_start, jog2_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog2 (future start)");

    // jog2's lateness from now_at_fi+1 is 0 (future).
    let now_post_fi = now_at_fi + 1;
    let jog2_lateness = now_post_fi.saturating_sub(jog2_start);
    assert_eq!(jog2_lateness, 0, "jog2 lateness must be 0 (future piece)");

    // jog1's stale start_time lateness is 3*TICK+1 > FAULT_TOL.
    let jog1_lateness = now_post_fi.saturating_sub(jog1_start);
    assert!(
        jog1_lateness > FAULT_TOL,
        "precondition: jog1_lateness={jog1_lateness} > fault_tol={FAULT_TOL}"
    );

    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    let retired = engine.retired_counts()[0];

    assert_eq!(
        err,
        0,
        "REGRESSION: engine raised {err} on jog2 tick \
         (now={now_post_fi}). jog2_start={jog2_start} (lateness=0, future). \
         jog1_start={jog1_start} (lateness={jog1_lateness} > fault_tol={FAULT_TOL}). \
         retired_after_tick={retired}. \
         FIX: force_idle must call ring.drain() so arm_next peeks jog2 \
         (future, no fault), not jog1's stale slot."
    );
}

// ── Test 4: control — normal drain, no force_idle, no fault ──────────────────

/// Control case: jog1 drains naturally (no force_idle), jog2 pushed with
/// future start_time.  No fault expected, no fault observed.
///
/// Confirms the bug was gated exclusively on the force_idle path and that
/// the fix did not regress the normal drain path.
///
/// This test is expected to PASS.
#[test]
fn natural_drain_no_force_idle_no_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    // Use a short piece so we can tick past its end in a predictable number of
    // steps (1 ms = 520_000 cycles >> TICK so lateness after drain is minimal).
    let jog1_start = TICK;
    let jog1_dur = 0.001_f32;
    let jog1_end_cyc = piece_end(jog1_start, jog1_dur);

    let rc = engine.push_pieces(0, &[const_piece(jog1_start, jog1_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog1");

    // Arm jog1.
    engine.tick(jog1_start, &shared, &mut storage);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "arm jog1 ok");
    assert_eq!(engine.retired_counts()[0], 0);

    // Tick to jog1's end — engine retires it normally via advance_counter.
    engine.tick(jog1_end_cyc, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault at drain"
    );
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "jog1 retired via advance_counter after natural expiry"
    );

    // Push jog2 with a future start_time — no force_idle involved.
    let jog2_start = jog1_end_cyc + LEAD_CYCLES;
    let rc = engine.push_pieces(0, &[const_piece(jog2_start, 0.010)], &mut storage);
    assert_eq!(rc, 0, "push jog2");

    // Tick just after jog1_end; jog2 is future (lateness == 0).
    engine.tick(jog1_end_cyc + 1, &shared, &mut storage);
    let err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        err,
        0,
        "CONTROL CASE: normal drain + future jog2 must NOT fault; got {err}"
    );
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "jog2 armed but still playing — retired must remain 1"
    );
}

// ── Test 5: regression — force_idle drains stale piece, advances retired cursor

/// Regression test: force_idle must drain the stale jog1 slot so that
/// `retired == 1` after the drain, and jog2 (pushed at `jog1_end_cyc`,
/// a future start relative to `now_post_fi`) arms without fault.
///
/// The observable footprint of the fix is `retired == 1`: `ring.drain()`
/// advanced `retired` from 0 to `head` (which was 1 after jog1 was pushed),
/// recording that jog1's slot was discarded.  `retired == 0` after the tick
/// would mean the drain never ran and the stale slot was re-armed — the old
/// MCU bug.
///
/// Setup mirrors the previous classifier test (jog2 start at `jog1_end_cyc`
/// which is well in the future relative to `now_post_fi = jog1_start + 2*TICK + 1`).
#[test]
fn force_idle_drains_stale_piece_retires_cursor() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let jog1_start = TICK;
    let jog1_dur = 0.010_f32;
    let jog1_end_cyc = piece_end(jog1_start, jog1_dur);

    let rc = engine.push_pieces(0, &[const_piece(jog1_start, jog1_dur)], &mut storage);
    assert_eq!(rc, 0);

    engine.tick(jog1_start, &shared, &mut storage);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "arm ok");
    engine.tick(jog1_start + TICK, &shared, &mut storage);
    engine.tick(jog1_start + 2 * TICK, &shared, &mut storage);
    assert_eq!(engine.retired_counts()[0], 0, "mid-window: not yet retired");

    // force_idle fires; jog1 piece still live.
    engine.runtime_force_idle(&shared);

    // Push jog2 — start_time at jog1_end (future relative to now_post_fi).
    let rc = engine.push_pieces(0, &[const_piece(jog1_end_cyc, 0.010)], &mut storage);
    assert_eq!(rc, 0);

    let now_post_fi = jog1_start + 2 * TICK + 1;
    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    let retired = engine.retired_counts()[0];

    // No spurious fault: force_idle drained the stale jog1 slot.
    assert_eq!(
        err,
        0,
        "REGRESSION: spurious fault {err} after force_idle drain fix; \
         retired={retired}. ring.drain() must have advanced retired to head \
         so arm_next sees jog2 (future, no fault), not jog1's stale slot."
    );

    // retired == 1 proves drain() advanced the cursor past jog1's slot.
    // retired == 0 would mean drain() never ran and the bug is still present.
    assert_eq!(
        retired,
        1,
        "REGRESSION: retired={retired} after force_idle + jog2 tick. \
         retired==1 proves ring.drain() advanced the cursor past jog1's stale slot \
         (the observable footprint of the fix). \
         retired==0 means drain() was not called and the stale cursor bug persists."
    );
}
