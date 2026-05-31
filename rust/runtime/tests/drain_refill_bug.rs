//! Reproduces the spurious `-308 PieceStartInPast` fault that fires on the
//! two-jog hardware repro.
//!
//! # Root cause (engine.rs:571-586, stepping_state.rs:144-149)
//!
//! `runtime_force_idle` iterates every configured axis and calls
//! `axis.reset_isr_cache()`, which sets `axis.armed = None` and zeroes the
//! step/position counters.  It does NOT call `axis.ring.advance_counter()`.
//!
//! When a piece is mid-play at force_idle time (`armed.is_some()`, piece window
//! still open), the ring descriptor's `tail` and `retired` fields are NOT
//! advanced.  The ring slot of the currently-playing piece therefore remains
//! occupied from the descriptor's perspective (`retired < head`, `tail ==
//! retired % ring_depth`).
//!
//! On the first post-force_idle ISR tick, `get_position_and_velocity`
//! (engine.rs:680-718) takes the following path:
//!
//!   (a) `axis.armed` is `None` — branch 1 (`now < piece_end`) is skipped.
//!   (b) `armed.take()` returns `None` — `is_some() == false` — the guard at
//!       line 704 does NOT call `advance_counter`.
//!   (c) `arm_next` peeks `axis.ring` at `tail` — returns the STALE mid-play
//!       piece that was playing before force_idle.
//!   (d) Fault check: `now.saturating_sub(stale.piece_start_cycles) > 2 *
//!       sample_period_cycles`.  The stale piece started many ticks ago (the
//!       piece was already playing for at least 2 ticks when force_idle fired),
//!       so the check fires `-308 PieceStartInPast` even though the piece's
//!       `piece_end_cycles` extends far into the future.
//!
//! The fault is SPURIOUS — the engine rejects a piece whose window is still
//! open because force_idle left the ring cursor stranded on the un-retired
//! slot.
//!
//! # Placement-invariance (hardware fact #3)
//!
//! The fault fires identically whether jog2 arrives 1.8 s or 127 ms after
//! jog1, because the trigger is the stale ring cursor left by force_idle, not
//! the timing of jog2's pieces.  Even if jog2's pieces have `start_time` far
//! in the future, `arm_next` peeks `tail`, which points at jog1's un-retired
//! slot — not at jog2's slot — and the fault check fires on jog1's stale
//! `start_time`.
//!
//! # Tests in this file
//!
//! 1. `force_idle_mid_piece_moving_axis_faults_spuriously`
//!    Single moving axis.  force_idle fires while a 10 ms piece is mid-window.
//!    Asserts the fault fires (the test is EXPECTED TO FAIL in the sense that
//!    it documents the bug: we assert `last_error == 0` and the engine returns
//!    `-308`, proving the engine is wrong).
//!
//! 2. `force_idle_mid_piece_held_z_axis_faults_spuriously`
//!    Held Z axis (exact hardware geometry: axis 2, 250 ms constant hold piece,
//!    520 MHz clock).  Reproduces the F446 fault path.
//!
//! 3. `force_idle_then_jog2_future_start_faults_spuriously`
//!    force_idle mid-piece, push jog2 with start_time 250 ms in the future,
//!    tick once.  Asserts `last_error == 0`.  The engine fires `-308` on jog1's
//!    stale slot even though jog2 is completely future — proving placement-
//!    invariance is an MCU bug, not a host scheduling defect.
//!
//! 4. `natural_drain_no_force_idle_no_fault` (control)
//!    Same geometry but without force_idle: jog1 drains naturally, jog2 is
//!    pushed with a future start_time, no fault.  Establishes that the normal
//!    path is correct and the bug is gated exclusively on the force_idle path.
//!
//! Tests 1-3 are the bug: they will FAIL (assert `0`, engine returns `-308`).
//! Test 4 is the control: it will PASS.

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::error::FaultCode;
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

/// Axis 0 (moving).  Push a 10 ms piece.  Arm it.  Tick 2 more times while
/// mid-window (well before piece_end).  Issue force_idle.  Tick once more.
///
/// EXPECTED (correct engine): `last_error == 0` — the ring should either be
/// empty (if force_idle drained it) or the ISR should hold the piece at its
/// current position without faulting.
///
/// ACTUAL (bug present): `last_error == -308 PieceStartInPast` — the ring
/// cursor was not advanced by force_idle, so `arm_next` re-peeks the stale
/// jog1 piece at `tail`; its `start_time` is now `3*TICK + 1` cycles in the
/// past, exceeding `FAULT_TOL = 2*TICK`.
///
/// This test is expected to FAIL (assertion fires), proving the MCU bug.
#[test]
#[ignore = "KNOWN BUG (dormant): runtime_force_idle (engine.rs:577) calls reset_isr_cache without ring.advance_counter, stranding the ring cursor on the mid-play slot -> spurious -308. Remove #[ignore] when fixed."]
fn force_idle_mid_piece_moving_axis_faults_spuriously() {
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
    // force_idle.  If the bug is present: retired == 0 (cursor not advanced).
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

    // Diagnostic: if the bug is present, retired is still 0 (cursor stranded).
    // This is the observable MCU-side footprint: advance_counter was never called.
    assert_eq!(
        err,
        0,
        "SPURIOUS FAULT: engine raised {err} (PieceStartInPast=-308) on a \
         piece whose window was still open at force_idle time \
         (piece_end={piece_end_cyc}, now_post_fi={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}). \
         retired_after_force_idle={retired_after_fi}. \
         ROOT CAUSE: runtime_force_idle (engine.rs:571-586) calls \
         axis.reset_isr_cache() (stepping_state.rs:144) which sets \
         armed=None without calling axis.ring.advance_counter(). \
         The ring cursor (tail, retired) is left pointing at the \
         un-retired mid-play piece's slot. On the next tick, arm_next \
         peeks that stale slot and the fault check fires."
    );
}

// ── Test 2: held Z axis, long 250ms hold piece ────────────────────────────────

/// Axis 2 (held Z, F446 path).  250 ms constant piece starting at `TICK`.
/// This is the exact geometry of the Z-axis hold piece in the hardware two-jog
/// repro.  force_idle fires at `piece_start + 2*TICK` (the piece has been live
/// for 2 ticks, window extends 130M more cycles).
///
/// EXPECTED: `last_error == 0`.
/// ACTUAL (bug): `-308 PieceStartInPast` because now - piece_start = 3*TICK+1
/// = 26_001 > FAULT_TOL = 26_000.
///
/// This test is expected to FAIL.
#[test]
#[ignore = "KNOWN BUG (dormant): same root cause as force_idle_mid_piece_moving_axis — reset_isr_cache without ring.advance_counter strands the held-Z hold-piece slot -> spurious -308. Remove #[ignore] when fixed."]
fn force_idle_mid_piece_held_z_axis_faults_spuriously() {
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
        "SPURIOUS FAULT on Z-axis hold piece: engine raised {err} \
         (PieceStartInPast=-308). \
         piece_end={piece_end_cyc}, now={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}, \
         remaining_window={remaining_window} cycles. \
         The piece window was still valid; the fault is spurious. \
         Same root cause as test 1: force_idle (engine.rs:577) calls \
         reset_isr_cache without advance_counter, stranding ring.tail \
         on the un-retired Z hold piece slot."
    );
}

// ── Test 3: placement-invariance — jog2 future start does not prevent fault ───

/// Arm jog1 piece, tick mid-window, force_idle, push jog2 with start_time
/// = now + LEAD_CYCLES (250 ms in the future), tick once.
///
/// EXPECTED: `last_error == 0` — jog2's lateness is 0 (future piece), so even
/// if the engine re-arms jog1's stale slot it should not fault... except it
/// does, on jog1's stale slot.
///
/// This test encodes the placement-invariance observation (hardware fact #3):
/// the fault fires regardless of jog2's start_time, because `arm_next` peeks
/// `ring.tail` which still points at jog1's un-retired slot (not at jog2's
/// slot, which sits at the next position in the ring).
///
/// This test is expected to FAIL.
#[test]
#[ignore = "KNOWN BUG (dormant): proves placement-invariance is the stranded cursor, not host scheduling — arm_next peeks jog1's un-retired slot, faulting -308 even though jog2 is fully future. Remove #[ignore] when fixed."]
fn force_idle_then_jog2_future_start_faults_spuriously() {
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

    // force_idle: clears armed but leaves ring cursor stranded at jog1's slot.
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

    // If the MCU bug is present, retired == 0 (jog1 never retired by
    // advance_counter, proving the fault is on jog1's stale slot, not jog2).
    assert_eq!(
        err,
        0,
        "PLACEMENT-INVARIANT SPURIOUS FAULT: engine raised {err} on jog2 tick \
         (now={now_post_fi}). jog2_start={jog2_start} (lateness=0, future). \
         jog1_start={jog1_start} (lateness={jog1_lateness} > fault_tol={FAULT_TOL}). \
         retired_after_fault={retired} (0 = fault on jog1 stale slot, MCU bug; \
         1 = fault on jog2 stale anchor, host bug). \
         The fault fires on jog1's stale ring.tail slot because force_idle \
         (engine.rs:577) cleared armed=None without calling advance_counter, \
         leaving ring.tail pointing at jog1's un-retired slot. jog2 was never \
         reached by arm_next."
    );
}

// ── Test 4: control — normal drain, no force_idle, no fault ──────────────────

/// Control case: jog1 drains naturally (no force_idle), jog2 pushed with
/// future start_time.  No fault expected, no fault observed.
///
/// Confirms the bug is gated exclusively on the force_idle path.
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

// ── Test 5: classifier — retired count distinguishes MCU vs host bug ──────────

/// Two fault paths produce the same `-308` error code.  The `retired` count
/// at fault time distinguishes them unambiguously:
///
///   - `retired == 0` after fault  →  fault is on jog1's STALE ring slot
///     (MCU bug: force_idle stranded the cursor without calling advance_counter)
///   - `retired == 1` after fault  →  jog1 retired normally; fault is on jog2's
///     stale anchor (host bug: host re-used jog1's end time as jog2's start)
///
/// This test exercises the MCU-bug path and asserts the retired count == 0,
/// confirming the fault is not on a genuinely-past jog2 piece.
///
/// The assertion `last_error == FaultCode::PieceStartInPast` is the bug
/// manifestation (it fires); the `retired == 0` assertion is the classifier.
///
/// This test is expected to compile and run to the `retired == 0` assertion
/// without panicking there — it validates the footprint, not the absence of
/// the fault.
#[test]
fn retired_count_zero_classifies_stale_cursor_mcu_bug() {
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

    // Push jog2 — start_time at jog1_end (stale anchor: host bug territory).
    let rc = engine.push_pieces(0, &[const_piece(jog1_end_cyc, 0.010)], &mut storage);
    assert_eq!(rc, 0);

    let now_post_fi = jog1_start + 2 * TICK + 1;
    engine.tick(now_post_fi, &shared, &mut storage);

    // The fault DOES fire.  Record both the error code and retired count.
    let err = shared.last_error.load(Ordering::Acquire);
    let retired = engine.retired_counts()[0];

    assert_eq!(
        err,
        FaultCode::PieceStartInPast.as_i32(),
        "classifier: the fault must have fired (MCU bug present)"
    );

    // THIS is the load-bearing classifier: retired == 0 proves the fault is
    // on jog1's stale slot (advance_counter was never called by force_idle),
    // NOT on jog2's stale anchor.  If retired were 1, jog1 would have been
    // retired normally and the fault would be on jog2 (host bug).
    assert_eq!(
        retired,
        0,
        "CLASSIFIER: retired must be 0 after force_idle stale-cursor fault. \
         retired=0 means jog1 was never retired (advance_counter not called by \
         force_idle); the fault is on jog1's un-retired stale ring slot, not on \
         jog2. This is the MCU bug fingerprint. \
         (retired=1 would indicate the fault is on jog2 — host bug.)"
    );
}
