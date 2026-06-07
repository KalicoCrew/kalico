use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;
const FAULT_TOL: u64 = TICK * 2;
const LEAD_CYCLES: u64 = 130_000_000;

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

    engine.tick(piece_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault at arm"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece armed, not yet retired"
    );

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

    engine.runtime_force_idle(&shared);

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
        err, 0,
        "REGRESSION: engine raised {err} (PieceStartInPast=-308) on a \
         piece whose window was still open at force_idle time \
         (piece_end={piece_end_cyc}, now_post_fi={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}). \
         retired_after_force_idle={retired_after_fi}. \
         FIX: runtime_force_idle must call axis.ring.drain() after \
         axis.reset_isr_cache() to advance the ring cursor past stale slots."
    );
}

#[test]
fn force_idle_mid_piece_held_z_axis_no_spurious_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 2, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let piece_start = TICK;
    let piece_dur = 0.250_f32;
    let piece_end_cyc = piece_end(piece_start, piece_dur);

    assert!(
        piece_end_cyc > piece_start + 1_000 * TICK,
        "precondition: hold piece must extend >> 1000 ticks past start"
    );

    let rc = engine.push_pieces(2, &[const_piece(piece_start, piece_dur)], &mut storage);
    assert_eq!(rc, 0, "push Z hold piece");

    engine.tick(piece_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault at arm"
    );

    engine.tick(piece_start + TICK, &shared, &mut storage);
    engine.tick(piece_start + 2 * TICK, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault mid-window"
    );

    engine.runtime_force_idle(&shared);

    let now_post_fi = piece_start + 2 * TICK + 1;
    let lateness = now_post_fi.saturating_sub(piece_start);
    let remaining_window = piece_end_cyc - now_post_fi;

    assert!(
        lateness > FAULT_TOL,
        "precondition: lateness={lateness} > fault_tol={FAULT_TOL}"
    );

    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        err, 0,
        "REGRESSION on Z-axis hold piece: engine raised {err} \
         (PieceStartInPast=-308). \
         piece_end={piece_end_cyc}, now={now_post_fi}, \
         lateness={lateness}, fault_tol={FAULT_TOL}, \
         remaining_window={remaining_window} cycles. \
         FIX: force_idle must call ring.drain() to advance the cursor past \
         the un-retired Z hold piece slot."
    );
}

#[test]
fn force_idle_then_jog2_future_start_no_spurious_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let jog1_start = TICK;
    let jog1_dur = 0.010_f32;
    let jog1_end_cyc = piece_end(jog1_start, jog1_dur);

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

    engine.runtime_force_idle(&shared);

    let jog2_start = now_at_fi + LEAD_CYCLES;
    let jog2_dur = 0.010_f32;
    let rc = engine.push_pieces(0, &[const_piece(jog2_start, jog2_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog2 (future start)");

    let now_post_fi = now_at_fi + 1;
    let jog2_lateness = now_post_fi.saturating_sub(jog2_start);
    assert_eq!(jog2_lateness, 0, "jog2 lateness must be 0 (future piece)");

    let jog1_lateness = now_post_fi.saturating_sub(jog1_start);
    assert!(
        jog1_lateness > FAULT_TOL,
        "precondition: jog1_lateness={jog1_lateness} > fault_tol={FAULT_TOL}"
    );

    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    let retired = engine.retired_counts()[0];

    assert_eq!(
        err, 0,
        "REGRESSION: engine raised {err} on jog2 tick \
         (now={now_post_fi}). jog2_start={jog2_start} (lateness=0, future). \
         jog1_start={jog1_start} (lateness={jog1_lateness} > fault_tol={FAULT_TOL}). \
         retired_after_tick={retired}. \
         FIX: force_idle must call ring.drain() so arm_next peeks jog2 \
         (future, no fault), not jog1's stale slot."
    );
}

#[test]
fn natural_drain_no_force_idle_no_fault() {
    let mut engine = make_engine();
    configure_axis(&mut engine, 0, 64);
    let mut storage = make_storage();
    let (_qs, shared) = setup_queues(&mut engine);

    let jog1_start = TICK;
    let jog1_dur = 0.001_f32;
    let jog1_end_cyc = piece_end(jog1_start, jog1_dur);

    let rc = engine.push_pieces(0, &[const_piece(jog1_start, jog1_dur)], &mut storage);
    assert_eq!(rc, 0, "push jog1");

    engine.tick(jog1_start, &shared, &mut storage);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "arm jog1 ok");
    assert_eq!(engine.retired_counts()[0], 0);

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

    let jog2_start = jog1_end_cyc + LEAD_CYCLES;
    let rc = engine.push_pieces(0, &[const_piece(jog2_start, 0.010)], &mut storage);
    assert_eq!(rc, 0, "push jog2");

    engine.tick(jog1_end_cyc + 1, &shared, &mut storage);
    let err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        err, 0,
        "CONTROL CASE: normal drain + future jog2 must NOT fault; got {err}"
    );
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "jog2 armed but still playing — retired must remain 1"
    );
}

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

    engine.runtime_force_idle(&shared);

    let rc = engine.push_pieces(0, &[const_piece(jog1_end_cyc, 0.010)], &mut storage);
    assert_eq!(rc, 0);

    let now_post_fi = jog1_start + 2 * TICK + 1;
    engine.tick(now_post_fi, &shared, &mut storage);

    let err = shared.last_error.load(Ordering::Acquire);
    let retired = engine.retired_counts()[0];

    assert_eq!(
        err, 0,
        "REGRESSION: spurious fault {err} after force_idle drain fix; \
         retired={retired}. ring.drain() must have advanced retired to head \
         so arm_next sees jog2 (future, no fault), not jog1's stale slot."
    );

    assert_eq!(
        retired, 1,
        "REGRESSION: retired={retired} after force_idle + jog2 tick. \
         retired==1 proves ring.drain() advanced the cursor past jog1's stale slot \
         (the observable footprint of the fix). \
         retired==0 means drain() was not called and the stale cursor bug persists."
    );
}
