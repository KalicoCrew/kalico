use core::sync::atomic::Ordering;

use runtime::clock::WidenState;
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::piece_ring::PieceEntry;
use runtime::state::{IsrState, SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::tick::isr_sample_tick;

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u32 = CLOCK_FREQ / SAMPLE_RATE;

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

fn make_isr(engine: Engine) -> IsrState {
    IsrState {
        engine,
        widen_state: WidenState::default(),
        last_tick_now: None,
    }
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn configure_axis0(engine: &mut Engine) {
    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        0.0125,
        64,
        &[pulse_binding()],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");
}

fn install_queue(engine: &mut Engine) -> ([*mut StepQueue; MAX_AXES], StepQueue) {
    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);
    (qs, q0)
}

fn const_piece(start_time: u64, dur_s: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0; 4],
        duration: dur_s,
        _reserved: 0,
    }
}

#[test]
fn idle_ticks_never_fault_even_with_large_gap() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "idle tick 0 must not fault"
    );

    isr_sample_tick(&mut isr, &shared, &mut storage, 1_000_000_000_u32);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "idle tick with huge gap must not fault — guard must never fire during idle"
    );

    isr_sample_tick(&mut isr, &shared, &mut storage, 1_000_013_000_u32);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "second idle tick must not fault"
    );
}

#[test]
fn active_motion_gap_latches_tick_interval_exceeded() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece_start: u64 = TICK_CYCLES as u64;
    let piece = const_piece(piece_start, 10.0);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first active tick (future piece held at t=0) must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "first active tick must set last_tick_now to Some"
    );

    let gap_raw = TICK_CYCLES * 3;
    isr_sample_tick(&mut isr, &shared, &mut storage, gap_raw);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "gap > 2×period during active motion must latch TickIntervalExceeded"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        detail, 3,
        "fault_detail must equal gap_ticks (gap/period = 3*TICK_CYCLES/TICK_CYCLES = 3), got {detail}"
    );
}

#[test]
fn steady_cadence_of_active_ticks_never_faults() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = const_piece(0, 10.0);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    for i in 0u32..60 {
        let raw = TICK_CYCLES * i;
        isr_sample_tick(&mut isr, &shared, &mut storage, raw);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "no fault expected at active tick {i} on a steady cadence"
        );
    }
}

#[test]
fn idle_active_gap_idle_active_rebaselines() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(
        isr.last_tick_now.is_none(),
        "idle tick must leave last_tick_now as None"
    );

    let piece_a = const_piece(0, 10.0);
    let rc = isr.engine.push_pieces(0, &[piece_a], &mut storage);
    assert_eq!(rc, 0);

    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first active tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "first active tick must set last_tick_now"
    );

    let raw_gap = TICK_CYCLES + TICK_CYCLES * 5;
    isr_sample_tick(&mut isr, &shared, &mut storage, raw_gap);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "5×period gap during active motion must latch TickIntervalExceeded"
    );

    shared.last_error.store(0, Ordering::Release);
    shared.fault_detail.store(0, Ordering::Release);

    let mut engine2 = make_engine();
    configure_axis0(&mut engine2);
    let (_qs2, mut _q02) = install_queue(&mut engine2);
    let mut isr2 = make_isr(engine2);
    let shared2 = SharedState::new();
    let mut storage2 = make_storage();

    isr_sample_tick(&mut isr2, &shared2, &mut storage2, 0);
    assert!(isr2.last_tick_now.is_none());

    let piece_b = const_piece(0, 10.0);
    let rc = isr2.engine.push_pieces(0, &[piece_b], &mut storage2);
    assert_eq!(rc, 0);

    isr_sample_tick(&mut isr2, &shared2, &mut storage2, TICK_CYCLES);
    assert_eq!(shared2.last_error.load(Ordering::Acquire), 0);
    assert!(isr2.last_tick_now.is_some());

    isr_sample_tick(&mut isr2, &shared2, &mut storage2, TICK_CYCLES * 2);
    assert_eq!(shared2.last_error.load(Ordering::Acquire), 0);

    let mut engine3 = make_engine();
    configure_axis0(&mut engine3);
    let (_qs3, mut _q03) = install_queue(&mut engine3);
    let mut isr3 = make_isr(engine3);
    let shared3 = SharedState::new();
    let mut storage3 = make_storage();

    isr_sample_tick(&mut isr3, &shared3, &mut storage3, 0);
    assert!(isr3.last_tick_now.is_none());

    let piece_c = const_piece(0, 10.0);
    isr3.engine.push_pieces(0, &[piece_c], &mut storage3);
    isr_sample_tick(&mut isr3, &shared3, &mut storage3, TICK_CYCLES);
    assert!(isr3.last_tick_now.is_some());

    isr3.last_tick_now = None;

    let large_raw: u32 = TICK_CYCLES * 1000;
    isr_sample_tick(&mut isr3, &shared3, &mut storage3, large_raw);
    assert_eq!(
        shared3.last_error.load(Ordering::Acquire),
        0,
        "active tick after idle (last_tick_now=None) must not fault, even with huge gap"
    );
    assert!(
        isr3.last_tick_now.is_some(),
        "active tick after idle re-baseline must set last_tick_now to Some"
    );
}

#[test]
fn gap_exactly_2x_period_does_not_fault() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = const_piece(0, 10.0);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(isr.last_tick_now.is_some());

    let raw = TICK_CYCLES * 2;
    isr_sample_tick(&mut isr, &shared, &mut storage, raw);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "gap == 2×period must not fault (strictly-greater-than threshold)"
    );
}

#[test]
fn large_gap_saturates_fault_detail_to_0xffff() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let mut isr = make_isr(engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = const_piece(0, 10.0);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(isr.last_tick_now.is_some());

    let gap_ticks_target: u32 = 0x1_0000;
    let gap_raw: u32 = gap_ticks_target * TICK_CYCLES;
    isr_sample_tick(&mut isr, &shared, &mut storage, gap_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "65536-tick gap must latch TickIntervalExceeded"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        detail, 0xFFFF,
        "fault_detail must saturate at 0xFFFF for a {gap_ticks_target}-tick gap, got {detail}"
    );
}
