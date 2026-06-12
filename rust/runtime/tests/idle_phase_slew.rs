use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;

fn make_engine_with_phase_axis() -> Engine {
    let mut engine = Engine::new(CLOCK_FREQ, SAMPLE_RATE);
    let binding = StepperBindingRust {
        stepper_oid: 5,
        tmc_cs_oid: 7,
        _pad: [0; 2],
    };
    let rc = engine.configure_axis(
        0,
        StepMode::Phase,
        0.000_625,
        64,
        &[binding],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");
    engine
}

fn install_queues(engine: &mut Engine, q0: &mut StepQueue) {
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = q0;
    engine.test_install_step_queues(qs);
}

#[test]
fn jog_slews_to_target_while_no_motion_is_armed() {
    let mut engine = make_engine_with_phase_axis();
    let mut q0 = StepQueue::new();
    install_queues(&mut engine, &mut q0);
    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];
    let shared = SharedState::new();
    shared.phase_motor_count.store(1, Ordering::Release);
    shared.phase_slot_idx[0].store(0, Ordering::Release);

    assert_eq!(engine.phase_jog_to(&shared, 5, 20, 1), 0);
    let q = engine.phase_state(5).expect("stepper must be found");
    assert!(!q.settled, "jog must leave a pending slew");

    for n in 1..=64_u64 {
        engine.tick(n * TICK_CYCLES, &shared, &mut storage);
    }

    let q = engine.phase_state(5).expect("stepper must be found");
    assert!(
        q.settled,
        "idle ticks must ramp the offset to the jog target (phase={})",
        q.phase
    );
    assert_eq!(q.phase, 20);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault during idle slew"
    );
}

#[test]
fn idle_pulse_axis_does_not_dispatch() {
    let mut engine = Engine::new(CLOCK_FREQ, SAMPLE_RATE);
    let binding = StepperBindingRust {
        stepper_oid: 5,
        tmc_cs_oid: 7,
        _pad: [0; 2],
    };
    assert_eq!(
        engine.configure_axis(
            0,
            StepMode::Pulse,
            0.000_625,
            64,
            &[binding],
            TOTAL_RING_PIECES
        ),
        0
    );
    let mut q0 = StepQueue::new();
    install_queues(&mut engine, &mut q0);
    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];
    let shared = SharedState::new();

    let active = engine.tick(TICK_CYCLES, &shared, &mut storage);
    assert!(!active, "idle pulse axis must not report active");
    assert_eq!(q0.tail, q0.head, "no steps enqueued while idle");
}
