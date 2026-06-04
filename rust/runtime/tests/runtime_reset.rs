//! Coverage for `Engine::reset()` — the host-issued, idempotent clean-state
//! reset that rewinds the ring bump allocator on every (re)connect.

use runtime::engine::{Engine, RuntimeStatus};
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

fn new_engine() -> Engine {
    Engine::new(520_000_000, 40_000)
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

#[test]
fn reset_clears_axis_state() {
    let mut e = new_engine();
    let b = pulse_binding();
    assert_eq!(
        e.configure_axis(0, StepMode::Pulse, 0.0125, 64, &[b], 512),
        0
    );
    assert_eq!(
        e.configure_axis(1, StepMode::Pulse, 0.0125, 64, &[b], 512),
        0
    );
    assert_eq!(e.num_axes, 2);

    e.reset();

    assert_eq!(e.num_axes, 0, "num_axes not cleared");
    assert!(
        e.stepping_axes.iter().all(|a| a.is_none()),
        "axes not cleared"
    );
    assert_eq!(e.status(), RuntimeStatus::Idle, "status not Idle");
    assert_eq!(e.last_error(), 0, "last_error not cleared");
}

#[test]
fn reset_reclaims_ring_allocation() {
    let mut e = new_engine();
    let b = pulse_binding();
    // Fill the 512-piece pool: two 256-deep axes -> cursor at 512.
    assert_eq!(
        e.configure_axis(0, StepMode::Pulse, 0.0125, 256, &[b], 512),
        0
    );
    assert_eq!(
        e.configure_axis(1, StepMode::Pulse, 0.0125, 256, &[b], 512),
        0
    );
    // A third allocation must now overflow (the bug, pre-reset).
    assert_ne!(
        e.configure_axis(2, StepMode::Pulse, 0.0125, 256, &[b], 512),
        0,
        "expected RING_FULL before reset"
    );

    e.reset();

    // After reset the cursor is rewound, so the same configuration fits again.
    assert_eq!(
        e.configure_axis(0, StepMode::Pulse, 0.0125, 256, &[b], 512),
        0
    );
    assert_eq!(
        e.configure_axis(1, StepMode::Pulse, 0.0125, 256, &[b], 512),
        0
    );
}

#[test]
fn reset_is_idempotent_on_fresh_engine() {
    let mut e = new_engine();
    e.reset();
    let b = pulse_binding();
    // A fresh reset must not consume allocator space — a full-pool axis fits.
    assert_eq!(
        e.configure_axis(0, StepMode::Pulse, 0.0125, 512, &[b], 512),
        0
    );
}

#[test]
fn reset_preserves_clock_config() {
    let mut e = new_engine();
    let sample_period = e.sample_period_cycles;
    let cps = e.cycles_per_second;
    e.reset();
    assert_eq!(
        e.sample_period_cycles, sample_period,
        "sample period changed"
    );
    assert!(
        (e.cycles_per_second - cps).abs() < f32::EPSILON,
        "cycles/s changed"
    );
}
