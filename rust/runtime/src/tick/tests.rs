#![allow(clippy::integer_division)]

use core::sync::atomic::Ordering;

use crate::clock::TEST_ONLY_TICK_RATE_HZ;
use crate::endstop::{
    ArmMsg, ArmPolicy, SourceConfig, SourceKind, VelocityAxis, poll_trip, set_pin_level,
};
use crate::engine::Engine;
use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const CLOCK_FREQ: u32 = 520_000_000;

/// `Engine::new` with the standard test clock rate must produce an engine
/// with the correct `sample_period_cycles`:
///   `cycles = round(clock_freq / sample_rate)`.
///
/// Guards against the sample-period computation being zeroed or
/// misconfigured, which would silently disable the fault-tolerance check
/// in `get_position_and_velocity` (the `> 2 * sample_period_cycles` guard
/// degenerates to `> 0` when `sample_period_cycles == 0`).
#[test]
fn engine_new_has_correct_sample_period() {
    let engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);
    let expected_cycles = (CLOCK_FREQ + TEST_ONLY_TICK_RATE_HZ / 2) / TEST_ONLY_TICK_RATE_HZ;
    assert_eq!(
        engine.sample_period_cycles, expected_cycles,
        "sample_period_cycles must equal round(clock_freq / sample_rate); \
         got {}, expected {expected_cycles}",
        engine.sample_period_cycles
    );
    assert!(
        engine.sample_period_cycles > 0,
        "sample_period_cycles must be > 0 (a zero value disables the fault-tolerance guard)"
    );
}

/// An `Engine` constructed with `new` and no pieces configured must have
/// `num_axes == 0` and all retired_counts equal to 0.
#[test]
fn engine_new_starts_idle() {
    let engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);
    assert_eq!(
        engine.num_axes, 0,
        "freshly-constructed engine must have 0 axes"
    );
    let rc = engine.retired_counts();
    assert!(
        rc.iter().all(|&c| c == 0),
        "all retired_counts must be 0 at startup; got {rc:?}"
    );
}

/// Regression: `endstop::tick` was never called from the engine modulation tick.
/// Pins were sampled into `PIN_LEVELS` every tick but never evaluated, so no
/// GPIO arm ever published a trip event and `poll_trip()` always returned `None`.
///
/// Verify: arm a GPIO source, assert the pin via `set_pin_level`, drive two engine
/// ticks (one idle — no armed motion — is enough), then assert `poll_trip()` is
/// populated with the expected arm_id, source index, and step counts.
///
/// The engine has no active motion pieces so all axes are idle and the axis loop
/// skips all axes. The endstop evaluation at the bottom of `engine.tick` is
/// reached unconditionally.
#[test]
#[allow(clippy::unwrap_used)]
fn engine_tick_evaluates_armed_gpio_endstop() {
    let _guard = crate::endstop::test_guard();

    let shared = SharedState::new();
    let mut storage = vec![
        PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 };
        crate::state::TOTAL_RING_PIECES
    ];
    let mut engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);

    // Configure axis 0 with stepper OID=0 so the engine knows about it and
    // position_count is seeded to a known value (0 after default construction).
    let binding = StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    };
    assert_eq!(
        engine.configure_axis(
            0,
            StepMode::Pulse,
            0.0125_f32,
            16,
            &[binding],
            crate::state::TOTAL_RING_PIECES,
        ),
        crate::error::KALICO_OK,
        "configure_axis must succeed"
    );
    // Write a known position so we can verify it ends up in the trip snapshot.
    engine
        .stepping_axes
        .get_mut(0)
        .and_then(|s| s.as_mut())
        .unwrap()
        .steppers
        .first()
        .unwrap()
        .position_count
        .store(1234, Ordering::Release);

    // Arm a TripImmediately Physical source on gpio 30, v_min=0, stepper OID=0.
    let mut sources = [SourceConfig::EMPTY; crate::endstop::MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: 30,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    crate::endstop::arm(ArmMsg {
        arm_id: 7,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
    })
    .expect("arm must succeed");

    // Assert the GPIO pin — engine tick must detect it.
    set_pin_level(30, true);

    // Drive one engine tick at clock=0. No motion pieces are loaded so the axis
    // loop produces nothing; the endstop evaluation runs regardless.
    engine.tick(0, &shared, &mut storage);

    let event = poll_trip().expect(
        "poll_trip() must return Some after engine.tick detects an asserted GPIO — \
         endstop::tick was not called from engine.tick (regression)",
    );
    assert_eq!(event.arm_id, 7, "arm_id must match");
    assert_eq!(
        event.trip_source_idx, 0,
        "trip_source_idx must be 0 (first source)"
    );
    assert_eq!(event.stepper_count, 1, "stepper_count must be 1");
    assert_eq!(
        event.steppers[0].oid, 0,
        "stepper oid must be 0"
    );
    assert_eq!(
        event.steppers[0].step_count, 1234,
        "step_count must match position_count seeded above: \
         stepper_counts is OID-indexed so position_count[oid=0]=1234 must appear here"
    );
}

/// Verify that a pin that is asserted at arm time but transitions through
/// `WaitForClear` is only detected after it clears and re-asserts — and that
/// this detection path reaches `poll_trip()` via the engine tick, not
/// `endstop::tick` being called directly.
#[test]
#[allow(clippy::unwrap_used)]
fn engine_tick_wait_for_clear_detected_via_engine_tick() {
    let _guard = crate::endstop::test_guard();

    let shared = SharedState::new();
    let mut storage = vec![
        PieceEntry { start_time: 0, coeffs: [0.0; 4], duration: 0.0, _reserved: 0 };
        crate::state::TOTAL_RING_PIECES
    ];
    let mut engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);

    let mut sources = [SourceConfig::EMPTY; crate::endstop::MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: 31,
        active_high: true,
        policy: ArmPolicy::WaitForClear,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    crate::endstop::arm(ArmMsg {
        arm_id: 8,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
    })
    .expect("arm must succeed");

    // Pin is asserted at arm time — WaitForClear must ignore it until it clears.
    set_pin_level(31, true);
    engine.tick(0, &shared, &mut storage);
    assert!(
        poll_trip().is_none(),
        "tick 0: WaitForClear must not trip while pin is persistently asserted"
    );

    // Pin clears — latches cleared flag.
    set_pin_level(31, false);
    engine.tick(1, &shared, &mut storage);
    assert!(poll_trip().is_none(), "tick 1: no trip while pin is low");

    // Pin re-asserts — now it should trip.
    set_pin_level(31, true);
    engine.tick(2, &shared, &mut storage);
    let event = poll_trip().expect(
        "poll_trip() must return Some after WaitForClear sequence driven through engine.tick",
    );
    assert_eq!(event.arm_id, 8);
    assert_eq!(event.trip_source_idx, 0);
}
