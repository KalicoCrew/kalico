//! Verify that fresh GPIO detection reports the trip but does NOT self-freeze.
//!
//! When the endstop siren is disabled (the local AbortNow on fresh GPIO
//! detection is suppressed), `tick()` must return `Continue` even though
//! the trip event has been queued. The cross-MCU relay is responsible for
//! sending `trsync_trigger`, which freezes via the top-of-tick AbortNow
//! path (TrippedReady|Tripping → AbortNow). That relay path is tested
//! separately and is unaffected by this change.
//!
//! See docs/superpowers/specs/2026-05-31-trsync-cross-mcu-homing-design.md

use runtime::endstop::{
    ArmMsg, ArmPolicy, SourceConfig, SourceKind, TripAction, VelocityAxis, arm, poll_trip,
    set_pin_level, tick,
};

/// Build a minimal `SourceConfig` for a physical GPIO source.
fn gpio_source(gpio: u16) -> SourceConfig {
    SourceConfig {
        kind: SourceKind::Physical,
        gpio,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    }
}

/// Build a single-source `ArmMsg` with the given source and `arm_id=1`.
fn arm_msg(source: SourceConfig) -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; runtime::endstop::MAX_SOURCES];
    sources[0] = source;
    ArmMsg {
        arm_id: 1,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [7, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    }
}

/// Fresh GPIO detection must return `Continue` (siren disabled) AND queue
/// the trip event so the relay can observe it.
#[test]
fn fresh_gpio_trip_returns_continue_and_queues_event() {
    // Use the crate's test_guard (exposed via feature = "test-helpers"):
    // acquires the global endstop mutex and resets all state. This mirrors
    // the pattern used by unit tests in src/endstop/tests.rs.
    let _guard = runtime::endstop::test_guard();

    // Arm a single active-high GPIO source on pin 20.
    arm(arm_msg(gpio_source(20))).expect("arm should succeed");

    // Assert the pin — source is now asserted.
    set_pin_level(20, true);

    // Tick at arm_clock (clock=0): the source should detect the assertion.
    // Siren is disabled: tick() must return Continue, NOT AbortNow.
    let action = tick(0, [0, 0, 0], &[0]);
    assert_eq!(
        action,
        TripAction::Continue,
        "fresh GPIO detection must return Continue (siren disabled); \
         got {action:?} — the local AbortNow has not been suppressed yet"
    );

    // The trip must still be reported: poll_trip() must return Some with
    // the correct arm_id so the relay can observe and dispatch it.
    let event = poll_trip().expect(
        "poll_trip() must return Some after a fresh GPIO trip — \
         the report (publish_snapshot + TRIP_EVENT_QUEUED) must still happen",
    );
    assert_eq!(
        event.arm_id, 1,
        "trip event arm_id must match the armed arm_id"
    );
    assert_eq!(
        event.trip_source_idx, 0,
        "trip event source index must be 0 (first and only source)"
    );
}
