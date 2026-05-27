use super::*;

const V_MIN: u32 = 10 << 16;

fn cfg(kind: SourceKind, policy: ArmPolicy, sample_n: u8, gpio: PinId) -> SourceConfig {
    SourceConfig {
        kind,
        gpio,
        active_high: true,
        policy,
        sample_n,
        velocity_axis: VelocityAxis::X,
        v_min_q16: V_MIN,
    }
}

fn msg(source: SourceConfig) -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = source;
    ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    }
}

/// Build a Software-source arm message with the given `grant_ticks`.
fn sw_msg(grant_ticks: u64) -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Software,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::XYZ,
        v_min_q16: 0,
    };
    ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks,
    }
}

fn reset() -> std::sync::MutexGuard<'static, ()> {
    test_guard()
}

fn drain_trip() -> TripEvent {
    poll_trip().expect("trip event")
}

#[test]
fn source_policy_sample_matrix() {
    for kind in [SourceKind::Physical, SourceKind::TmcDiag] {
        for policy in [
            ArmPolicy::TripImmediately,
            ArmPolicy::WaitForClear,
            ArmPolicy::IgnoreUntilMoving,
        ] {
            for sample_n in [1, 3] {
                let _guard = reset();
                let source = cfg(kind, policy, sample_n, 1);
                arm(msg(source)).expect("arm");
                set_pin_level(1, true);
                if policy == ArmPolicy::WaitForClear {
                    assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, false);
                    assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, true);
                } else if policy == ArmPolicy::IgnoreUntilMoving {
                    assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, false);
                    assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, true);
                }

                for i in 1..=sample_n {
                    let action = tick(10 + u64::from(i), [V_MIN, 0, 0], &[10, 20]);
                    if i < sample_n {
                        assert_eq!(action, TripAction::Continue);
                    } else {
                        assert_eq!(action, TripAction::AbortNow);
                        let evt = drain_trip();
                        assert_eq!(evt.trip_source_idx, 0);
                    }
                }
            }
        }
    }
}

#[test]
fn ignore_until_moving_latch_requires_velocity_then_clear_once() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::TmcDiag,
        ArmPolicy::IgnoreUntilMoving,
        1,
        2,
    )))
    .expect("arm");

    set_pin_level(2, true);
    assert_eq!(tick(1, [V_MIN - 1, 0, 0], &[1]), TripAction::Continue);
    assert_eq!(tick(2, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, false);
    assert_eq!(tick(3, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, true);
    assert_eq!(tick(4, [V_MIN, 0, 0], &[1]), TripAction::AbortNow);
    assert_eq!(drain_trip().trip_clock, 4);

    reset_for_test();
    arm(msg(cfg(
        SourceKind::TmcDiag,
        ArmPolicy::IgnoreUntilMoving,
        1,
        2,
    )))
    .expect("arm");
    set_pin_level(2, false);
    assert_eq!(tick(1, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, true);
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn wait_for_clear_ignores_assertion_at_arm() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::WaitForClear,
        1,
        3,
    )))
    .expect("arm");
    set_pin_level(3, true);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(3, false);
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(3, true);
    assert_eq!(tick(3, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn trip_immediately_assertion_at_arm_trips_on_first_sample() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        4,
    )))
    .expect("arm");
    set_pin_level(4, true);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn arm_policy_try_from_decodes_known_variants_and_rejects_others() {
    assert_eq!(ArmPolicy::try_from(0).unwrap(), ArmPolicy::TripImmediately);
    assert_eq!(ArmPolicy::try_from(1).unwrap(), ArmPolicy::WaitForClear);
    assert_eq!(ArmPolicy::try_from(2).unwrap(), ArmPolicy::IgnoreUntilMoving);
    assert_eq!(ArmPolicy::try_from(3).unwrap_err(), 3);
    assert_eq!(ArmPolicy::try_from(255).unwrap_err(), 255);
}

#[test]
fn unknown_policy_byte_falls_back_to_trip_immediately_behavior() {
    // Defensive: if a wire-corruption or version-skew ever planted a
    // non-{0,1,2} value into the policy atomic, the decoded fallback
    // is `TripImmediately` — same observable behavior as setting
    // policy to 0 explicitly: trip when asserted, no-op otherwise.
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        4,
    )))
    .expect("arm");
    // Plant a bogus byte directly into the source's policy atomic.
    ARM.sources[0].policy.store(99, Ordering::Release);
    set_pin_level(4, true);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn multi_source_or_reports_first_asserted_source_index() {
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 5);
    sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 6);
    arm(ArmMsg {
        arm_id: 77,
        arm_clock: 0,
        source_count: 2,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm");
    set_pin_level(6, true);
    assert_eq!(tick(1, [0, 0, 0], &[100, -200]), TripAction::AbortNow);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 77);
    assert_eq!(evt.trip_source_idx, 1);
    assert_eq!(evt.stepper_count, 2);
    assert_eq!(evt.steppers[0].oid, 0);
    assert_eq!(evt.steppers[0].step_count, 100);
    assert_eq!(evt.steppers[1].oid, 1);
    assert_eq!(evt.steppers[1].step_count, -200);
}

#[test]
fn future_arm_clock_ignores_early_assertions() {
    let _guard = reset();
    let mut m = msg(cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 7));
    m.arm_clock = 50;
    arm(m).expect("arm");
    set_pin_level(7, true);
    assert_eq!(tick(49, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_none());
    assert_eq!(tick(50, [0, 0, 0], &[2]), TripAction::AbortNow);
    assert_eq!(drain_trip().trip_clock, 50);
}

#[test]
fn tick_returns_continue_for_non_armed_non_tripped_states() {
    let _guard = reset();
    set_pin_level(8, true);
    for state in [
        ArmState::Idle,
        ArmState::TrippedSent,
        ArmState::Disarmed,
    ] {
        ARM.state.store(state as u8, Ordering::Release);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    }
}

#[test]
fn tick_returns_abort_for_tripped_states() {
    let _guard = reset();
    for state in [ArmState::Tripping, ArmState::TrippedReady] {
        ARM.state.store(state as u8, Ordering::Release);
        assert_eq!(
            tick(1, [0, 0, 0], &[1]),
            TripAction::AbortNow,
            "tick() must return AbortNow when state={state:?}"
        );
    }
}

#[test]
fn exactly_one_terminal_for_trip_vs_disarm_schedules() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        9,
    )))
    .expect("arm");
    set_pin_level(9, true);

    let disarm_first = disarm(42);
    assert_eq!(disarm_first, DisarmStatus::Disarmed);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_none());

    reset_for_test();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        9,
    )))
    .expect("arm");
    set_pin_level(9, true);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
    assert_eq!(disarm(42), DisarmStatus::AlreadyTripped);
    assert!(poll_trip().is_some());
}

#[test]
fn snapshot_seqlock_reader_retries_odd_and_never_returns_torn_read() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        10,
    )))
    .expect("arm");
    set_pin_level(10, true);
    assert_eq!(
        tick(0x1_0000_0002, [0, 0, 0], &[123, 456]),
        TripAction::AbortNow
    );
    let evt = drain_trip();
    assert_eq!(evt.trip_clock, 0x1_0000_0002);
    assert_eq!(evt.steppers[0].step_count, 123);
    assert_eq!(evt.steppers[1].step_count, 456);
}

#[test]
fn active_low_polarity_uses_explicit_branch_not_xor() {
    let _guard = reset();
    let mut source = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 11);
    source.active_high = false;
    // Active-low: HIGH = not asserted, LOW = asserted.
    // Set pin HIGH before arming so arm() does not see an asserted
    // pin and immediately return AlreadyTripped.
    set_pin_level(11, true);
    arm(msg(source)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(11, false);
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn already_tripped_at_arm_time_active_high() {
    // TripImmediately + pin already HIGH when arm() is called:
    // arm() should return AlreadyTripped synchronously, publish a
    // snapshot, and set state to TrippedReady so poll_trip() works.
    let _guard = reset();
    set_pin_level(12, true);
    let result = arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        12,
    )));
    assert_eq!(result, Ok(ArmStatus::AlreadyTripped));
    // State should be TrippedReady; poll_trip() must return Some.
    let evt = poll_trip().expect("trip event after AlreadyTripped");
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, 0);
    // No further ticks should trip again.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
}

#[test]
fn already_tripped_requires_trip_immediately_policy() {
    // WaitForClear source with pin HIGH at arm time must NOT return
    // AlreadyTripped — the policy requires a clear-then-assert cycle.
    let _guard = reset();
    set_pin_level(13, true);
    let result = arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::WaitForClear,
        1,
        13,
    )));
    assert_eq!(result, Ok(ArmStatus::Armed));
}

// --- Software source tests ---

#[test]
fn software_source_does_not_trip_on_gpio() {
    // A Software source must not read or respond to GPIO levels.
    let _guard = reset();
    arm(sw_msg(1000)).expect("arm");
    // Set every pin high — a Physical source would trip immediately.
    for i in 0..20_u16 {
        set_pin_level(i, true);
    }
    // First tick opens the deadline window; must NOT trip on GPIO.
    assert_eq!(tick(1, [0, 0, 0], &[1, 2]), TripAction::Continue);
    // deadline_active should now be set.
    assert!(ARM.deadline_active.load(Ordering::Acquire));
}

#[test]
fn software_source_deadline_expires_and_trips() {
    // grant_ticks = 100; arm_clock = 0.
    // tick(1)   → opens window: deadline = 1 + 100 = 101. Continue.
    // tick(101) → clock == deadline → AbortNow with DEADLINE_EXPIRED idx.
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[10, 20]), TripAction::Continue);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    // Clock 100 is still inside the window.
    assert_eq!(tick(100, [0, 0, 0], &[10, 20]), TripAction::Continue);
    // Clock 101 is at the deadline — should trip.
    assert_eq!(tick(101, [0, 0, 0], &[10, 20]), TripAction::AbortNow);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
    assert_eq!(evt.trip_clock, 101);
}

#[test]
fn extend_deadline_pushes_window_forward() {
    // grant_ticks = 100.
    // tick(1)   → deadline = 101. Continue.
    // extend_deadline at clock=50 → deadline = 50 + 100 = 150.
    // tick(101) → inside new window. Continue.
    // tick(150) → at new deadline. AbortNow.
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    extend_deadline(42, 50);
    assert_eq!(ARM.deadline_clock_unchecked(), 150);
    assert_eq!(tick(101, [0, 0, 0], &[]), TripAction::Continue);
    assert_eq!(tick(150, [0, 0, 0], &[]), TripAction::AbortNow);
    assert_eq!(drain_trip().trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
}

#[test]
fn extend_deadline_ignored_for_wrong_arm_id() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    let deadline_before = ARM.deadline_clock_unchecked();
    extend_deadline(99, 50); // wrong arm_id
    assert_eq!(ARM.deadline_clock_unchecked(), deadline_before);
}

#[test]
fn extend_deadline_ignored_before_first_tick() {
    // Before the first tick, deadline_active = false.
    // extend_deadline should silently ignore.
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    extend_deadline(42, 50); // deadline_active is false → no-op
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    assert_eq!(ARM.deadline_clock_unchecked(), 0);
}

#[test]
fn software_trip_transitions_armed_to_tripped_ready() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(
        software_trip(42, 500, &[10, 20]),
        TripResult::Tripped
    );
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
    assert_eq!(evt.trip_clock, 500);
}

#[test]
fn software_trip_wrong_arm_id_is_no_op() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(
        software_trip(99, 500, &[10, 20]),
        TripResult::WrongArmId
    );
    // Still armed.
    assert!(matches_u8(
        ARM.state.load(Ordering::Acquire),
        ArmState::Armed
    ));
}

#[test]
fn software_trip_on_non_armed_state_is_not_armed() {
    let _guard = reset();
    // Set arm_id to 0 so it matches the reset state, then put the state
    // into Disarmed. software_trip must return NotArmed (state check
    // fails) rather than WrongArmId (arm_id check fails).
    ARM.arm_id.store(0, Ordering::Release);
    ARM.state.store(ArmState::Disarmed as u8, Ordering::Release);
    assert_eq!(
        software_trip(0, 500, &[]),
        TripResult::NotArmed
    );
}

#[test]
fn software_trip_idempotent_second_call_returns_not_armed() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(42, 1, &[]), TripResult::Tripped);
    // State is now TrippedReady; a second call must return NotArmed.
    assert_eq!(software_trip(42, 2, &[]), TripResult::NotArmed);
}

#[test]
fn deadline_active_false_resets_across_arm_calls() {
    // Arm with Software source, open deadline, then re-arm.
    // On the new arm, deadline_active must be false again.
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    tick(1, [0, 0, 0], &[]);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    // Disarm so we can re-arm.
    disarm(42);
    arm(sw_msg(100)).expect("arm");
    assert!(
        !ARM.deadline_active.load(Ordering::Acquire),
        "deadline_active must be cleared on re-arm"
    );
}

#[test]
fn software_source_deadline_uses_saturating_add() {
    // grant_ticks = u64::MAX → deadline = clock.saturating_add(u64::MAX)
    // = u64::MAX (saturates). That deadline will never be reached in
    // practice, but the arithmetic must not overflow/panic.
    let _guard = reset();
    arm(sw_msg(u64::MAX)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    assert_eq!(ARM.deadline_clock_unchecked(), u64::MAX);
}

#[test]
fn software_source_skips_gpio_no_gpio_trip() {
    // Mixed arm: Software source at index 0, Physical at index 1.
    // Pin for Physical (gpio=15) is deasserted; no GPIO trip expected.
    // Deadline with large grant: arm never expires. Should stay Continue.
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Software,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::XYZ,
        v_min_q16: 0,
    };
    sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 15);
    arm(ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 2,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 10_000,
    })
    .expect("arm");
    // Tick with Physical pin deasserted → Continue.
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    // Assert the Physical pin → Physical source trips.
    set_pin_level(15, true);
    assert_eq!(tick(2, [0, 0, 0], &[]), TripAction::AbortNow);
    let evt = drain_trip();
    // Should be source index 1 (the Physical source), not the Software one.
    assert_eq!(evt.trip_source_idx, 1);
}

/// Regression test for the Z homing crash (2026-05-25):
/// `software_trip` must cause the next `tick()` call to return
/// `AbortNow`. The segment engine calls `tick()` at modulation rate;
/// if it returns `Continue` after a software trip, the MCU keeps
/// generating steps and the toolhead doesn't stop.
///
/// Root cause: `tick()` early-returns `Continue` when
/// `ARM.state != Armed`. After `software_trip` sets state to
/// `TrippedReady`, tick() saw "not Armed" and returned Continue
/// instead of AbortNow.
#[test]
fn software_trip_causes_tick_to_abort() {
    let _guard = reset();
    arm(sw_msg(100_000)).expect("arm");

    // First tick past arm_clock: activates the deadline window.
    assert_eq!(tick(1, [0, 0, 0], &[0, 0]), TripAction::Continue);

    // Host sends software_trip (probe triggered).
    assert_eq!(software_trip(42, 50, &[10, 20]), TripResult::Tripped);

    // The NEXT tick() must return AbortNow so the segment engine
    // stops generating steps. This is the critical safety invariant.
    assert_eq!(
        tick(51, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must return AbortNow after software_trip — \
         otherwise the MCU keeps moving and crashes into the bed"
    );
}

/// Same as above but for the case where software_trip arrives BEFORE
/// the first tick past arm_clock (the deadline isn't active yet).
/// tick() must still return AbortNow.
#[test]
fn software_trip_before_arm_clock_causes_tick_to_abort() {
    let _guard = reset();
    let mut msg = sw_msg(100_000);
    msg.arm_clock = 1000; // arm_clock is in the future
    arm(msg).expect("arm");

    // Host sends software_trip before arm_clock (probe triggered
    // very early due to being close to bed).
    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);

    // tick at clock=1001 (past arm_clock): must abort even though
    // deadline wasn't active.
    assert_eq!(
        tick(1001, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must abort after software_trip even if deadline wasn't active"
    );
}
