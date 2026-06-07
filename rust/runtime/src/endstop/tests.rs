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
    assert_eq!(
        ArmPolicy::try_from(2).unwrap(),
        ArmPolicy::IgnoreUntilMoving
    );
    assert_eq!(ArmPolicy::try_from(3).unwrap_err(), 3);
    assert_eq!(ArmPolicy::try_from(255).unwrap_err(), 255);
}

#[test]
fn unknown_policy_byte_falls_back_to_trip_immediately_behavior() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        4,
    )))
    .expect("arm");
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
    for state in [ArmState::Idle, ArmState::TrippedSent, ArmState::Disarmed] {
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
    // For active-low, HIGH = not asserted. Set HIGH before arming so arm()
    // does not see an asserted pin and immediately return AlreadyTripped.
    set_pin_level(11, true);
    arm(msg(source)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(11, false);
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::AbortNow);
}

#[test]
fn already_tripped_at_arm_time_active_high() {
    let _guard = reset();
    set_pin_level(12, true);
    let result = arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        12,
    )));
    assert_eq!(result, Ok(ArmStatus::AlreadyTripped));
    let evt = poll_trip().expect("trip event after AlreadyTripped");
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, 0);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
}

#[test]
fn already_tripped_requires_trip_immediately_policy() {
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

#[test]
fn software_source_does_not_trip_on_gpio() {
    let _guard = reset();
    arm(sw_msg(1000)).expect("arm");
    for i in 0..20_u16 {
        set_pin_level(i, true);
    }
    assert_eq!(tick(1, [0, 0, 0], &[1, 2]), TripAction::Continue);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
}

#[test]
fn software_source_deadline_expires_and_trips() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[10, 20]), TripAction::Continue);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    assert_eq!(tick(100, [0, 0, 0], &[10, 20]), TripAction::Continue);
    assert_eq!(tick(101, [0, 0, 0], &[10, 20]), TripAction::AbortNow);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
    assert_eq!(evt.trip_clock, 101);
}

#[test]
fn extend_deadline_pushes_window_forward() {
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
    extend_deadline(99, 50);
    assert_eq!(ARM.deadline_clock_unchecked(), deadline_before);
}

#[test]
fn extend_deadline_ignored_before_first_tick() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    extend_deadline(42, 50);
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    assert_eq!(ARM.deadline_clock_unchecked(), 0);
}

#[test]
fn software_trip_transitions_armed_to_tripped_ready() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
    assert_eq!(evt.trip_clock, 500);
}

#[test]
fn software_trip_wrong_arm_id_is_no_op() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(99, 500, &[10, 20]), TripResult::WrongArmId);
    assert!(matches_u8(
        ARM.state.load(Ordering::Acquire),
        ArmState::Armed
    ));
}

#[test]
fn software_trip_on_non_armed_state_is_not_armed() {
    let _guard = reset();
    // Use arm_id=0 to match reset state so the state check (NotArmed) fires
    // before the arm_id check (WrongArmId) — both values must agree.
    ARM.arm_id.store(0, Ordering::Release);
    ARM.state.store(ArmState::Disarmed as u8, Ordering::Release);
    assert_eq!(software_trip(0, 500, &[]), TripResult::NotArmed);
}

#[test]
fn software_trip_idempotent_second_call_returns_not_armed() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(42, 1, &[]), TripResult::Tripped);
    assert_eq!(software_trip(42, 2, &[]), TripResult::NotArmed);
}

#[test]
fn deadline_active_false_resets_across_arm_calls() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    tick(1, [0, 0, 0], &[]);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    disarm(42);
    arm(sw_msg(100)).expect("arm");
    assert!(
        !ARM.deadline_active.load(Ordering::Acquire),
        "deadline_active must be cleared on re-arm"
    );
}

#[test]
fn software_source_deadline_uses_saturating_add() {
    let _guard = reset();
    arm(sw_msg(u64::MAX)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    assert_eq!(ARM.deadline_clock_unchecked(), u64::MAX);
}

#[test]
fn software_source_skips_gpio_no_gpio_trip() {
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
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    set_pin_level(15, true);
    assert_eq!(tick(2, [0, 0, 0], &[]), TripAction::AbortNow);
    let evt = drain_trip();
    assert_eq!(evt.trip_source_idx, 1);
}

#[test]
fn software_trip_causes_tick_to_abort() {
    let _guard = reset();
    arm(sw_msg(100_000)).expect("arm");

    assert_eq!(tick(1, [0, 0, 0], &[0, 0]), TripAction::Continue);

    assert_eq!(software_trip(42, 50, &[10, 20]), TripResult::Tripped);

    assert_eq!(
        tick(51, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must return AbortNow after software_trip — \
         otherwise the MCU keeps moving and crashes into the bed"
    );
}

#[test]
fn software_trip_before_arm_clock_causes_tick_to_abort() {
    let _guard = reset();
    let mut msg = sw_msg(100_000);
    msg.arm_clock = 1000;
    arm(msg).expect("arm");

    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);

    assert_eq!(
        tick(1001, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must abort after software_trip even if deadline wasn't active"
    );
}
