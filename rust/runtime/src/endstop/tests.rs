use super::*;

fn cfg(kind: SourceKind, policy: ArmPolicy, sample_n: u8, gpio: PinId) -> SourceConfig {
    SourceConfig {
        kind,
        gpio,
        active_high: true,
        policy,
        sample_n,
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
    }
}

fn sw_msg() -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Software,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
    };
    ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
    }
}

fn reset() -> std::sync::MutexGuard<'static, ()> {
    test_guard()
}

fn drain_trip() -> TripEvent {
    poll_trip().expect("trip event")
}

#[test]
fn arm_policy_try_from_decodes_known_variants_and_rejects_others() {
    assert_eq!(ArmPolicy::try_from(0).unwrap(), ArmPolicy::TripImmediately);
    assert_eq!(ArmPolicy::try_from(1).unwrap(), ArmPolicy::WaitForClear);
    assert_eq!(ArmPolicy::try_from(2).unwrap_err(), 2);
    assert_eq!(ArmPolicy::try_from(255).unwrap_err(), 255);
}

#[test]
fn arm_validates_source_and_stepper_counts() {
    let _guard = reset();
    let mut m = msg(cfg(SourceKind::TmcDiag, ArmPolicy::TripImmediately, 1, 1));
    m.source_count = 0;
    assert_eq!(arm(m), Err(ArmError::EmptySources));

    let mut m = msg(cfg(SourceKind::TmcDiag, ArmPolicy::TripImmediately, 1, 1));
    m.stepper_count = 0;
    assert_eq!(arm(m), Err(ArmError::EmptySteppers));

    let mut m = msg(cfg(SourceKind::TmcDiag, ArmPolicy::TripImmediately, 0, 1));
    m.sources[0].sample_n = 0;
    assert_eq!(arm(m), Err(ArmError::InvalidSampleN));
}

#[test]
fn arm_then_disarm_lifecycle() {
    let _guard = reset();
    assert_eq!(arm(sw_msg()), Ok(ArmStatus::Armed));
    assert!(matches_u8(ARM.state.load(Ordering::Acquire), ArmState::Armed));
    assert_eq!(disarm(42), DisarmStatus::Disarmed);
    assert!(matches_u8(
        ARM.state.load(Ordering::Acquire),
        ArmState::Disarmed
    ));
}

#[test]
fn re_arm_while_armed_is_busy() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(arm(sw_msg()), Err(ArmError::Busy));
}

// --- software_trip: the trip path used by both the local C poll task and the
//     cross-MCU relay (runtime_stop_on_trigger -> kalico_software_trip).

#[test]
fn software_trip_transitions_armed_to_tripped_ready_and_publishes() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
    assert_eq!(evt.trip_clock, 500);
}

#[test]
fn software_trip_wrong_arm_id_is_no_op() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(software_trip(99, 500, &[10, 20]), TripResult::WrongArmId);
    assert!(matches_u8(ARM.state.load(Ordering::Acquire), ArmState::Armed));
}

#[test]
fn software_trip_on_non_armed_state_is_not_armed() {
    let _guard = reset();
    ARM.arm_id.store(0, Ordering::Release);
    ARM.state.store(ArmState::Disarmed as u8, Ordering::Release);
    assert_eq!(software_trip(0, 500, &[]), TripResult::NotArmed);
}

#[test]
fn software_trip_idempotent_second_call_returns_not_armed() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(software_trip(42, 1, &[]), TripResult::Tripped);
    assert_eq!(software_trip(42, 2, &[]), TripResult::NotArmed);
}

#[test]
fn snapshot_seqlock_carries_clock_and_step_counts() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(
        software_trip(42, 0x1_0000_0002, &[123, 456]),
        TripResult::Tripped
    );
    let evt = drain_trip();
    assert_eq!(evt.trip_clock, 0x1_0000_0002);
    assert_eq!(evt.steppers[0].oid, 0);
    assert_eq!(evt.steppers[0].step_count, 123);
    assert_eq!(evt.steppers[1].oid, 1);
    assert_eq!(evt.steppers[1].step_count, 456);
}

// --- disarm-ordering contract: a stale relay HOST_REQUEST trsync_trigger fires
//     software_trip after the host has already disarmed; it must be a clean
//     no-op (the one deliberate exception to fail-loud — see commit message).

#[test]
fn software_trip_on_disarmed_arm_is_a_no_op() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");
    assert_eq!(disarm(42), DisarmStatus::Disarmed);

    let snapshot_version_before = ARM.snapshot.version.load(Ordering::Acquire);
    assert_eq!(
        software_trip(42, 500, &[10, 20]),
        TripResult::NotArmed,
        "software_trip after disarm must return NotArmed"
    );
    assert_eq!(
        ARM.snapshot.version.load(Ordering::Acquire),
        snapshot_version_before,
        "snapshot version must not advance: no publish_snapshot call expected"
    );
    assert!(
        poll_trip().is_none(),
        "no trip event must be queued after software_trip on disarmed arm"
    );
}

#[test]
fn software_trip_with_mismatched_arm_id_leaves_live_arm_intact() {
    let _guard = reset();
    arm(sw_msg()).expect("arm");

    let snapshot_version_before = ARM.snapshot.version.load(Ordering::Acquire);
    assert_eq!(
        software_trip(99, 500, &[10, 20]),
        TripResult::WrongArmId,
        "software_trip with wrong arm_id must return WrongArmId"
    );
    assert_eq!(
        ARM.snapshot.version.load(Ordering::Acquire),
        snapshot_version_before,
        "snapshot version must not advance: no publish_snapshot call expected"
    );
    assert!(
        poll_trip().is_none(),
        "no trip event must be queued after mismatched software_trip"
    );

    // The real arm (id=42) must still trip normally.
    assert_eq!(
        software_trip(42, 600, &[10, 20]),
        TripResult::Tripped,
        "the correct arm_id must still trip after a mismatched software_trip was ignored"
    );
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
    assert_eq!(evt.trip_clock, 600);
}
