use super::*;

#[test]
fn encode_sources_round_trip_one_source() {
    let s = SourceSpec {
        kind: SourceKind::TmcDiag,
        gpio: 0x1234,
        active_high: true,
        policy: ArmPolicy::IgnoreUntilMoving,
        sample_n: 3,
        velocity_axis: 0x03, // X | Y
        v_min_q16: 0xDEADBEEF,
    };
    let buf = encode_sources(&[s]).unwrap();
    assert_eq!(buf.len(), SOURCE_RECORD_LEN);
    assert_eq!(buf[0], 1); // TmcDiag
    assert_eq!(&buf[1..3], &[0x34, 0x12]); // gpio LE
    assert_eq!(buf[3], 1); // active_high
    assert_eq!(buf[4], 2); // IgnoreUntilMoving
    assert_eq!(buf[5], 3); // sample_n
    assert_eq!(buf[6], 0x03); // velocity_axis
    assert_eq!(&buf[7..11], &[0xEF, 0xBE, 0xAD, 0xDE]); // v_min_q16 LE
}

#[test]
fn encode_sources_rejects_overflow() {
    let s = SourceSpec {
        kind: SourceKind::Physical,
        gpio: 0,
        active_high: false,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: 0x07,
        v_min_q16: 0,
    };
    let too_many = vec![s; MAX_SOURCES + 1];
    assert!(matches!(
        encode_sources(&too_many),
        Err(EndstopError::TooManySources(_))
    ));
}

#[test]
fn encode_steppers_round_trip() {
    let oids = [3u8, 7, 11];
    let buf = encode_steppers(&oids).unwrap();
    assert_eq!(buf, oids.to_vec());
}

#[test]
fn encode_steppers_rejects_overflow() {
    let too_many = vec![0u8; MAX_STEPPERS + 1];
    assert!(matches!(
        encode_steppers(&too_many),
        Err(EndstopError::TooManySteppers(_))
    ));
}

#[test]
fn arm_status_round_trip() {
    for s in [
        ArmStatus::Armed,
        ArmStatus::AlreadyTripped,
        ArmStatus::Rejected,
    ] {
        assert_eq!(ArmStatus::from_u8(s as u8), Some(s));
    }
    assert_eq!(ArmStatus::from_u8(99), None);
}

#[test]
fn disarm_status_round_trip() {
    for s in [
        DisarmStatus::Disarmed,
        DisarmStatus::AlreadyTripped,
        DisarmStatus::Unknown,
    ] {
        assert_eq!(DisarmStatus::from_u8(s as u8), Some(s));
    }
    assert_eq!(DisarmStatus::from_u8(99), None);
}
