use super::*;

#[test]
fn take_completion_event_fires_once_after_no_trip_retire() {
    let h = HomingState::new();
    h.begin(7);
    h.mark_dispatched_segment(3);
    h.complete_if_retired(3);
    assert_eq!(h.state(), HomingSegmentState::Completed);
    assert_eq!(h.take_completion_event(), Some(7));
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn take_completion_event_does_not_fire_when_idle() {
    let h = HomingState::new();
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn take_completion_event_does_not_fire_after_trip() {
    let h = HomingState::new();
    h.begin(8);
    h.mark_dispatched_segment(2);
    h.state.store(
        HomingSegmentState::Tripped as u8,
        std::sync::atomic::Ordering::Release,
    );
    h.complete_if_retired(2);
    assert_eq!(h.state(), HomingSegmentState::Tripped);
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn from_u8_round_trips_all_variants() {
    let pairs: &[(u8, HomingSegmentState)] = &[
        (0, HomingSegmentState::Idle),
        (1, HomingSegmentState::Active),
        (2, HomingSegmentState::Completed),
        (3, HomingSegmentState::Tripped),
    ];
    for &(raw, expected) in pairs {
        assert_eq!(HomingSegmentState::from_u8(raw), expected, "raw={raw}");
    }
    assert_eq!(HomingSegmentState::from_u8(4), HomingSegmentState::Idle);
    assert_eq!(HomingSegmentState::from_u8(255), HomingSegmentState::Idle);
}
