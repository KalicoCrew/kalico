use super::*;

#[test]
fn take_completion_event_fires_once_after_no_trip_retire() {
    let h = HomingState::new();
    h.begin(7);
    h.mark_dispatched_segment(3);
    // Retire past the homing segment; no trip occurred.
    h.complete_if_retired(3);
    assert_eq!(h.state(), HomingSegmentState::Completed);
    assert_eq!(h.take_completion_event(), Some(7));
    // Take-once: a second call returns None.
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn take_completion_event_does_not_fire_when_idle() {
    let h = HomingState::new();
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn take_completion_event_does_not_fire_after_trip() {
    // If the segment retired *and* a trip was latched, the trip path
    // owns the terminal — we don't double-fire as past-end-time.
    let h = HomingState::new();
    h.begin(8);
    h.mark_dispatched_segment(2);
    // Simulate a trip flipping the state to Tripped.
    h.state.store(
        HomingSegmentState::Tripped as u8,
        std::sync::atomic::Ordering::Release,
    );
    // Even if credit-freed retires the segment afterwards, no completion event.
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
        (4, HomingSegmentState::DeadlineExpired),
    ];
    for &(raw, expected) in pairs {
        assert_eq!(HomingSegmentState::from_u8(raw), expected, "raw={raw}");
    }
    // Unknown values fall back to Idle.
    assert_eq!(HomingSegmentState::from_u8(5), HomingSegmentState::Idle);
    assert_eq!(HomingSegmentState::from_u8(255), HomingSegmentState::Idle);
}

#[test]
fn complete_if_retired_does_not_overwrite_deadline_expired() {
    // DeadlineExpired is a terminal state; complete_if_retired must
    // leave it alone (same as Tripped).
    let h = HomingState::new();
    h.begin(9);
    h.mark_dispatched_segment(5);
    h.state.store(
        HomingSegmentState::DeadlineExpired as u8,
        std::sync::atomic::Ordering::Release,
    );
    h.complete_if_retired(5);
    assert_eq!(h.state(), HomingSegmentState::DeadlineExpired);
    assert_eq!(h.take_completion_event(), None);
}

#[test]
fn take_completion_event_does_not_fire_after_deadline_expired() {
    let h = HomingState::new();
    h.begin(10);
    h.mark_dispatched_segment(6);
    h.state.store(
        HomingSegmentState::DeadlineExpired as u8,
        std::sync::atomic::Ordering::Release,
    );
    assert_eq!(h.take_completion_event(), None);
}
