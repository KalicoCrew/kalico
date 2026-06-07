use super::*;
use crate::pump::AxisKey;

#[test]
fn homing_enqueue_params_active_returns_drip_constants() {
    let (lead, max_piece) = homing_enqueue_params(true);
    assert_eq!(lead, DRIP_MAX_AHEAD_SECS);
    assert_eq!(max_piece, Some(DRIP_PIECE_SECS));
}

#[test]
fn homing_enqueue_params_inactive_returns_normal_constants() {
    let (lead, max_piece) = homing_enqueue_params(false);
    assert_eq!(lead, crate::pump::MAX_LEAD_SECS);
    assert_eq!(max_piece, None);
}

#[test]
fn axis_keys_recorded_after_begin_and_cleared_by_take() {
    let h = HomingState::new();
    h.begin(1);
    let keys = [
        AxisKey { mcu_id: 1, axis: 0 },
        AxisKey { mcu_id: 1, axis: 1 },
    ];
    h.record_axis_keys(&keys);
    let taken = h.take_axis_keys();
    assert_eq!(taken.len(), 2);
    assert!(taken.contains(&keys[0]));
    assert!(taken.contains(&keys[1]));
    assert!(h.take_axis_keys().is_empty(), "second take must return empty");
}

#[test]
fn record_axis_keys_deduplicates() {
    let h = HomingState::new();
    h.begin(2);
    let key = AxisKey { mcu_id: 5, axis: 0 };
    h.record_axis_keys(&[key]);
    h.record_axis_keys(&[key]);
    assert_eq!(h.take_axis_keys().len(), 1);
}

#[test]
fn begin_clears_previously_recorded_axis_keys() {
    let h = HomingState::new();
    h.begin(1);
    h.record_axis_keys(&[AxisKey { mcu_id: 1, axis: 0 }]);
    h.begin(2);
    assert!(h.take_axis_keys().is_empty());
}

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
