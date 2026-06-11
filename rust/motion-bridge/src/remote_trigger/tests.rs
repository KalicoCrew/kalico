use super::{relay_decision, relay_trip_clock, RelayAction};

#[test]
fn non_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(1), false), RelayAction::Ignore);
}

#[test]
fn terminal_report_fires() {
    assert_eq!(relay_decision(Some(0), false), RelayAction::Fire);
}

#[test]
fn second_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(0), true), RelayAction::Ignore);
}

#[test]
fn malformed_report_without_can_trigger_is_ignored() {
    assert_eq!(relay_decision(None, false), RelayAction::Ignore);
}

#[test]
fn nonzero_report_clock_expands_against_reference() {
    // reference 0x1_0000_1000, clock32 just below the low-32 reference:
    // small negative delta, same epoch.
    assert_eq!(relay_trip_clock(0x0000_0F00, 0x1_0000_1000), 0x1_0000_0F00);
}

#[test]
fn clock32_ahead_of_reference_expands_forward() {
    assert_eq!(relay_trip_clock(0x0000_2000, 0x1_0000_1000), 0x1_0000_2000);
}

#[test]
fn expansion_handles_wrap_boundary() {
    // reference just past a 32-bit wrap; clock32 from just before it.
    assert_eq!(relay_trip_clock(0xFFFF_FF00, 0x2_0000_0010), 0x1_FFFF_FF00);
}

#[test]
fn zero_clock_means_host_commanded_trigger_substitute_reference() {
    // trsync_trigger path reports clock=0 (trsync.c:176); substitute the
    // router's current estimate.
    assert_eq!(relay_trip_clock(0, 0x1_0000_1000), 0x1_0000_1000);
}
