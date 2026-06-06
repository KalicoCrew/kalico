use super::*;
use time::macros::datetime;

#[test]
fn levels_lowercase() {
    assert_eq!(level_str(&Level::INFO), "info");
    assert_eq!(level_str(&Level::WARN), "warn");
    assert_eq!(level_str(&Level::ERROR), "error");
    assert_eq!(level_str(&Level::DEBUG), "debug");
    assert_eq!(level_str(&Level::TRACE), "trace");
}

#[test]
fn time_is_rfc3339_millis_z() {
    let t = datetime!(2026-06-01 14:02:11.482482 UTC);
    assert_eq!(format_time(t), "2026-06-01T14:02:11.482Z");
}

#[test]
fn subsystem_mapping() {
    assert_eq!(subsystem_for_target("motion_bridge::bridge"), "bridge");
    assert_eq!(subsystem_for_target("motion_bridge::planner"), "motion");
    assert_eq!(
        subsystem_for_target("kalico_host_rt::host_io::reactor"),
        "mcu-comms"
    );
    assert_eq!(
        subsystem_for_target("motion_bridge::probe_homing"),
        "homing"
    );
    assert_eq!(subsystem_for_target("some::unknown::path"), "host-rust");
}
