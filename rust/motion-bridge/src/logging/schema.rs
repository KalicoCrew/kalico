use time::OffsetDateTime;
use time::format_description::FormatItem;
use time::macros::format_description;
use tracing::Level;

pub const SOURCE_HOST_RUST: &str = "host-rust";

const TIME_FMT: &[FormatItem<'static>] =
    format_description!("[year]-[month]-[day]T[hour]:[minute]:[second].[subsecond digits:3]Z");

pub fn level_str(level: &Level) -> &'static str {
    match *level {
        Level::TRACE => "trace",
        Level::DEBUG => "debug",
        Level::INFO => "info",
        Level::WARN => "warn",
        Level::ERROR => "error",
    }
}

pub fn format_time(t: OffsetDateTime) -> String {
    t.format(&TIME_FMT)
        .unwrap_or_else(|_| "1970-01-01T00:00:00.000Z".to_string())
}

pub fn subsystem_for_target(target: &str) -> &'static str {
    if target.contains("clock") {
        "clocksync"
    } else if target.contains("probe_homing") || target.contains("homing") {
        "homing"
    } else if target.contains("planner") {
        "motion"
    } else if target.contains("pump") {
        "motion"
    } else if target.contains("reactor")
        || target.contains("transport")
        || target.contains("kalico_native")
    {
        "mcu-comms"
    } else if target.contains("bridge") {
        "bridge"
    } else {
        "host-rust"
    }
}

#[cfg(test)]
mod tests {
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
}
