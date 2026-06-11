//! Decision logic for the remote-trigger relay: a reactor interceptor on a
//! probe MCU's RX thread translating terminal `trsync_state` reports into
//! the bridge's endstop-trip dispatch. Kept free of I/O for testability;
//! the interceptor closure lives in bridge.rs.

#[derive(Debug, PartialEq, Eq)]
pub enum RelayAction {
    Fire,
    Ignore,
}

pub fn relay_decision(can_trigger: Option<u32>, already_fired: bool) -> RelayAction {
    match can_trigger {
        Some(0) if !already_fired => RelayAction::Fire,
        _ => RelayAction::Ignore,
    }
}

/// `trsync_state.clock` is a report-time clock, not a trip timestamp
/// (`trsync.c:190`), and the host-commanded `trsync_trigger` path sends 0
/// (`trsync.c:176`). Expand a nonzero report clock to 64 bits against the
/// router's current clock estimate for the probe MCU; substitute that
/// estimate outright for the zero case. The result is provisional-only —
/// precise trigger timestamps come from the probe's own latched record.
pub fn relay_trip_clock(clock32: u32, reference_clock64: u64) -> u64 {
    if clock32 == 0 {
        return reference_clock64;
    }
    let delta = clock32.wrapping_sub(reference_clock64 as u32) as i32 as i64;
    reference_clock64.wrapping_add(delta as u64)
}

#[cfg(test)]
mod tests;
