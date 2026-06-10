use std::sync::{Arc, Mutex};

use crate::pump::AxisKey;
use kalico_host_rt::passthrough_queue::PassthroughRouter;

#[derive(Debug, thiserror::Error)]
pub enum ReconstructError {
    #[error(
        "clock unsynced: {description} (endstop_mcu={endstop_mcu}, \
         axis_mcu={axis_mcu}, trip_clock={trip_clock})"
    )]
    ClockUnsynced {
        description: String,
        endstop_mcu: u32,
        axis_mcu: u32,
        trip_clock: u64,
    },
}

pub fn reconstruct_axis_position(
    endstop_mcu: u32,
    trip_clock: u64,
    axis_key: AxisKey,
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_clock: u64,
) -> Result<f64, String> {
    let axis_mcu = axis_key.mcu_id;

    let axis_clock = {
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        crate::motion_history::clock_between_mcus(
            &router_guard,
            crate::types::mcu_handle_from_raw(endstop_mcu),
            crate::types::mcu_handle_from_raw(axis_mcu),
            trip_clock,
        )
        .map_err(|description| {
            ReconstructError::ClockUnsynced {
                description,
                endstop_mcu,
                axis_mcu,
                trip_clock,
            }
            .to_string()
        })?
    };

    if axis_clock <= window_start_clock {
        return Err(format!(
            "endstop trip clock {axis_clock} predates this homing move \
             (window starts at {window_start_clock}) — stale trip or \
             mis-synced clock"
        ));
    }

    let store = history.lock().unwrap_or_else(|p| p.into_inner());
    store
        .state_at_clock(axis_key, axis_clock, None)
        .map(|st| st.position)
        .map_err(|e| e.to_string())
}

pub fn broadcast_stop<S, F>(
    mcu_ids: &std::collections::HashSet<u32, S>,
    axis_mcu: u32,
    call: F,
) -> Result<u64, String>
where
    S: std::hash::BuildHasher,
    F: Fn(u32) -> Result<kalico_protocol::messages::StopResponse, String>,
{
    let mut errors: Vec<String> = Vec::new();
    let mut axis_discard_clock: Option<u64> = None;
    for &mcu_id in mcu_ids {
        match call(mcu_id) {
            Ok(resp) if resp.result != 0 => {
                errors.push(format!(
                    "Stop rejected by mcu {mcu_id}: result={}",
                    resp.result
                ));
            }
            Ok(resp) => {
                if mcu_id == axis_mcu {
                    axis_discard_clock = Some(resp.discard_clock);
                }
            }
            Err(e) => errors.push(e),
        }
    }
    if !errors.is_empty() {
        return Err(format!(
            "EndstopTrip Stop broadcast failed: {}",
            errors.join("; ")
        ));
    }
    axis_discard_clock
        .ok_or_else(|| format!("EndstopTrip: axis MCU {axis_mcu} did not report a discard clock"))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveFaultRoute {
    HomingError,
    Fatal,
}

pub fn route_drive_fault(fault_mcu: u32, homing_axis_mcu: Option<u32>) -> DriveFaultRoute {
    if homing_axis_mcu == Some(fault_mcu) {
        DriveFaultRoute::HomingError
    } else {
        DriveFaultRoute::Fatal
    }
}

pub fn post_homing_fault_is_benign(now_ns: u64, settled_at_ns: u64) -> bool {
    settled_at_ns != 0 && now_ns.saturating_sub(settled_at_ns) < 2_000_000_000
}

#[cfg(test)]
mod tests;
