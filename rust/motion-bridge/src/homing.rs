use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use runtime::piece_ring::PieceEntry;

use crate::dispatch::McuAxisConfig;
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

    #[error(
        "endstop trip clock outside all stored trajectory pieces \
         (trip_clock={trip_clock}, axis_clock={axis_clock}, \
         window_start={window_start}, window_end={window_end}) — \
         stale/mis-synced clock or trip from a prior stream"
    )]
    EndstopTripOutsideTrajectory {
        trip_clock: u64,
        axis_clock: u64,
        window_start: u64,
        window_end: u64,
    },

    #[error("no trajectory pieces recorded for axis {0:?} — was HomeDrip dispatched?")]
    NoTrajectoryPieces(AxisKey),

    #[error("MCU clock frequency unknown for mcu {mcu_id}")]
    UnknownClockFreq { mcu_id: u32 },
}

/// Evaluate a cubic Bernstein polynomial at parameter `u` in [0, 1].
///
/// Bernstein coefficients are stored in `coeffs` as `[b0, b1, b2, b3]`
/// where `p(u) = Σ C(3,k) * (1-u)^(3-k) * u^k * b_k`.
#[inline]
pub fn eval_bernstein_cubic(coeffs: [f32; 4], u: f64) -> f64 {
    let v = 1.0 - u;
    let u = u;
    let b0 = coeffs[0] as f64;
    let b1 = coeffs[1] as f64;
    let b2 = coeffs[2] as f64;
    let b3 = coeffs[3] as f64;
    v * v * v * b0 + 3.0 * v * v * u * b1 + 3.0 * v * u * u * b2 + u * u * u * b3
}

/// Find the axis position at `axis_clock` using stored trajectory pieces.
///
/// The piece whose window `[start_time, start_time + duration*freq)` contains
/// `axis_clock` is identified; Bernstein coefficients are evaluated at
/// `u = (axis_clock - start_time) / (duration * freq)`.
fn eval_piece_at_clock(
    pieces: &[PieceEntry],
    axis_clock: u64,
    clock_freq: f64,
    trip_clock: u64,
) -> Result<f64, ReconstructError> {
    if pieces.is_empty() {
        return Err(ReconstructError::EndstopTripOutsideTrajectory {
            trip_clock,
            axis_clock,
            window_start: 0,
            window_end: 0,
        });
    }

    let window_start = pieces.first().map(|p| p.start_time).unwrap_or(0);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let window_end = pieces
        .last()
        .map(|p| p.end_time(clock_freq as f32))
        .unwrap_or(0);

    for piece in pieces {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let piece_end = piece.end_time(clock_freq as f32);
        if axis_clock >= piece.start_time && axis_clock < piece_end {
            let duration_ticks = (piece.duration as f64) * clock_freq;
            let u = if duration_ticks > 0.0 {
                ((axis_clock - piece.start_time) as f64) / duration_ticks
            } else {
                0.0
            };
            let u_clamped = u.clamp(0.0, 1.0);
            return Ok(eval_bernstein_cubic(piece.coeffs, u_clamped));
        }
    }

    Err(ReconstructError::EndstopTripOutsideTrajectory {
        trip_clock,
        axis_clock,
        window_start,
        window_end,
    })
}

/// Reconstruct the motor-space position of `axis_key` at the instant `trip_clock`
/// fired on `endstop_mcu`.
///
/// Same-MCU: `axis_clock = trip_clock` (no conversion needed, exact).
/// Cross-MCU: project `trip_clock` to host seconds, then back to axis MCU ticks.
pub fn reconstruct_axis_position(
    endstop_mcu: u32,
    trip_clock: u64,
    axis_key: AxisKey,
    router: &Arc<Mutex<PassthroughRouter>>,
    homing_traj: &Arc<Mutex<HashMap<AxisKey, Vec<PieceEntry>>>>,
    configs: &[McuAxisConfig],
) -> Result<f64, String> {
    let axis_mcu = axis_key.mcu_id;

    let axis_clock = if axis_mcu == endstop_mcu {
        trip_clock
    } else {
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        let endstop_handle = crate::types::mcu_handle_from_raw(endstop_mcu);
        let axis_handle = crate::types::mcu_handle_from_raw(axis_mcu);

        let host_trip = router_guard
            .clock_to_host_secs(endstop_handle, trip_clock)
            .ok_or_else(|| {
                ReconstructError::ClockUnsynced {
                    description: format!(
                        "clock_to_host_secs returned None for endstop_mcu {endstop_mcu}"
                    ),
                    endstop_mcu,
                    axis_mcu,
                    trip_clock,
                }
                .to_string()
            })?;

        router_guard
            .host_time_to_mcu_clock(axis_handle, host_trip)
            .map_err(|e| {
                ReconstructError::ClockUnsynced {
                    description: format!(
                        "host_time_to_mcu_clock failed for axis_mcu {axis_mcu}: {e:?}"
                    ),
                    endstop_mcu,
                    axis_mcu,
                    trip_clock,
                }
                .to_string()
            })?
    };

    let clock_freq = {
        let cfg = configs
            .iter()
            .find(|c| c.mcu_id == axis_mcu)
            .ok_or_else(|| {
                ReconstructError::UnknownClockFreq { mcu_id: axis_mcu }.to_string()
            })?;
        let _ = cfg;
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        let axis_handle = crate::types::mcu_handle_from_raw(axis_mcu);
        router_guard
            .ack_clock_and_freq(axis_handle)
            .map(|(_, freq)| freq)
            .unwrap_or(0.0)
    };

    if clock_freq == 0.0 {
        return Err(ReconstructError::UnknownClockFreq { mcu_id: axis_mcu }.to_string());
    }

    let traj = homing_traj.lock().unwrap_or_else(|p| p.into_inner());
    let pieces = traj.get(&axis_key).ok_or_else(|| {
        ReconstructError::NoTrajectoryPieces(axis_key).to_string()
    })?;

    eval_piece_at_clock(pieces, axis_clock, clock_freq, trip_clock)
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests;
