//! Cross-MCU homing trip relay — the bridge reactor's analog of mainline
//! Klipper's C `trdispatch`. On the first trip report from any participating
//! source, broadcast `trsync_trigger` to every participating sink trsync.
//!
//! Sources report via either `kalico_endstop_tripped` (bridge GPIO) or
//! `trsync_state` with `can_trigger==0` (classic/Beacon). Sinks are firmware
//! trsyncs armed with `runtime_stop_on_trigger` whose signal freezes the
//! curve evaluator.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;

/// Reason carried by a relayed trigger. Matches `MCU_trsync.REASON_ENDSTOP_HIT`.
pub const REASON_ENDSTOP_HIT: u8 = 1;

/// Identifies a sink trsync to receive `trsync_trigger` when a trip is
/// detected. `mcu` must resolve to a `KalicoHostIo` in the `sink_ios` table
/// passed to [`prepare`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SinkSpec {
    pub mcu: u32,
    pub trsync_oid: u8,
}

/// Identifies a trip source to monitor. The `mcu` field selects which
/// `KalicoHostIo` the interceptor is registered on (passed as part of the
/// `sources` slice to [`prepare`]).
pub enum SourceSpec {
    /// Bridge GPIO endstop — listens for `kalico_endstop_tripped` filtered
    /// by `arm_id`.
    BridgeGpio { mcu: u32, arm_id: u32 },
    /// Classic/Beacon trsync — listens for `trsync_state` (by oid) and
    /// triggers when `can_trigger == 0`.
    Trsync { mcu: u32, trsync_oid: u8 },
}

/// Build the `trsync_trigger` command string for a sink trsync `oid`.
///
/// # Example
///
/// ```
/// use motion_bridge::trip_dispatch::build_trigger_cmd;
/// assert_eq!(build_trigger_cmd(42), "trsync_trigger oid=42 reason=1");
/// ```
pub fn build_trigger_cmd(oid: u8) -> String {
    format!("trsync_trigger oid={oid} reason={REASON_ENDSTOP_HIT}")
}

/// Pure one-shot fan-out, unit-testable without real transport.
///
/// The first call to [`on_trip`] invokes `send` once per sink. All subsequent
/// calls are no-ops — mirroring how `trdispatch` clears `can_trigger` so only
/// the first trip event propagates.
pub struct FanOut {
    sinks: Vec<SinkSpec>,
    fired: AtomicBool,
}

impl FanOut {
    /// Create a new `FanOut` targeting the given `sinks`. Not yet fired.
    pub fn new(sinks: Vec<SinkSpec>) -> Self {
        Self { sinks, fired: AtomicBool::new(false) }
    }

    /// On the first call, invoke `send(mcu, cmd)` once per sink in order.
    /// Subsequent calls are no-ops (one-shot, like trdispatch clearing
    /// `can_trigger`).
    pub fn on_trip(&self, mut send: impl FnMut(u32, &str)) {
        if self
            .fired
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }
        for s in &self.sinks {
            send(s.mcu, &build_trigger_cmd(s.trsync_oid));
        }
    }
}

/// Opaque handle returned by [`prepare`]. Holds the interceptor registrations
/// and the shared `triggered` flag. Pass to [`cleanup`] when the homing move
/// is complete.
pub struct TripDispatchHandle {
    pub(crate) triggered: Arc<AtomicBool>,
    pub(crate) registrations: Vec<(Arc<KalicoHostIo>, InterceptorId)>,
}

impl TripDispatchHandle {
    /// Returns `true` if any source has fired a trip since [`prepare`].
    pub fn was_triggered(&self) -> bool {
        self.triggered.load(Ordering::Acquire)
    }
}

/// Wire up the relay: register one interceptor per source; each interceptor's
/// closure fans `trsync_trigger` out to all sinks on the first trip (one-shot
/// via [`FanOut`]).
///
/// `sink_ios` maps MCU id → `KalicoHostIo` for all sink MCUs listed in
/// `sinks`. The fan-out sends directly via [`KalicoHostIo::send_fire_and_forget`].
///
/// # Errors
///
/// Returns `TransportError::Closed` if any source's reactor has exited, or
/// another `TransportError` variant if interceptor registration fails.
pub fn prepare(
    sources: Vec<(SourceSpec, Arc<KalicoHostIo>)>,
    sinks: Vec<SinkSpec>,
    sink_ios: Vec<(u32, Arc<KalicoHostIo>)>,
) -> Result<TripDispatchHandle, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(sinks));
    let mut registrations = Vec::new();

    for (spec, src_io) in sources {
        let fan = Arc::clone(&fan);
        let triggered = Arc::clone(&triggered);
        let sink_ios = sink_ios.clone();
        let (name, oid_filter, want_arm_id) = match &spec {
            SourceSpec::BridgeGpio { arm_id, .. } => {
                ("kalico_endstop_tripped", None, Some(*arm_id))
            }
            SourceSpec::Trsync { trsync_oid, .. } => {
                ("trsync_state", Some(u32::from(*trsync_oid)), None)
            }
        };
        let id = src_io.register_frame_interceptor(
            name,
            oid_filter,
            Box::new(move |params| {
                if let Some(want) = want_arm_id {
                    // BridgeGpio: filter by arm_id; skip if this callback is
                    // for a different arm.
                    if params.get_u32("arm_id") != want {
                        return;
                    }
                } else {
                    // Trsync: only trigger when can_trigger transitions to 0
                    // (probe hit / soft-trip). Non-zero means still armed —
                    // keep waiting.
                    if params.get_u32("can_trigger") != 0 {
                        return;
                    }
                }
                fan.on_trip(|mcu, cmd| {
                    if let Some((_, io)) = sink_ios.iter().find(|(m, _)| *m == mcu) {
                        let _ = io.send_fire_and_forget(cmd);
                    }
                });
                triggered.store(true, Ordering::Release);
            }),
        )?;
        registrations.push((src_io, id));
    }

    Ok(TripDispatchHandle { triggered, registrations })
}

/// Unregister all source interceptors installed by [`prepare`].
///
/// Always call this after [`prepare`], even on error paths, so the next homing
/// cycle can register fresh interceptors for the same (msg_name, oid) keys.
pub fn cleanup(handle: TripDispatchHandle) {
    for (io, id) in handle.registrations {
        let _ = io.unregister_frame_interceptor(id);
    }
}

#[cfg(test)]
mod tests;
