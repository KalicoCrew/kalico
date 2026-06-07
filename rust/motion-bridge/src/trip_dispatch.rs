//! Cross-MCU homing trip relay — the bridge reactor's analog of mainline
//! Klipper's C `trdispatch`. On the first trip report from any participating
//! source, broadcast `trsync_trigger` to every participating sink trsync.
//!
//! Sources report via either `kalico_endstop_tripped` (bridge GPIO) or
//! `trsync_state` with `can_trigger==0` (classic/Beacon). Sinks are firmware
//! trsyncs armed with `runtime_stop_on_trigger` whose signal freezes the
//! curve evaluator.
//!
//! Participants (typically the same MCUs as sources/sinks) report liveness via
//! `trsync_state` with `can_trigger==1`. Each such report feeds
//! [`extension::ExtensionEngine`], which decides when to push a new
//! `trsync_set_timeout` to each participant's firmware trsync. This prevents
//! the firmware's built-in expire timer from freezing the move during a live
//! homing sequence.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use kalico_host_rt::clock::instant_to_f64;
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

/// Identifies a trsync that participates in liveness extension. Each
/// participant's `trsync_state can_trigger=1` reports keep every MCU's
/// firmware expire timer pushed forward. `mcu` must resolve to a
/// `KalicoHostIo` in the `participants` slice passed to [`prepare`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantSpec {
    pub mcu: u32,
    pub trsync_oid: u8,
}

/// Identifies a trip source to monitor. The `mcu` field selects which
/// `KalicoHostIo` the interceptor is registered on (passed as part of the
/// `sources` slice to [`prepare`]).
#[derive(Debug)]
pub enum SourceSpec {
    /// Bridge GPIO endstop — listens for `kalico_endstop_tripped` filtered
    /// by `arm_id`.
    BridgeGpio { mcu: u32, arm_id: u32 },
    /// Classic/Beacon trsync — listens for `trsync_state` (by oid) and
    /// triggers when `can_trigger == 0`.
    ///
    /// # Caller contract
    ///
    /// A `trsync_state` frame with `can_trigger == 0` can also arrive
    /// **before** the homing arm (e.g. during MCU init). The caller MUST
    /// register the dispatch only when arming the move — mirroring
    /// `probe_homing.rs`'s "call before home_start" contract — so that a
    /// spurious pre-arm trip is not relayed to the sink trsyncs.
    Trsync { mcu: u32, trsync_oid: u8 },
}

/// Build the `trsync_trigger` command string for a sink trsync `oid`.
///
/// # Example
///
/// ```
/// use motion_bridge_native::trip_dispatch::build_trigger_cmd;
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
#[derive(Debug)]
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
/// `participants` lists every trsync whose `can_trigger=1` reports keep the
/// firmware expire timers extended. `expire_timeout_s` is the per-MCU expire
/// window. `clock_of(mcu_id)` returns `(projected_now_ticks, freq)` for the
/// named MCU, or `None` when the clock is not yet synced. An un-synced clock
/// is warned once per participant and the extension is skipped; a homing move
/// reaching prepare with no synced clocks will surface via this warning before
/// the firmware timer fires.
///
/// # Caller contract
///
/// Call `prepare` only when arming the move (never at init time), so that
/// any pre-arm `trsync_state` frames with `can_trigger == 0` are not yet
/// intercepted. See [`SourceSpec::Trsync`] for details.
///
/// # Errors
///
/// Returns `TransportError::Closed` if any source's reactor has exited, or
/// another `TransportError` variant if interceptor registration fails. On a
/// partial failure (some sources registered, then a later one errors), all
/// previously registered interceptors are unregistered before returning.
pub fn prepare(
    sources: Vec<(SourceSpec, Arc<KalicoHostIo>)>,
    sinks: Vec<SinkSpec>,
    sink_ios: Vec<(u32, Arc<KalicoHostIo>)>,
    participants: Vec<(ParticipantSpec, Arc<KalicoHostIo>)>,
    expire_timeout_s: f64,
    clock_of: impl Fn(u32) -> Option<(u64, f64)> + Send + Sync + 'static,
) -> Result<TripDispatchHandle, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(sinks));
    let mut registrations: Vec<(Arc<KalicoHostIo>, InterceptorId)> = Vec::new();

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
        let id = match src_io.register_frame_interceptor(
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
        ) {
            Ok(id) => id,
            Err(e) => {
                for (io, prev_id) in registrations {
                    let _ = io.unregister_frame_interceptor(prev_id);
                }
                return Err(e);
            }
        };
        registrations.push((src_io, id));
    }

    if !participants.is_empty() {
        // mainline: min_extend = 0.8 × report_ticks, report = 0.3 × timeout
        let min_extend_s = 0.8 * 0.3 * expire_timeout_s;
        let engine = Arc::new(std::sync::Mutex::new(
            extension::ExtensionEngine::new(
                participants
                    .iter()
                    .map(|_| extension::Participant { last_status_time: 0.0, expire_time: 0.0 })
                    .collect(),
                expire_timeout_s,
                min_extend_s,
            ),
        ));
        let clock_of = Arc::new(clock_of);

        let participant_io: Vec<(u32, u8, Arc<KalicoHostIo>)> = participants
            .iter()
            .map(|(p, io)| (p.mcu, p.trsync_oid, Arc::clone(io)))
            .collect();

        for (idx, (spec, part_io)) in participants.into_iter().enumerate() {
            let engine = Arc::clone(&engine);
            let clock_of = Arc::clone(&clock_of);
            let participant_io = participant_io.clone();
            let mcu = spec.mcu;

            let id = match part_io.register_frame_interceptor(
                "trsync_state",
                Some(u32::from(spec.trsync_oid)),
                Box::new(move |params| {
                    if params.get_u32("can_trigger") == 0 {
                        return;
                    }

                    let clock32 = params.get_u32("clock");
                    let (now_ticks, freq) = match clock_of(mcu) {
                        Some(v) => v,
                        None => {
                            tracing::warn!(
                                participant_idx = idx,
                                mcu_id = mcu,
                                "trip_dispatch: clock not synced for participant — \
                                 extension skipped"
                            );
                            return;
                        }
                    };
                    let host_now = instant_to_f64(Instant::now());
                    let report_ticks =
                        extension::clock32_to_64(now_ticks, clock32);
                    let status_time =
                        extension::ticks_to_host_time(report_ticks, now_ticks, host_now, freq);

                    let sends = engine
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .on_report(idx, status_time);

                    for (target_idx, expire_t) in sends {
                        let (target_mcu, target_oid, ref target_io) =
                            participant_io[target_idx];
                        let (target_now_ticks, target_freq) = match clock_of(target_mcu) {
                            Some(v) => v,
                            None => {
                                tracing::warn!(
                                    participant_idx = target_idx,
                                    mcu_id = target_mcu,
                                    "trip_dispatch: clock not synced for target \
                                     participant — extension send skipped"
                                );
                                continue;
                            }
                        };
                        let target_host_now = instant_to_f64(Instant::now());
                        let expire_ticks = extension::host_time_to_ticks(
                            expire_t,
                            target_now_ticks,
                            target_host_now,
                            target_freq,
                        );
                        let cmd = format!(
                            "trsync_set_timeout oid={} clock={}",
                            target_oid,
                            expire_ticks & 0xFFFF_FFFF
                        );
                        let _ = target_io.send_fire_and_forget(&cmd);
                    }
                }),
            ) {
                Ok(id) => id,
                Err(e) => {
                    for (io, prev_id) in registrations {
                        let _ = io.unregister_frame_interceptor(prev_id);
                    }
                    return Err(e);
                }
            };
            registrations.push((part_io, id));
        }
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

pub mod extension;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod extension_tests;
