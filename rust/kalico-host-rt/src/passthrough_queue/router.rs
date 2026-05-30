//! Top-level passthrough router — the boundary the bridge calls.
//! Owns one `McuState` + `NotifyTable` + `ReceiveWindow` per MCU.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock};
use std::time::Instant;

use indexmap::IndexMap;

use super::config_stage::{ConfigStage, ConfigStagePhase};
use super::debug_log::{DebugEntry, DebugLog};
use super::entry::{NotifyId, PassthroughEntry};
use super::mcu_state::{CommandQueueId, McuState, PushError};
use super::notify::{NotifyCallback, NotifyResponse, NotifyTable};
use super::receive_window::ReceiveWindow;
use super::stats::{PassthroughStats, StatsCounters};
use crate::clock::Clock;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);

impl McuHandle {
    pub fn raw(&self) -> u32 {
        self.0
    }

    /// Reconstruct an `McuHandle` from a raw `u32` previously obtained via
    /// [`raw()`](Self::raw). The caller is responsible for ensuring the
    /// value refers to a live MCU — the router will return
    /// `RouterError::UnknownMcu` if it does not.
    pub fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
}

#[derive(Debug)]
pub enum RouterError {
    UnknownMcu(McuHandle),
    Push(PushError),
    WindowFull,
}

impl std::fmt::Display for RouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownMcu(h) => write!(f, "unknown MCU handle {}", h.0),
            Self::Push(e) => write!(f, "push error: {e}"),
            Self::WindowFull => write!(f, "receive window full"),
        }
    }
}

impl std::error::Error for RouterError {}

struct McuRecord {
    #[allow(dead_code)]
    label: String,
    state: McuState,
    notify_table: NotifyTable,
    window: ReceiveWindow,
    sent_times: HashMap<NotifyId, f64>,
    config_stage: ConfigStage,
    /// MCU oscillator frequency in Hz.
    clock_freq: f64,
    /// Offset for host_time -> mcu_clock conversion.
    clock_offset: f64,
    /// Last known MCU clock value.
    last_clock: u64,
    /// Callbacks fired on the non-empty -> empty transition.
    flush_callbacks: Vec<Box<dyn Fn() + Send>>,
    /// Tracks whether the queues were non-empty at some point, so the
    /// callback only fires on a genuine non-empty -> empty transition.
    was_non_empty: bool,
    /// Per-MCU stats counters.
    stats: StatsCounters,
    /// Rolling debug log for crash diagnostics.
    debug_log: DebugLog,
}

impl std::fmt::Debug for McuRecord {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuRecord")
            .field("label", &self.label)
            .field("state", &self.state)
            .field("notify_table", &self.notify_table)
            .field("window", &self.window)
            .field("sent_times_count", &self.sent_times.len())
            .field("config_stage", &self.config_stage)
            .field("clock_freq", &self.clock_freq)
            .field("last_clock", &self.last_clock)
            .field("flush_callbacks_count", &self.flush_callbacks.len())
            .field("was_non_empty", &self.was_non_empty)
            .field("stats", &self.stats)
            .field("debug_log", &self.debug_log)
            .finish()
    }
}

pub struct PassthroughRouter {
    mcus: IndexMap<McuHandle, McuRecord>,
    next_handle: u32,
    clock: Arc<dyn Clock + Send + Sync>,
}

impl std::fmt::Debug for PassthroughRouter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PassthroughRouter")
            .field("mcus", &self.mcus)
            .field("next_handle", &self.next_handle)
            .finish()
    }
}

/// Convert an `Instant` to a monotonic f64 seconds relative to a
/// process-lifetime anchor. Only deltas between two values are meaningful.
fn instant_to_f64(instant: Instant) -> f64 {
    static ANCHOR: OnceLock<Instant> = OnceLock::new();
    let anchor = ANCHOR.get_or_init(Instant::now);
    if instant >= *anchor {
        instant.duration_since(*anchor).as_secs_f64()
    } else {
        -(anchor.duration_since(instant).as_secs_f64())
    }
}

impl PassthroughRouter {
    pub fn with_clock(clock: Arc<dyn Clock + Send + Sync>) -> Self {
        Self {
            mcus: IndexMap::new(),
            next_handle: 0,
            clock,
        }
    }

    /// Iterate over all claimed MCU handles.
    pub fn mcu_handles(&self) -> impl Iterator<Item = &McuHandle> {
        self.mcus.keys()
    }

    /// Register a new MCU and return its handle.
    pub fn claim_mcu(&mut self, label: &str) -> McuHandle {
        let handle = McuHandle(self.next_handle);
        self.next_handle += 1;
        self.mcus.insert(
            handle,
            McuRecord {
                label: label.to_owned(),
                state: McuState::new(),
                notify_table: NotifyTable::new(),
                window: ReceiveWindow::new(),
                sent_times: HashMap::new(),
                config_stage: ConfigStage::new(),
                clock_freq: 0.0,
                clock_offset: 0.0,
                last_clock: 0,
                flush_callbacks: Vec::new(),
                was_non_empty: false,
                stats: StatsCounters::new(),
                debug_log: DebugLog::new(),
            },
        );
        handle
    }

    /// Remove an MCU. Outstanding notify callbacks are dropped.
    pub fn release_mcu(&mut self, handle: McuHandle) {
        self.mcus.swap_remove(&handle);
    }

    pub fn alloc_command_queue(&mut self, mcu: McuHandle) -> Result<CommandQueueId, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.state.alloc_command_queue())
    }

    pub fn register_notify(
        &mut self,
        mcu: McuHandle,
        cb: NotifyCallback,
    ) -> Result<NotifyId, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.notify_table.register(cb))
    }

    pub fn push(
        &mut self,
        mcu: McuHandle,
        queue_id: CommandQueueId,
        entry: PassthroughEntry,
    ) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.state.push(queue_id, entry).map_err(RouterError::Push)
    }

    pub fn promote_all(&mut self, mcu: McuHandle, ack_clock: u64) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.state.promote_all(ack_clock);
        Ok(())
    }

    /// Pop the next entry for emission if the receive window allows it.
    /// Records the emit in the window and stores the sent timestamp for
    /// any notify-bearing entry. Also tracks the non-empty state for flush
    /// callback triggering.
    pub fn pop_next_for_emission(
        &mut self,
        mcu: McuHandle,
    ) -> Result<Option<PassthroughEntry>, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        if !rec.window.can_emit() {
            return Ok(None);
        }
        let entry = rec.state.pop_next();
        if let Some(ref e) = entry {
            rec.was_non_empty = true;
            let bytes_len = e.bytes().len() as u64;
            rec.window.record_emit(bytes_len);
            rec.stats.bytes_write += bytes_len;
            rec.stats.send_seq += 1;
            let now = instant_to_f64(self.clock.now());
            rec.debug_log
                .record_sent(rec.stats.send_seq, e.bytes().to_vec(), now);
            if !e.notify_id().is_none() {
                rec.sent_times.insert(e.notify_id(), now);
            }
        }
        Ok(entry)
    }

    /// Dispatch a response for a previously-sent notify-bearing entry.
    ///
    /// # Wire contract
    ///
    /// `response_bytes` MUST be the message **body**: `[msgid VLQ |
    /// fields...]` — i.e. the same bytes that `MsgProtoParser::decode_body`
    /// expects. No frame header (length / seq) and no trailer (CRC / sync
    /// byte). The bytes are passed through verbatim into
    /// [`NotifyResponse::bytes`] without inspection or transformation;
    /// callers (e.g. `RouterTransport::submit_and_wait`) decode them
    /// directly with the parser.
    pub fn dispatch_response(
        &mut self,
        mcu: McuHandle,
        notify_id: NotifyId,
        response_bytes: Vec<u8>,
    ) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.stats.bytes_read += response_bytes.len() as u64;
        rec.stats.receive_seq += 1;
        let sent_time = rec.sent_times.remove(&notify_id).unwrap_or(0.0);
        let receive_time = instant_to_f64(self.clock.now());
        rec.debug_log
            .record_received(rec.stats.receive_seq, response_bytes.clone(), receive_time);
        rec.notify_table.dispatch(
            notify_id,
            NotifyResponse {
                bytes: response_bytes,
                sent_time,
                receive_time,
            },
        );
        Ok(())
    }

    pub fn record_ack(&mut self, mcu: McuHandle, acked_bytes: u64) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.window.record_ack(acked_bytes);
        Ok(())
    }

    // ── Config-stage API ────────────────────────────────────────────────

    pub fn add_config_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_config_cmd(bytes))
    }

    pub fn add_init_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_init_cmd(bytes))
    }

    pub fn add_restart_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_restart_cmd(bytes))
    }

    /// Transition to `SendingConfig` — begin draining config commands.
    pub fn begin_config_phase(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.config_stage.begin_config_send();
        Ok(())
    }

    /// Get the next config/init entry, or `None` when all have been sent.
    pub fn next_config_entry(&mut self, mcu: McuHandle) -> Result<Option<Vec<u8>>, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.next_config_entry())
    }

    /// Current config-stage phase for the given MCU.
    pub fn config_phase(&self, mcu: McuHandle) -> Result<ConfigStagePhase, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.phase())
    }

    // ── Clock estimation API ────────────────────────────────────────────

    /// Update the clock estimation parameters for an MCU.
    /// Called by the clock-sync subsystem whenever it refines its estimate.
    pub fn set_clock_est(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset: f64,
        last_clock: u64,
    ) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        // Keep the host timestamp paired with the MCU clock value supplied
        // by the clocksync estimate. Rebasing the timestamp to "now" while
        // retaining an older MCU clock makes projections lag by callback
        // transit latency.
        rec.clock_offset = offset;
        rec.last_clock = last_clock;
        Ok(())
    }

    /// Update clock estimation when the incoming host timestamp is in a
    /// caller-owned monotonic clock domain. `host_now_same_epoch` must be
    /// sampled in that same domain at the time this method is called; the
    /// router rebases the estimate into its own deterministic clock domain.
    pub fn set_clock_est_rebased(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset: f64,
        last_clock: u64,
        host_now_same_epoch: f64,
    ) -> Result<(), RouterError> {
        let bridge_now = instant_to_f64(self.clock.now());
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = bridge_now - (host_now_same_epoch - offset);
        rec.last_clock = last_clock;

        Ok(())
    }

    /// Update clock-sync state from a freshly recorded RTT-aware sample.
    /// Used by the bridge's periodic `kalico_clock_sync_request` driver
    /// (`spawn_periodic_clock_sync`) to keep `compute_ack_clock` from
    /// drifting once klippy's bridge-mode clocksync stops emitting
    /// `clock` responses (klippy's `_get_clock_event` raw_send is a
    /// no-op in bridge mode — there is no other update path).
    ///
    /// `host_send` is the wire-send instant; `mcu_at_send` is the MCU
    /// clock value at that instant (RTT-corrected by the caller).
    pub fn set_clock_est_from_sample(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        host_send: Instant,
        mcu_at_send: u64,
    ) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = instant_to_f64(host_send);
        rec.last_clock = mcu_at_send;
        Ok(())
    }

    /// Compute the projected MCU ack-clock from the current host time and
    /// clock estimation parameters.
    ///
    /// `projected_clock = last_clock + (host_now_secs - clock_offset) * clock_freq`
    ///
    /// Returns 0 if clock estimation has not been set (freq == 0).
    pub fn compute_ack_clock(&self, mcu: McuHandle) -> Result<u64, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        if rec.clock_freq == 0.0 {
            return Ok(0);
        }
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Ok(projected)
    }

    /// Shared host clock "now" in seconds — the time base the dispatch anchor uses. Reads the same clock source as the ack-clock projections (via a different formula).
    pub fn host_now_secs(&self) -> f64 {
        instant_to_f64(self.clock.now())
    }

    /// Convert a host-time-seconds value to the projected MCU clock for
    /// the given MCU, using the linear estimate set by `set_clock_est`.
    /// Returns 0 if the estimate has not been initialised (`freq == 0`).
    pub fn host_time_to_mcu_clock(
        &self,
        mcu: McuHandle,
        host_time_secs: f64,
    ) -> Result<u64, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        if rec.clock_freq == 0.0 {
            return Ok(0);
        }
        let delta = (host_time_secs - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Ok(projected)
    }

    // ── Stats ────────────────────────────────────────────────────────────

    /// Snapshot current statistics for the given MCU.
    pub fn get_stats(&self, mcu: McuHandle) -> Result<PassthroughStats, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        let mut snap = rec.stats.snapshot();
        snap.ready_bytes = rec.state.total_ready_bytes();
        snap.upcoming_bytes = rec.state.total_upcoming_bytes();
        Ok(snap)
    }

    // ── Debug log / extract_old ───────────────────────────────────────────

    /// Drain the rolling debug log for crash diagnostics.
    /// Returns `(old_sent, old_received)`.
    pub fn extract_old(
        &mut self,
        mcu: McuHandle,
    ) -> Result<(Vec<DebugEntry>, Vec<DebugEntry>), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.debug_log.extract_old())
    }

    // ── Flush callbacks ─────────────────────────────────────────────────

    /// Register a callback that fires on the non-empty -> empty transition
    /// for the given MCU's queues.
    pub fn register_flush_callback(
        &mut self,
        mcu: McuHandle,
        cb: Box<dyn Fn() + Send>,
    ) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.flush_callbacks.push(cb);
        Ok(())
    }

    /// Check whether the MCU's queues transitioned from non-empty to empty.
    /// If so, fire all registered flush callbacks.
    ///
    /// Call this after draining emissions for a tick.
    pub fn check_flush(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        if rec.was_non_empty && rec.state.is_all_ready_empty() {
            rec.was_non_empty = false;
            for cb in &rec.flush_callbacks {
                cb();
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
