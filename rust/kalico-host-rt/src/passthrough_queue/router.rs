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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.state.alloc_command_queue())
    }

    pub fn register_notify(
        &mut self,
        mcu: McuHandle,
        cb: NotifyCallback,
    ) -> Result<NotifyId, RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.notify_table.register(cb))
    }

    pub fn push(
        &mut self,
        mcu: McuHandle,
        queue_id: CommandQueueId,
        entry: PassthroughEntry,
    ) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.state.push(queue_id, entry).map_err(RouterError::Push)
    }

    pub fn promote_all(&mut self, mcu: McuHandle, ack_clock: u64) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
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
            rec.debug_log.record_sent(
                rec.stats.send_seq,
                e.bytes().to_vec(),
                now,
            );
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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.stats.bytes_read += response_bytes.len() as u64;
        rec.stats.receive_seq += 1;
        let sent_time = rec.sent_times.remove(&notify_id).unwrap_or(0.0);
        let receive_time = instant_to_f64(self.clock.now());
        rec.debug_log.record_received(
            rec.stats.receive_seq,
            response_bytes.clone(),
            receive_time,
        );
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

    pub fn record_ack(
        &mut self,
        mcu: McuHandle,
        acked_bytes: u64,
    ) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.window.record_ack(acked_bytes);
        Ok(())
    }

    // ── Config-stage API ────────────────────────────────────────────────

    pub fn add_config_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_config_cmd(bytes))
    }

    pub fn add_init_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_init_cmd(bytes))
    }

    pub fn add_restart_cmd(&mut self, mcu: McuHandle, bytes: Vec<u8>) -> Result<bool, RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.add_restart_cmd(bytes))
    }

    /// Transition to `SendingConfig` — begin draining config commands.
    pub fn begin_config_phase(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.config_stage.begin_config_send();
        Ok(())
    }

    /// Get the next config/init entry, or `None` when all have been sent.
    pub fn next_config_entry(&mut self, mcu: McuHandle) -> Result<Option<Vec<u8>>, RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        // Klipper's `offset` is in its `reactor.monotonic()` frame
        // (Python perf_counter, anchored at process start). The bridge's
        // `instant_to_f64` is anchored at its own first-call OnceLock,
        // which is a different epoch. Using `offset` directly in
        // `compute_ack_clock`'s `(host_now - clock_offset)` would produce
        // a hugely negative delta (saturated to 0), which makes the
        // projection collapse to raw `last_clock`.
        //
        // Rebase `offset` into the bridge's frame: at this exact instant
        // the Klipper-frame "now" is whatever value Klipper just sent,
        // which is `offset + (last_clock_now_klippy_frame_ - last_clock)
        // / freq`. We don't know Klipper's "now" directly, but we know
        // that the host_now we'd compute on the bridge side IS the
        // co-temporal value; so we rebase by storing
        //   bridge_offset := instant_to_f64(now) - (offset_received - offset_received) = instant_to_f64(now)
        // and reinterpret last_clock as "the MCU clock value at this
        // bridge-frame instant". The projection
        //   projected = last_clock + (host_now - bridge_offset) * freq
        // then correctly extrapolates forward.
        rec.clock_offset = instant_to_f64(self.clock.now());
        rec.last_clock = last_clock;
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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
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
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        rec.flush_callbacks.push(cb);
        Ok(())
    }

    /// Check whether the MCU's queues transitioned from non-empty to empty.
    /// If so, fire all registered flush callbacks.
    ///
    /// Call this after draining emissions for a tick.
    pub fn check_flush(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
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
mod tests {
    use super::*;
    use crate::clock::MockClock;
    use crate::passthrough_queue::entry::NotifyId;
    use std::sync::Mutex;
    use std::time::Duration;

    fn make_router() -> (PassthroughRouter, Arc<MockClock>) {
        let clock = MockClock::new();
        let router =
            PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
        (router, clock)
    }

    fn entry(min_clock: u64, req_clock: u64) -> PassthroughEntry {
        PassthroughEntry::new(vec![0x01], min_clock, req_clock, NotifyId::none())
    }

    fn entry_with_notify(min_clock: u64, req_clock: u64, nid: NotifyId) -> PassthroughEntry {
        PassthroughEntry::new(vec![0x01], min_clock, req_clock, nid)
    }

    #[test]
    fn two_mcus_claim_release_independently() {
        let (mut router, _) = make_router();
        let a = router.claim_mcu("mcu_a");
        let b = router.claim_mcu("mcu_b");
        assert_ne!(a, b);

        router.release_mcu(a);
        // b should still work
        let q = router.alloc_command_queue(b);
        assert!(q.is_ok());
        // a should be gone
        assert!(router.alloc_command_queue(a).is_err());
    }

    #[test]
    fn alloc_command_queue_per_mcu() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q1 = router.alloc_command_queue(mcu).unwrap();
        let q2 = router.alloc_command_queue(mcu).unwrap();
        assert_ne!(q1, q2);
    }

    #[test]
    fn push_routes_correctly_through_mcu_state() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        router.push(mcu, q, entry(0, 100)).unwrap();
        router.push(mcu, q, entry(0, 50)).unwrap();

        let e = router.pop_next_for_emission(mcu).unwrap().unwrap();
        assert_eq!(e.req_clock(), 50);
    }

    #[test]
    fn register_notify_and_dispatch_response_round_trip() {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let captured = Arc::new(Mutex::new(None));
        let captured2 = Arc::clone(&captured);
        let nid = router
            .register_notify(
                mcu,
                Box::new(move |resp| {
                    *captured2.lock().unwrap() = Some(resp);
                }),
            )
            .unwrap();

        router.push(mcu, q, entry_with_notify(0, 10, nid)).unwrap();

        // Emit it — records sent_time
        let _ = router.pop_next_for_emission(mcu).unwrap();

        // Advance clock so receive_time > sent_time
        clock.advance(Duration::from_millis(50));

        router
            .dispatch_response(mcu, nid, vec![0xBE, 0xEF])
            .unwrap();

        let resp = captured.lock().unwrap().take().unwrap();
        assert_eq!(resp.bytes, vec![0xBE, 0xEF]);
        // receive_time should be after sent_time
        assert!(resp.receive_time >= resp.sent_time);
    }

    #[test]
    fn pop_next_for_emission_respects_window_gate() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        for i in 0..20 {
            router.push(mcu, q, entry(0, i)).unwrap();
        }

        // The default window should let some through but eventually block.
        let mut emitted = 0u32;
        while router.pop_next_for_emission(mcu).unwrap().is_some() {
            emitted += 1;
            if emitted > 100 {
                panic!("window gate did not kick in");
            }
        }
        assert!(emitted > 0);
        assert!(emitted < 20);
    }

    #[test]
    fn record_ack_frees_window_capacity() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        for i in 0..20 {
            router.push(mcu, q, entry(0, i)).unwrap();
        }

        // Drain until blocked
        while router.pop_next_for_emission(mcu).unwrap().is_some() {}

        // Ack frees capacity
        router.record_ack(mcu, 50).unwrap();

        // Should be able to emit more now
        let got = router.pop_next_for_emission(mcu).unwrap();
        assert!(got.is_some());
    }

    // ── Clock estimation tests ──────────────────────────────────────────

    #[test]
    fn set_clock_est_stores_values() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");

        // Before setting, compute_ack_clock returns 0 (freq == 0).
        assert_eq!(router.compute_ack_clock(mcu).unwrap(), 0);

        router.set_clock_est(mcu, 48_000_000.0, 0.0, 1000).unwrap();

        // After setting, compute_ack_clock returns non-zero.
        let ack = router.compute_ack_clock(mcu).unwrap();
        assert!(ack >= 1000, "ack_clock should be at least last_clock");
    }

    #[test]
    fn compute_ack_clock_projects_from_host_time() {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");

        // Record the base host time, then set clock_offset = base host time.
        let base_host = instant_to_f64(clock.now());
        router
            .set_clock_est(mcu, 1_000_000.0, base_host, 0)
            .unwrap();

        // At t=0 from offset, projected clock = 0 + 0 * freq = 0.
        let ack0 = router.compute_ack_clock(mcu).unwrap();
        assert_eq!(ack0, 0);

        // Advance 1 second — projected clock ~ 0 + 1.0 * 1_000_000.
        // Allow +-1 for f64 rounding in instant_to_f64 deltas.
        clock.advance(Duration::from_secs(1));
        let ack1 = router.compute_ack_clock(mcu).unwrap();
        let diff = (ack1 as i64 - 1_000_000_i64).unsigned_abs();
        assert!(diff <= 1, "expected ~1_000_000, got {ack1}");
    }

    #[test]
    fn compute_ack_clock_unknown_mcu_errors() {
        let (router, _) = make_router();
        let bogus = McuHandle(999);
        assert!(router.compute_ack_clock(bogus).is_err());
    }

    // ── Flush callback tests ────────────────────────────────────────────

    #[test]
    fn flush_callback_fires_on_non_empty_to_empty_transition() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let count = Arc::new(Mutex::new(0u32));
        let count2 = Arc::clone(&count);
        router
            .register_flush_callback(mcu, Box::new(move || {
                *count2.lock().unwrap() += 1;
            }))
            .unwrap();

        // Enqueue and emit one entry.
        router.push(mcu, q, entry(0, 10)).unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();

        // Now queues are empty — check_flush should fire the callback.
        router.check_flush(mcu).unwrap();
        assert_eq!(*count.lock().unwrap(), 1);
    }

    #[test]
    fn flush_callback_does_not_fire_if_never_non_empty() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let _q = router.alloc_command_queue(mcu).unwrap();

        let count = Arc::new(Mutex::new(0u32));
        let count2 = Arc::clone(&count);
        router
            .register_flush_callback(mcu, Box::new(move || {
                *count2.lock().unwrap() += 1;
            }))
            .unwrap();

        // Never pushed anything — check_flush should NOT fire.
        router.check_flush(mcu).unwrap();
        assert_eq!(*count.lock().unwrap(), 0);
    }

    #[test]
    fn flush_multiple_callbacks_all_fire() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let c1 = Arc::new(Mutex::new(0u32));
        let c2 = Arc::new(Mutex::new(0u32));
        let c1b = Arc::clone(&c1);
        let c2b = Arc::clone(&c2);

        router
            .register_flush_callback(mcu, Box::new(move || {
                *c1b.lock().unwrap() += 1;
            }))
            .unwrap();
        router
            .register_flush_callback(mcu, Box::new(move || {
                *c2b.lock().unwrap() += 1;
            }))
            .unwrap();

        router.push(mcu, q, entry(0, 10)).unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();
        router.check_flush(mcu).unwrap();

        assert_eq!(*c1.lock().unwrap(), 1);
        assert_eq!(*c2.lock().unwrap(), 1);
    }

    // ── Stats tests ──────────────────────────────────────────────────────

    #[test]
    fn stats_increment_on_emit() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let s0 = router.get_stats(mcu).unwrap();
        assert_eq!(s0.bytes_write, 0);
        assert_eq!(s0.send_seq, 0);

        router.push(mcu, q, entry(0, 10)).unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();

        let s1 = router.get_stats(mcu).unwrap();
        assert_eq!(s1.bytes_write, 1); // entry() produces 1-byte payload
        assert_eq!(s1.send_seq, 1);
    }

    #[test]
    fn stats_increment_on_response_receive() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let nid = router.register_notify(mcu, Box::new(|_| {})).unwrap();
        router.push(mcu, q, entry_with_notify(0, 10, nid)).unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();

        router.dispatch_response(mcu, nid, vec![0xAA, 0xBB]).unwrap();

        let s = router.get_stats(mcu).unwrap();
        assert_eq!(s.bytes_read, 2);
        assert_eq!(s.receive_seq, 1);
    }

    #[test]
    fn stats_are_per_mcu() {
        let (mut router, _) = make_router();
        let mcu_a = router.claim_mcu("a");
        let mcu_b = router.claim_mcu("b");
        let qa = router.alloc_command_queue(mcu_a).unwrap();
        let qb = router.alloc_command_queue(mcu_b).unwrap();

        router.push(mcu_a, qa, entry(0, 10)).unwrap();
        let _ = router.pop_next_for_emission(mcu_a).unwrap();

        router.push(mcu_b, qb, entry(0, 20)).unwrap();
        router.push(mcu_b, qb, entry(0, 30)).unwrap();
        let _ = router.pop_next_for_emission(mcu_b).unwrap();
        let _ = router.pop_next_for_emission(mcu_b).unwrap();

        let sa = router.get_stats(mcu_a).unwrap();
        let sb = router.get_stats(mcu_b).unwrap();

        assert_eq!(sa.send_seq, 1);
        assert_eq!(sb.send_seq, 2);
    }

    #[test]
    fn stats_ready_bytes_reflects_live_queue() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        router.push(mcu, q, entry(0, 10)).unwrap();
        router.push(mcu, q, entry(0, 20)).unwrap();

        let s = router.get_stats(mcu).unwrap();
        assert_eq!(s.ready_bytes, 2); // 2 entries x 1 byte each
    }

    // ── Debug log / extract_old tests ──────────────────────────────────

    #[test]
    fn extract_old_captures_sent_and_received() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let nid = router.register_notify(mcu, Box::new(|_| {})).unwrap();
        router
            .push(mcu, q, entry_with_notify(0, 10, nid))
            .unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();
        router
            .dispatch_response(mcu, nid, vec![0xDE, 0xAD])
            .unwrap();

        let (sent, received) = router.extract_old(mcu).unwrap();
        assert_eq!(sent.len(), 1);
        assert_eq!(received.len(), 1);
        assert_eq!(received[0].bytes, vec![0xDE, 0xAD]);
    }

    #[test]
    fn extract_old_capped_at_100() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        for i in 0..120 {
            router.push(mcu, q, entry(0, i)).unwrap();
        }
        // Emit all 120 — window might block, but we only need enough
        // to exceed 100.
        let mut emitted = 0;
        while router.pop_next_for_emission(mcu).unwrap().is_some() {
            emitted += 1;
            // Free window capacity for next batch.
            router.record_ack(mcu, 1).unwrap();
        }
        assert!(emitted > 100, "need >100 emits, got {emitted}");

        let (sent, _) = router.extract_old(mcu).unwrap();
        assert_eq!(sent.len(), 100);
    }

    #[test]
    fn flush_does_not_fire_twice_without_new_entries() {
        let (mut router, _) = make_router();
        let mcu = router.claim_mcu("mcu");
        let q = router.alloc_command_queue(mcu).unwrap();

        let count = Arc::new(Mutex::new(0u32));
        let count2 = Arc::clone(&count);
        router
            .register_flush_callback(mcu, Box::new(move || {
                *count2.lock().unwrap() += 1;
            }))
            .unwrap();

        router.push(mcu, q, entry(0, 10)).unwrap();
        let _ = router.pop_next_for_emission(mcu).unwrap();
        router.check_flush(mcu).unwrap();
        assert_eq!(*count.lock().unwrap(), 1);

        // Second check_flush without new entries — should not fire again.
        router.check_flush(mcu).unwrap();
        assert_eq!(*count.lock().unwrap(), 1);
    }
}
