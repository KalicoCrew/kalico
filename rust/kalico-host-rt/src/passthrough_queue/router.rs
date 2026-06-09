use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use indexmap::IndexMap;

use super::config_stage::{ConfigStage, ConfigStagePhase};
use super::debug_log::{DebugEntry, DebugLog};
use super::entry::{NotifyId, PassthroughEntry};
use super::mcu_state::{CommandQueueId, McuState, PushError};
use super::notify::{NotifyCallback, NotifyResponse, NotifyTable};
use super::receive_window::ReceiveWindow;
use super::stats::{PassthroughStats, StatsCounters};
use crate::clock::{Clock, instant_to_f64};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);

impl McuHandle {
    pub fn raw(&self) -> u32 {
        self.0
    }

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
    clock_freq: f64,
    clock_offset: f64,
    last_clock: u64,
    flush_callbacks: Vec<Box<dyn Fn() + Send>>,
    was_non_empty: bool,
    stats: StatsCounters,
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

impl PassthroughRouter {
    pub fn with_clock(clock: Arc<dyn Clock + Send + Sync>) -> Self {
        Self {
            mcus: IndexMap::new(),
            next_handle: 0,
            clock,
        }
    }

    pub fn mcu_handles(&self) -> impl Iterator<Item = &McuHandle> {
        self.mcus.keys()
    }

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

    pub fn begin_config_phase(&mut self, mcu: McuHandle) -> Result<(), RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.config_stage.begin_config_send();
        Ok(())
    }

    pub fn next_config_entry(&mut self, mcu: McuHandle) -> Result<Option<Vec<u8>>, RouterError> {
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.next_config_entry())
    }

    pub fn config_phase(&self, mcu: McuHandle) -> Result<ConfigStagePhase, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        Ok(rec.config_stage.phase())
    }

    pub fn set_clock_est(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset: f64,
        last_clock: u64,
    ) -> Result<(), RouterError> {
        log::info!(
            "[clock-seed] set_clock_est mcu={:?} freq={:.1} offset={:.9} last_clock={}",
            mcu,
            freq,
            offset,
            last_clock
        );
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = offset;
        rec.last_clock = last_clock;
        Ok(())
    }

    /// Set the router's clock record from a Python-side clocksync estimate.
    ///
    /// `offset_raw` is `time_avg + min_half_rtt` in CLOCK_MONOTONIC_RAW seconds
    /// (what Python's `_handle_clock` computes and the mirror callback exports).
    /// `host_now_raw` is accepted for API compatibility but is NOT used in the
    /// projection — using it would embed the Python→Rust GIL-hop latency ε
    /// directly into `clock_offset`, biasing every subsequent projection by ε
    /// (up to tens of ms on a loaded Pi 3B).
    ///
    /// Instead, `CLOCK_MONOTONIC_RAW` is read here in Rust at the same instant
    /// as `instant_to_f64(self.clock.now())`, so the conversion constant
    /// `raw_at_anchor = raw_now - instant_now` is computed without any
    /// cross-runtime latency and `clock_offset = offset_raw - raw_at_anchor`
    /// is exact up to µs sample skew.
    pub fn set_clock_est_rebased(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        offset_raw: f64,
        last_clock: u64,
        _host_now_raw: f64,
    ) -> Result<(), RouterError> {
        let bridge_now_instant = instant_to_f64(self.clock.now());
        let bridge_now_raw = crate::clock::monotonic_raw_secs();
        let clock_offset = offset_raw - (bridge_now_raw - bridge_now_instant);
        log::info!(
            "[clock-seed] set_clock_est_rebased mcu={:?} freq={:.1} offset_raw={:.9} \
             bridge_now_raw={:.9} bridge_now_instant={:.9} clock_offset={:.9} last_clock={}",
            mcu,
            freq,
            offset_raw,
            bridge_now_raw,
            bridge_now_instant,
            clock_offset,
            last_clock
        );
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = clock_offset;
        rec.last_clock = last_clock;
        Ok(())
    }

    pub fn set_clock_est_from_sample(
        &mut self,
        mcu: McuHandle,
        freq: f64,
        host_send: Instant,
        mcu_at_send: u64,
    ) -> Result<(), RouterError> {
        let clock_offset = instant_to_f64(host_send);
        log::info!(
            "[clock-seed] set_clock_est_from_sample mcu={:?} freq={:.1} \
             clock_offset={:.9} mcu_at_send={}",
            mcu,
            freq,
            clock_offset,
            mcu_at_send
        );
        let rec = self
            .mcus
            .get_mut(&mcu)
            .ok_or(RouterError::UnknownMcu(mcu))?;
        rec.clock_freq = freq;
        rec.clock_offset = clock_offset;
        rec.last_clock = mcu_at_send;
        Ok(())
    }

    /// Convert an MCU tick count to a wall-clock `OffsetDateTime`.
    ///
    /// Returns `None` when no clock record has been set for this MCU
    /// (i.e. `clock_freq == 0.0` — no `set_clock_est_rebased` call yet).
    ///
    /// `estimated = true` when the tick is more than one frequency-second from
    /// the anchor, i.e. significant extrapolation.
    pub fn wall_time_at_mcu(
        &self,
        mcu: McuHandle,
        mcu_ticks: u64,
    ) -> Option<(time::OffsetDateTime, bool)> {
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (rec.last_clock as f64);
        let mcu_host_instant = rec.clock_offset + delta_ticks / rec.clock_freq;
        let now_instant = instant_to_f64(self.clock.now());
        let delta_from_now = mcu_host_instant - now_instant;
        let wall_now = std::time::SystemTime::now();
        let wall_time = if delta_from_now >= 0.0 {
            wall_now
                .checked_add(std::time::Duration::from_secs_f64(delta_from_now))
                .unwrap_or(wall_now)
        } else {
            wall_now
                .checked_sub(std::time::Duration::from_secs_f64(-delta_from_now))
                .unwrap_or(wall_now)
        };
        let estimated = delta_ticks.abs() / rec.clock_freq > 1.0;
        Some((time::OffsetDateTime::from(wall_time), estimated))
    }

    pub fn ack_clock_and_freq(&self, mcu: McuHandle) -> Option<(u64, f64)> {
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        let host_now = instant_to_f64(self.clock.now());
        let delta = (host_now - rec.clock_offset) * rec.clock_freq;
        #[allow(clippy::cast_sign_loss)]
        let projected = rec.last_clock.wrapping_add(delta.max(0.0) as u64);
        Some((projected, rec.clock_freq))
    }

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

    pub fn host_now_secs(&self) -> f64 {
        instant_to_f64(self.clock.now())
    }

    pub fn clock_to_host_secs(&self, mcu: McuHandle, mcu_clock: u64) -> Option<f64> {
        let rec = self.mcus.get(&mcu)?;
        if rec.clock_freq == 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_clock as f64) - (rec.last_clock as f64);
        Some(rec.clock_offset + delta_ticks / rec.clock_freq)
    }

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
        log::trace!(
            "[project] host_time_to_mcu_clock mcu={:?} host_secs={:.9} clock_offset={:.9} \
             last_clock={} clock_freq={:.1} result_ns={}",
            mcu,
            host_time_secs,
            rec.clock_offset,
            rec.last_clock,
            rec.clock_freq,
            projected
        );
        Ok(projected)
    }

    pub fn log_seg0_deficit(&self, mcu: McuHandle, seg0_host_secs: f64, t0: f64) {
        let rec = match self.mcus.get(&mcu) {
            Some(r) => r,
            None => {
                log::warn!("[seg0-deficit] mcu={:?} UNKNOWN", mcu);
                return;
            }
        };
        if rec.clock_freq == 0.0 {
            log::warn!(
                "[seg0-deficit] mcu={:?} clock_freq=0 (not yet synced) t0={:.6} seg0_host={:.6}",
                mcu,
                t0,
                seg0_host_secs
            );
            return;
        }
        let start_time = self
            .host_time_to_mcu_clock(mcu, seg0_host_secs)
            .unwrap_or(0);
        let ack_now = self.compute_ack_clock(mcu).unwrap_or(0);
        let deficit_ticks = start_time as i64 - ack_now as i64;
        let deficit_us = (deficit_ticks as f64 / rec.clock_freq) * 1e6;
        log::warn!(
            "[seg0-deficit] mcu={:?} freq={:.1} offset={:.6} last_clock={} t0={:.6} seg0_host={:.6} start_time={} ack_now={} deficit_ticks={} deficit_us={:.1} (negative=>in past)",
            mcu,
            rec.clock_freq,
            rec.clock_offset,
            rec.last_clock,
            t0,
            seg0_host_secs,
            start_time,
            ack_now,
            deficit_ticks,
            deficit_us
        );
    }

    pub fn get_stats(&self, mcu: McuHandle) -> Result<PassthroughStats, RouterError> {
        let rec = self.mcus.get(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        let mut snap = rec.stats.snapshot();
        snap.ready_bytes = rec.state.total_ready_bytes();
        snap.upcoming_bytes = rec.state.total_upcoming_bytes();
        Ok(snap)
    }

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
