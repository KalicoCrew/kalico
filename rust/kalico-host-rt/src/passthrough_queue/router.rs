//! Top-level passthrough router — the boundary the bridge calls.
//! Owns one `McuState` + `NotifyTable` + `ReceiveWindow` per MCU.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock};
use std::time::Instant;

use indexmap::IndexMap;

use super::config_stage::{ConfigStage, ConfigStagePhase};
use super::entry::{NotifyId, PassthroughEntry};
use super::mcu_state::{CommandQueueId, McuState, PushError};
use super::notify::{NotifyCallback, NotifyResponse, NotifyTable};
use super::receive_window::ReceiveWindow;
use crate::clock::Clock;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct McuHandle(u32);

impl McuHandle {
    pub fn raw(&self) -> u32 {
        self.0
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
    /// any notify-bearing entry.
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
            let bytes_len = e.bytes().len() as u64;
            rec.window.record_emit(bytes_len);
            if !e.notify_id().is_none() {
                let now = instant_to_f64(self.clock.now());
                rec.sent_times.insert(e.notify_id(), now);
            }
        }
        Ok(entry)
    }

    /// Dispatch a response for a previously-sent notify-bearing entry.
    pub fn dispatch_response(
        &mut self,
        mcu: McuHandle,
        notify_id: NotifyId,
        response_bytes: Vec<u8>,
    ) -> Result<(), RouterError> {
        let rec = self.mcus.get_mut(&mcu).ok_or(RouterError::UnknownMcu(mcu))?;
        let sent_time = rec.sent_times.remove(&notify_id).unwrap_or(0.0);
        let receive_time = instant_to_f64(self.clock.now());
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
}
