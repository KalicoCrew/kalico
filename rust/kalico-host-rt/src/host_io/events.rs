//! EventDispatcher subsystem. Spec §6. (Phase-C stub; Phase D adds the rest.)

use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::sync::Arc;
use std::time::Instant;

use arc_swap::ArcSwap;

use crate::credit::CreditCounter;
use crate::fault::FaultLatch;
use crate::host_io::runtime_events::{RuntimeEvent, StatusEvent, TraceEvent};

#[derive(Debug, Clone)]
pub enum HostEvent {
    TraceSubscriberOverflow { dropped_count: u64, at: Instant },
    TraceSubscriberDisconnected { at: Instant },
    TraceSubscriberReattached { events_lost_during_gap: u64, at: Instant },
}

// ─── D6: TraceRing ───────────────────────────────────────────────────────────

#[derive(Debug)]
pub struct TraceRing {
    capacity:               usize,
    sticky_overflow:        bool,
    subscriber:             Option<SyncSender<TraceEvent>>,
    drop_count_since_event: u64,
    host_event_tx:          Option<SyncSender<HostEvent>>,
}

impl TraceRing {
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            sticky_overflow: false,
            subscriber: None,
            drop_count_since_event: 0,
            host_event_tx: None,
        }
    }

    pub fn dispatch(&mut self, mut event: TraceEvent) {
        if self.sticky_overflow {
            event.flags |= 0x01; // OVERFLOW flag — mark before try_send (Bug 3 fix)
        }

        match self.subscriber.as_ref() {
            Some(tx) => match tx.try_send(event) {
                Ok(_) => {
                    self.sticky_overflow = false; // Bug 3 fix: clear only on successful delivery
                }
                Err(TrySendError::Full(_)) => {
                    // sticky_overflow stays true; drop count increments
                    self.sticky_overflow = true;
                    self.drop_count_since_event += 1;
                    if let Some(host_tx) = &self.host_event_tx {
                        let _ = host_tx.try_send(HostEvent::TraceSubscriberOverflow {
                            dropped_count: self.drop_count_since_event,
                            at: Instant::now(),
                        });
                    }
                }
                Err(TrySendError::Disconnected(_)) => {
                    self.subscriber = None;
                    self.sticky_overflow = false;     // Bug 2 fix: don't carry overflow to new subscriber
                    self.drop_count_since_event = 1;  // Bug 1 fix: count the event that triggered disconnect
                    if let Some(host_tx) = &self.host_event_tx {
                        let _ = host_tx.try_send(HostEvent::TraceSubscriberDisconnected {
                            at: Instant::now(),
                        });
                    }
                }
            },
            None => self.drop_count_since_event += 1,
        }
    }

    pub fn subscribe(
        &mut self,
        tx: SyncSender<TraceEvent>,
    ) -> Result<(), crate::transport::SubscribeError> {
        if self.subscriber.is_some() {
            return Err(crate::transport::SubscribeError::AlreadySubscribed { channel: "trace" });
        }
        if self.drop_count_since_event > 0 {
            if let Some(host_tx) = &self.host_event_tx {
                let _ = host_tx.try_send(HostEvent::TraceSubscriberReattached {
                    events_lost_during_gap: self.drop_count_since_event,
                    at: Instant::now(),
                });
            }
            self.drop_count_since_event = 0;
        }
        self.subscriber = Some(tx);
        Ok(())
    }

    pub fn set_host_event_tx(&mut self, tx: SyncSender<HostEvent>) {
        self.host_event_tx = Some(tx);
    }
}

// ─── D7: RuntimeEventDispatcher ──────────────────────────────────────────────

#[derive(Debug, Default)]
pub struct RuntimeEventDispatcher {
    subscriber: Option<SyncSender<RuntimeEvent>>,
}

impl RuntimeEventDispatcher {
    pub fn dispatch(&mut self, event: RuntimeEvent) {
        if let Some(tx) = &self.subscriber {
            match tx.try_send(event) {
                Ok(_) => {}
                Err(TrySendError::Full(e)) => {
                    log::warn!("runtime-event subscriber overflow; dropping: {e:?}");
                }
                Err(TrySendError::Disconnected(_)) => {
                    self.subscriber = None;
                }
            }
        }
    }

    pub fn subscribe(
        &mut self,
        tx: SyncSender<RuntimeEvent>,
    ) -> Result<(), crate::transport::SubscribeError> {
        if self.subscriber.is_some() {
            return Err(crate::transport::SubscribeError::AlreadySubscribed {
                channel: "runtime_event",
            });
        }
        self.subscriber = Some(tx);
        Ok(())
    }
}

// ─── D8: HostEventDispatcher ─────────────────────────────────────────────────
//
// Drains a shared inbox written by `TraceRing` (and any other reactor-internal
// host-event source) and forwards to the user-attached subscriber. The inbox
// must exist at construction time so `TraceRing::set_host_event_tx` can be
// wired at `EventDispatcher::new` — before any subscriber attaches. The
// reactor calls `drain_pending` once per loop iteration.

#[derive(Debug)]
pub struct HostEventDispatcher {
    inbox_rx:   Receiver<HostEvent>,
    subscriber: Option<SyncSender<HostEvent>>,
}

impl HostEventDispatcher {
    pub fn new(inbox_rx: Receiver<HostEvent>) -> Self {
        Self { inbox_rx, subscriber: None }
    }

    /// Forward any events queued in the inbox to the attached subscriber.
    /// Drops events on the floor when no subscriber is attached.
    pub fn drain_pending(&mut self) {
        while let Ok(event) = self.inbox_rx.try_recv() {
            self.dispatch(event);
        }
    }

    pub fn dispatch(&mut self, event: HostEvent) {
        if let Some(tx) = &self.subscriber {
            match tx.try_send(event) {
                Ok(_) => {}
                Err(TrySendError::Full(_)) => {
                    log::warn!("host-event subscriber overflow; dropping");
                }
                Err(TrySendError::Disconnected(_)) => {
                    self.subscriber = None;
                }
            }
        }
    }

    pub fn subscribe(
        &mut self,
        tx: SyncSender<HostEvent>,
    ) -> Result<(), crate::transport::SubscribeError> {
        if self.subscriber.is_some() {
            return Err(crate::transport::SubscribeError::AlreadySubscribed {
                channel: "host_event",
            });
        }
        self.subscriber = Some(tx);
        Ok(())
    }

    pub fn sender_handle(&self) -> Option<SyncSender<HostEvent>> {
        self.subscriber.clone()
    }
}

// ─── D9: EventDispatcher composition ─────────────────────────────────────────

pub struct EventDispatcher {
    pub credit_counter:           Option<Arc<CreditCounter>>,
    pub fault_latch:              FaultLatch,
    pub trace_ring:               TraceRing,
    pub status_snapshot:          Arc<ArcSwap<StatusEvent>>,
    pub runtime_event_dispatcher: RuntimeEventDispatcher,
    pub host_event_dispatcher:    HostEventDispatcher,
}

impl EventDispatcher {
    pub fn new(
        status_snapshot:     Arc<ArcSwap<StatusEvent>>,
        trace_capacity:      usize,
        host_event_capacity: usize,
    ) -> Self {
        // Spec §6.4 / §6.8: TraceRing emits HostEvents (overflow / disconnect /
        // reattach) into a shared bounded channel; HostEventDispatcher drains
        // it on each reactor loop iteration and forwards to the user subscriber.
        let (host_tx, host_rx) = sync_channel::<HostEvent>(host_event_capacity);
        let mut trace_ring = TraceRing::new(trace_capacity);
        trace_ring.set_host_event_tx(host_tx);
        Self {
            credit_counter: None,
            fault_latch: FaultLatch::default(),
            trace_ring,
            status_snapshot,
            runtime_event_dispatcher: RuntimeEventDispatcher::default(),
            host_event_dispatcher: HostEventDispatcher::new(host_rx),
        }
    }

    pub fn dispatch(&mut self, event: RuntimeEvent) {
        match event {
            RuntimeEvent::CreditFreed(e) => {
                if let Some(counter) = &self.credit_counter {
                    counter.on_credit_freed(e.free_slots);
                }
            }
            RuntimeEvent::Fault(e) => {
                self.fault_latch.dispatch(e);
            }
            RuntimeEvent::Trace(e) => {
                self.trace_ring.dispatch(e);
            }
            RuntimeEvent::Status(e) => {
                self.handle_status_frame(&e);
            }
            ev @ RuntimeEvent::UnknownOutput { .. } => {
                self.runtime_event_dispatcher.dispatch(ev);
            }
        }
    }

    /// Per spec §6.5. Update snapshot AND synthesize FaultEvent if engine_status
    /// is FAULT and no fault has been latched on the host.
    fn handle_status_frame(&mut self, frame: &StatusEvent) {
        self.status_snapshot.store(Arc::new(frame.clone()));

        const ENGINE_STATUS_FAULT: u8 = 3;
        if frame.engine_status == ENGINE_STATUS_FAULT && self.fault_latch.cell.is_none() {
            let synthesized = crate::host_io::runtime_events::FaultEvent {
                fault_code:   frame.last_fault,
                fault_detail: frame.fault_detail,
                segment_id:   frame.current_segment_id,
                synthesized:  true,
            };
            self.fault_latch.dispatch(synthesized);
        }
    }
}

// ─── D10a: Tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod dispatch_tests {
    use super::*;
    use std::sync::Arc;
    use arc_swap::ArcSwap;
    use crate::host_io::runtime_events::{FaultEvent, RuntimeEvent, StatusEvent};

    fn make_dispatcher() -> EventDispatcher {
        let snap = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
        EventDispatcher::new(snap, 256, 64)
    }

    fn fault_status(engine_status: u8, last_fault: u16, segment_id: u32) -> RuntimeEvent {
        RuntimeEvent::Status(StatusEvent {
            engine_status,
            current_segment_id: segment_id,
            last_fault,
            fault_detail: 0,
        })
    }

    #[test]
    fn fault_status_synthesizes_when_no_edge_observed() {
        let mut d = make_dispatcher();
        d.dispatch(fault_status(3, 17, 42));
        let cell = d.fault_latch.cell.as_ref().expect("fault should be synthesized");
        assert_eq!(cell.fault_code, 17);
        assert_eq!(cell.synthesized, true);
        assert_eq!(cell.segment_id, 42);
    }

    #[test]
    fn synthesis_idempotent_across_repeated_status_frames() {
        let mut d = make_dispatcher();
        d.dispatch(fault_status(3, 17, 42));
        d.dispatch(fault_status(3, 17, 42));
        // Still latched once (cell still present, still synthesized).
        let cell = d.fault_latch.cell.as_ref().unwrap();
        assert_eq!(cell.synthesized, true);
    }

    #[test]
    fn edge_event_upgrades_synthesized_in_place() {
        let mut d = make_dispatcher();
        d.dispatch(fault_status(3, 17, 42));
        // Edge event with exact segment_id preferred.
        d.dispatch(RuntimeEvent::Fault(FaultEvent {
            fault_code: 17,
            fault_detail: 0,
            segment_id: 39,
            synthesized: false,
        }));
        let cell = d.fault_latch.cell.as_ref().unwrap();
        assert!(!cell.synthesized, "edge upgrade clears synthesized");
        assert_eq!(cell.segment_id, 39, "edge segment_id preferred");
    }

    #[test]
    fn status_without_fault_does_not_synthesize() {
        let mut d = make_dispatcher();
        d.dispatch(fault_status(1, 0, 0)); // engine_status != 3
        assert!(d.fault_latch.cell.is_none());
    }
}
