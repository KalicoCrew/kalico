//! EventDispatcher subsystem. Spec §6. (Phase-C stub; Phase D adds the rest.)

use std::sync::mpsc::{SyncSender, TrySendError};
use std::time::Instant;

use crate::host_io::runtime_events::{RuntimeEvent, TraceEvent};

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
            event.flags |= 0x01; // OVERFLOW flag
            self.sticky_overflow = false;
        }

        match self.subscriber.as_ref() {
            Some(tx) => match tx.try_send(event) {
                Ok(_) => {}
                Err(TrySendError::Full(_)) => {
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
                    self.drop_count_since_event = 0;
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

#[derive(Debug, Default)]
pub struct HostEventDispatcher {
    subscriber: Option<SyncSender<HostEvent>>,
}

impl HostEventDispatcher {
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
