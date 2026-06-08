use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::time::Instant;

use arc_swap::ArcSwap;

use crate::fault::FaultLatch;
use crate::host_io::runtime_events::{
    CreditFreedEvent, McuLogEvent, RuntimeEvent, StatusEvent, TraceEvent,
};

#[derive(Debug, Clone)]
pub enum HostEvent {
    TraceSubscriberOverflow {
        dropped_count: u64,
        at: Instant,
    },
    TraceSubscriberDisconnected {
        at: Instant,
    },
    TraceSubscriberReattached {
        events_lost_during_gap: u64,
        at: Instant,
    },
}

#[derive(Debug)]
pub struct TraceRing {
    #[allow(dead_code)] // stored at construction, future use for overflow policy
    capacity: usize,
    sticky_overflow: bool,
    subscriber: Option<SyncSender<TraceEvent>>,
    drop_count_since_event: u64,
    host_event_tx: Option<SyncSender<HostEvent>>,
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
            event.flags |= 0x01;
        }

        match self.subscriber.as_ref() {
            Some(tx) => match tx.try_send(event) {
                Ok(()) => {
                    self.sticky_overflow = false;
                }
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
                    self.sticky_overflow = false;
                    self.drop_count_since_event = 1;
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

#[derive(Debug, Default)]
pub struct RuntimeEventDispatcher {
    subscriber: Option<SyncSender<RuntimeEvent>>,
}

impl RuntimeEventDispatcher {
    pub fn dispatch(&mut self, event: RuntimeEvent) {
        if let Some(tx) = &self.subscriber {
            match tx.try_send(event) {
                Ok(()) => {}
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

#[derive(Debug)]
pub struct HostEventDispatcher {
    inbox_rx: Receiver<HostEvent>,
    subscriber: Option<SyncSender<HostEvent>>,
}

impl HostEventDispatcher {
    pub fn new(inbox_rx: Receiver<HostEvent>) -> Self {
        Self {
            inbox_rx,
            subscriber: None,
        }
    }

    pub fn drain_pending(&mut self) {
        while let Ok(event) = self.inbox_rx.try_recv() {
            self.dispatch(event);
        }
    }

    pub fn dispatch(&mut self, event: HostEvent) {
        if let Some(tx) = &self.subscriber {
            match tx.try_send(event) {
                Ok(()) => {}
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

// Manual Debug — heartbeat_callback and mcu_log_hook are trait objects and cannot derive.
impl std::fmt::Debug for EventDispatcher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventDispatcher")
            .field("fault_latch", &self.fault_latch)
            .field("trace_ring", &self.trace_ring)
            .field("status_snapshot", &"<ArcSwap<StatusEvent>>")
            .field("runtime_event_dispatcher", &self.runtime_event_dispatcher)
            .field("host_event_dispatcher", &self.host_event_dispatcher)
            .field("status_retired_watermark", &self.status_retired_watermark)
            .field(
                "heartbeat_callback",
                if self.heartbeat_callback.is_some() {
                    &"Some(<fn>)"
                } else {
                    &"None"
                },
            )
            .field(
                "mcu_log_hook",
                if self.mcu_log_hook.is_some() {
                    &"Some(<fn>)"
                } else {
                    &"None"
                },
            )
            .finish_non_exhaustive()
    }
}

pub struct EventDispatcher {
    pub fault_latch: FaultLatch,
    pub trace_ring: TraceRing,
    pub status_snapshot: Arc<ArcSwap<StatusEvent>>,
    pub runtime_event_dispatcher: RuntimeEventDispatcher,
    pub host_event_dispatcher: HostEventDispatcher,
    status_retired_watermark: u32,
    pub heartbeat_callback: Option<Arc<dyn Fn(&[u32]) + Send + Sync>>,
    pub mcu_log_hook: Option<Box<dyn Fn(McuLogEvent) + Send + Sync>>,
}

impl EventDispatcher {
    pub fn new(
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        trace_capacity: usize,
        host_event_capacity: usize,
    ) -> Self {
        let (host_tx, host_rx) = sync_channel::<HostEvent>(host_event_capacity);
        let mut trace_ring = TraceRing::new(trace_capacity);
        trace_ring.set_host_event_tx(host_tx);
        Self {
            fault_latch: FaultLatch::default(),
            trace_ring,
            status_snapshot,
            runtime_event_dispatcher: RuntimeEventDispatcher::default(),
            host_event_dispatcher: HostEventDispatcher::new(host_rx),
            status_retired_watermark: 0,
            heartbeat_callback: None,
            mcu_log_hook: None,
        }
    }

    pub fn set_mcu_log_hook<F>(&mut self, f: F)
    where
        F: Fn(McuLogEvent) + Send + Sync + 'static,
    {
        self.mcu_log_hook = Some(Box::new(f));
    }

    pub fn dispatch(&mut self, event: RuntimeEvent) {
        match event {
            RuntimeEvent::CreditFreed(e) => {
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::CreditFreed(e));
            }
            RuntimeEvent::Fault(e) => {
                let signed_code = e.fault_code as i16 as i32;
                log::warn!(
                    "[KALICO-FAULT] received FaultEvent \
                     fault_code={} (wire_u16={}) fault_detail={:#010x} \
                     segment_id={:#010x} synthesized={} \
                     (segment_id is the -311 stacked PC = addr2line target: \
                     the instruction the interrupted context was about to \
                     execute, i.e. the code holding the CPU/PRIMASK across the \
                     late tick; 0 for non-311 faults. \
                     see runtime::error::FaultCode: \
                     -308=PieceStartInPast -309=RingFull \
                     -310=StepsPerSampleExceeded -311=TickIntervalExceeded \
                     -302=MathNonFinite -303=PieceAdvanceUnderflow \
                     -300=StepQueueOverflow)",
                    signed_code,
                    e.fault_code,
                    e.fault_detail,
                    e.segment_id,
                    e.synthesized,
                );
                self.fault_latch.dispatch(e.clone());
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Fault(e));
            }
            RuntimeEvent::Trace(e) => {
                self.trace_ring.dispatch(e);
            }
            RuntimeEvent::Status(e) => {
                let synth_credit = self.handle_status_frame(&e);
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Status(e));
                if let Some(c) = synth_credit {
                    self.dispatch(RuntimeEvent::CreditFreed(c));
                }
            }
            RuntimeEvent::Heartbeat { retired_counts } => {
                if let Some(cb) = &self.heartbeat_callback {
                    cb(&retired_counts);
                }
            }
            RuntimeEvent::EndstopTrip(_)
            | RuntimeEvent::UnknownOutput { .. }
            | RuntimeEvent::PassthroughResponse { .. } => {
                self.runtime_event_dispatcher.dispatch(event);
            }
            RuntimeEvent::McuLog(e) => {
                if let Some(hook) = &self.mcu_log_hook {
                    hook(e.clone());
                }
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::McuLog(e));
            }
        }
    }

    fn handle_status_frame(&mut self, frame: &StatusEvent) -> Option<CreditFreedEvent> {
        const ENGINE_STATUS_FAULT: u8 = 3;
        const Q_N_MINUS_1: u8 = 7;

        self.status_snapshot.store(Arc::new(frame.clone()));

        if frame.engine_status == ENGINE_STATUS_FAULT && self.fault_latch.cell.is_none() {
            let synthesized = crate::host_io::runtime_events::FaultEvent {
                fault_code: frame.last_fault,
                fault_detail: frame.fault_detail,
                segment_id: frame.current_segment_id,
                synthesized: true,
            };
            self.fault_latch.dispatch(synthesized);
        }

        let watermark = frame.retired_through_segment_id;
        #[allow(clippy::cast_possible_wrap)]
        let advanced = (watermark.wrapping_sub(self.status_retired_watermark) as i32) > 0;
        if advanced {
            self.status_retired_watermark = watermark;
            let free_slots = Q_N_MINUS_1.saturating_sub(frame.queue_depth);
            Some(CreditFreedEvent {
                retired_through_segment_id: watermark,
                free_slots,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod dispatch_tests;
