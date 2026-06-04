//! `EventDispatcher` subsystem. Spec §6. (Phase-C stub; Phase D adds the rest.)

use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::time::Instant;

use arc_swap::ArcSwap;

use crate::fault::FaultLatch;
use crate::host_io::runtime_events::{CreditFreedEvent, McuLogEvent, RuntimeEvent, StatusEvent, TraceEvent};

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

// ─── D6: TraceRing ───────────────────────────────────────────────────────────

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
            event.flags |= 0x01; // OVERFLOW flag — mark before try_send (Bug 3 fix)
        }

        match self.subscriber.as_ref() {
            Some(tx) => match tx.try_send(event) {
                Ok(()) => {
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
                    self.sticky_overflow = false; // Bug 2 fix: don't carry overflow to new subscriber
                    self.drop_count_since_event = 1; // Bug 1 fix: count the event that triggered disconnect
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

// ─── D8: HostEventDispatcher ─────────────────────────────────────────────────
//
// Drains a shared inbox written by `TraceRing` (and any other reactor-internal
// host-event source) and forwards to the user-attached subscriber. The inbox
// must exist at construction time so `TraceRing::set_host_event_tx` can be
// wired at `EventDispatcher::new` — before any subscriber attaches. The
// reactor calls `drain_pending` once per loop iteration.

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

// ─── D9: EventDispatcher composition ─────────────────────────────────────────

/// Manual `Debug` — `heartbeat_callback` and `mcu_log_hook` are trait objects
/// and cannot derive.
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
    /// Last `retired_through_segment_id` observed on a `StatusEvent`. Drives
    /// the synthesis of `CreditFreed` from the 10 Hz periodic status frame —
    /// see [`Self::handle_status_frame`] for the motivation (USB-CDC TX
    /// congestion can drop fire-and-forget `CreditFreed` frames, but the
    /// periodic status frame is monotonic state so we can recover credit
    /// flow from it deterministically).
    status_retired_watermark: u32,
    /// Optional callback fired on every `StatusHeartbeat` (0x0083) with the
    /// per-axis consumed-piece counts. Pump-private: the event is consumed
    /// here and NOT forwarded to the general `runtime_rx` channel.
    /// Set via [`ReactorCommand::AttachHeartbeatCallback`].
    pub heartbeat_callback: Option<Arc<dyn Fn(&[u32]) + Send + Sync>>,
    /// Optional hook fired on every decoded `McuLog (0x0084)` event with an
    /// owned `McuLogEvent`. The hook is called before the event is forwarded
    /// to the general `runtime_rx` channel. Set via [`Self::set_mcu_log_hook`].
    pub mcu_log_hook: Option<Box<dyn Fn(McuLogEvent) + Send + Sync>>,
}

impl EventDispatcher {
    pub fn new(
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        trace_capacity: usize,
        host_event_capacity: usize,
    ) -> Self {
        // Spec §6.4 / §6.8: TraceRing emits HostEvents (overflow / disconnect /
        // reattach) into a shared bounded channel; HostEventDispatcher drains
        // it on each reactor loop iteration and forwards to the user subscriber.
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

    /// Attach a closure that fires on every decoded `McuLog (0x0084)` event.
    ///
    /// The closure receives an owned [`McuLogEvent`] so it can move the value
    /// into a channel or process it without holding a borrow on the dispatcher.
    /// Replaces any previously set hook. The hook fires before the event is
    /// forwarded to the general `runtime_rx` subscriber channel.
    pub fn set_mcu_log_hook<F>(&mut self, f: F)
    where
        F: Fn(McuLogEvent) + Send + Sync + 'static,
    {
        self.mcu_log_hook = Some(Box::new(f));
    }

    pub fn dispatch(&mut self, event: RuntimeEvent) {
        match event {
            RuntimeEvent::CreditFreed(e) => {
                // Forward to the python bridge poller so motion-bridge can
                // observe credit_freed events for diagnostics. The credit
                // counter and slot-pool retirement wiring have been removed
                // (Task 10); only the forwarding path remains.
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::CreditFreed(e));
            }
            RuntimeEvent::Fault(e) => {
                // Log before shutdown() races with the USB-CDC drop so the
                // fault code reaches the journal even if the FaultEvent frame
                // is lost after reset. `journalctl -u klippy -g KALICO-FAULT`
                // MCU wire encoding: negative i32 as (i32 as i16) as u16;
                // reverse: (u16 as i16) as i32.
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
                // Forward to the python bridge poller too — fault visibility
                // matters end-to-end. Pre-Phase-C this rode the legacy
                // `kalico_fault` Klipper output; now kalico-native delivers
                // FaultEvent into RuntimeEvent::Fault.
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Fault(e));
            }
            RuntimeEvent::Trace(e) => {
                self.trace_ring.dispatch(e);
            }
            RuntimeEvent::Status(e) => {
                let synth_credit = self.handle_status_frame(&e);
                // Also forward to the general runtime-event channel so callers of
                // `take_runtime_event_subscription` (e.g. the Python bridge poller)
                // can observe status heartbeats.  The snapshot and fault-synthesis
                // paths above are orthogonal and still fire first.
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Status(e));
                // v2 architectural credit-flow: status frames carry the retirement
                // watermark. On advance, synthesize a CreditFreed and dispatch
                // through the normal CreditFreed path so the bridge poller can
                // observe it via take_runtime_event.
                if let Some(c) = synth_credit {
                    self.dispatch(RuntimeEvent::CreditFreed(c));
                }
            }
            RuntimeEvent::Heartbeat { retired_counts } => {
                // Pump-private: consumed here, NOT forwarded to the general
                // runtime_rx channel. The heartbeat callback feeds the host
                // pump's flow-control logic directly over a channel.
                if let Some(cb) = &self.heartbeat_callback {
                    cb(&retired_counts);
                }
            }
            RuntimeEvent::EndstopTripped(_)
            | RuntimeEvent::UnknownOutput { .. }
            | RuntimeEvent::PassthroughResponse { .. } => {
                self.runtime_event_dispatcher.dispatch(event);
            }
            RuntimeEvent::McuLog(e) => {
                if let Some(hook) = &self.mcu_log_hook {
                    hook(e.clone());
                }
                // Also forward to the general runtime channel so callers that
                // subscribe to take_runtime_event_subscription can observe it.
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::McuLog(e));
            }
        }
    }

    /// Per spec §6.5. Update snapshot AND synthesize `FaultEvent` if `engine_status`
    /// is FAULT and no fault has been latched on the host.
    ///
    /// Returns `Some(CreditFreedEvent)` when the retirement watermark advanced
    /// past the previously observed value — the caller dispatches that
    /// synthesized event through the normal `CreditFreed` machinery so the
    /// bridge poller can observe retirement progress even when the firmware's
    /// fire-and-forget `CreditFreed` frame was dropped under TX congestion.
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

        // v2 credit-flow synthesis. Use signed-difference comparison so
        // wraparound (~4 billion segments) doesn't trigger spurious advances
        // after MCU reset. The firmware encodes free_slots = Q_N-1 -
        // queue_depth (saturated at 0) — replicate the same computation here.
        let watermark = frame.retired_through_segment_id;
        #[allow(clippy::cast_possible_wrap)] // intentional signed-difference comparison for wrap detection
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

// ─── D10a: Tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod dispatch_tests;
