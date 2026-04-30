//! EventDispatcher subsystem. Spec §6. (Phase-C stub; Phase D adds the rest.)

use std::time::Instant;

#[derive(Debug, Clone)]
pub enum HostEvent {
    TraceSubscriberOverflow { dropped_count: u64, at: Instant },
    TraceSubscriberDisconnected { at: Instant },
    TraceSubscriberReattached { events_lost_during_gap: u64, at: Instant },
}
