//! Core data types for passthrough queue entries.

/// Sentinel `req_clock` value: entries with this priority are only emitted
/// when no non-background entries exist across any queue. Mirrors
/// `BACKGROUND_PRIORITY_CLOCK` in serialqueue.c.
pub const BACKGROUND_PRIORITY_CLOCK: u64 = u64::MAX;

/// Opaque identifier used to correlate a sent command with its MCU response.
/// A value of 0 means "no notification requested."
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NotifyId(u64);

impl NotifyId {
    pub const fn none() -> Self {
        Self(0)
    }

    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    pub fn is_none(&self) -> bool {
        self.0 == 0
    }

    pub fn raw(&self) -> u64 {
        self.0
    }
}

/// A single message queued for transmission to an MCU.
///
/// - `min_clock`: the promotion gate — the entry stays in `upcoming` until
///   `ack_clock >= min_clock`.
/// - `req_clock`: the emission priority key — lower values go first when
///   picking from the ready queue.
#[derive(Debug, Clone)]
pub struct PassthroughEntry {
    bytes: Vec<u8>,
    min_clock: u64,
    req_clock: u64,
    notify_id: NotifyId,
}

impl PassthroughEntry {
    pub fn new(bytes: Vec<u8>, min_clock: u64, req_clock: u64, notify_id: NotifyId) -> Self {
        Self {
            bytes,
            min_clock,
            req_clock,
            notify_id,
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn min_clock(&self) -> u64 {
        self.min_clock
    }

    pub fn req_clock(&self) -> u64 {
        self.req_clock
    }

    pub fn notify_id(&self) -> NotifyId {
        self.notify_id
    }

    /// Returns `true` when this entry carries the background-priority
    /// sentinel, meaning it should only be emitted when no non-background
    /// entries exist.
    pub fn is_background_priority(&self) -> bool {
        self.req_clock == BACKGROUND_PRIORITY_CLOCK
    }
}

#[cfg(test)]
mod tests;
