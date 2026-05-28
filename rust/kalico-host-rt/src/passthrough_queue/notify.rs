//! Query/response correlation: register a callback with a unique
//! `NotifyId`, and dispatch exactly once when the MCU responds.

use std::collections::HashMap;

use super::entry::NotifyId;

#[derive(Debug, Clone, Default)]
pub struct NotifyResponse {
    /// Message **body** bytes: `[msgid VLQ | fields...]`. No frame header
    /// (length / seq) and no trailer (CRC / sync byte). Suitable for direct
    /// consumption by `MsgProtoParser::decode_body`. See
    /// [`crate::passthrough_queue::router::PassthroughRouter::dispatch_response`]
    /// for the producer-side contract.
    pub bytes: Vec<u8>,
    pub sent_time: f64,
    pub receive_time: f64,
}

pub type NotifyCallback = Box<dyn FnOnce(NotifyResponse) + Send>;

pub struct NotifyTable {
    callbacks: HashMap<NotifyId, NotifyCallback>,
    next_id: u64,
}

impl std::fmt::Debug for NotifyTable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NotifyTable")
            .field("pending", &self.callbacks.len())
            .field("next_id", &self.next_id)
            .finish()
    }
}

impl Default for NotifyTable {
    fn default() -> Self {
        Self {
            callbacks: HashMap::new(),
            // Start at 1 so id=0 is always "none".
            next_id: 1,
        }
    }
}

impl NotifyTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a callback and return its unique `NotifyId`.
    pub fn register(&mut self, cb: NotifyCallback) -> NotifyId {
        let id = NotifyId::new(self.next_id);
        self.next_id += 1;
        self.callbacks.insert(id, cb);
        id
    }

    /// Fire the callback for `id` with `response`, consuming it.
    /// A second dispatch for the same `id` is a no-op.
    pub fn dispatch(&mut self, id: NotifyId, response: NotifyResponse) {
        if let Some(cb) = self.callbacks.remove(&id) {
            cb(response);
        }
    }

    pub fn pending_count(&self) -> usize {
        self.callbacks.len()
    }
}

#[cfg(test)]
mod tests;
