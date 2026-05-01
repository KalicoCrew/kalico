//! Query/response correlation: register a callback with a unique
//! `NotifyId`, and dispatch exactly once when the MCU responds.

use std::collections::HashMap;

use super::entry::NotifyId;

#[derive(Debug, Clone, Default)]
pub struct NotifyResponse {
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
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[test]
    fn dispatch_fires_callback_once() {
        let mut table = NotifyTable::new();
        let fired = Arc::new(Mutex::new(0u32));
        let fired2 = Arc::clone(&fired);

        let id = table.register(Box::new(move |_resp| {
            *fired2.lock().unwrap() += 1;
        }));

        table.dispatch(id, NotifyResponse::default());
        assert_eq!(*fired.lock().unwrap(), 1);

        // Second dispatch is a no-op
        table.dispatch(id, NotifyResponse::default());
        assert_eq!(*fired.lock().unwrap(), 1);
    }

    #[test]
    fn unique_ids() {
        let mut table = NotifyTable::new();
        let id1 = table.register(Box::new(|_| {}));
        let id2 = table.register(Box::new(|_| {}));
        assert_ne!(id1, id2);
        assert!(!id1.is_none());
        assert!(!id2.is_none());
    }

    #[test]
    fn dispatch_propagates_response_payload() {
        let mut table = NotifyTable::new();
        let captured = Arc::new(Mutex::new(Vec::new()));
        let captured2 = Arc::clone(&captured);

        let id = table.register(Box::new(move |resp| {
            *captured2.lock().unwrap() = resp.bytes;
        }));

        table.dispatch(
            id,
            NotifyResponse {
                bytes: vec![0xDE, 0xAD],
                sent_time: 1.0,
                receive_time: 2.0,
            },
        );

        assert_eq!(*captured.lock().unwrap(), vec![0xDE, 0xAD]);
    }
}
