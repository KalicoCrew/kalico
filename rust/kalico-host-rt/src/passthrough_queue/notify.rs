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
            next_id: 1,
        }
    }
}

impl NotifyTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, cb: NotifyCallback) -> NotifyId {
        let id = NotifyId::new(self.next_id);
        self.next_id += 1;
        self.callbacks.insert(id, cb);
        id
    }

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
