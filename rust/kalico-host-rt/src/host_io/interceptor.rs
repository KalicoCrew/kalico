use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::transport::MessageParams;

/// Newtype wrapper so `ReactorCommand` can remain `#[derive(Debug)]`. The
/// inner `Box<dyn Fn(&MessageParams) + Send + Sync>` does not implement
/// `Debug`, so we provide a trivial opaque representation.
pub struct InterceptorCallback(pub Box<dyn Fn(&MessageParams) + Send + Sync>);

impl std::fmt::Debug for InterceptorCallback {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("InterceptorCallback(<fn>)")
    }
}

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InterceptorId(u64);

impl InterceptorId {
    fn next() -> Self {
        Self(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

pub(crate) struct InterceptorEntry {
    pub id: InterceptorId,
    pub callback: Box<dyn Fn(&MessageParams) + Send + Sync>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct InterceptorKey {
    msg_name: String,
    oid: Option<u32>,
}

pub(crate) struct InterceptorTable {
    entries: HashMap<InterceptorKey, Vec<InterceptorEntry>>,
}

impl InterceptorTable {
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    pub fn register(
        &mut self,
        msg_name: String,
        oid: Option<u32>,
        callback: InterceptorCallback,
    ) -> InterceptorId {
        let id = InterceptorId::next();
        let key = InterceptorKey { msg_name, oid };
        self.entries
            .entry(key)
            .or_default()
            .push(InterceptorEntry { id, callback: callback.0 });
        id
    }

    pub fn unregister(&mut self, id: InterceptorId) {
        self.entries.retain(|_, entries| {
            entries.retain(|e| e.id != id);
            !entries.is_empty()
        });
    }

    pub fn dispatch(&self, msg_name: &str, oid: Option<u32>, params: &MessageParams) {
        let key = InterceptorKey {
            msg_name: msg_name.to_owned(),
            oid,
        };
        if let Some(entries) = self.entries.get(&key) {
            for entry in entries {
                (entry.callback)(params);
            }
        }
    }
}

impl std::fmt::Debug for InterceptorEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("InterceptorEntry")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}
