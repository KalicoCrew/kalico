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

    pub fn entry_count(&self) -> usize {
        self.entries.values().map(|v| v.len()).sum()
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    #[test]
    fn dispatch_fires_matching_callback() {
        let mut table = InterceptorTable::new();
        let count = Arc::new(AtomicU32::new(0));
        let count_clone = Arc::clone(&count);

        table.register(
            "trsync_state".into(),
            Some(0),
            InterceptorCallback(Box::new(move |_| {
                count_clone.fetch_add(1, Ordering::Relaxed);
            })),
        );

        let mut params = MessageParams::new();
        params.insert("oid", crate::transport::MessageValue::U32(0));
        params.insert("can_trigger", crate::transport::MessageValue::U32(0));

        table.dispatch("trsync_state", Some(0), &params);
        assert_eq!(count.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn dispatch_ignores_wrong_oid() {
        let mut table = InterceptorTable::new();
        let count = Arc::new(AtomicU32::new(0));
        let count_clone = Arc::clone(&count);

        table.register(
            "trsync_state".into(),
            Some(0),
            InterceptorCallback(Box::new(move |_| {
                count_clone.fetch_add(1, Ordering::Relaxed);
            })),
        );

        let params = MessageParams::new();
        table.dispatch("trsync_state", Some(1), &params);
        assert_eq!(count.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn dispatch_ignores_wrong_name() {
        let mut table = InterceptorTable::new();
        let count = Arc::new(AtomicU32::new(0));
        let count_clone = Arc::clone(&count);

        table.register(
            "trsync_state".into(),
            Some(0),
            InterceptorCallback(Box::new(move |_| {
                count_clone.fetch_add(1, Ordering::Relaxed);
            })),
        );

        let params = MessageParams::new();
        table.dispatch("analog_in_state", Some(0), &params);
        assert_eq!(count.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn unregister_removes_callback() {
        let mut table = InterceptorTable::new();
        let count = Arc::new(AtomicU32::new(0));
        let count_clone = Arc::clone(&count);

        let id = table.register(
            "trsync_state".into(),
            Some(0),
            InterceptorCallback(Box::new(move |_| {
                count_clone.fetch_add(1, Ordering::Relaxed);
            })),
        );

        let params = MessageParams::new();
        table.dispatch("trsync_state", Some(0), &params);
        assert_eq!(count.load(Ordering::Relaxed), 1);

        table.unregister(id);
        table.dispatch("trsync_state", Some(0), &params);
        assert_eq!(count.load(Ordering::Relaxed), 1, "should not fire after unregister");
    }

    #[test]
    fn callback_receives_params() {
        let mut table = InterceptorTable::new();
        let seen_value = Arc::new(AtomicU32::new(999));
        let seen_clone = Arc::clone(&seen_value);

        table.register(
            "trsync_state".into(),
            Some(0),
            InterceptorCallback(Box::new(move |params| {
                seen_clone.store(params.get_u32("can_trigger"), Ordering::Relaxed);
            })),
        );

        let mut params = MessageParams::new();
        params.insert("oid", crate::transport::MessageValue::U32(0));
        params.insert("can_trigger", crate::transport::MessageValue::U32(0));
        params.insert("trigger_reason", crate::transport::MessageValue::U32(1));

        table.dispatch("trsync_state", Some(0), &params);
        assert_eq!(seen_value.load(Ordering::Relaxed), 0);
    }
}
