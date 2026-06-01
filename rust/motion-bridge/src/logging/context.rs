//! Process-global session/print correlation context for Rust host logs.
//!
//! Written rarely (session bind at startup, print start/end) via a PyO3 setter;
//! read on every log event by the tracing layer. `ArcSwap` gives readers a
//! wait-free pointer load with no writer lock — the same idiom already used for
//! `ArcSwap<StatusEvent>` in `kalico-host-rt`.

use std::sync::Arc;

use arc_swap::ArcSwap;

/// Sentinel emitted when a record is produced before `session_id` is bound.
/// Mirrors the Python `structured_log.UNBOUND_SESSION` so both sources agree.
pub const UNBOUND_SESSION: &str = "__unbound__";

#[derive(Debug, Clone)]
pub struct SessionContext {
    pub session_id: String,
    pub print_id: String,
}

impl Default for SessionContext {
    fn default() -> Self {
        SessionContext {
            session_id: UNBOUND_SESSION.to_string(),
            print_id: String::new(),
        }
    }
}

fn global() -> &'static ArcSwap<SessionContext> {
    use std::sync::OnceLock;
    static CTX: OnceLock<ArcSwap<SessionContext>> = OnceLock::new();
    CTX.get_or_init(|| ArcSwap::from_pointee(SessionContext::default()))
}

/// Atomically replace the current context. Carrying the *old* `print_id` for a
/// record already in flight during a swap is acceptable and expected.
pub fn set_context(session_id: String, print_id: String) {
    global().store(Arc::new(SessionContext {
        session_id,
        print_id,
    }));
}

/// Load the current context (wait-free). Cloned `Arc`, one atomic op.
pub fn load_context() -> Arc<SessionContext> {
    global().load_full()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_to_unbound_sentinel() {
        // NOTE: relies on no prior set_context in this test binary's process for
        // the very first read; we assert the sentinel shape, not exclusivity.
        let c = SessionContext::default();
        assert_eq!(c.session_id, "__unbound__");
        assert_eq!(c.print_id, "");
    }

    #[test]
    fn set_then_load_roundtrips() {
        set_context("k-1748700131-4412".to_string(), "print-1748700500".to_string());
        let c = load_context();
        assert_eq!(c.session_id, "k-1748700131-4412");
        assert_eq!(c.print_id, "print-1748700500");
    }

    #[test]
    fn print_id_can_be_cleared() {
        set_context("k-1".to_string(), "print-x".to_string());
        set_context("k-1".to_string(), String::new());
        assert_eq!(load_context().print_id, "");
    }
}
