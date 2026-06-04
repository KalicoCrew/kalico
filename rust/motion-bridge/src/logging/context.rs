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

    /// All tests that call `set_context` acquire `CONTEXT_TEST_LOCK` from the
    /// parent module for their entire duration. This serialises against layer
    /// tests (which also write the process-global ArcSwap) running in parallel
    /// in the same test binary.
    ///
    /// Note: `arc_swap_concurrent_coherence` intentionally spawns a writer
    /// thread that calls `set_context` while this test holds the lock — that is
    /// correct; the lock serialises against OTHER tests, not the test's own thread.
    use crate::logging::CONTEXT_TEST_LOCK;

    #[test]
    fn defaults_to_unbound_sentinel() {
        // NOTE: relies on no prior set_context in this test binary's process for
        // the very first read; we assert the sentinel shape, not exclusivity.
        let c = SessionContext::default();
        assert_eq!(c.session_id, "__unbound__");
        assert_eq!(c.print_id, "");
    }

    /// Merged test: exercises set→load roundtrip and print_id clearing in a
    /// single test function so parallel test threads cannot interleave on the
    /// process-global `ArcSwap`.
    #[test]
    fn set_load_and_clear_sequence() {
        let _guard = CONTEXT_TEST_LOCK.lock().unwrap();

        // set→load roundtrip
        set_context("k-1748700131-4412".to_string(), "print-1748700500".to_string());
        let c = load_context();
        assert_eq!(c.session_id, "k-1748700131-4412");
        assert_eq!(c.print_id, "print-1748700500");

        // print_id can be cleared (second store on same session)
        set_context("k-1".to_string(), "print-x".to_string());
        set_context("k-1".to_string(), String::new());
        assert_eq!(load_context().print_id, "");
    }

    /// Spec §14: verifies that concurrent `set_context` swaps and `load_context`
    /// reads never produce a *torn* `SessionContext` — a pair where `session_id`
    /// and `print_id` belong to different generations.
    ///
    /// Two valid generations are defined with matching suffixes so a mismatch is
    /// unambiguous:
    ///   gen A: session_id = "k-AAA",   print_id = "print-AAA"
    ///   gen B: session_id = "k-BBB",   print_id = "print-BBB"
    ///
    /// A writer thread alternates between the two for many iterations.  The main
    /// thread concurrently loads and asserts each result is one of the two valid
    /// pairs.  `ArcSwap::store` publishes one complete `Arc<SessionContext>` and
    /// `load_full` returns one complete Arc, so a torn read is impossible by
    /// design — this test documents and exercises that guarantee.
    #[test]
    fn arc_swap_concurrent_coherence() {
        let _guard = CONTEXT_TEST_LOCK.lock().unwrap();

        const WRITER_ITERS: usize = 50_000;
        const READER_ITERS: usize = 100_000;

        let writer = std::thread::spawn(|| {
            for i in 0..WRITER_ITERS {
                if i % 2 == 0 {
                    set_context("k-AAA".to_string(), "print-AAA".to_string());
                } else {
                    set_context("k-BBB".to_string(), "print-BBB".to_string());
                }
            }
        });

        for _ in 0..READER_ITERS {
            let ctx = load_context();
            // A torn read would be a mixed pair such as ("k-AAA", "print-BBB").
            // The suffix must always match; any other combination is a coherence
            // violation.
            let coherent = (ctx.session_id == "k-AAA" && ctx.print_id == "print-AAA")
                || (ctx.session_id == "k-BBB" && ctx.print_id == "print-BBB")
                // Allow the initial/sentinel state present before the writer's
                // first store lands or after a prior test left the global in an
                // arbitrary state.
                || ctx.session_id != "k-AAA" && ctx.session_id != "k-BBB";
            assert!(
                coherent,
                "torn read detected: session_id={:?} print_id={:?}",
                ctx.session_id, ctx.print_id
            );
        }

        writer.join().expect("writer thread must not panic");
    }
}
