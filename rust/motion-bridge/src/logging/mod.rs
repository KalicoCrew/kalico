//! Structured logging for the Rust host (Stage 2). Emits the Stage 1 NDJSON
//! schema into `<events_dir>/host-rust.jsonl`.

pub mod context;
pub mod layer;
pub mod schema;
pub mod writer;

use std::path::Path;
use std::sync::OnceLock;

use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::filter::EnvFilter;
use tracing_subscriber::prelude::*;

use crate::logging::layer::JsonlLayer;
use crate::logging::writer::{
    RotatingJsonlWriter, DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL,
};

// `set_context` is the only re-export with an in-crate consumer (the PyO3
// setter in bridge.rs). `load_context` (used by layer.rs via the direct
// `context::` path) and `UNBOUND_SESSION` stay reachable as
// `logging::context::{load_context, UNBOUND_SESSION}` without a convenience
// re-export, which would otherwise warn as unused in the cdylib build.
pub use crate::logging::context::set_context;

// Keep the appender worker alive for the process lifetime; dropping it flushes
// and stops the worker thread. Stored once at init.
static GUARD: OnceLock<WorkerGuard> = OnceLock::new();
static INITIALIZED: OnceLock<()> = OnceLock::new();

/// Process-global mutex serialising every test that writes the `ArcSwap`
/// session context. All tests in `context::tests` and `layer::tests` that call
/// `set_context` must hold this lock for their entire duration so that parallel
/// `cargo test` threads cannot interleave writes and produce spurious
/// assertion failures on the shared global.
#[cfg(test)]
pub(crate) static CONTEXT_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Errors from logging initialization (fail-loudly; surfaced to Python).
#[derive(Debug, thiserror::Error)]
pub enum LogInitError {
    #[error("logging already initialized")]
    AlreadyInitialized,
    #[error("opening host-rust.jsonl failed: {0}")]
    Io(#[from] std::io::Error),
}

/// Install the global tracing subscriber writing schema-conformant NDJSON to
/// `<events_dir>/host-rust.jsonl`. Idempotent-by-failure: a second call is a
/// hard error (fail-loudly), so a duplicate bridge construction is caught.
///
/// Default level is `info` (drops trace/debug at emit per spec §9); `RUST_LOG`
/// overrides for debugging. Known-noisy debug logs (clocksync) are already
/// below `info` and dropped by the default.
pub fn init_logging(events_dir: &Path) -> Result<(), LogInitError> {
    // Use `set` itself as the atomic guard: exactly one concurrent caller wins;
    // all others bail before touching any global state. This eliminates the
    // TOCTOU window of the old get-then-later-set pattern.
    //
    // NOTE: if initialization fails after this point (e.g. file open error),
    // `INITIALIZED` remains set and a retry returns `AlreadyInitialized`. That
    // is intentional — a failed init is a hard startup error (fail-loudly).
    if INITIALIZED.set(()).is_err() {
        return Err(LogInitError::AlreadyInitialized);
    }
    let path = events_dir.join("host-rust.jsonl");
    let rotating = RotatingJsonlWriter::new(
        &path,
        DEFAULT_MAX_BYTES,
        DEFAULT_BACKUP_COUNT,
        FSYNC_INTERVAL,
    )?;
    // lossy(false): block the producer under backpressure rather than silently
    // drop (matches Stage 1's bounded-queue fail-loud posture, §7.1).
    let (non_blocking, guard) = tracing_appender::non_blocking::NonBlockingBuilder::default()
        .lossy(false)
        .finish(rotating);

    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let subscriber = tracing_subscriber::registry()
        .with(filter)
        .with(JsonlLayer::new(non_blocking));

    // Capture all existing `log::*` calls into tracing (zero call-site edits).
    tracing_log::LogTracer::init().map_err(|e| {
        std::io::Error::other(e.to_string())
    })?;
    tracing::subscriber::set_global_default(subscriber).map_err(|e| {
        std::io::Error::other(e.to_string())
    })?;

    let _ = GUARD.set(guard);
    Ok(())
}

/// The Rust twin of `structured_log.event`: a structured event requiring
/// `subsystem` and `event`. Use for new / hot-path code.
///
/// ```ignore
/// klog!(info, "homing", "homing.endstop_trip", "endstop trip on Z";
///       axis = "z", trigger_mm = 12.40);
/// ```
#[macro_export]
macro_rules! klog {
    ($level:ident, $subsystem:expr, $event:expr, $msg:expr $(; $($k:ident = $v:expr),* $(,)?)?) => {
        tracing::$level!(
            subsystem = $subsystem,
            event = $event,
            $($($k = $v,)*)?
            $msg
        );
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn double_init_is_a_hard_error() {
        // First init may already have happened in another test in this binary;
        // assert that *some* init succeeds and a subsequent one errors.
        let dir = std::env::temp_dir().join(format!(
            "kalico-init-test-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let first = init_logging(&dir);
        // Either this call initialized it, or a prior test did. In both cases a
        // *second* explicit call here must be AlreadyInitialized.
        let second = init_logging(&dir);
        assert!(matches!(second, Err(LogInitError::AlreadyInitialized)));
        let _ = first;
    }
}
