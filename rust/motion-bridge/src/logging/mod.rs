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
    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL, RotatingJsonlWriter,
};

pub use crate::logging::context::set_context;

static GUARD: OnceLock<WorkerGuard> = OnceLock::new();
static INITIALIZED: OnceLock<()> = OnceLock::new();

#[cfg(test)]
pub(crate) static CONTEXT_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[derive(Debug, thiserror::Error)]
pub enum LogInitError {
    #[error("opening host-rust.jsonl failed: {0}")]
    Io(#[from] std::io::Error),
}

pub fn init_logging(events_dir: &Path) -> Result<(), LogInitError> {
    if INITIALIZED.set(()).is_err() {
        return Ok(());
    }
    let path = events_dir.join("host-rust.jsonl");
    let rotating = RotatingJsonlWriter::new(
        &path,
        DEFAULT_MAX_BYTES,
        DEFAULT_BACKUP_COUNT,
        FSYNC_INTERVAL,
    )?;
    let (non_blocking, guard) = tracing_appender::non_blocking::NonBlockingBuilder::default()
        .lossy(false)
        .finish(rotating);

    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let subscriber = tracing_subscriber::registry()
        .with(filter)
        .with(JsonlLayer::new(non_blocking));

    tracing_log::LogTracer::init().map_err(|e| std::io::Error::other(e.to_string()))?;
    tracing::subscriber::set_global_default(subscriber)
        .map_err(|e| std::io::Error::other(e.to_string()))?;

    let _ = GUARD.set(guard);
    Ok(())
}

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
    fn double_init_is_idempotent() {
        let dir = std::env::temp_dir().join(format!("kalico-init-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let first = init_logging(&dir);
        let second = init_logging(&dir);
        assert!(matches!(second, Ok(())));
        let _ = first;
    }
}
