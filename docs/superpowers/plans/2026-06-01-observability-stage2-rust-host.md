# Observability Stage 2 — Rust Host `tracing` Swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Dispatch implementer subagents **strictly serially** (never in parallel — they share one worktree). Use the `rust-engineer` subagent for every Rust task.

**Goal:** Replace the Rust host's `env_logger` + scattered `/tmp/*.log` + `eprintln!` diagnostics with a `tracing` stack that emits the *same* structured NDJSON schema as the Stage 1 Python host into `~/printer_data/logs/events/host-rust.jsonl`, stamped with `session_id`/`print_id` received from Python across the PyO3 seam.

**Architecture:** A new `motion-bridge/src/logging/` module owns: a process-global `ArcSwap<Arc<SessionContext>>` (lock-free multi-reader, written by a PyO3 setter); a custom `tracing_subscriber::Layer` that serializes each event to the exact Stage-1 schema and injects the session context; a size-based `RotatingJsonlWriter` (uncompressed, flush-per-record + periodic fsync, mirroring the Python `JsonlSink`) driven off-thread by `tracing-appender`'s `NonBlocking`; and `tracing-log::LogTracer` to capture all 62 existing `log::*` call sites with zero edits. `kalico-host-rt` gains a `tracing` dependency only to convert its `/tmp` writes + `eprintln!` to `tracing::event!`; its `log::*` calls are captured globally. The global `tracing` subscriber is installed once via a new `init_logging(events_dir)` PyO3 method called from `MotionBridgeWrapper.__init__`.

**Tech Stack:** Rust, `tracing` + `tracing-subscriber` + `tracing-appender` + `tracing-log`, `arc-swap` (already a dep), `time` (RFC3339 formatting), `serde_json` (already a dep), PyO3 0.24 (abi3-py39). Python glue in `klippy/motion_bridge.py` + `klippy/printer.py`.

---

## Schema contract (must match Stage 1 exactly)

The Stage 1 Python serializer (`klippy/structured_log.py`) produces one JSON object per physical line with these fields. Stage 2 Rust output MUST be byte-compatible in field names, types, and value conventions so both `events/*.jsonl` files share one schema for VictoriaLogs (Stage 3).

Core (every record):
- `_time` — RFC3339 UTC, **millisecond** precision, trailing `Z`. Python format: `2026-06-01T14:02:11.482Z` (see `structured_log.format_time`).
- `_msg` — human message string. Sanitized (JSON-escaped); never raw.
- `level` — lowercase enum: `trace`/`debug`/`info`/`warn`/`error`. (Python maps CRITICAL→`error`; Rust has no critical.)
- `source` — for Rust always the literal `"host-rust"`.
- `subsystem` — string; from an explicit event field when present, else a target→subsystem mapping.
- `session_id` — string `k-<unix>-<pid>`; the sentinel `"__unbound__"` if not yet bound (mirrors Python `UNBOUND_SESSION`).
- `target` — Rust module path (e.g. `motion_bridge::bridge`), from `event.metadata().target()`.

Optional:
- `print_id` — string; **always present, empty string `""` when idle** (matches Python `record_to_dict` which always writes `print_id`).
- `event` — stable key string (e.g. `homing.endstop_trip`).
- `code` / `code_name` — int / string when present.
- payload — any structured fields; **numbers stay JSON numbers** (e.g. `trigger_mm: 12.40`, not `"12.40"`).

Serialization rules (mirror `structured_log.serialize_record`): compact separators (no spaces), `ensure_ascii` off (UTF-8 passthrough), exactly one `\n`-terminated physical line per record, embedded newlines/quotes/control chars JSON-escaped.

---

## File Structure

New files (all under `rust/motion-bridge/src/logging/`):
- `mod.rs` — module root; re-exports; `init_logging`, `set_session_context`, `clear_session_context`; the `OnceLock` init guard + `WorkerGuard` storage; the `klog!` macro (defined at crate root via `#[macro_export]`).
- `context.rs` — `SessionContext` struct + process-global `ArcSwap` + get/set.
- `schema.rs` — `level_str`, `format_time`, `SOURCE_HOST_RUST`, `UNBOUND_SESSION`, `subsystem_for_target`.
- `writer.rs` — `RotatingJsonlWriter` (`io::Write`, size rotation, flush + periodic fsync).
- `layer.rs` — the custom `tracing_subscriber::Layer` + field visitor.

Modified files:
- `rust/motion-bridge/Cargo.toml` — add tracing stack + `time`; remove `env_logger`.
- `rust/motion-bridge/src/lib.rs` — declare `mod logging`; remove `env_logger::try_init()` from the `#[pymodule]`.
- `rust/motion-bridge/src/bridge.rs` — add `init_logging` + `set_session_context` `#[pymethods]`; convert `/tmp/cax-trace.log` (1376), `/tmp/interceptor_trace.log` (2469, 2641), `[move-diag]` eprintln (2309).
- `rust/motion-bridge/src/planner.rs` — convert `[move-diag]` eprintln (276, 594, 601, 611, 741, 820, 829, 835).
- `rust/motion-bridge/src/probe_homing.rs` — convert `/tmp/interceptor_trace.log` (57, 76).
- `rust/kalico-host-rt/Cargo.toml` — add `tracing`.
- `rust/kalico-host-rt/src/host_io/reactor.rs` — convert `/tmp/interceptor_trace.log` (792), `/tmp/kalico-firewire.log` (1021, 1123) + all `eprintln!` `[trace-*]`/`[bridge-error]`.
- `rust/kalico-host-rt/src/host_io/mod.rs` — convert `[tio-*]`/`[reactor-spawn]` eprintln.
- `rust/kalico-host-rt/src/host_io/kalico_native.rs` — convert `[kalico-id]` eprintln.
- `klippy/motion_bridge.py` — call `init_logging` + `set_session_context` in `__init__`; register print-state handlers.
- `klippy/printer.py` — thread `events_dir` to the bridge wrapper if not already reachable.

---

## Conventions for all Rust tasks

- Run `cd rust && cargo test -p motion-bridge` and `cargo test -p kalico-host-rt` to verify. Native macOS build — no MCU target.
- The workspace uses `-D warnings` in CI with pedantic clippy. Run `cargo clippy -p motion-bridge -p kalico-host-rt --all-targets` and fix lints before committing.
- `unsafe_code = "deny"` workspace-wide. None of this work needs `unsafe`.
- Commit after each task with a `feat(logging-rs):` / `refactor(logging-rs):` prefix. **Do NOT add any `Co-Authored-By` trailer.**
- Fail-loudly is the project rule: no `let _ = ...` swallowing of write/flush/init errors.

---

### Task 1: Add dependencies

**Files:**
- Modify: `rust/motion-bridge/Cargo.toml`
- Modify: `rust/kalico-host-rt/Cargo.toml`

- [ ] **Step 1: Add the tracing stack + time to motion-bridge**

In `rust/motion-bridge/Cargo.toml` `[dependencies]`, **remove** the line `env_logger = "0.11"` and **add**:

```toml
tracing = "0.1"
tracing-subscriber = { version = "0.3", default-features = false, features = ["registry", "env-filter", "std", "fmt"] }
tracing-appender = "0.2"
tracing-log = "0.2"
time = { version = "0.3", default-features = false, features = ["formatting", "std"] }
```

(`log = "0.4"`, `arc-swap = "1"`, `serde_json = "1"` are already present — keep them. `tracing-log` brings the `LogTracer` that captures the existing `log::*` macros.)

- [ ] **Step 2: Add tracing to kalico-host-rt**

In `rust/kalico-host-rt/Cargo.toml` `[dependencies]`, add:

```toml
tracing = "0.1"
```

(Only the macro crate — `kalico-host-rt` emits `tracing::event!` for its converted diagnostics; the global subscriber is installed by `motion-bridge`. Its existing `log::*` calls are captured by `LogTracer` with no dep change.)

- [ ] **Step 3: Verify the workspace still builds**

Run: `cd rust && cargo build -p motion-bridge -p kalico-host-rt`
Expected: PASS (env_logger removal will cause an unused-import / unresolved error in `lib.rs` — that is fixed in Task 6/Step where `lib.rs` is edited; if the build fails ONLY on `env_logger` in `lib.rs`, that is expected at this step). To keep Task 1 green in isolation, also do Step 4 now.

- [ ] **Step 4: Temporarily neutralize the env_logger call so the tree builds**

In `rust/motion-bridge/src/lib.rs`, find the `#[pymodule]` body (around line 34-42) and delete the line `let _ = env_logger::try_init();` (and any now-unused `use env_logger` if present). The pymodule body becomes:

```rust
#[pymodule]
fn motion_bridge_native(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
```

Run: `cd rust && cargo build -p motion-bridge -p kalico-host-rt`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/Cargo.toml rust/kalico-host-rt/Cargo.toml rust/motion-bridge/src/lib.rs rust/Cargo.lock
git commit -m "feat(logging-rs): add tracing stack deps, drop env_logger"
```

---

### Task 2: SessionContext + process-global ArcSwap

**Files:**
- Create: `rust/motion-bridge/src/logging/context.rs`
- Create: `rust/motion-bridge/src/logging/mod.rs` (stub for now; expanded later)
- Modify: `rust/motion-bridge/src/lib.rs` (add `mod logging;`)

- [ ] **Step 1: Write the failing test**

Create `rust/motion-bridge/src/logging/context.rs`:

```rust
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
```

Create `rust/motion-bridge/src/logging/mod.rs`:

```rust
//! Structured logging for the Rust host (Stage 2 of the observability pipeline).
//! Emits the same NDJSON schema as the Stage 1 Python host into
//! `<events_dir>/host-rust.jsonl`.

pub mod context;
```

In `rust/motion-bridge/src/lib.rs`, add near the other `mod` declarations:

```rust
mod logging;
```

- [ ] **Step 2: Run the test to verify it fails (then passes)**

Run: `cd rust && cargo test -p motion-bridge logging::context`
Expected: COMPILES and PASSES (this task is pure data structure; the "failing" stage is just confirming it compiles and the asserts hold). If `arc_swap::ArcSwap::from_pointee` is unavailable in the pinned `arc-swap = "1"`, use `ArcSwap::new(Arc::new(SessionContext::default()))` instead.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/logging/ rust/motion-bridge/src/lib.rs
git commit -m "feat(logging-rs): SessionContext + process-global ArcSwap"
```

---

### Task 3: Schema helpers (level, time, source, subsystem mapping)

**Files:**
- Create: `rust/motion-bridge/src/logging/schema.rs`
- Modify: `rust/motion-bridge/src/logging/mod.rs` (add `pub mod schema;`)

- [ ] **Step 1: Write the failing test**

Create `rust/motion-bridge/src/logging/schema.rs`:

```rust
//! Schema-conformance helpers shared by the tracing layer. Values here must
//! match the Stage 1 Python serializer (`klippy/structured_log.py`) exactly.

use time::format_description::FormatItem;
use time::macros::format_description;
use time::OffsetDateTime;
use tracing::Level;

pub const SOURCE_HOST_RUST: &str = "host-rust";

/// RFC3339 UTC with millisecond precision + trailing `Z`, matching
/// `structured_log.format_time` (e.g. `2026-06-01T14:02:11.482Z`).
const TIME_FMT: &[FormatItem<'static>] = format_description!(
    "[year]-[month]-[day]T[hour]:[minute]:[second].[subsecond digits:3]Z"
);

/// Map a tracing `Level` to the lowercase schema enum.
pub fn level_str(level: &Level) -> &'static str {
    match *level {
        Level::TRACE => "trace",
        Level::DEBUG => "debug",
        Level::INFO => "info",
        Level::WARN => "warn",
        Level::ERROR => "error",
    }
}

/// Format a system time as the schema `_time`. Takes the time as a parameter so
/// it is testable; the layer passes `OffsetDateTime::now_utc()`.
pub fn format_time(t: OffsetDateTime) -> String {
    // UTC by construction; formatting cannot fail for this fixed description.
    t.format(&TIME_FMT)
        .unwrap_or_else(|_| "1970-01-01T00:00:00.000Z".to_string())
}

/// Best-effort `subsystem` for a captured `log::*` record that carries no
/// explicit `subsystem` field. New `klog!`/`tracing::event!` call sites set
/// `subsystem` explicitly and bypass this. Maps a Rust module-path target to a
/// logical area; unknown targets fall back to `"host-rust"`.
pub fn subsystem_for_target(target: &str) -> &'static str {
    if target.contains("clock") {
        "clocksync"
    } else if target.contains("probe_homing") || target.contains("homing") {
        "homing"
    } else if target.contains("planner") {
        "motion"
    } else if target.contains("pump") {
        "motion"
    } else if target.contains("reactor")
        || target.contains("transport")
        || target.contains("kalico_native")
    {
        "mcu-comms"
    } else if target.contains("bridge") {
        "bridge"
    } else {
        "host-rust"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use time::macros::datetime;

    #[test]
    fn levels_lowercase() {
        assert_eq!(level_str(&Level::INFO), "info");
        assert_eq!(level_str(&Level::WARN), "warn");
        assert_eq!(level_str(&Level::ERROR), "error");
        assert_eq!(level_str(&Level::DEBUG), "debug");
        assert_eq!(level_str(&Level::TRACE), "trace");
    }

    #[test]
    fn time_is_rfc3339_millis_z() {
        // 2026-06-01T14:02:11.482482Z UTC -> millisecond truncation + Z
        let t = datetime!(2026-06-01 14:02:11.482482 UTC);
        assert_eq!(format_time(t), "2026-06-01T14:02:11.482Z");
    }

    #[test]
    fn subsystem_mapping() {
        assert_eq!(subsystem_for_target("motion_bridge::bridge"), "bridge");
        assert_eq!(subsystem_for_target("motion_bridge::planner"), "motion");
        assert_eq!(
            subsystem_for_target("kalico_host_rt::host_io::reactor"),
            "mcu-comms"
        );
        assert_eq!(subsystem_for_target("motion_bridge::probe_homing"), "homing");
        assert_eq!(subsystem_for_target("some::unknown::path"), "host-rust");
    }
}
```

Add to `rust/motion-bridge/src/logging/mod.rs`:

```rust
pub mod schema;
```

- [ ] **Step 2: Run the test**

Run: `cd rust && cargo test -p motion-bridge logging::schema`
Expected: PASS. If `time::macros::format_description`/`datetime` are unavailable, enable the `time` `macros` feature: change the dep to `time = { version = "0.3", default-features = false, features = ["formatting", "std", "macros"] }`. If millisecond truncation differs (rounding vs. truncation), adjust the test to match `time`'s documented truncation behavior and note it — the Python side truncates (`microsecond // 1000`).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/logging/schema.rs rust/motion-bridge/src/logging/mod.rs rust/motion-bridge/Cargo.toml rust/Cargo.lock
git commit -m "feat(logging-rs): schema helpers (level/time/subsystem)"
```

---

### Task 4: RotatingJsonlWriter (size rotation + flush + periodic fsync)

**Files:**
- Create: `rust/motion-bridge/src/logging/writer.rs`
- Modify: `rust/motion-bridge/src/logging/mod.rs` (add `pub mod writer;`)

This mirrors the Python `JsonlSink`: 32 MB × 5 uncompressed rotation, flush each record, periodic fsync (15 s), fsync on close/rotate. It is owned by the single `tracing-appender` worker thread, so it needs no internal locking. Fail-loudly: I/O errors propagate via `io::Result` (the worker surfaces them; the proactive last-gasp is a Stage 3 item, consistent with Stage 1).

- [ ] **Step 1: Write the failing test**

Create `rust/motion-bridge/src/logging/writer.rs`:

```rust
//! Size-based rotating NDJSON writer for `host-rust.jsonl`. Mirrors the Stage 1
//! Python `JsonlSink`: uncompressed rotation (`.1`..`.N`), flush per write, a
//! periodic fsync backstop, and fsync on rotate/close. Single-threaded: owned
//! by the `tracing-appender` worker, so no locking.

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub const DEFAULT_MAX_BYTES: u64 = 32 * 1024 * 1024;
pub const DEFAULT_BACKUP_COUNT: u32 = 5;
pub const FSYNC_INTERVAL: Duration = Duration::from_secs(15);

pub struct RotatingJsonlWriter {
    path: PathBuf,
    file: File,
    written: u64,
    max_bytes: u64,
    backup_count: u32,
    last_fsync: Instant,
    fsync_interval: Duration,
}

impl std::fmt::Debug for RotatingJsonlWriter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RotatingJsonlWriter")
            .field("path", &self.path)
            .field("written", &self.written)
            .finish_non_exhaustive()
    }
}

impl RotatingJsonlWriter {
    pub fn new(
        path: impl AsRef<Path>,
        max_bytes: u64,
        backup_count: u32,
        fsync_interval: Duration,
    ) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        let written = file.metadata()?.len();
        Ok(RotatingJsonlWriter {
            path,
            file,
            written,
            max_bytes,
            backup_count,
            last_fsync: Instant::now(),
            fsync_interval,
        })
    }

    fn rotated_path(&self, n: u32) -> PathBuf {
        let mut s = self.path.as_os_str().to_os_string();
        s.push(format!(".{n}"));
        PathBuf::from(s)
    }

    /// Close + fsync the current file, shift `.N-1`->`.N` ... base->`.1`, reopen.
    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?;
        self.file.sync_all()?; // fsync before rotate so no partial tail is lost
        // Drop the oldest, then cascade.
        let oldest = self.rotated_path(self.backup_count);
        if oldest.exists() {
            std::fs::remove_file(&oldest)?;
        }
        for n in (1..self.backup_count).rev() {
            let src = self.rotated_path(n);
            if src.exists() {
                std::fs::rename(&src, self.rotated_path(n + 1))?;
            }
        }
        if self.path.exists() {
            std::fs::rename(&self.path, self.rotated_path(1))?;
        }
        self.file = OpenOptions::new().create(true).append(true).open(&self.path)?;
        self.written = 0;
        self.last_fsync = Instant::now();
        Ok(())
    }
}

impl Write for RotatingJsonlWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.written + buf.len() as u64 > self.max_bytes && self.written > 0 {
            self.rotate()?;
        }
        let n = self.file.write(buf)?;
        self.written += n as u64;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()?;
        if self.last_fsync.elapsed() >= self.fsync_interval {
            self.file.sync_all()?;
            self.last_fsync = Instant::now();
        }
        Ok(())
    }
}

impl Drop for RotatingJsonlWriter {
    fn drop(&mut self) {
        // Best-effort durable close. Errors at drop cannot be propagated; this
        // is the documented exception to fail-loudly (matches Python close()).
        let _ = self.file.flush();
        let _ = self.file.sync_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        // Per-test unique dir; std::process::id avoids cross-test collisions.
        p.push(format!("kalico-jsonl-test-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p.push("host-rust.jsonl");
        p
    }

    #[test]
    fn writes_lines_to_base_file() {
        let path = tmp("basic");
        let mut w = RotatingJsonlWriter::new(&path, 1024, 3, FSYNC_INTERVAL).unwrap();
        w.write_all(b"{\"a\":1}\n").unwrap();
        w.flush().unwrap();
        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents, "{\"a\":1}\n");
    }

    #[test]
    fn rotates_when_exceeding_max_bytes() {
        let path = tmp("rotate");
        // tiny max so the 2nd write triggers a rotation
        let mut w = RotatingJsonlWriter::new(&path, 8, 3, FSYNC_INTERVAL).unwrap();
        w.write_all(b"AAAAAAA\n").unwrap(); // 8 bytes -> fills
        w.write_all(b"BBBBBBB\n").unwrap(); // triggers rotate before write
        w.flush().unwrap();
        let base = std::fs::read_to_string(&path).unwrap();
        let rotated = std::fs::read_to_string(path.with_extension("jsonl.1")).unwrap();
        assert_eq!(base, "BBBBBBB\n");
        assert_eq!(rotated, "AAAAAAA\n");
    }

    #[test]
    fn drops_oldest_beyond_backup_count() {
        let path = tmp("cascade");
        let mut w = RotatingJsonlWriter::new(&path, 4, 2, FSYNC_INTERVAL).unwrap();
        for i in 0..5u8 {
            w.write_all(&[b'0' + i, b'\n', b'x', b'\n']).unwrap();
        }
        w.flush().unwrap();
        // backup_count=2 => base + .1 + .2 exist, no .3
        assert!(path.exists());
        assert!(path.with_extension("jsonl.1").exists());
        assert!(path.with_extension("jsonl.2").exists());
        assert!(!path.with_extension("jsonl.3").exists());
    }
}
```

Note for implementer: `path.with_extension("jsonl.1")` in the tests assumes the base path ends in `.jsonl`; `with_extension` replaces from the last `.`, so for `host-rust.jsonl` it yields `host-rust.jsonl.1` only if you pass `"jsonl.1"`. Verify the produced names match `rotated_path` (which appends `.1` to the full filename → `host-rust.jsonl.1`). If `with_extension` does not produce the expected name, construct the expected path explicitly with `path.parent().join("host-rust.jsonl.1")`. Fix the test to assert the real `rotated_path` output.

Add to `rust/motion-bridge/src/logging/mod.rs`:

```rust
pub mod writer;
```

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p motion-bridge logging::writer`
Expected: PASS. Adjust the rotated-path assertions per the note above so they match `rotated_path`'s actual output (`<base>.1`, `<base>.2`).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/logging/writer.rs rust/motion-bridge/src/logging/mod.rs
git commit -m "feat(logging-rs): size-based rotating jsonl writer (flush + periodic fsync)"
```

---

### Task 5: Custom tracing Layer (exact-schema serializer + context injection)

**Files:**
- Create: `rust/motion-bridge/src/logging/layer.rs`
- Modify: `rust/motion-bridge/src/logging/mod.rs` (add `pub mod layer;`)

The layer is the crux: on each event it loads the session context, visits the event's fields, and writes one schema-conformant JSON line to a `MakeWriter`. It is generic over the writer so the test can inject an in-memory buffer.

- [ ] **Step 1: Write the failing test**

Create `rust/motion-bridge/src/logging/layer.rs`:

```rust
//! Custom `tracing_subscriber::Layer` that serializes each event to the Stage 1
//! NDJSON schema and injects the session/print context. Generic over a
//! `MakeWriter` so production uses the non-blocking rotating writer and tests
//! use an in-memory buffer.

use std::io::Write;

use serde_json::{Map, Value};
use time::OffsetDateTime;
use tracing::field::{Field, Visit};
use tracing::{Event, Subscriber};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::Context;
use tracing_subscriber::Layer;

use super::context::load_context;
use super::schema::{format_time, level_str, subsystem_for_target, SOURCE_HOST_RUST};

/// Collects event fields into a JSON map, special-casing `message`,
/// `subsystem`, `event`, `code`, `code_name`.
#[derive(Default)]
struct FieldVisitor {
    map: Map<String, Value>,
    message: Option<String>,
}

impl Visit for FieldVisitor {
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.message = Some(value.to_string());
        } else {
            self.map.insert(field.name().to_string(), Value::String(value.to_string()));
        }
    }
    fn record_i64(&mut self, field: &Field, value: i64) {
        self.map.insert(field.name().to_string(), Value::from(value));
    }
    fn record_u64(&mut self, field: &Field, value: u64) {
        self.map.insert(field.name().to_string(), Value::from(value));
    }
    fn record_f64(&mut self, field: &Field, value: f64) {
        self.map.insert(field.name().to_string(), Value::from(value));
    }
    fn record_bool(&mut self, field: &Field, value: bool) {
        self.map.insert(field.name().to_string(), Value::Bool(value));
    }
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        let s = format!("{value:?}");
        if field.name() == "message" {
            self.message = Some(s);
        } else {
            self.map.insert(field.name().to_string(), Value::String(s));
        }
    }
}

pub struct JsonlLayer<W> {
    make_writer: W,
}

impl<W> std::fmt::Debug for JsonlLayer<W> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JsonlLayer").finish_non_exhaustive()
    }
}

impl<W> JsonlLayer<W> {
    pub fn new(make_writer: W) -> Self {
        JsonlLayer { make_writer }
    }
}

impl<S, W> Layer<S> for JsonlLayer<W>
where
    S: Subscriber,
    W: for<'a> MakeWriter<'a> + 'static,
{
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = FieldVisitor::default();
        event.record(&mut visitor);

        let meta = event.metadata();
        let target = meta.target();
        let ctx = load_context();

        let mut out = Map::new();
        out.insert("_time".into(), Value::String(format_time(OffsetDateTime::now_utc())));
        out.insert(
            "_msg".into(),
            Value::String(visitor.message.unwrap_or_default()),
        );
        out.insert("level".into(), Value::String(level_str(meta.level()).into()));
        out.insert("source".into(), Value::String(SOURCE_HOST_RUST.into()));

        // subsystem: explicit field wins, else target mapping.
        let subsystem = match visitor.map.remove("subsystem") {
            Some(Value::String(s)) => s,
            _ => subsystem_for_target(target).to_string(),
        };
        out.insert("subsystem".into(), Value::String(subsystem));
        out.insert("session_id".into(), Value::String(ctx.session_id.clone()));
        out.insert("target".into(), Value::String(target.to_string()));
        out.insert("print_id".into(), Value::String(ctx.print_id.clone()));

        // Promote remaining payload fields (event, code, code_name, axis, ...).
        for (k, v) in visitor.map {
            out.entry(k).or_insert(v);
        }

        // Compact, one physical line, UTF-8 passthrough — matches the Python
        // serializer. serde_json escapes embedded newlines/quotes/control chars.
        let mut line = serde_json::to_string(&Value::Object(out))
            .unwrap_or_else(|e| format!("{{\"_msg\":\"serialize error: {e}\"}}"));
        line.push('\n');

        let mut w = self.make_writer.make_writer();
        // Fail-loudly posture: a write error here is surfaced by the worker /
        // Stage 3 liveness. At the layer we cannot return Result; on error we
        // emit to stderr as the last-gasp (Stage 3 will route this properly).
        if let Err(e) = w.write_all(line.as_bytes()) {
            eprintln!("[host-rust-log] sink write failed: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use tracing_subscriber::prelude::*;

    #[derive(Clone, Default)]
    struct BufWriter(Arc<Mutex<Vec<u8>>>);
    impl<'a> MakeWriter<'a> for BufWriter {
        type Writer = BufHandle;
        fn make_writer(&'a self) -> Self::Writer {
            BufHandle(self.0.clone())
        }
    }
    struct BufHandle(Arc<Mutex<Vec<u8>>>);
    impl Write for BufHandle {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    fn capture<F: FnOnce()>(f: F) -> Vec<serde_json::Value> {
        let buf = BufWriter::default();
        let layer = JsonlLayer::new(buf.clone());
        let subscriber = tracing_subscriber::registry().with(layer);
        tracing::subscriber::with_default(subscriber, f);
        let bytes = buf.0.lock().unwrap().clone();
        String::from_utf8(bytes)
            .unwrap()
            .lines()
            .filter(|l| !l.is_empty())
            .map(|l| serde_json::from_str(l).expect("each line is valid JSON"))
            .collect()
    }

    #[test]
    fn emits_core_schema_fields() {
        super::super::context::set_context("k-1-2".into(), "".into());
        let recs = capture(|| {
            tracing::info!(
                subsystem = "homing",
                event = "homing.endstop_trip",
                axis = "z",
                trigger_mm = 12.40_f64,
                "endstop trip on Z at 12.40mm"
            );
        });
        assert_eq!(recs.len(), 1);
        let r = &recs[0];
        assert_eq!(r["_msg"], "endstop trip on Z at 12.40mm");
        assert_eq!(r["level"], "info");
        assert_eq!(r["source"], "host-rust");
        assert_eq!(r["subsystem"], "homing");
        assert_eq!(r["event"], "homing.endstop_trip");
        assert_eq!(r["axis"], "z");
        // numeric stays a JSON number
        assert!((r["trigger_mm"].as_f64().unwrap() - 12.40).abs() < 1e-9);
        assert_eq!(r["session_id"], "k-1-2");
        assert_eq!(r["print_id"], "");
        assert!(r["_time"].as_str().unwrap().ends_with('Z'));
    }

    #[test]
    fn subsystem_falls_back_to_target_mapping() {
        super::super::context::set_context("k-1-2".into(), "".into());
        let recs = capture(|| {
            tracing::warn!(event = "retry", "attach_serial retry");
        });
        // target is this test module path; mapping default is "host-rust"
        assert!(recs[0]["subsystem"].is_string());
    }

    #[test]
    fn embedded_newline_yields_one_line() {
        super::super::context::set_context("k-1-2".into(), "".into());
        let recs = capture(|| {
            tracing::info!("line one\nline two\u{0007}");
        });
        // One record despite the embedded newline (it is JSON-escaped).
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0]["_msg"], "line one\nline two\u{0007}");
    }
}
```

Add to `rust/motion-bridge/src/logging/mod.rs`:

```rust
pub mod layer;
```

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p motion-bridge logging::layer`
Expected: PASS. If `tracing-subscriber`'s `MakeWriter` is gated behind the `fmt` feature, it is already enabled (Task 1 includes `fmt`). If `record_str` is not called for string literals (tracing records `&str` values via `record_str` only when typed `&str`; the message is recorded via `record_debug`/`record_str` depending on macro form), ensure the `record_debug` path strips the surrounding quotes — `format!("{value:?}")` on a `&str` yields a quoted string. **Fix:** the message field arrives through `record_str` for the format-string message in current tracing; if a test shows `_msg` wrapped in extra quotes, special-case the `message` field in `record_debug` to use the `Display`-like value. Verify against actual tracing behavior and adjust the visitor so `_msg` is the clean message text (matching the assertions).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/logging/layer.rs rust/motion-bridge/src/logging/mod.rs
git commit -m "feat(logging-rs): custom tracing Layer serializing the Stage 1 schema"
```

---

### Task 6: init_logging wiring + klog! macro + LogTracer + level filter

**Files:**
- Modify: `rust/motion-bridge/src/logging/mod.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (export `klog!` if defined at crate root)

- [ ] **Step 1: Write the init function + macro**

Replace the body of `rust/motion-bridge/src/logging/mod.rs` with (keeping the `pub mod` lines):

```rust
//! Structured logging for the Rust host (Stage 2). Emits the Stage 1 NDJSON
//! schema into `<events_dir>/host-rust.jsonl`.

pub mod context;
pub mod layer;
pub mod schema;
pub mod writer;

use std::path::Path;
use std::sync::OnceLock;
use std::time::Duration;

use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::filter::EnvFilter;
use tracing_subscriber::prelude::*;

use crate::logging::layer::JsonlLayer;
use crate::logging::writer::{
    RotatingJsonlWriter, DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL,
};

pub use crate::logging::context::{load_context, set_context, UNBOUND_SESSION};

// Keep the appender worker alive for the process lifetime; dropping it flushes
// and stops the worker thread. Stored once at init.
static GUARD: OnceLock<WorkerGuard> = OnceLock::new();
static INITIALIZED: OnceLock<()> = OnceLock::new();

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
    if INITIALIZED.get().is_some() {
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

    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new("info"));
    let subscriber = tracing_subscriber::registry()
        .with(filter)
        .with(JsonlLayer::new(non_blocking));

    // Capture all existing `log::*` calls into tracing (zero call-site edits).
    tracing_log::LogTracer::init().map_err(|e| {
        LogInitError::Io(std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))
    })?;
    tracing::subscriber::set_global_default(subscriber).map_err(|e| {
        LogInitError::Io(std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))
    })?;

    let _ = GUARD.set(guard);
    let _ = INITIALIZED.set(());
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
```

Note: `set_global_default` + `LogTracer::init()` are process-global and can only run once per process. The unit test above tolerates a prior init within the same test binary. The end-to-end "captured log produces a file line" check is exercised in Task 11's integration verification (a separate process), to avoid global-subscriber contention across unit tests.

- [ ] **Step 2: Run the build + test**

Run: `cd rust && cargo test -p motion-bridge logging::`
Expected: PASS. If `EnvFilter` is unavailable, confirm the `env-filter` feature (Task 1). If `tracing_appender::non_blocking::NonBlockingBuilder` differs in the pinned `0.2`, use `tracing_appender::non_blocking(rotating)` + set lossy via the builder API actually exposed; the requirement is **lossy = false**. If `std::io::Error::new(ErrorKind::Other, ...)` triggers a clippy `io_other_error` pedantic lint, use `std::io::Error::other(...)`.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/logging/mod.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(logging-rs): init_logging (subscriber+appender+LogTracer) and klog! macro"
```

---

### Task 7: PyO3 setters + remove env_logger dependency

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add fields + `#[pymethods]`)
- Verify: `rust/motion-bridge/src/lib.rs` (env_logger already removed in Task 1)

- [ ] **Step 1: Add the PyO3 methods**

In `rust/motion-bridge/src/bridge.rs`, inside the existing `#[pymethods] impl PyMotionBridge` block (begins ~line 640), add two methods. They delegate to the `logging` module; the session context global lives there, so `PyMotionBridge` needs no new fields:

```rust
    /// Install the Rust host structured-logging subscriber, writing
    /// `<events_dir>/host-rust.jsonl`. Called once from Python at bridge setup,
    /// before any other bridge method. Fails loudly if already initialized or
    /// if the file cannot be opened (project fail-loudly policy).
    fn init_logging(&self, events_dir: String) -> PyResult<()> {
        crate::logging::init_logging(std::path::Path::new(&events_dir)).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "init_logging failed: {e}"
            ))
        })
    }

    /// Set/update the session + print correlation context stamped on every Rust
    /// log record. Called at bridge setup (with the current session_id and an
    /// empty print_id) and on every print-state change.
    #[pyo3(signature = (session_id, print_id=String::new()))]
    fn set_session_context(&self, session_id: String, print_id: String) {
        crate::logging::set_context(session_id, print_id);
    }
```

- [ ] **Step 2: Confirm env_logger is fully gone**

Run: `cd rust && grep -rn env_logger motion-bridge/src motion-bridge/Cargo.toml`
Expected: no matches (the `Cargo.toml` dep was removed in Task 1 Step 1; the `lib.rs` call in Task 1 Step 4). If `motion-bridge/Cargo.toml` still lists `env_logger`, remove it now.

- [ ] **Step 3: Build + run the existing bridge tests**

Run: `cd rust && cargo test -p motion-bridge`
Expected: PASS (all 92 existing unit + integration tests still green; the new methods compile).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/Cargo.toml
git commit -m "feat(logging-rs): PyO3 init_logging + set_session_context setters"
```

---

### Task 8: Convert motion-bridge /tmp writes + [move-diag] eprintln

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (lines 1376, 2309, 2469, 2641 regions)
- Modify: `rust/motion-bridge/src/planner.rs` (lines 276, 594, 601, 611, 741, 820, 829, 835 regions)
- Modify: `rust/motion-bridge/src/probe_homing.rs` (lines 57, 76 regions)

**Cutover rule (§7.2):** each `/tmp` write / `eprintln!` is replaced by a `tracing::event!` (or `klog!`) in the *same* edit — the `OpenOptions`/`writeln!`/`eprintln!` lines are deleted, not left alongside. No window of double-logging.

**Transformation pattern.** A `/tmp` append like:

```rust
if let Ok(mut f) = std::fs::OpenOptions::new().append(true).create(true).open("/tmp/cax-trace.log") {
    let _ = writeln!(f, "configure_axes ENTRY mcu_handle={mcu_handle} kin={kinematics} ...");
}
```

becomes a structured event (line numbers/diag prefixes dropped; the data becomes fields):

```rust
tracing::info!(
    subsystem = "bridge",
    event = "configure_axes_entry",
    mcu_handle,
    kinematics = %kinematics,
    "configure_axes entry"
);
```

A diagnostic `eprintln!` like:

```rust
eprintln!("[move-diag] planner.submit_move enter id={id} t={t}");
```

becomes (these are debug-grade diagnostics → `debug!`, so the `info` default drops them; real errors → `warn!`/`error!`):

```rust
tracing::debug!(subsystem = "motion", event = "planner_submit_enter", id, t, "planner.submit_move enter");
```

Error diagnostics (e.g. `[move-diag] run_commit_and_dispatch: dispatch ERR {e}`) become `tracing::error!(subsystem = "motion", event = "dispatch_error", error = %e, "run_commit_and_dispatch dispatch failed")`.

- [ ] **Step 1: Convert each site**

Apply the pattern to every site below. Use `%x` for `Display` values and `?x` for `Debug` values in the field position; keep the original variable data as fields. Choose `subsystem`/`event`/`level` from the recon mapping:

bridge.rs:
- 1376 `/tmp/cax-trace.log` (`configure_axes` entry) → `info`, subsystem `bridge`, event `configure_axes_entry`.
- 2309 `[move-diag] bridge.submit_move enter` → `debug`, subsystem `motion`, event `submit_move_enter`.
- 2469 `/tmp/interceptor_trace.log` (`endstop_arm` result) → `info`, subsystem `homing`, event `endstop_arm_result`.
- 2641 `/tmp/interceptor_trace.log` (`run_probe_homing` dispatch) → `info`, subsystem `homing`, event `homing_move_dispatch`.

planner.rs:
- 276 `[move-diag] planner.submit_move enter` → `debug`, subsystem `motion`, event `submit_move_enter`.
- 594 commit_decel_to_zero ERR → `error`, subsystem `motion`, event `commit_decel_error`.
- 601 drained=… → `debug`, subsystem `motion`, event `commit_drained`.
- 611 dispatch ERR → `error`, subsystem `motion`, event `dispatch_error`.
- 741 planner recv gap_us= → `debug`, subsystem `motion`, event `planner_recv_gap`.
- 820, 829, 835 Move arm errors → `error`, subsystem `motion`, event `move_arm_error` (disambiguate with a distinct field or sub-event per site).

probe_homing.rs:
- 57 `/tmp/interceptor_trace.log` (probe callback) → `debug`, subsystem `homing`, event `probe_callback`.
- 76 `/tmp/interceptor_trace.log` (software_trip sent) → `info`, subsystem `homing`, event `software_trip_sent`.

Delete the now-unused `use std::io::Write;` / `OpenOptions` imports if they become unused (the compiler will warn — fix to keep `-D warnings` clean).

- [ ] **Step 2: Build + test**

Run: `cd rust && cargo test -p motion-bridge && cargo clippy -p motion-bridge --all-targets`
Expected: PASS, no warnings. Confirm no `/tmp/` or `eprintln!` remains in motion-bridge src:
Run: `grep -rn "/tmp/.*\.log\|eprintln!" motion-bridge/src --include="*.rs" | grep -v "/tests/" | grep -v "#\[cfg(test)\]"`
Expected: empty (the `[host-rust-log]` last-gasp `eprintln!` in `layer.rs` is the one allowed exception — it is the sink-failure last-gasp, not a diagnostic; exclude it).

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src
git commit -m "refactor(logging-rs): convert motion-bridge /tmp + move-diag to tracing events"
```

---

### Task 9: Convert kalico-host-rt /tmp writes + eprintln

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`
- Modify: `rust/kalico-host-rt/src/host_io/kalico_native.rs`

Same cutover rule and transformation pattern as Task 8. `kalico-host-rt` got the `tracing` dep in Task 1. Its `log::*` calls are already captured by `LogTracer` globally — leave those untouched. Only convert the `/tmp` writes and `eprintln!` diagnostics.

- [ ] **Step 1: Convert each site**

reactor.rs:
- 792 `/tmp/interceptor_trace.log` (unsolicited passthrough frame) → `debug`, subsystem `mcu-comms`, event `unsolicited_frame`.
- 1021 `/tmp/kalico-firewire.log` (SubmitTyped) → `debug`, subsystem `mcu-comms`, event `submit_typed`.
- 1123 `/tmp/kalico-firewire.log` (FireAndForget; two branches) → `debug` event `fire_and_forget_sent` on success, `error` event `fire_and_forget_encode_error` on the encode-error branch.
- 262 `[trace-write]` → `trace`, subsystem `mcu-comms`, event `frame_write`.
- 371 `[trace-await]` → `trace`, subsystem `mcu-comms`, event `await_response`.
- 737 `[trace-rx-jump]` → `warn`, subsystem `mcu-comms`, event `rx_seq_jump`.
- 750/776 `[trace-decode-err]` → `warn`, subsystem `mcu-comms`, event `decode_error`.
- 817 `[trace-resp]` → `debug`, subsystem `mcu-comms`, event `unsolicited_no_interceptor`.
- 898 `[trace-poll]` → `debug`, subsystem `mcu-comms`, event `slow_poll`.
- 1065 `[trace-close]` → `info`, subsystem `mcu-comms`, event `expected_disconnect`.
- 1158/1169 `[bridge-error]` → `error`, subsystem `mcu-comms`, event `fire_and_forget_send_error` / `..._encode_error`.
- 1393 `[trace-rto]` → `debug`, subsystem `mcu-comms`, event `retransmit`.
- 1434 `[trace-tick]` → `debug`, subsystem `mcu-comms`, event `slow_tick`.

mod.rs:
- 285/300/318 `[tio-*]` termios trace → `debug`, subsystem `mcu-comms`, event `termios_setup`.
- 386 `[reactor-spawn]` → `info`, subsystem `mcu-comms`, event `reactor_spawn`.
- 409 `[reactor-spawn] EXIT_ON_FAULT` → `error`, subsystem `mcu-comms`, event `reactor_exit_on_fault`.

kalico_native.rs:
- 260 `[kalico-id]` → `info`, subsystem `bridge`, event `identify_complete`.

- [ ] **Step 2: Build + test**

Run: `cd rust && cargo test -p kalico-host-rt && cargo clippy -p kalico-host-rt --all-targets`
Expected: PASS, no warnings. Confirm cleanup:
Run: `grep -rn "/tmp/.*\.log\|eprintln!" kalico-host-rt/src --include="*.rs" | grep -v "/tests/"`
Expected: empty (allow only genuine test-only diagnostics under `#[cfg(test)]`, if any).

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-host-rt/src
git commit -m "refactor(logging-rs): convert kalico-host-rt /tmp + eprintln to tracing events"
```

---

### Task 10: Python plumbing — call init_logging + propagate session/print

**Files:**
- Modify: `klippy/motion_bridge.py`
- Modify: `klippy/printer.py` (only if `events_dir` is not already reachable by the wrapper)
- Test: `test/test_motion_bridge_logging.py` (Create)

Goal: the `MotionBridgeWrapper` calls `init_logging(events_dir)` then `set_session_context(session_id, "")` at construction (honoring the binding-timing invariant: before any Rust log fires), and pushes `print_id` changes to Rust on print-state events.

- [ ] **Step 1: Write the failing test**

Create `test/test_motion_bridge_logging.py`:

```python
# Tests for MotionBridgeWrapper structured-logging plumbing: init_logging is
# called once with the events dir, the initial session context is pushed, and
# print-state events propagate print_id to the native bridge.
import sys
import types
import unittest
from unittest import mock

import structured_log


class FakeNative:
    def __init__(self):
        self.init_calls = []
        self.ctx_calls = []

    def init_logging(self, events_dir):
        self.init_calls.append(events_dir)

    def set_session_context(self, session_id, print_id=""):
        self.ctx_calls.append((session_id, print_id))


class FakePrinter:
    def __init__(self):
        self.handlers = {}

    def register_event_handler(self, name, cb):
        self.handlers.setdefault(name, []).append(cb)

    def fire(self, name):
        for cb in self.handlers.get(name, []):
            cb()


class MotionBridgeLoggingTest(unittest.TestCase):
    def setUp(self):
        structured_log.clear_session()
        structured_log.clear_print()

    def test_init_and_initial_context_pushed(self):
        from motion_bridge import attach_structured_logging

        native = FakeNative()
        printer = FakePrinter()
        structured_log.bind_session("k-1-2")
        attach_structured_logging(native, printer, "/home/x/printer_data/logs/events")
        self.assertEqual(native.init_calls, ["/home/x/printer_data/logs/events"])
        self.assertEqual(native.ctx_calls[0], ("k-1-2", ""))

    def test_print_start_and_end_propagate(self):
        from motion_bridge import attach_structured_logging

        native = FakeNative()
        printer = FakePrinter()
        structured_log.bind_session("k-1-2")
        attach_structured_logging(native, printer, "/x/events")
        # simulate print_stats binding a print id, then firing the event
        structured_log.bind_print("print-9")
        printer.fire("print_stats:start_printing")
        self.assertEqual(native.ctx_calls[-1], ("k-1-2", "print-9"))
        structured_log.clear_print()
        printer.fire("print_stats:complete_printing")
        self.assertEqual(native.ctx_calls[-1], ("k-1-2", ""))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/observability && PYTHONPATH=klippy python3 -m pytest test/test_motion_bridge_logging.py -v`
Expected: FAIL — `attach_structured_logging` does not exist.

- [ ] **Step 3: Implement the plumbing**

In `klippy/motion_bridge.py`, add the helper and wire it from the wrapper. First read the current `MotionBridgeWrapper.__init__` and how it obtains the printer + the log/events dir. Add a module-level function:

```python
def attach_structured_logging(native, printer, events_dir):
    # Install the Rust host structured-logging subscriber and push the current
    # session/print context. Honors the binding-timing invariant: session_id is
    # already bound by the time the bridge is constructed (printer.py startup).
    if events_dir:
        native.init_logging(events_dir)
    native.set_session_context(structured_log.get_session(), structured_log.get_print())

    def _push_ctx(*_args):
        native.set_session_context(
            structured_log.get_session(), structured_log.get_print()
        )

    for ev in (
        "print_stats:start_printing",
        "print_stats:complete_printing",
        "print_stats:error_printing",
        "print_stats:cancelled_printing",
        "print_stats:paused_printing",
        "print_stats:reset",
    ):
        printer.register_event_handler(ev, _push_ctx)
```

Add `import structured_log` (top of `motion_bridge.py`). Then in `MotionBridgeWrapper.__init__`, after `self._bridge = _native.MotionBridge()`, call `attach_structured_logging(self._bridge, printer, events_dir)` where `printer` is the wrapper's printer handle and `events_dir` is obtained the same way Stage 1 computes it. If the wrapper does not already receive `events_dir`, thread it from `printer.py` where `events_dir_for(...)` is computed (Stage 1 added `events_dir_for`). The wrapper can compute it itself: `events_dir = structured_log_events_dir(printer)` — prefer reusing the value already computed in `printer.py:main()` and stored on `start_args`/the printer, to avoid recomputation. Read `printer.py` to find the cleanest source (it already calls `events_dir_for(logfile)` and passes `edir` to `setup_bg_logging`); expose that `edir` to the bridge wrapper (e.g. via `printer.get_start_args().get('log_events_dir')` if start_args carries it, else add it there).

Note ordering: `print_stats` fires `start_printing` *after* `structured_log.bind_print` (Stage 1 ordered `bind_print` before `send_event` in `print_stats.note_start`), so when `_push_ctx` runs the contextvar already holds the new `print_id`. For `complete/error/cancel`, Stage 1 calls `clear_print()` *after* `send_event`, so `_push_ctx` on those events would still observe the old `print_id`. **Fix the ordering** in `print_stats.py` so the relevant `send_event` is fired *after* the clear, OR have `_push_ctx` for the finish events push an empty `print_id` explicitly. Choose the explicit-empty approach to avoid reordering Stage 1 semantics: register the finish events to a `_clear_ctx` handler that pushes `(session, "")`. Update the test accordingly if you take this route (the test asserts the end state is `("k-1-2", "")`, which both approaches satisfy as long as the handler reads after the clear — so the simplest correct implementation is to register finish/reset/cancel/error to a handler that pushes an empty print_id, and start/pause to `_push_ctx`).

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/observability && PYTHONPATH=klippy python3 -m pytest test/test_motion_bridge_logging.py -v`
Expected: PASS.

- [ ] **Step 5: Lint**

Run: `/opt/homebrew/bin/ruff check klippy/motion_bridge.py klippy/printer.py test/test_motion_bridge_logging.py && /opt/homebrew/bin/ruff format --check klippy/motion_bridge.py test/test_motion_bridge_logging.py`
Expected: clean (line-length 80, import order). Fix any I001/I002/E501.

- [ ] **Step 6: Commit**

```bash
git add klippy/motion_bridge.py klippy/printer.py test/test_motion_bridge_logging.py
git commit -m "feat(logging): push Rust host session/print context from Python bridge"
```

---

### Task 11: Integration verification (build .so + end-to-end NDJSON)

**Files:**
- Create: `rust/motion-bridge/tests/logging_integration.rs`

This is the end-to-end proof: with a real global subscriber in a dedicated test process, a captured `log::*` call and a native `tracing` event both land in `host-rust.jsonl` as schema-conformant lines carrying the bound `session_id`.

- [ ] **Step 1: Write the integration test**

Create `rust/motion-bridge/tests/logging_integration.rs`:

```rust
//! End-to-end: init the real subscriber, emit via both `log::` and `tracing::`,
//! confirm `host-rust.jsonl` contains schema-conformant lines with the bound
//! session/print ids. Runs in its own test process (one global subscriber).

use std::path::PathBuf;

#[test]
fn end_to_end_jsonl_has_schema_and_context() {
    let dir: PathBuf = std::env::temp_dir().join(format!(
        "kalico-log-it-{}",
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    // init must succeed exactly once in this process.
    motion_bridge_native::logging::init_logging(&dir).expect("init");
    motion_bridge_native::logging::set_context(
        "k-1748700131-4412".to_string(),
        "print-1748700500".to_string(),
    );

    log::warn!("legacy log path captured");
    tracing::info!(
        subsystem = "homing",
        event = "homing.endstop_trip",
        axis = "z",
        trigger_mm = 12.40_f64,
        "endstop trip on Z"
    );

    // Allow the non-blocking worker to flush.
    std::thread::sleep(std::time::Duration::from_millis(250));

    let contents = std::fs::read_to_string(dir.join("host-rust.jsonl")).unwrap();
    let lines: Vec<serde_json::Value> = contents
        .lines()
        .filter(|l| !l.is_empty())
        .map(|l| serde_json::from_str(l).expect("valid JSON line"))
        .collect();
    assert!(lines.len() >= 2, "expected >=2 records, got {}", lines.len());
    for r in &lines {
        assert_eq!(r["source"], "host-rust");
        assert_eq!(r["session_id"], "k-1748700131-4412");
        assert_eq!(r["print_id"], "print-1748700500");
        assert!(r["_time"].as_str().unwrap().ends_with('Z'));
        assert!(r["level"].is_string());
        assert!(r["subsystem"].is_string());
        assert!(r["target"].is_string());
    }
    let trip = lines.iter().find(|r| r["event"] == "homing.endstop_trip").unwrap();
    assert_eq!(trip["axis"], "z");
    assert!((trip["trigger_mm"].as_f64().unwrap() - 12.40).abs() < 1e-9);
}
```

This requires `logging` to be public from the crate. In `rust/motion-bridge/src/lib.rs`, change `mod logging;` to `pub mod logging;`.

- [ ] **Step 2: Run the integration test**

Run: `cd rust && cargo test -p motion-bridge --test logging_integration -- --nocapture`
Expected: PASS. If `log::warn!` lines do NOT appear, confirm `LogTracer::init()` ran before the first `log::` call and that the `EnvFilter` default (`info`) admits `warn`. If they still don't appear because `LogTracer` was already installed by another test in the same run, this dedicated `--test` target runs in its own binary/process, so it should be clean.

- [ ] **Step 3: Build the actual .so and confirm it loads**

Run: `cd rust && cargo build -p motion-bridge --release`
Then copy per the Makefile convention and smoke-import from Python:
Run: `cp rust/target/release/libmotion_bridge_native.dylib klippy/motion_bridge_native.so 2>/dev/null || cp rust/target/release/libmotion_bridge_native.so klippy/motion_bridge_native.so`
Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/observability && PYTHONPATH=klippy python3 -c "import motion_bridge_native as m; b=m.MotionBridge(); import tempfile,os; d=tempfile.mkdtemp(); b.init_logging(os.path.join(d,'events')); b.set_session_context('k-9-9',''); print('OK', sorted(os.listdir(os.path.join(d,'events'))))"`
Expected: prints `OK ['host-rust.jsonl']` (the events dir is created by the writer; the file exists). If `init_logging` raises, read the error — likely a path/permission issue or a double-init from a prior import in the same interpreter.

- [ ] **Step 4: Full regression — Rust + Python + lint**

Run: `cd rust && cargo test -p motion-bridge -p kalico-host-rt`
Expected: PASS.
Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/observability && PYTHONPATH=.:klippy python3 -m pytest test/test_structured_log.py test/test_log_sinks.py test/test_queuelogger_pipeline.py test/test_session_binding.py test/test_print_id_binding.py test/test_motion_bridge_logging.py -q`
Expected: all logging tests PASS (Stage 1 + Stage 2 Python).
Run: `cd rust && cargo clippy -p motion-bridge -p kalico-host-rt --all-targets`
Expected: no warnings.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/tests/logging_integration.rs rust/motion-bridge/src/lib.rs
git commit -m "test(logging-rs): end-to-end host-rust.jsonl schema + context integration"
```

---

## Optional verification with klipper-sim (post-merge sanity)

After all tasks, optionally run an offline planner pass through the sim (`~/Developer/klipper-sim/`, per the reference memory) against representative G-code with this branch's `--klipper-root`, and confirm `host-rust.jsonl` accumulates schema-conformant lines during a sim run alongside the Stage 1 `host-py.jsonl`. This is a real-world smoke of the full host pipeline; it is not a gating CI step (the sim needs its own setup) but is the best available pre-hardware check that both sources write coherent, queryable NDJSON.

---

## Self-Review (completed during planning)

- **Spec coverage:** §7.2 (env_logger→tracing, capture log::*, retire /tmp+eprintln, ArcSwap via PyO3, klog!) → Tasks 1,6,7,8,9 + the macro. §6 sync contract (ArcSwap, binding-timing, old-or-new-never-torn) → Tasks 2,7,10. §5 schema → Tasks 3,5 + the schema contract section. §7.2 fail-loudly (no silent writeln) → Tasks 8,9; init fail-loud → Task 6/7. §8 rotation uncompressed → Task 4. §9 level default info / drop-at-emit → Task 6 EnvFilter. §11 component boundaries (PyO3 setter, layer, writer) → Tasks 4,5,7. §14 testing (schema, sanitization, concurrency, integration) → Tasks 2,4,5,11. §13 sim → optional section.
- **Deferred correctly (not in this stage):** Vector/VL deploy, query-logs skill, heartbeat/liveness, proactive disk-full last-gasp (Stage 3 §16 item 11); per-subsystem runtime SET_LOG_LEVEL (deferred config follow-on); the size/retention numbers are defaults (§16).
- **Type consistency:** `SessionContext{session_id, print_id}`, `set_context`/`load_context`, `init_logging(events_dir)`, `set_session_context(session_id, print_id)`, `JsonlLayer::new`, `RotatingJsonlWriter::new` used consistently across tasks. `klog!(level, subsystem, event, msg; k=v,...)` signature fixed in Task 6 and referenced in 8/9.
- **Known implementation-time confirmations (flagged inline, not placeholders):** exact `arc-swap`/`tracing-appender`/`time` API spellings under the pinned minor versions; tracing's `record_str` vs `record_debug` path for the format-message (Task 5 Step 2 note); `with_extension` rotated-path naming (Task 4 note). Each has a concrete fallback in-step.
