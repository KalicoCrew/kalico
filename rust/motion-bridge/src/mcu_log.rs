//! Re-emit closure factory for MCU structured-log events.
//!
//! `build_mcu_log_hook` constructs the closure injected into
//! `EventDispatcher::set_mcu_log_hook` (via `KalicoHostIo::set_mcu_log_hook`).
//! It captures:
//!   - `Arc<RwLock<ClockSyncEstimator>>` — shared with the clock-sync thread
//!   - `Arc<Mutex<RotatingJsonlWriter>>` — dedicated NDJSON writer for
//!     `events/<source>.jsonl` (separate from `host-rust.jsonl`)
//!   - `source: String` — `"mcu-h7"` or `"mcu-f4"`

use std::io::Write;
use std::sync::{Arc, Mutex, RwLock};

use serde_json::{Map, Value};
use time::OffsetDateTime;

use kalico_host_rt::clock_sync::ClockSyncEstimator;
use kalico_host_rt::host_io::runtime_events::McuLogEvent;
use runtime::error::FaultCode;
use runtime::log_codes::{compose_msg, event_info, subsystem_name};

use crate::logging::context::load_context;
use crate::logging::schema::format_time;
use crate::logging::writer::RotatingJsonlWriter;

/// Map a raw `u8` MCU level to a `&'static str` for the schema `level` field.
///
/// Any unrecognised value is treated as `"error"` (fail-loudly posture).
fn mcu_level_str(level: u8) -> &'static str {
    match level {
        0 => "trace",
        1 => "debug",
        2 => "warn",
        // 3 and all unrecognised levels map to "error".
        _ => "error",
    }
}

/// Build the re-emit closure for MCU structured-log events.
///
/// The returned closure is `Fn(McuLogEvent) + Send + Sync + 'static` and can be
/// passed directly to [`kalico_host_rt::host_io::KalicoHostIo::set_mcu_log_hook`].
///
/// # Arguments
///
/// * `clock`  — shared clock-sync estimator (write-locked by the clock-sync
///   thread; read-locked here for `wall_time_at_mcu`).
/// * `writer` — dedicated rotating NDJSON writer for `events/<source>.jsonl`.
/// * `source` — value written into the `source` field, e.g. `"mcu-h7"`.
///
/// # Timestamp fallback
///
/// When the estimator has no samples yet (`wall_time_at_mcu` returns `None`),
/// the closure reconstructs an approximate wall-clock from `host_recv`
/// (the `Instant` stamped at decode time) and sets `time_estimated = true`.
pub fn build_mcu_log_hook(
    clock: Arc<RwLock<ClockSyncEstimator>>,
    writer: Arc<Mutex<RotatingJsonlWriter>>,
    source: String,
) -> impl Fn(McuLogEvent) + Send + Sync + 'static {
    move |e: McuLogEvent| {
        // 1. Resolve timestamp.
        let (time_str, time_estimated) = {
            let guard = clock
                .read()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if let Some((dt, estimated)) = guard.wall_time_at_mcu(e.mcu_tick) {
                (format_time(dt), estimated)
            } else {
                // No clock-sync samples yet — fall back to host_recv.
                // `host_recv` is an `Instant`; convert via SystemTime.
                let elapsed = e.host_recv.elapsed();
                let sys = std::time::SystemTime::now()
                    .checked_sub(elapsed)
                    .unwrap_or(std::time::SystemTime::UNIX_EPOCH);
                (format_time(OffsetDateTime::from(sys)), true)
            }
        };

        // 2. Resolve names.
        let subsys_name = subsystem_name(e.subsystem);
        let (event_name, template) = event_info(e.subsystem, e.event);
        let msg = compose_msg(template, e.args[0], e.args[1]);

        // 3. Resolve fault code (sign-wrapped u16; 0 means no fault code).
        let (code_val, code_name_val): (Option<u16>, Option<&'static str>) = if e.code != 0 {
            let name = FaultCode::from_u16(e.code)
                .map(FaultCode::code_name)
                .unwrap_or("unknown");
            (Some(e.code), Some(name))
        } else {
            (None, None)
        };

        // 4. Stamp session context.
        let ctx = load_context();

        // 5. Compose the NDJSON record.
        let mut rec = Map::new();
        rec.insert("_time".into(), Value::String(time_str));
        rec.insert("_msg".into(), Value::String(msg));
        rec.insert(
            "level".into(),
            Value::String(mcu_level_str(e.level).to_owned()),
        );
        rec.insert("source".into(), Value::String(source.clone()));
        rec.insert("subsystem".into(), Value::String(subsys_name.to_owned()));
        rec.insert("event".into(), Value::String(event_name.to_owned()));
        rec.insert("session_id".into(), Value::String(ctx.session_id.clone()));
        rec.insert("print_id".into(), Value::String(ctx.print_id.clone()));
        rec.insert(
            "target".into(),
            Value::String(format!("mcu::{subsys_name}")),
        );
        rec.insert("mcu_tick".into(), Value::from(e.mcu_tick));
        rec.insert("seq".into(), Value::from(e.seq));
        rec.insert("arg0".into(), Value::from(e.args[0]));
        rec.insert("arg1".into(), Value::from(e.args[1]));
        rec.insert("time_estimated".into(), Value::Bool(time_estimated));
        if let Some(code) = code_val {
            rec.insert("code".into(), Value::from(code));
        }
        if let Some(name) = code_name_val {
            rec.insert("code_name".into(), Value::String(name.to_owned()));
        }

        let mut line = serde_json::to_string(&Value::Object(rec))
            .unwrap_or_else(|err| format!("{{\"_msg\":\"mcu-log serialize error: {err}\"}}"));
        line.push('\n');

        // 6. Write to the dedicated MCU JSONL file.
        let mut w = writer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Err(err) = w.write_all(line.as_bytes()) {
            // Fail-loudly on write error. Use eprintln because this closure
            // runs on the reactor dispatch thread where the tracing subscriber
            // may not be safely reachable.
            eprintln!("[mcu-log] JSONL write failed: {err}");
        }
    }
}
