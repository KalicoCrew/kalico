use std::io::Write;
use std::sync::{Arc, Mutex};

use serde_json::{Map, Value};
use time::OffsetDateTime;

use kalico_host_rt::host_io::runtime_events::McuLogEvent;
use kalico_host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use runtime::error::FaultCode;
use runtime::log_codes::{compose_msg, event_info, subsystem_name};

use crate::logging::context::load_context;
use crate::logging::schema::format_time;
use crate::logging::writer::RotatingJsonlWriter;

fn mcu_level_str(level: u8) -> &'static str {
    match level {
        0 => "trace",
        1 => "debug",
        2 => "warn",
        _ => "error",
    }
}

pub fn build_mcu_log_hook(
    router: Arc<Mutex<PassthroughRouter>>,
    mcu: McuHandle,
    writer: Arc<Mutex<RotatingJsonlWriter>>,
    source: String,
) -> impl Fn(McuLogEvent) + Send + Sync + 'static {
    move |e: McuLogEvent| {
        let (time_str, time_estimated) = {
            let guard = router
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if let Some((dt, estimated)) = guard.wall_time_at_mcu(mcu, e.mcu_tick) {
                (format_time(dt), estimated)
            } else {
                let elapsed = e.host_recv.elapsed();
                let sys = std::time::SystemTime::now()
                    .checked_sub(elapsed)
                    .unwrap_or(std::time::SystemTime::UNIX_EPOCH);
                (format_time(OffsetDateTime::from(sys)), true)
            }
        };

        let subsys_name = subsystem_name(e.subsystem);
        let (event_name, template) = event_info(e.subsystem, e.event);
        let msg = compose_msg(template, e.args[0], e.args[1]);

        let (code_val, code_name_val): (Option<u16>, Option<&'static str>) = if e.code != 0 {
            let name = FaultCode::from_u16(e.code)
                .map(FaultCode::code_name)
                .unwrap_or("unknown");
            (Some(e.code), Some(name))
        } else {
            (None, None)
        };

        let ctx = load_context();

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

        let mut w = writer
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Err(err) = w.write_all(line.as_bytes()) {
            eprintln!("[mcu-log] JSONL write failed: {err}");
        }
    }
}
