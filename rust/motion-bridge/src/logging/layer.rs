use std::io::Write;

use serde_json::{Map, Value};
use time::OffsetDateTime;
use tracing::field::{Field, Visit};
use tracing::{Event, Subscriber};
use tracing_subscriber::Layer;
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::Context;

use super::context::load_context;
use super::schema::{SOURCE_HOST_RUST, format_time, level_str, subsystem_for_target};

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
            self.map
                .insert(field.name().to_string(), Value::String(value.to_string()));
        }
    }

    fn record_i64(&mut self, field: &Field, value: i64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_u64(&mut self, field: &Field, value: u64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_f64(&mut self, field: &Field, value: f64) {
        self.map
            .insert(field.name().to_string(), Value::from(value));
    }

    fn record_bool(&mut self, field: &Field, value: bool) {
        self.map
            .insert(field.name().to_string(), Value::Bool(value));
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
        out.insert(
            "_time".into(),
            Value::String(format_time(OffsetDateTime::now_utc())),
        );
        out.insert(
            "_msg".into(),
            Value::String(visitor.message.unwrap_or_default()),
        );
        out.insert(
            "level".into(),
            Value::String(level_str(meta.level()).into()),
        );
        out.insert("source".into(), Value::String(SOURCE_HOST_RUST.into()));

        let subsystem = match visitor.map.remove("subsystem") {
            Some(Value::String(s)) => s,
            _ => subsystem_for_target(target).to_string(),
        };
        out.insert("subsystem".into(), Value::String(subsystem));
        out.insert("session_id".into(), Value::String(ctx.session_id.clone()));
        out.insert("target".into(), Value::String(target.to_string()));
        out.insert("print_id".into(), Value::String(ctx.print_id.clone()));

        for (k, v) in visitor.map {
            out.entry(k).or_insert(v);
        }

        let mut line = serde_json::to_string(&Value::Object(out))
            .unwrap_or_else(|e| format!("{{\"_msg\":\"serialize error: {e}\"}}"));
        line.push('\n');

        let mut w = self.make_writer.make_writer();
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

    use crate::logging::CONTEXT_TEST_LOCK;

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
        let _ctx_guard = CONTEXT_TEST_LOCK.lock().unwrap();
        super::super::context::set_context("k-1-2".into(), String::new());
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
        assert!((r["trigger_mm"].as_f64().unwrap() - 12.40).abs() < 1e-9);
        assert_eq!(r["session_id"], "k-1-2");
        assert_eq!(r["print_id"], "");
        assert!(r["_time"].as_str().unwrap().ends_with('Z'));
    }

    #[test]
    fn subsystem_falls_back_to_target_mapping() {
        let _ctx_guard = CONTEXT_TEST_LOCK.lock().unwrap();
        super::super::context::set_context("k-1-2".into(), String::new());
        let recs = capture(|| {
            tracing::warn!(event = "retry", "attach_serial retry");
        });
        assert!(recs[0]["subsystem"].is_string());
    }

    #[test]
    fn embedded_newline_yields_one_line() {
        let _ctx_guard = CONTEXT_TEST_LOCK.lock().unwrap();
        super::super::context::set_context("k-1-2".into(), String::new());
        let recs = capture(|| {
            tracing::info!("line one\nline two\u{0007}");
        });
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0]["_msg"], "line one\nline two\u{0007}");
    }

    #[test]
    fn message_with_literal_quotes_is_preserved() {
        let _ctx_guard = CONTEXT_TEST_LOCK.lock().unwrap();
        super::super::context::set_context("k-1-2".into(), String::new());
        let recs = capture(|| {
            tracing::info!("{}", "\"hello\"");
        });
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0]["_msg"], "\"hello\"");
    }
}
