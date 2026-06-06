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
mod tests;
