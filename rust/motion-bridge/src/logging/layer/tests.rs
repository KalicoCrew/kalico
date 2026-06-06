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
