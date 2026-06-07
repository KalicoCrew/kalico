use std::sync::{Arc, Mutex};
use std::time::Instant;

use time::OffsetDateTime;

use kalico_host_rt::clock::RealClock;
use kalico_host_rt::host_io::runtime_events::McuLogEvent;
use kalico_host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use motion_bridge_native::logging::context;
use motion_bridge_native::logging::writer::RotatingJsonlWriter;
use motion_bridge_native::logging::writer::{
    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL,
};
use motion_bridge_native::mcu_log::build_mcu_log_hook;

/// Serialise tests that call `context::set_context` so they don't race on the
/// process-global `ArcSwap<SessionContext>`.  Integration tests run in the same
/// binary (one process), so a per-file mutex is sufficient.
static CTX_LOCK: Mutex<()> = Mutex::new(());

fn tmp_jsonl(dir_suffix: &str, filename: &str) -> std::path::PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "kalico-mcu-log-test-{}-{dir_suffix}",
        std::process::id()
    ));
    std::fs::create_dir_all(&p).unwrap();
    p.push(filename);
    p
}

fn make_router_with_clock_record(label: &str) -> (Arc<Mutex<PassthroughRouter>>, McuHandle) {
    let router = Arc::new(Mutex::new(PassthroughRouter::with_clock(Arc::new(
        RealClock,
    ))));
    let mcu = router.lock().unwrap().claim_mcu(label);
    router
        .lock()
        .unwrap()
        .set_clock_est_from_sample(mcu, 100_000_000.0, Instant::now(), 15 * 100_000_000)
        .unwrap();
    (router, mcu)
}

fn make_empty_router(label: &str) -> (Arc<Mutex<PassthroughRouter>>, McuHandle) {
    let router = Arc::new(Mutex::new(PassthroughRouter::with_clock(Arc::new(
        RealClock,
    ))));
    let mcu = router.lock().unwrap().claim_mcu(label);
    (router, mcu)
}

#[test]
fn re_emit_closure_produces_schema_conformant_line() {
    let _ctx_guard = CTX_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    context::set_context("k-test-session".to_string(), "print-42".to_string());

    let path = tmp_jsonl("reemit", "mcu-h7.jsonl");
    let writer = Arc::new(Mutex::new(
        RotatingJsonlWriter::new(
            &path,
            DEFAULT_MAX_BYTES,
            DEFAULT_BACKUP_COUNT,
            FSYNC_INTERVAL,
        )
        .unwrap(),
    ));

    let (router, mcu) = make_router_with_clock_record("mcu-h7");
    let hook = build_mcu_log_hook(router, mcu, Arc::clone(&writer), "mcu-h7".to_string());

    let event = McuLogEvent {
        mcu_tick: 15 * 100_000_000u64,
        level: 2,
        subsystem: 2,
        event: 1,
        code: 0xFEC9,
        seq: 7,
        args: [100, 200],
        host_recv: Instant::now(),
    };

    hook(event);

    {
        let mut w = writer.lock().unwrap();
        use std::io::Write;
        w.flush().unwrap();
    }

    let content = std::fs::read_to_string(&path).unwrap();
    let line = content.lines().next().expect("at least one line");
    let rec: serde_json::Value = serde_json::from_str(line).expect("valid JSON");

    assert_eq!(rec["source"], "mcu-h7");
    assert_eq!(rec["level"], "warn");
    assert_eq!(rec["subsystem"], "tick");
    assert_eq!(rec["event"], "tick.interval_exceeded");
    assert_eq!(rec["session_id"], "k-test-session");
    assert_eq!(rec["print_id"], "print-42");
    assert_eq!(rec["seq"], 7);
    assert_eq!(rec["code"], 0xFEC9u64);
    assert_eq!(rec["code_name"], "TickIntervalExceeded");
    assert!(rec["_msg"].as_str().unwrap().contains("100"));
    let time_str = rec["_time"].as_str().unwrap();
    assert!(time_str.ends_with('Z'), "_time must end with Z: {time_str}");
    OffsetDateTime::parse(time_str, &time::format_description::well_known::Rfc3339)
        .unwrap_or_else(|e| panic!("_time '{time_str}' is not valid RFC3339: {e}"));
    assert!(
        rec.get("time_estimated").is_some(),
        "time_estimated field must be present"
    );
    assert_eq!(rec["arg0"], 100u64);
    assert_eq!(rec["arg1"], 200u64);
}

#[test]
fn fallback_stamps_time_estimated_true_when_no_clock_sync_samples() {
    let _ctx_guard = CTX_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    context::set_context(
        "k-fallback-session".to_string(),
        "print-fallback".to_string(),
    );

    let path = tmp_jsonl("fallback", "mcu-h7-fallback.jsonl");
    let writer = Arc::new(Mutex::new(
        RotatingJsonlWriter::new(
            &path,
            DEFAULT_MAX_BYTES,
            DEFAULT_BACKUP_COUNT,
            FSYNC_INTERVAL,
        )
        .unwrap(),
    ));

    let (router, mcu) = make_empty_router("mcu-h7");
    let hook = build_mcu_log_hook(router, mcu, Arc::clone(&writer), "mcu-h7".to_string());

    let event = McuLogEvent {
        mcu_tick: 5 * 100_000_000u64,
        level: 3,
        subsystem: 0,
        event: 0,
        code: 0xFEC9,
        seq: 1,
        args: [42, 0],
        host_recv: Instant::now(),
    };

    hook(event);

    {
        let mut w = writer.lock().unwrap();
        use std::io::Write;
        w.flush().unwrap();
    }

    let content = std::fs::read_to_string(&path).unwrap();
    let line = content.lines().next().expect("at least one NDJSON line");
    let rec: serde_json::Value = serde_json::from_str(line).expect("valid JSON");

    let time_str = rec["_time"].as_str().expect("_time must be a string");
    assert!(
        time_str.ends_with('Z'),
        "_time must end with Z (got: {time_str})"
    );
    OffsetDateTime::parse(time_str, &time::format_description::well_known::Rfc3339)
        .unwrap_or_else(|e| panic!("_time '{time_str}' is not valid RFC3339: {e}"));

    assert_eq!(
        rec["time_estimated"],
        serde_json::Value::Bool(true),
        "time_estimated must be true when the router has no clock record (fallback path)"
    );
}

#[test]
fn source_matches_label() {
    let _ctx_guard = CTX_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    context::set_context("k-label-test".to_string(), "print-label".to_string());

    for label in &["mcu-h7", "mcu-f4"] {
        let filename = format!("{label}.jsonl");
        let path = tmp_jsonl(&format!("source-label-{label}"), &filename);

        let writer = Arc::new(Mutex::new(
            RotatingJsonlWriter::new(
                &path,
                DEFAULT_MAX_BYTES,
                DEFAULT_BACKUP_COUNT,
                FSYNC_INTERVAL,
            )
            .unwrap(),
        ));

        let (router, mcu) = make_router_with_clock_record(label);
        let hook = build_mcu_log_hook(router, mcu, Arc::clone(&writer), (*label).to_string());

        let event = McuLogEvent {
            mcu_tick: 3 * 100_000_000u64,
            level: 3,
            subsystem: 1,
            event: 0,
            code: 0,
            seq: 1,
            args: [0, 0],
            host_recv: Instant::now(),
        };

        hook(event);

        {
            let mut w = writer.lock().unwrap();
            use std::io::Write;
            w.flush().unwrap();
        }

        let content = std::fs::read_to_string(&path).unwrap();
        let line = content.lines().next().expect("at least one line");
        let rec: serde_json::Value = serde_json::from_str(line).expect("valid JSON");

        assert_eq!(
            rec["source"], *label,
            "source field must match the label '{label}', got {:?}",
            rec["source"]
        );
    }
}
