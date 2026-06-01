//! End-to-end: init the real subscriber, emit via both `log::` and `tracing::`,
//! confirm `host-rust.jsonl` contains schema-conformant lines with the bound
//! session/print ids. Runs in its own test process (one global subscriber).

use std::path::PathBuf;

#[test]
fn end_to_end_jsonl_has_schema_and_context() {
    let dir: PathBuf =
        std::env::temp_dir().join(format!("kalico-log-it-{}", std::process::id()));
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
    let trip = lines
        .iter()
        .find(|r| r["event"] == "homing.endstop_trip")
        .unwrap();
    assert_eq!(trip["axis"], "z");
    assert!((trip["trigger_mm"].as_f64().unwrap() - 12.40).abs() < 1e-9);
}
