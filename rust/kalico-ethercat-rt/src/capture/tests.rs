use std::path::PathBuf;
use std::sync::mpsc::sync_channel;

use super::*;

fn sample(n: i32) -> DriveSample {
    DriveSample {
        target_counts: n,
        position_demand: n + 1,
        position_actual: n + 2,
        following_error: -3,
        torque_actual: 42,
        statusword: 0x0627,
        error_code: 0,
    }
}

fn record(cycle: u64) -> CaptureRecord {
    CaptureRecord {
        cycle_index: cycle,
        flags: FLAG_TORQUE_ENABLED | FLAG_MOTION_ACTIVE,
        drive: sample(1000),
    }
}

fn cfg(path: &PathBuf) -> CaptureConfig {
    CaptureConfig {
        path: path.to_str().unwrap().to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drive_name: "x".to_owned(),
        cycle_ns: 1_000_000,
        counts_per_mm: 3276.8,
        started_mono_ns: 7,
    }
}

fn tmp_path(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "kalico-capture-{}-{}.scap",
        tag,
        std::process::id()
    ))
}

#[test]
fn record_encodes_to_fixed_little_endian_layout() {
    let r = CaptureRecord {
        cycle_index: 0x0102030405060708,
        flags: 0x03,
        drive: DriveSample {
            target_counts: -2,
            position_demand: 0x11223344,
            position_actual: -1,
            following_error: 5,
            torque_actual: -300,
            statusword: 0x0627,
            error_code: 0x7380,
        },
    };
    let b = encode_record(&r);
    assert_eq!(b.len(), RECORD_SIZE);
    assert_eq!(&b[0..8], &0x0102030405060708u64.to_le_bytes());
    assert_eq!(b[8], 0x03);
    assert_eq!(&b[9..13], &(-2i32).to_le_bytes());
    assert_eq!(&b[13..17], &0x11223344i32.to_le_bytes());
    assert_eq!(&b[17..21], &(-1i32).to_le_bytes());
    assert_eq!(&b[21..25], &5i32.to_le_bytes());
    assert_eq!(&b[25..27], &(-300i16).to_le_bytes());
    assert_eq!(&b[27..29], &0x0627u16.to_le_bytes());
    assert_eq!(&b[29..31], &0x7380u16.to_le_bytes());
}

#[test]
fn header_is_one_json_line_describing_the_record() {
    let path = tmp_path("hdr");
    let h = header_json(&cfg(&path));
    assert!(h.ends_with('\n'));
    assert_eq!(h.lines().count(), 1);
    for needle in [
        "\"version\":1",
        "\"cycle_ns\":1000000",
        "\"record_size\":31",
        "\"started_utc\":\"2026-06-10T12:00:00Z\"",
        "\"started_mono_ns\":7",
        "\"name\":\"x\"",
        "\"counts_per_mm\":3276.8",
        "{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}",
        "{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}",
        "{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9}",
        "{\"name\":\"position_demand\",\"dtype\":\"i32\",\"offset\":13}",
        "{\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":17}",
        "{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":21}",
        "{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":25}",
        "{\"name\":\"statusword\",\"dtype\":\"u16\",\"offset\":27}",
        "{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":29}",
    ] {
        assert!(h.contains(needle), "header missing {needle}: {h}");
    }
}

#[test]
fn lifecycle_start_push_stop_produces_parseable_file() {
    let path = tmp_path("happy");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert!(!cap.is_active());
    assert_eq!(cap.start(cfg(&path)), 0);
    assert!(cap.is_active());
    for i in 0..50u64 {
        cap.push(record(i));
    }
    let out = cap.stop();
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, 50);
    assert_eq!(out.overflow_cycle, None);
    assert!(!cap.is_active());

    let bytes = std::fs::read(&path).unwrap();
    let nl = bytes.iter().position(|&b| b == b'\n').unwrap();
    let header = std::str::from_utf8(&bytes[..nl]).unwrap();
    assert!(header.contains("\"version\":1"));
    let body = &bytes[nl + 1..];
    assert_eq!(body.len(), 50 * RECORD_SIZE);
    assert_eq!(&body[..RECORD_SIZE], &encode_record(&record(0)));
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn double_start_rejected_and_capture_survives() {
    let path = tmp_path("dbl");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert_eq!(cap.start(cfg(&path)), 0);
    assert_eq!(cap.start(cfg(&path)), ERR_CAPTURE_ACTIVE);
    assert!(cap.is_active());
    let out = cap.stop();
    assert_eq!(out.result, 0);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn stop_without_start_rejected() {
    let mut cap = Capture::new();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_NOT_ACTIVE);
    assert_eq!(out.samples, 0);
}

#[test]
fn unwritable_path_fails_start() {
    let mut cap = Capture::new();
    let c = cfg(&PathBuf::from("/dev/null/nope/x.scap"));
    assert_eq!(cap.start(c), ERR_CAPTURE_FILE);
    assert!(!cap.is_active());
}

#[test]
fn quote_in_drive_name_rejected_before_touching_disk() {
    let path = tmp_path("badname");
    let mut cap = Capture::new();
    let mut c = cfg(&path);
    c.drive_name = "x\"evil".to_owned();
    assert_eq!(cap.start(c), ERR_CAPTURE_BAD_ARG);
    assert!(!path.exists());
}

#[test]
fn overflow_kills_capture_and_renames_file() {
    let path = tmp_path("ovf");
    let _ = std::fs::remove_file(&path);
    let failed = path.with_extension("failed.scap");
    let _ = std::fs::remove_file(&failed);

    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(4);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..10u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_OVERFLOW);
    assert_eq!(out.overflow_cycle, Some(4));
    assert_eq!(out.samples, 4);
    assert!(!path.exists(), "failed capture must not keep .scap name");
    assert!(failed.exists(), "failed capture must be renamed");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn pushes_after_overflow_are_ignored() {
    let path = tmp_path("ovf2");
    let _ = std::fs::remove_file(&path);
    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(2);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..100u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.overflow_cycle, Some(2), "first refused cycle is recorded");
    let failed = path.with_extension("failed.scap");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn writer_death_latches_file_error() {
    let path = tmp_path("wdeath");
    let _ = std::fs::remove_file(&path);
    let failed = path.with_extension("failed.scap");
    let _ = std::fs::remove_file(&failed);

    let mut cap = Capture::with_capacity(4);
    let (start_result, writer_done) = cap.start_writer_fails(cfg(&path));
    assert_eq!(start_result, 0);

    writer_done.recv().expect("writer must signal done before failing");

    for i in 0..3u64 {
        cap.push(record(i));
    }
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_FILE);
    assert!(!path.exists(), "failed capture must not keep .scap name");
    assert!(failed.exists(), "failed capture must be renamed");
    std::fs::remove_file(&failed).unwrap();
}
