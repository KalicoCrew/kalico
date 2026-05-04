//! Phase-1 corpus replay test. Spec §4.13.
//!
//! Three tests:
//!   1. replay_all_captures_decode_without_error — every .bin in tests/captures/
//!      decodes cleanly (passes vacuously when corpus is empty).
//!   2. corpus_covers_required_decode_surfaces (#[ignore]) — asserts the corpus
//!      exercises all REQUIRED_SURFACES.  Requires a populated corpus to pass;
//!      un-ignore once H723 corpus is collected (F2).
//!   3. corpus_covers_required_encode_surfaces — encode-side coverage, runs without
//!      captures.

use std::path::PathBuf;

use kalico_host_rt::host_io::parser::{DataDictionary, DecodedFrame, MsgProtoParser};
use kalico_host_rt::host_io::wire::{extract_packet, MESSAGE_MIN};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn repo_root() -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR")); // = rust/kalico-host-rt
    p.pop(); // -> rust/
    p.pop(); // -> repo root
    p
}

fn captures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/captures")
}

fn build_parser() -> MsgProtoParser {
    let dict_path = repo_root().join("out/klipper.dict");
    let blob = std::fs::read(&dict_path).expect("out/klipper.dict must exist; run `make`");
    let text = std::str::from_utf8(&blob).expect("klipper.dict is JSON");
    let dict: DataDictionary = serde_json::from_str(text).expect("klipper.dict must parse");
    MsgProtoParser::from_dictionary(dict).expect("parser must build from dictionary")
}

/// All surface names that must appear in the corpus (spec §4.13 REQUIRED_SURFACES).
const REQUIRED_SURFACES: &[&str] = &[
    "kalico_push_segment",
    "kalico_push_response",
    "kalico_clock_sync_request",
    "kalico_clock_sync_response",
    "kalico_load_curve",
    "kalico_load_curve_response",
    "kalico_stream_arm",
    "kalico_stream_arm_response",
    "kalico_credit_freed",
    "kalico_fault",
    "kalico_status_v6",
    "kalico_trace",
];

// ---------------------------------------------------------------------------
// Test 1
// ---------------------------------------------------------------------------

#[test]
fn replay_all_captures_decode_without_error() {
    let parser = build_parser();

    let bin_files: Vec<_> = std::fs::read_dir(&captures_dir())
        .expect("captures dir must exist")
        .filter_map(|entry| {
            let entry = entry.expect("dir entry");
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("bin") {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    // Passes vacuously when no corpus has been collected yet.
    if bin_files.is_empty() {
        return;
    }

    for path in &bin_files {
        let raw = std::fs::read(path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", path.display(), e));
        let mut buf = raw;
        let mut frame_count: usize = 0;
        let mut decode_errors: usize = 0;

        loop {
            match extract_packet(&mut buf) {
                None => break,
                Some(packet) => {
                    if packet.len() <= MESSAGE_MIN {
                        // Empty-body frame (min-size): skip per spec.
                        continue;
                    }
                    frame_count += 1;
                    if let Err(e) = parser.decode(&packet) {
                        eprintln!(
                            "decode error in {}: {:?} on packet {:02x?}",
                            path.display(),
                            e,
                            packet
                        );
                        decode_errors += 1;
                    }
                }
            }
        }

        assert!(
            frame_count > 0,
            "capture {} had no decodable frames",
            path.display()
        );
        assert_eq!(
            decode_errors,
            0,
            "capture {} had {} decode error(s)",
            path.display(),
            decode_errors
        );
    }
}

// ---------------------------------------------------------------------------
// Test 2 — requires H723 corpus; ignored until F2 corpus is collected
// ---------------------------------------------------------------------------

/// Un-ignore once H723 corpus is collected (F2).
#[test]
#[ignore]
fn corpus_covers_required_decode_surfaces() {
    let parser = build_parser();

    let mut observed: std::collections::HashSet<String> = std::collections::HashSet::new();

    let bin_files: Vec<_> = std::fs::read_dir(&captures_dir())
        .expect("captures dir must exist")
        .filter_map(|entry| {
            let entry = entry.expect("dir entry");
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("bin") {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    for path in &bin_files {
        let raw = std::fs::read(path)
            .unwrap_or_else(|e| panic!("failed to read {}: {}", path.display(), e));
        let mut buf = raw;

        loop {
            match extract_packet(&mut buf) {
                None => break,
                Some(packet) => {
                    if packet.len() <= MESSAGE_MIN {
                        continue;
                    }
                    if let Ok(frame) = parser.decode(&packet) {
                        match frame {
                            DecodedFrame::Response { name, .. } => {
                                observed.insert(name);
                            }
                            DecodedFrame::Output { name, .. } => {
                                observed.insert(name);
                            }
                        }
                    }
                }
            }
        }
    }

    let missing: Vec<&str> = REQUIRED_SURFACES
        .iter()
        .copied()
        .filter(|s| !observed.contains(*s))
        .collect();

    assert!(
        missing.is_empty(),
        "corpus is missing coverage for: {:?}\nobserved: {:?}",
        missing,
        observed
    );
}

// ---------------------------------------------------------------------------
// Test 3 — encode-side surface coverage (no captures required)
// ---------------------------------------------------------------------------

#[test]
fn corpus_covers_required_encode_surfaces() {
    let parser = build_parser();

    let commands = [
        "kalico_clock_sync_request request_id=42 host_send_time_lo=0 host_send_time_hi=0",
        "kalico_stream_arm t_start_t0_lo=0 t_start_t0_hi=0 arm_lead_cycles=10",
        // kalico_push_segment / kalico_load_curve retired Phase C of the
        // kalico-native transport spec — they no longer ride Klipper protocol.
        // The native LoadCurve / PushSegment frames are exercised by
        // kalico-protocol round-trip tests + sim_handshake.
    ];

    for cmd in &commands {
        let result = parser.encode(cmd);
        assert!(
            result.is_ok(),
            "encode({:?}) failed: {:?}",
            cmd,
            result.err()
        );
        let bytes = result.unwrap();
        assert!(
            !bytes.is_empty(),
            "encode({:?}) returned empty bytes",
            cmd
        );
    }
}
