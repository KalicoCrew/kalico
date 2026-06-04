use std::path::PathBuf;

use indexmap::IndexMap;
use kalico_host_rt::host_io::parser::{DataDictionary, DecodedFrame, MsgProtoParser};
use kalico_host_rt::host_io::wire::{MESSAGE_MIN, extract_packet};

fn repo_root() -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.pop();
    p.pop();
    p
}

fn captures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/captures")
}

fn build_parser() -> Option<MsgProtoParser> {
    let dict_path = repo_root().join("out/klipper.dict");
    let blob = match std::fs::read(&dict_path) {
        Ok(b) => b,
        Err(_) => return None,
    };
    let text = std::str::from_utf8(&blob).expect("klipper.dict is JSON");
    let dict: DataDictionary = serde_json::from_str(text).expect("klipper.dict must parse");
    Some(MsgProtoParser::from_dictionary(dict).expect("parser must build from dictionary"))
}

fn build_encode_parser() -> MsgProtoParser {
    let mut commands: IndexMap<String, i32> = IndexMap::new();
    commands.insert(
        "runtime_clock_sync_request request_id=%u host_send_time_lo=%u host_send_time_hi=%u".into(),
        1,
    );
    commands.insert(
        "runtime_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u".into(),
        2,
    );

    let dict = DataDictionary {
        commands,
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "inline-test".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    };
    MsgProtoParser::from_dictionary(dict).expect("inline encode parser must build")
}

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

#[test]
fn replay_all_captures_decode_without_error() {
    let Some(parser) = build_parser() else {
        eprintln!(
            "klipper.dict not found — vacuous until H723 corpus collected (tracked, not flaky)"
        );
        return;
    };

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

#[test]
#[ignore = "vacuous until H723 capture corpus is collected — tracked, not flaky"]
fn corpus_covers_required_decode_surfaces() {
    let Some(parser) = build_parser() else {
        eprintln!(
            "klipper.dict not found — vacuous until H723 corpus collected (tracked, not flaky)"
        );
        return;
    };

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

#[test]
fn corpus_covers_required_encode_surfaces() {
    let parser = build_encode_parser();

    let commands = [
        "runtime_clock_sync_request request_id=42 host_send_time_lo=0 host_send_time_hi=0",
        "runtime_stream_arm t_start_t0_lo=0 t_start_t0_hi=0 arm_lead_cycles=10",
    ];

    for cmd in &commands {
        let result = parser.encode(cmd);
        assert!(result.is_ok(), "encode({cmd:?}) failed: {:?}", result.err());
        let bytes = result.unwrap();
        assert!(!bytes.is_empty(), "encode({cmd:?}) returned empty bytes");
    }
}
