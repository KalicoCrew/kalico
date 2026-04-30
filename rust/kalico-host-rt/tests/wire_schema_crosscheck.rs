//! Closure-review regression test for finding #2 (HIGH, SHIP-BLOCKER):
//! every host outbound command emission MUST match the firmware's
//! `DECL_COMMAND` format string in `src/runtime_tick.c` exactly — same
//! field names, same ordering. Drift between the two sides causes the
//! Klipper msgproto encoder to silently produce frames the firmware
//! refuses to decode.
//!
//! Approach: parse the firmware's `DECL_COMMAND(...)` format strings out
//! of `src/runtime_tick.c` programmatically, then check each
//! known-host-emitted command name has its field-name list reproduced
//! verbatim by the host-rt encoder.
//!
//! This is the cross-check the prior review chain (4-round spec + 5-round
//! plan + per-phase code reviews) didn't have, which is why the
//! producer-side `curve_handle_packed=` / `kin=` / lo-before-hi mismatch
//! shipped despite extensive review.

use std::collections::HashMap;
use std::time::Duration;

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::producer::{SegmentPushParams, push_segment};
use kalico_host_rt::transport::MessageValue;

mod mock_transport;
use mock_transport::{MockTransport, mp_with};

/// Path to the firmware's `runtime_tick.c` relative to the workspace root.
/// `CARGO_MANIFEST_DIR` is `rust/kalico-host-rt`; go up two levels.
fn firmware_runtime_tick_path() -> std::path::PathBuf {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("manifest dir");
    let mut p = std::path::PathBuf::from(manifest_dir);
    p.pop(); // rust/
    p.pop(); // workspace root
    p.push("src");
    p.push("runtime_tick.c");
    p
}

/// Extract `name -> [field_name, ...]` mapping from every
/// `DECL_COMMAND(handler, "name field1=%X field2=%Y ...")` declaration in
/// the file. Multi-line format strings are supported (C-style adjacent
/// string literal concatenation).
fn parse_decl_commands(src: &str) -> HashMap<String, Vec<String>> {
    let mut out = HashMap::new();
    // Pull each `DECL_COMMAND(...)` block. Brittle but adequate for the
    // shape `runtime_tick.c` actually uses.
    let mut rest = src;
    while let Some(start) = rest.find("DECL_COMMAND(") {
        let after_open = &rest[start + "DECL_COMMAND(".len()..];
        // Find the closing `)` at top level. There are no nested parens
        // inside DECL_COMMAND in this file.
        let Some(close) = after_open.find(");") else {
            break;
        };
        let block = &after_open[..close];
        rest = &after_open[close..];

        // Concatenate string-literal fragments inside the block.
        let mut concat = String::new();
        let mut in_str = false;
        let mut chars = block.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                '"' => in_str = !in_str,
                '\\' if in_str => {
                    if let Some(&next) = chars.peek() {
                        // Skip the next char (handles \" / \n / etc.)
                        chars.next();
                        match next {
                            'n' => concat.push('\n'),
                            't' => concat.push('\t'),
                            other => concat.push(other),
                        }
                    }
                }
                _ if in_str => concat.push(c),
                _ => {}
            }
        }
        // First whitespace-separated token is the command name; the rest
        // are `name=%X` pairs we strip down to just `name`.
        let mut tokens = concat.split_whitespace();
        let Some(name) = tokens.next() else {
            continue;
        };
        let mut fields = Vec::new();
        for tok in tokens {
            if let Some((field, _ty)) = tok.split_once('=') {
                fields.push(field.to_string());
            }
        }
        out.insert(name.to_string(), fields);
    }
    out
}

#[test]
fn parse_decl_commands_finds_known_handlers() {
    let path = firmware_runtime_tick_path();
    let src =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let cmds = parse_decl_commands(&src);
    // Sanity — these are the commands hosts emit.
    for required in [
        "kalico_push_segment",
        "kalico_load_curve",
        "kalico_stream_open",
        "kalico_stream_arm",
        "kalico_stream_terminal",
        "kalico_stream_flush",
        "kalico_clock_sync_request",
        "kalico_query_pool_state",
    ] {
        assert!(
            cmds.contains_key(required),
            "DECL_COMMAND parser missed `{required}` — adjust regex/token \
             splitter in this test"
        );
    }
}

#[test]
fn host_push_segment_field_names_and_ordering_match_firmware() {
    let path = firmware_runtime_tick_path();
    let src = std::fs::read_to_string(&path).expect("read runtime_tick.c");
    let cmds = parse_decl_commands(&src);
    let firmware_fields = cmds
        .get("kalico_push_segment")
        .expect("kalico_push_segment in DECL_COMMAND set");

    // Capture the host's emission via MockTransport.
    let mut io = MockTransport::new();
    let credit = CreditCounter::new(1);
    io.enqueue_response(
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("accepted_segment_id", MessageValue::U32(1)),
            ("credit_epoch", MessageValue::U32(1)),
        ]),
    );
    push_segment(&mut io, &credit, &SegmentPushParams {
        id: 1,
        x_handle_packed: 0x0001_0000,
        y_handle_packed: 0,
        z_handle_packed: 0,
        e_handle_packed: 0,
        t_start: 100,
        t_end: 200,
        kinematics: 0,
        e_mode: 0,
        extrusion_ratio: 0.0,
    }).expect("happy push for schema cross-check");
    let line = io.last_sent().expect("MockTransport recorded send");

    // Strip leading command name and split on whitespace; confirm field
    // *names* and *ordering* match the firmware DECL_COMMAND verbatim.
    let mut tokens = line.split_whitespace();
    let cmd_name = tokens.next().unwrap();
    assert_eq!(cmd_name, "kalico_push_segment");
    let host_fields: Vec<String> = tokens
        .map(|t| t.split_once('=').unwrap().0.to_string())
        .collect();
    assert_eq!(
        &host_fields, firmware_fields,
        "host `kalico_push_segment` field names/ordering must match firmware \
         DECL_COMMAND verbatim. host=`{line}` firmware fields={firmware_fields:?}"
    );
}

#[test]
fn host_stream_arm_field_names_and_ordering_match_firmware() {
    // Same shape as the push_segment cross-check but for stream::arm.
    // We don't have a Mock-only path through stream::arm (it depends on
    // a real ClockSyncEstimator anchor), so we hand-verify by comparing
    // the firmware's expected field list against the literal format
    // string the host uses in `src/stream.rs`.
    let path = firmware_runtime_tick_path();
    let src = std::fs::read_to_string(&path).expect("read runtime_tick.c");
    let cmds = parse_decl_commands(&src);
    let firmware_fields = cmds
        .get("kalico_stream_arm")
        .expect("kalico_stream_arm in DECL_COMMAND set");
    assert_eq!(
        firmware_fields,
        &vec![
            "t_start_t0_lo".to_string(),
            "t_start_t0_hi".to_string(),
            "arm_lead_cycles".to_string(),
        ],
        "firmware schema for kalico_stream_arm changed — update host \
         emission in `rust/kalico-host-rt/src/stream.rs` accordingly"
    );

    // Inspect the host source directly. The stream.rs format string is
    // a static literal; we read the file and assert the expected
    // ordering pattern exists.
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("manifest dir");
    let stream_path = std::path::PathBuf::from(manifest_dir)
        .join("src")
        .join("stream.rs");
    let stream_src = std::fs::read_to_string(&stream_path)
        .unwrap_or_else(|e| panic!("read {}: {e}", stream_path.display()));
    assert!(
        stream_src.contains(
            "kalico_stream_arm t_start_t0_lo={lo} t_start_t0_hi={hi} arm_lead_cycles={alc}"
        ),
        "host `kalico_stream_arm` emission diverged from firmware \
         DECL_COMMAND. Update src/stream.rs format string."
    );
}

#[test]
fn host_clock_sync_request_field_names_and_ordering_match_firmware() {
    let path = firmware_runtime_tick_path();
    let src = std::fs::read_to_string(&path).expect("read runtime_tick.c");
    let cmds = parse_decl_commands(&src);
    let firmware_fields = cmds
        .get("kalico_clock_sync_request")
        .expect("kalico_clock_sync_request in DECL_COMMAND set");
    assert_eq!(
        firmware_fields,
        &vec![
            "request_id".to_string(),
            "host_send_time_lo".to_string(),
            "host_send_time_hi".to_string(),
        ]
    );

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("manifest dir");
    let stream_path = std::path::PathBuf::from(manifest_dir)
        .join("src")
        .join("stream.rs");
    let stream_src = std::fs::read_to_string(&stream_path).expect("read stream.rs");
    assert!(
        stream_src.contains(
            "kalico_clock_sync_request request_id=1 host_send_time_lo=0 host_send_time_hi=0"
        ),
        "host `kalico_clock_sync_request` emission diverged from firmware \
         DECL_COMMAND. Update src/stream.rs format string."
    );
}

// Suppress dead-code warning on `Duration` import when test binary is
// compiled without using every helper from `mock_transport`.
const _: Duration = Duration::from_millis(0);
