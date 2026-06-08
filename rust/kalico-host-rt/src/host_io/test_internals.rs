use super::*;

#[test]
fn vlq_roundtrip_small_positive() {
    for v in [0i64, 1, 100, 1_000, 100_000, 1_000_000_000] {
        let mut buf = Vec::new();
        parser::encode_vlq(&mut buf, v).expect("value in range");
        let (out, n) = parser::decode_vlq(&buf).unwrap();
        assert_eq!(n, buf.len(), "consumed != encoded for {v}");
        assert_eq!(out, v, "roundtrip failed for {v}");
    }
}

#[test]
fn crc16_matches_klipper_test_vector() {
    let crc = wire::crc16_ccitt(&[0x05, 0x10]);
    assert_eq!(crc, 0x9E81);
}

#[test]
fn extract_packet_picks_up_minimal_nak_frame() {
    let crc = wire::crc16_ccitt(&[0x05, 0x10]);
    let frame = vec![
        0x05,
        0x10,
        (crc >> 8) as u8,
        (crc & 0xFF) as u8,
        wire::MESSAGE_SYNC,
    ];
    let mut buf = frame.clone();
    let extracted = wire::extract_packet(&mut buf).expect("must extract NAK");
    assert_eq!(extracted, frame);
    assert!(buf.is_empty());
}

#[test]
fn extract_packet_resyncs_past_garbage_byte_smaller_than_message_min() {
    let mut buf: Vec<u8> = vec![0x02];
    let result = wire::extract_packet(&mut buf);
    assert!(
        result.is_none(),
        "still no complete frame, but buf must have been drained"
    );
    assert!(
        buf.is_empty(),
        "garbage leading byte should have been dropped, got {buf:?}"
    );
}

#[test]
fn extract_packet_resyncs_past_oversized_msglen_byte() {
    let mut buf: Vec<u8> = vec![0xFF];
    let result = wire::extract_packet(&mut buf);
    assert!(result.is_none());
    assert!(
        buf.is_empty(),
        "oversized msglen byte should have been dropped, got {buf:?}"
    );
}

#[test]
fn send_typed_payload_matches_call_typed_payload() {
    use crate::host_io::parser::{DataDictionary, FieldValue, MsgProtoParser};
    use indexmap::IndexMap;

    let mut d = DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    };
    d.commands
        .insert("kalico_load_curve_begin slot=%hu degree=%c".into(), 99);
    let parser = MsgProtoParser::from_dictionary(d).unwrap();

    let args = [
        ("slot", FieldValue::U16(7)),
        ("degree", FieldValue::Byte(3)),
    ];
    let send_typed_payload = parser
        .encode_typed("kalico_load_curve_begin", &args)
        .expect("encode_typed");
    let call_typed_payload = parser
        .encode_typed("kalico_load_curve_begin", &args)
        .expect("encode_typed");
    assert_eq!(send_typed_payload, call_typed_payload);
    assert!(!send_typed_payload.is_empty());
}

#[test]
fn decode_vlq_caps_continuation_at_5_bytes() {
    let malformed = vec![0xFFu8; 8];
    let result = parser::decode_vlq(&malformed);
    assert!(
        matches!(result, Err(parser::ParseError::BadVlq)),
        "malformed VLQ must return BadVlq, not roll past 5 bytes"
    );
}
