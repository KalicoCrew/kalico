use super::*;

fn build_packet(msgid: i32, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::new();
    let mut body = Vec::new();
    encode_vlq(&mut body, i64::from(msgid)).unwrap();
    body.extend_from_slice(payload);
    let msglen = MESSAGE_MIN + body.len();
    frame.push(msglen as u8);
    frame.push(0x10); // dest|seq=0
    frame.extend_from_slice(&body);
    frame.extend_from_slice(&[0, 0]); // dummy CRC
    frame.push(0x7E);
    frame
}

#[test]
fn decode_response_round_trips() {
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
    d.responses.insert("rsp val=%u".into(), 7);
    let parser = MsgProtoParser::from_dictionary(d).unwrap();

    let mut payload = Vec::new();
    encode_vlq(&mut payload, 12345).unwrap();
    let packet = build_packet(7, &payload);

    match parser.decode(&packet).unwrap() {
        DecodedFrame::Response { name, params } => {
            assert_eq!(name, "rsp");
            assert_eq!(params.get_u32("val"), 12345);
        }
        other => panic!("expected Response, got {:?}", other),
    }
}

#[test]
fn decode_unknown_msgid_returns_error() {
    let d = DataDictionary {
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
    let parser = MsgProtoParser::from_dictionary(d).unwrap();
    let packet = build_packet(99, &[]);
    match parser.decode(&packet) {
        Err(ParseError::UnknownMsgid(99)) => {}
        other => panic!("expected UnknownMsgid(99), got {:?}", other),
    }
}

#[test]
fn decode_output_recovers_field_names() {
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
    d.output.insert(
        "kalico_credit_freed retired_through_segment_id=%u free_slots=%c".into(),
        50,
    );
    let p = MsgProtoParser::from_dictionary(d).unwrap();

    let mut payload = Vec::new();
    encode_vlq(&mut payload, 42).unwrap();
    encode_vlq(&mut payload, 11).unwrap();
    let packet = build_packet(50, &payload);

    match p.decode(&packet).unwrap() {
        DecodedFrame::Output { name, params } => {
            assert_eq!(name, "kalico_credit_freed");
            assert_eq!(params.get_u32("retired_through_segment_id"), 42);
            assert_eq!(params.get_u32("free_slots"), 11);
        }
        other => panic!("expected Output, got {:?}", other),
    }
}

#[test]
fn decode_output_canonical_produces_msg_form() {
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
    d.output.insert(
        "kalico_credit_freed retired_through_segment_id=%u free_slots=%c".into(),
        50,
    );
    let p = MsgProtoParser::from_dictionary(d).unwrap();

    let mut payload = Vec::new();
    encode_vlq(&mut payload, 42).unwrap();
    encode_vlq(&mut payload, 11).unwrap();
    let packet = build_packet(50, &payload);

    let (name, params) = p.decode_output_canonical(&packet).unwrap();
    assert_eq!(name, "#output");
    let msg = params.try_get_str("#msg").unwrap();
    assert!(msg.contains("kalico_credit_freed"));
    assert!(msg.contains("42"));
    assert!(msg.contains("11"));
}
