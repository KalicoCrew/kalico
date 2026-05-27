use super::*;

fn empty_dict() -> DataDictionary {
    DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    }
}

#[test]
fn rejects_duplicate_msgid_across_sections() {
    let mut d = empty_dict();
    d.commands.insert("cmd_a arg=%u".into(), 5);
    d.responses.insert("rsp_b arg=%u".into(), 5);
    match MsgProtoParser::from_dictionary(d) {
        Err(ParseError::DuplicateMsgid(5)) => {}
        other => panic!("expected DuplicateMsgid(5), got {:?}", other),
    }
}

#[test]
fn rejects_duplicate_format_string() {
    let mut d = empty_dict();
    d.commands.insert("cmd arg=%u".into(), 5);
    d.responses.insert("cmd arg=%u".into(), 6);
    match MsgProtoParser::from_dictionary(d) {
        Err(ParseError::DuplicateFormatString(_)) => {}
        other => panic!("expected DuplicateFormatString, got {:?}", other),
    }
}

#[test]
fn accepts_disjoint_categories() {
    let mut d = empty_dict();
    d.commands.insert("cmd_a arg=%u".into(), 1);
    d.responses.insert("rsp_a arg=%u".into(), 2);
    d.output.insert("evt_a arg=%u".into(), 3);
    let p = MsgProtoParser::from_dictionary(d).unwrap();
    assert!(matches!(
        p.by_msgid.get(&1),
        Some(DispatchSpec::Response(_))
    ));
    assert!(matches!(
        p.by_msgid.get(&2),
        Some(DispatchSpec::Response(_))
    ));
    assert!(matches!(p.by_msgid.get(&3), Some(DispatchSpec::Output(_))));
}

/// Spec §4.7: free-form `output(...)` formats whose fields aren't
/// `name=%type`-tagged must accept-and-fall-back, not reject the dict.
#[test]
fn accepts_free_form_output_format() {
    let mut d = empty_dict();
    d.output.insert("debug_blob count=%u %s".into(), 7);
    let p = MsgProtoParser::from_dictionary(d).expect("free-form output must parse");
    match p.by_msgid.get(&7) {
        Some(DispatchSpec::Output(spec)) => {
            assert!(spec.is_free_form, "must mark as free-form");
            assert_eq!(spec.fields.len(), 2, "two %-codes recovered positionally");
            assert!(spec.field_names.is_empty());
        }
        other => panic!("expected free-form Output, got {other:?}"),
    }
}

#[test]
fn free_form_output_decodes_to_canonical_msg() {
    let mut d = empty_dict();
    d.output.insert("debug_blob %u %s".into(), 8);
    let parser = MsgProtoParser::from_dictionary(d).unwrap();
    // Body: msgid VLQ + u32 VLQ value 5 + length-prefixed string "hi"
    let mut body = Vec::new();
    encode_vlq(&mut body, 8).unwrap(); // msgid
    encode_vlq(&mut body, 5).unwrap(); // %u value
    body.push(2); // %s length prefix
    body.extend_from_slice(b"hi");

    let mut packet = vec![0u8, 0u8]; // 2-byte header (len + dest|seq)
    packet.extend_from_slice(&body);
    packet.extend_from_slice(&[0, 0, 0]); // 3-byte trailer (CRC + sync)

    let frame = parser.decode(&packet).expect("decode succeeds");
    match frame {
        DecodedFrame::Output { name, params } => {
            assert_eq!(name, "#output", "free-form must surface as #output");
            let msg = params.try_get_str("#msg").unwrap_or("");
            assert!(
                msg.contains("5"),
                "formatted message contains %u value: {msg:?}"
            );
            assert!(
                msg.contains("hi"),
                "formatted message contains %s value: {msg:?}"
            );
        }
        other => panic!("expected Output, got {other:?}"),
    }
}
