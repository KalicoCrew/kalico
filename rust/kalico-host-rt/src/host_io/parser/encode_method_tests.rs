use super::*;

fn parser_with_one_command() -> MsgProtoParser {
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
    d.commands.insert("ping val=%u".into(), 42);
    MsgProtoParser::from_dictionary(d).unwrap()
}

#[test]
fn string_and_typed_encode_to_same_bytes() {
    let p = parser_with_one_command();
    let bytes_str = p.encode("ping val=100").unwrap();
    let bytes_typed = p
        .encode_typed("ping", &[("val", FieldValue::U32(100))])
        .unwrap();
    assert_eq!(bytes_str, bytes_typed);
}

#[test]
fn encode_rejects_unknown_command() {
    let p = parser_with_one_command();
    match p.encode("unknown_cmd") {
        Err(ParseError::UnknownCommand(_)) => {}
        other => panic!("expected UnknownCommand, got {:?}", other),
    }
}

#[test]
fn encode_rejects_missing_field() {
    let p = parser_with_one_command();
    match p.encode("ping") {
        Err(ParseError::MissingField(_)) => {}
        other => panic!("expected MissingField, got {:?}", other),
    }
}

#[test]
fn enum_encode_rejects_unknown_name() {
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
    d.commands.insert("config_pin pin=%c".into(), 1);
    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(0));
    d.enumerations.insert("pin".to_string(), pin_table);

    let p = MsgProtoParser::from_dictionary(d).unwrap();
    match p.encode("config_pin pin=PZZZ") {
        Err(ParseError::UnknownEnumValue { value, .. }) => assert_eq!(value, "PZZZ"),
        other => panic!("expected UnknownEnumValue, got {:?}", other),
    }
}
