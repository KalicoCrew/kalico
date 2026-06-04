use super::*;

#[test]
fn parses_kalico_push_segment_format() {
    let s =
        "kalico_push_segment id=%u x_handle=%u y_handle=%u z_handle=%u e_handle=%u kinematics=%c";
    let (name, fields) = parse_format_string(s).unwrap();
    assert_eq!(name, "kalico_push_segment");
    assert_eq!(fields.len(), 6);
    assert_eq!(fields[0], ("id".to_string(), FieldType::U32));
    assert_eq!(fields[5], ("kinematics".to_string(), FieldType::Byte));
}

#[test]
fn parses_progmem_buffer_in_identify_response() {
    let s = "identify_response offset=%u data=%.*s";
    let (name, fields) = parse_format_string(s).unwrap();
    assert_eq!(name, "identify_response");
    assert_eq!(fields[1].1, FieldType::ProgmemBuffer);
}

#[test]
fn rejects_unknown_format_code_hc() {
    let s = "bad_cmd val=%hc";
    match parse_format_string(s) {
        Err(ParseError::UnknownFormatCode(c)) if c == "%hc" => {}
        other => panic!("expected UnknownFormatCode(%hc), got {:?}", other),
    }
}
