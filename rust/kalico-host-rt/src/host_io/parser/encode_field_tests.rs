use super::*;

#[test]
fn encodes_string_length_prefixed() {
    let mut buf = Vec::new();
    encode_field_value(&mut buf, FieldType::String, &FieldValue::String("hi")).unwrap();
    assert_eq!(buf, vec![2, b'h', b'i']);
}

#[test]
fn encodes_byte_via_vlq() {
    let mut buf = Vec::new();
    encode_field_value(&mut buf, FieldType::Byte, &FieldValue::Byte(0xFF)).unwrap();
    assert_eq!(buf, vec![0x81, 0x7F]);
}

#[test]
fn byte_field_accepts_signed_negative() {
    // Klipper's reference msgproto (PT_byte → PT_uint32 VLQ) accepts
    // signed values for %c. The bridge path's config_stepper emits
    // invert_step=-1 (commit 8649861c9); rejecting it here breaks every
    // config_stepper on every bridge-mode MCU. Regression guard.
    use indexmap::IndexMap;
    let enums: IndexMap<String, EnumTable> = IndexMap::new();
    for v in &["-1", "-128", "0", "127", "255"] {
        let mut buf = Vec::new();
        encode_field_str(&mut buf, &WrappedField::Plain(FieldType::Byte), v, &enums)
            .unwrap_or_else(|e| panic!("Byte should accept {v:?}: {e:?}"));
        assert!(!buf.is_empty(), "encoded payload non-empty for {v:?}");
    }
}

#[test]
fn byte_field_still_rejects_truly_out_of_range() {
    use indexmap::IndexMap;
    let enums: IndexMap<String, EnumTable> = IndexMap::new();
    for v in &["-129", "256", "1000", "-1000"] {
        let mut buf = Vec::new();
        let r = encode_field_str(&mut buf, &WrappedField::Plain(FieldType::Byte), v, &enums);
        assert!(
            matches!(r, Err(ParseError::OutOfRange { .. })),
            "Byte should reject {v:?}, got {r:?}"
        );
    }
}

#[test]
fn rejects_string_too_long() {
    let s = "x".repeat(300);
    let mut buf = Vec::new();
    match encode_field_value(&mut buf, FieldType::String, &FieldValue::String(&s)) {
        Err(ParseError::OutOfRange { .. }) => {}
        other => panic!("expected OutOfRange, got {:?}", other),
    }
}

#[test]
fn parse_hex_buffer_round_trips() {
    assert_eq!(
        parse_hex_buffer("0123abcd").unwrap(),
        vec![0x01, 0x23, 0xAB, 0xCD]
    );
    assert_eq!(parse_hex_buffer("").unwrap(), Vec::<u8>::new());
    assert!(matches!(parse_hex_buffer("0z"), Err(ParseError::BadHex(_))));
    assert!(matches!(parse_hex_buffer("1"), Err(ParseError::BadHex(_))));
}
