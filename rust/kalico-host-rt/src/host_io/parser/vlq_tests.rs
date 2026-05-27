use super::*;

#[test]
fn round_trips_representative_values() {
    // Klipper's signed VLQ preserves the original i64 value exactly —
    // including u32::MAX (4294967295) which is distinct from -1 on the
    // wire (5 bytes vs 1 byte). The caller truncates to i32 via `as i32`
    // after decode when the field type requires it (see decode_response).
    for v in [
        0i64,
        1,
        -1,
        100,
        100_000,
        i64::from(i32::MIN),
        i64::from(u32::MAX),
    ] {
        let mut buf = Vec::new();
        encode_vlq(&mut buf, v).unwrap();
        let (decoded, consumed) = decode_vlq(&buf).unwrap();
        assert_eq!(consumed, buf.len(), "consumed != encoded length for {}", v);
        assert_eq!(decoded, v, "round-trip for {} produced {}", v, decoded);
    }
}

#[test]
fn encode_vlq_rejects_out_of_range() {
    let mut buf = Vec::new();
    match encode_vlq(&mut buf, i64::from(u32::MAX) + 1) {
        Err(ParseError::OutOfRange { .. }) => {}
        other => panic!("expected OutOfRange, got {:?}", other),
    }
    match encode_vlq(&mut buf, i64::from(i32::MIN) - 1) {
        Err(ParseError::OutOfRange { .. }) => {}
        other => panic!("expected OutOfRange, got {:?}", other),
    }
}
