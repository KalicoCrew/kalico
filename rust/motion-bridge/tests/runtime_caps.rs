use kalico_protocol::codec::{Cursor, Decode, Encode};
use kalico_protocol::messages::{RUNTIME_CAPS_RESPONSE_BODY_LEN, RuntimeCapsResponse};

#[test]
fn query_runtime_caps_roundtrip_via_codec() {
    let original = RuntimeCapsResponse {
        total_piece_memory: 63488,
    };

    let mut body = Vec::new();
    original.encode(&mut body);
    assert_eq!(
        body.len(),
        RUNTIME_CAPS_RESPONSE_BODY_LEN,
        "encoded body must match the documented 4-byte layout",
    );

    let mut c = Cursor::new(&body);
    let decoded = RuntimeCapsResponse::decode_from(&mut c)
        .expect("RuntimeCapsResponse decodes from its own encoding");
    assert_eq!(decoded, original);
}

#[test]
fn query_runtime_caps_short_body_errors() {
    let body = [0u8; 2];
    let mut c = Cursor::new(&body);
    let r = RuntimeCapsResponse::decode_from(&mut c);
    assert!(r.is_err(), "short body must fail to decode, got {r:?}");
}
