use super::*;

#[test]
fn identify_roundtrip_lengths() {
    let buf = encode_identify(0x1234_5678, 0x01);
    assert_eq!(buf.len(), BOOTSTRAP_IDENTIFY_LEN);
}

#[test]
fn identify_response_roundtrip() {
    let resp = IdentifyResponse {
        proto_version: 0x01,
        firmware_ver: 0xDEAD_BEEF,
        build_hash: [0x42; 20],
        schema_hash: [0xAB; 32],
        reset_epoch: 0xCAFE_BABE,
        capabilities: 0x0000_0000_0000_0001,
        mcu_serial: *b"abcdef012345",
    };
    let buf = encode_identify_response(7, &resp);
    let (cid, decoded) = decode_identify_response(&buf).unwrap();
    assert_eq!(cid, 7);
    assert_eq!(decoded.proto_version, 0x01);
    assert_eq!(decoded.firmware_ver, 0xDEAD_BEEF);
    assert_eq!(decoded.schema_hash, [0xAB; 32]);
    assert_eq!(decoded.reset_epoch, 0xCAFE_BABE);
    assert_eq!(decoded.capabilities, 1);
    assert_eq!(&decoded.mcu_serial, b"abcdef012345");
}
