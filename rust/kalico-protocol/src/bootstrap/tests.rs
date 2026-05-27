use super::*;

#[test]
fn identify_byte_layout() {
    let m = Identify {
        proto_version: 0x01,
    };
    let bytes = m.encode_body_to_array();
    assert_eq!(bytes.len(), 1);
    assert_eq!(bytes, [0x01]);
    assert_eq!(Identify::decode_body(&bytes), Ok(m));
}

#[test]
fn identify_decode_rejects_wrong_length() {
    assert!(matches!(
        Identify::decode_body(&[]),
        Err(BootstrapDecodeError::WrongLength {
            expected: 1,
            got: 0
        })
    ));
    assert!(matches!(
        Identify::decode_body(&[1, 2]),
        Err(BootstrapDecodeError::WrongLength {
            expected: 1,
            got: 2
        })
    ));
}

#[test]
fn identify_response_offsets_are_frozen() {
    // Hand-counted from spec §5. If any of these fail, a protocol break
    // has been introduced — DO NOT update them, fix the layout instead.
    assert_eq!(IDR_OFF_PROTO_VERSION, 0);
    assert_eq!(IDR_OFF_FIRMWARE_VER, 1);
    assert_eq!(IDR_OFF_BUILD_HASH, 5);
    assert_eq!(IDR_OFF_SCHEMA_HASH, 25);
    assert_eq!(IDR_OFF_RESET_EPOCH, 57);
    assert_eq!(IDR_OFF_CAPABILITIES, 61);
    assert_eq!(IDR_OFF_MCU_SERIAL, 69);
    assert_eq!(IDENTIFY_RESPONSE_BODY_LEN, 81);
}

#[test]
fn identify_response_byte_layout() {
    // Construct with field values that make every byte position visible.
    let build_hash: [u8; 20] = std::array::from_fn(|i| 0x40 + i as u8);
    let schema_hash: [u8; 32] = std::array::from_fn(|i| 0x60 + i as u8);
    let mcu_serial: [u8; 12] = std::array::from_fn(|i| 0xA0 + i as u8);
    let m = IdentifyResponse {
        proto_version: 0x01,
        firmware_ver: 0x1122_3344,
        build_hash,
        schema_hash,
        reset_epoch: 0xDEAD_BEEF,
        capabilities: 0x0102_0304_0506_0708,
        mcu_serial,
    };
    let bytes = m.encode_body_to_array();
    assert_eq!(bytes.len(), 81);

    // Hand-rolled byte-by-byte verification.
    assert_eq!(bytes[0], 0x01);
    // firmware_ver little-endian.
    assert_eq!(&bytes[1..5], &[0x44, 0x33, 0x22, 0x11]);
    // build_hash verbatim.
    assert_eq!(&bytes[5..25], &build_hash);
    // schema_hash verbatim.
    assert_eq!(&bytes[25..57], &schema_hash);
    // reset_epoch little-endian.
    assert_eq!(&bytes[57..61], &[0xEF, 0xBE, 0xAD, 0xDE]);
    // capabilities little-endian.
    assert_eq!(
        &bytes[61..69],
        &[0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01]
    );
    // mcu_serial verbatim.
    assert_eq!(&bytes[69..81], &mcu_serial);

    // Round-trip.
    assert_eq!(IdentifyResponse::decode_body(&bytes), Ok(m));
}

#[test]
fn identify_response_decode_rejects_wrong_length() {
    assert!(matches!(
        IdentifyResponse::decode_body(&[0u8; 80]),
        Err(BootstrapDecodeError::WrongLength {
            expected: 81,
            got: 80
        })
    ));
}
