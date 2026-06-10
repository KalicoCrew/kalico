use super::*;

#[test]
fn sdo_kinds_map_to_u16_and_back() {
    for kind in [
        MessageKind::SdoRead,
        MessageKind::SdoReadResponse,
        MessageKind::SdoWrite,
        MessageKind::SdoWriteResponse,
    ] {
        assert_eq!(MessageKind::from_u16(kind.as_u16()), Some(kind));
        assert!(!kind.is_event(), "SDO kinds must not be in the event range");
    }
}

#[test]
fn sdo_read_roundtrip() {
    let msg = SdoRead {
        index: 0x2002,
        subindex: 3,
    };
    assert_eq!(roundtrip(&msg), msg);
    assert_eq!(msg.encoded_to_vec().len(), 3);
}

#[test]
fn sdo_read_response_roundtrip() {
    let msg = SdoReadResponse {
        result: 0x0601_0002,
        size: 2,
        data: [0x64, 0x00, 0x00, 0x00],
    };
    assert_eq!(roundtrip(&msg), msg);
    assert_eq!(msg.encoded_to_vec().len(), 9);
}

#[test]
fn sdo_write_roundtrip_negative_value() {
    let msg = SdoWrite {
        index: 0x2010,
        subindex: 1,
        size: 4,
        value: -4096,
    };
    assert_eq!(roundtrip(&msg), msg);
    assert_eq!(msg.encoded_to_vec().len(), 12);
}

#[test]
fn sdo_write_response_roundtrip() {
    let msg = SdoWriteResponse {
        result: ERR_SDO_VERIFY_MISMATCH,
        readback_size: 2,
        readback_data: [0xF4, 0x01, 0x00, 0x00],
    };
    assert_eq!(roundtrip(&msg), msg);
    assert_eq!(msg.encoded_to_vec().len(), 9);
}
