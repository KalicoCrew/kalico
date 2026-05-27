use super::*;

#[test]
fn version_check_v1_passes() {
    let payload = [0x01_u8, 0x02, 0x03];
    assert!(check_version(&payload).is_ok());
}

#[test]
fn version_check_unknown_rejects() {
    let payload = [0xFF_u8, 0x02, 0x03];
    assert_eq!(
        check_version(&payload),
        Err(FaultCode::ProtocolVersionUnsupported)
    );
}

#[test]
fn version_check_empty_rejects() {
    let payload: [u8; 0] = [];
    assert_eq!(
        check_version(&payload),
        Err(FaultCode::ProtocolVersionUnsupported)
    );
}

#[test]
fn version_v1_bare_minimum_one_byte() {
    let payload = [FORMAT_VERSION_V1];
    assert!(check_version(&payload).is_ok());
}
