use super::*;

#[test]
fn failure_text_maps_codes() {
    assert!(failure_text(0x0601_0002).contains("CoE abort 0x06010002"));
    assert!(failure_text(ERR_SDO_VERIFY_MISMATCH).contains("readback mismatch"));
    assert!(failure_text(ERR_SDO_UNSUPPORTED_SIZE).contains("size"));
    assert!(failure_text(ERR_SDO_TRANSPORT).contains("transport"));
    assert!(failure_text(ERR_SDO_VALUE_RANGE).contains("does not fit"));
    assert_eq!(failure_text(-999), "endpoint error -999");
}
