use super::*;

#[test]
fn roundtrip_empty_payload() {
    let frame = encode_frame(CHANNEL_CONTROL, &[]);
    let (ch, payload) = decode_frame(&frame).unwrap();
    assert_eq!(ch, CHANNEL_CONTROL);
    assert!(payload.is_empty());
}

#[test]
fn roundtrip_various_sizes() {
    for size in [1usize, 7, 64, 256, 1024, 8192, 16000] {
        let payload: Vec<u8> = (0..size).map(|i| (i & 0xFF) as u8).collect();
        let frame = encode_frame(CHANNEL_EVENTS, &payload);
        let (ch, decoded) = decode_frame(&frame).unwrap();
        assert_eq!(ch, CHANNEL_EVENTS);
        assert_eq!(decoded, payload.as_slice());
    }
}

#[test]
fn crc_mismatch_detected() {
    let mut frame = encode_frame(CHANNEL_CONTROL, b"hello kalico");
    // Flip a payload byte; CRC was computed before the flip.
    frame[5] ^= 0xFF;
    let err = decode_frame(&frame).unwrap_err();
    assert!(matches!(err, FrameError::CrcMismatch { .. }), "{err:?}");
}

#[test]
fn bad_sync_rejected() {
    let mut frame = encode_frame(CHANNEL_CONTROL, b"x");
    frame[0] = 0x7E;
    let err = decode_frame(&frame).unwrap_err();
    assert!(matches!(err, FrameError::BadSync(0x7E)));
}

#[test]
fn len_too_small_rejected() {
    // sync + len=2 + channel byte + bogus crc — len field below min.
    let mut buf = vec![FRAME_SYNC];
    buf.extend_from_slice(&2u16.to_le_bytes());
    buf.push(0);
    buf.extend_from_slice(&0u16.to_le_bytes());
    let err = decode_frame(&buf).unwrap_err();
    assert!(matches!(err, FrameError::LenTooSmall(2, _)));
}
