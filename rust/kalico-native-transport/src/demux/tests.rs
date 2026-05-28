use super::*;
use crate::frame::{CHANNEL_CONTROL, encode_frame};

fn good_klipper_frame(payload: &[u8], seq: u8) -> Vec<u8> {
    // Build a valid Klipper frame: [len][seq|DEST][payload][crc_hi][crc_lo][0x7E]
    let len = 5 + payload.len();
    assert!(len <= 64);
    let mut buf = Vec::with_capacity(len);
    buf.push(len as u8);
    buf.push((seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST);
    buf.extend_from_slice(payload);
    let crc = crc16_ccitt(&buf);
    buf.push((crc >> 8) as u8);
    buf.push((crc & 0xFF) as u8);
    buf.push(MESSAGE_SYNC);
    buf
}

fn fake_klipper_frame(payload: &[u8]) -> Vec<u8> {
    good_klipper_frame(payload, 0)
}

#[test]
fn klipper_validates_good_crc_and_trailer() {
    let frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let mut d = Demuxer::new();
    let (frames, errors) = d.feed_slice(&frame);
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 1, "expected one frame");
    match &frames[0] {
        Frame::Klipper(kf) => assert_eq!(kf.bytes(), &frame[..]),
        other => panic!("expected Klipper frame, got {other:?}"),
    }
}

#[test]
fn klipper_bad_crc_emits_stream_error() {
    let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let len = frame.len();
    frame[len - 3] ^= 0xFF; // corrupt CRC hi
    let mut d = Demuxer::new();
    let (_, errors) = d.feed_slice(&frame);
    assert!(!errors.is_empty(), "expected a StreamError, got none");
}

#[test]
fn klipper_bad_trailer_emits_stream_error() {
    let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
    let last = frame.len() - 1;
    frame[last] = 0x00; // not 0x7E
    let mut d = Demuxer::new();
    let (_, errors) = d.feed_slice(&frame);
    assert!(!errors.is_empty(), "expected a StreamError, got none");
}

#[test]
fn klipper_then_kalico_then_klipper() {
    let k1 = fake_klipper_frame(&[1, 2, 3]);
    let kal = encode_frame(CHANNEL_CONTROL, b"hello there kalico");
    let k2 = fake_klipper_frame(&[9, 9]);

    let mut d = Demuxer::new();
    let mut stream = Vec::new();
    stream.extend_from_slice(&k1);
    stream.extend_from_slice(&kal);
    stream.extend_from_slice(&k2);
    let (frames, errors) = d.feed_slice(&stream);

    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 3);
    match &frames[0] {
        Frame::Klipper(kf) => assert_eq!(kf.bytes(), &k1[..]),
        other => panic!("expected Klipper frame, got {other:?}"),
    }
    match &frames[1] {
        Frame::Kalico { channel, payload } => {
            assert_eq!(*channel, CHANNEL_CONTROL);
            assert_eq!(payload.as_slice(), b"hello there kalico");
        }
        other => panic!("expected kalico frame, got {other:?}"),
    }
    match &frames[2] {
        Frame::Klipper(kf) => assert_eq!(kf.bytes(), &k2[..]),
        other => panic!("expected Klipper frame, got {other:?}"),
    }
}

#[test]
fn kalico_payload_with_7e_does_not_resync() {
    // Payload contains the Klipper inter-frame sync byte; demuxer must
    // not break out of kalico state mid-frame.
    let payload = vec![0x7E; 200];
    let kal = encode_frame(CHANNEL_CONTROL, &payload);
    let mut d = Demuxer::new();
    let (frames, errors) = d.feed_slice(&kal);
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 1);
    match &frames[0] {
        Frame::Kalico {
            channel,
            payload: p,
        } => {
            assert_eq!(*channel, CHANNEL_CONTROL);
            assert_eq!(p, &payload);
        }
        other => panic!("expected kalico frame, got {other:?}"),
    }
}

#[test]
fn kalico_payload_with_55_does_not_confuse() {
    let payload = vec![FRAME_SYNC; 200];
    let kal = encode_frame(CHANNEL_CONTROL, &payload);
    let mut d = Demuxer::new();
    let (frames, errors) = d.feed_slice(&kal);
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 1);
    assert!(matches!(&frames[0], Frame::Kalico { .. }));
}

#[test]
fn partial_frames_split_across_feeds() {
    let kal = encode_frame(CHANNEL_CONTROL, &(0u8..200).collect::<Vec<u8>>());
    let mut d = Demuxer::new();
    // Feed in 17-byte chunks.
    let mut total = 0;
    for chunk in kal.chunks(17) {
        let (frames, _) = d.feed_slice(chunk);
        for f in frames {
            if matches!(f, Frame::Kalico { .. }) {
                total += 1;
            }
        }
    }
    assert_eq!(total, 1);
}

#[test]
fn malformed_kalico_len_recovers() {
    // sync + len=2 (below min). Demuxer flags error and resyncs.
    let mut d = Demuxer::new();
    let mut bytes = vec![FRAME_SYNC];
    bytes.extend_from_slice(&2u16.to_le_bytes());
    let (frames, errors) = d.feed_slice(&bytes);
    assert!(frames.is_empty(), "expected no frames, got {frames:?}");
    assert_eq!(errors.len(), 1);
    assert!(matches!(&errors[0], StreamError::KalicoLenBelowMin { .. }));
    // Now feed a valid Klipper frame; should still parse.
    let k = fake_klipper_frame(&[1]);
    let (frames, errors) = d.feed_slice(&k);
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 1);
    assert!(matches!(&frames[0], Frame::Klipper(_)));
}

#[test]
fn stray_7e_between_frames_tolerated() {
    let mut d = Demuxer::new();
    let kal = encode_frame(CHANNEL_CONTROL, b"abc");
    let (frames, errors) = d.feed_slice(&[0x7E, 0x7E, 0x7E]);
    assert!(frames.is_empty());
    assert!(errors.is_empty());
    let (frames, errors) = d.feed_slice(&kal);
    assert!(errors.is_empty(), "unexpected errors: {errors:?}");
    assert_eq!(frames.len(), 1);
    assert!(matches!(&frames[0], Frame::Kalico { .. }));
}

#[test]
fn out_of_frame_garbage_dropped() {
    let mut d = Demuxer::new();
    // 0x80 is not Klipper-len-range (5..=64), not 0x55, not 0x7E.
    let (frames, errors) = d.feed_slice(&[0x80, 0x81, 0x82]);
    assert!(frames.is_empty());
    assert!(errors.is_empty());
    let kal = encode_frame(CHANNEL_CONTROL, b"x");
    let (frames, _) = d.feed_slice(&kal);
    assert_eq!(frames.len(), 1);
}

#[test]
fn klipper_bad_crc_followed_immediately_by_valid_frame_recovers() {
    // Produce a stream where a "false length latch" byte starts a fake
    // klipper frame that overlaps the start of a real, valid frame.
    // After the false frame fails validation, 1-byte-shift resync MUST
    // recover and emit the real frame.
    let real = good_klipper_frame(&[0xAA, 0xBB], 0);
    // Prepend one byte in the Klipper-len-range (5..=64) to force a false latch.
    // 5 = minimum Klipper length, which consumes 4 bytes of `real` as payload;
    // the false frame's trailer check fails, triggering 1-byte-shift resync.
    let mut stream = Vec::new();
    // False latch: 5 (minimum Klipper len). The false frame consumes
    // real[0..4] as its payload bytes, then fails trailer validation
    // (real[3]=0xBB ≠ 0x7E). The replay queue then re-feeds real[0..4],
    // and together with the remaining real[4..7] the demuxer reassembles
    // and validates the true frame.
    stream.push(5u8);
    stream.extend_from_slice(&real);
    let mut d = Demuxer::new();
    let (frames, errors) = d.feed_slice(&stream);
    // Expect: at least one StreamError + the real KlipperFrame.
    assert!(
        !errors.is_empty(),
        "expected stream error from false latch, got {errors:?}"
    );
    let klippers: Vec<&[u8]> = frames
        .iter()
        .filter_map(|f| match f {
            Frame::Klipper(kf) => Some(kf.bytes()),
            _ => None,
        })
        .collect();
    assert!(
        klippers.iter().any(|b| *b == real.as_slice()),
        "expected the real frame to be recovered after resync; got {klippers:?}"
    );
}

#[test]
fn klipper_false_length_latch_recovers_to_valid_frame() {
    // Stream: a false-length-latch byte in 5..=64 range that partially
    // overlaps the start of a real frame, followed by the real frame's
    // remaining bytes. The demuxer must recover the real frame via resync.
    let real = good_klipper_frame(&[0xAA], 0);
    let mut stream = Vec::new();
    // False latch 5 (minimum Klipper len) consumes 4 bytes of `real` as payload;
    // the trailer check fails, replay re-feeds those 4 bytes, and the remaining
    // live bytes from `real` complete the valid frame.
    stream.push(5u8);
    stream.extend_from_slice(&real);
    let mut d = Demuxer::new();
    let (frames, _errors) = d.feed_slice(&stream);
    let klippers: Vec<&[u8]> = frames
        .iter()
        .filter_map(|f| match f {
            Frame::Klipper(kf) => Some(kf.bytes()),
            _ => None,
        })
        .collect();
    assert!(
        klippers.iter().any(|b| *b == real.as_slice()),
        "expected the real frame to be recovered; got {klippers:?}"
    );
}

#[test]
fn klipper_bad_dest_emits_stream_error() {
    // Build a real frame, then clobber the seq byte's DEST flag (upper nibble).
    // Both DEST-clear and DEST-with-extra-bits should fail.
    let mut frame = good_klipper_frame(&[0x01, 0x02], 0);
    frame[1] = 0x05; // DEST bit clear, low nibble 5
    // CRC is now stale; recompute so we test ONLY the DEST check, not CRC.
    let crc_off = frame.len() - 3;
    let crc = crc16_ccitt(&frame[..crc_off]);
    frame[crc_off] = (crc >> 8) as u8;
    frame[crc_off + 1] = (crc & 0xFF) as u8;
    let mut d = Demuxer::new();
    let (_, errors) = d.feed_slice(&frame);
    assert!(
        !errors.is_empty(),
        "expected StreamError for bad DEST flag, got none"
    );
    assert!(
        errors
            .iter()
            .any(|e| matches!(e, StreamError::KlipperBadSeqDest { .. })),
        "expected KlipperBadSeqDest variant, got {errors:?}"
    );
}
