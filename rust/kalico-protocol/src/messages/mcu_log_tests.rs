use super::*;

#[test]
fn mcu_log_encode_decode_round_trip() {
    use crate::codec::{Cursor, Decode, Encode};

    let orig = McuLog {
        mcu_tick: 0x0001_2345_6789_ABCD_u64,
        level: 2,
        subsystem: 1,
        event: 0x0003,
        code: 0xFECC, // sign-wrapped -308 (PieceStartInPast)
        seq: 7,
        args: [0xDEAD_BEEF, 0x0000_0042],
    };

    let mut buf = Vec::new();
    orig.encode(&mut buf);
    // Fixed layout: 8+1+1+2+2+2+4+4 = 24 bytes
    assert_eq!(buf.len(), 24);

    let mut c = Cursor::new(&buf);
    let decoded = McuLog::decode_from(&mut c).expect("decode must succeed");
    assert_eq!(decoded, orig);
    assert_eq!(c.remaining(), 0);
}

#[test]
fn mcu_log_is_event_kind() {
    assert!(MessageKind::McuLog.is_event());
    assert_eq!(MessageKind::McuLog.as_u16(), 0x0084);
    assert_eq!(MessageKind::from_u16(0x0084), Some(MessageKind::McuLog));
    assert_eq!(
        MessageKind::from_u16(0x0085),
        Some(MessageKind::EndstopTrip)
    );
    assert_eq!(MessageKind::from_u16(0x0086), None);
}

#[test]
fn mcu_log_is_schema_validated() {
    assert!(MessageKind::McuLog.is_schema_validated());
}

#[test]
fn mcu_log_zero_args_round_trip() {
    use crate::codec::{Cursor, Decode, Encode};
    let orig = McuLog {
        mcu_tick: 0,
        level: 0,
        subsystem: 0,
        event: 0,
        code: 0,
        seq: 0,
        args: [0, 0],
    };
    let mut buf = Vec::new();
    orig.encode(&mut buf);
    let decoded = McuLog::decode_from(&mut Cursor::new(&buf)).unwrap();
    assert_eq!(decoded, orig);
}

#[test]
fn mcu_log_decode_rejects_truncated_body() {
    use crate::codec::{Cursor, Decode, DecodeError, Encode};

    let orig = McuLog {
        mcu_tick: 0x0001_2345_6789_ABCD_u64,
        level: 2,
        subsystem: 1,
        event: 0x0003,
        code: 0xFECC,
        seq: 7,
        args: [0xDEAD_BEEF, 0x0000_0042],
    };
    let mut buf = Vec::new();
    orig.encode(&mut buf);
    assert_eq!(buf.len(), 24, "sanity: full encode must be 24 bytes");
    let short = &buf[..23];
    let mut c = Cursor::new(short);
    match McuLog::decode_from(&mut c) {
        Err(DecodeError::UnexpectedEof) => {}
        other => panic!("expected UnexpectedEof on 23-byte body, got {other:?}"),
    }
}

#[test]
fn mcu_log_byte_layout_is_little_endian() {
    use crate::codec::Encode;
    let msg = McuLog {
        mcu_tick: 0x0102_0304_0506_0708_u64,
        level: 0xAB,
        subsystem: 0xCD,
        event: 0x1234,
        code: 0x5678,
        seq: 0x9ABC,
        args: [0xDEAD_BEEF, 0xCAFE_1234],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 24);
    // mcu_tick LE (bytes 0..8)
    assert_eq!(&buf[0..8], &0x0102_0304_0506_0708_u64.to_le_bytes());
    // level (byte 8)
    assert_eq!(buf[8], 0xAB);
    // subsystem (byte 9)
    assert_eq!(buf[9], 0xCD);
    // event LE (bytes 10..12)
    assert_eq!(&buf[10..12], &0x1234_u16.to_le_bytes());
    // code LE (bytes 12..14)
    assert_eq!(&buf[12..14], &0x5678_u16.to_le_bytes());
    // seq LE (bytes 14..16)
    assert_eq!(&buf[14..16], &0x9ABC_u16.to_le_bytes());
    // arg0 LE (bytes 16..20)
    assert_eq!(&buf[16..20], &0xDEAD_BEEF_u32.to_le_bytes());
    // arg1 LE (bytes 20..24)
    assert_eq!(&buf[20..24], &0xCAFE_1234_u32.to_le_bytes());
}
