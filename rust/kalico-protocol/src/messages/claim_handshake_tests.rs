use super::roundtrip;
use super::*;

#[test]
fn claim_handshake_reply_roundtrips_ok_slave() {
    let reply = ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 1,
            state: SlaveState::Ok,
            fault_code: 0,
        }],
    };
    let decoded = roundtrip(&reply);
    assert_eq!(decoded.slave_statuses.len(), 1);
    assert_eq!(decoded.slave_statuses[0].slave_idx, 1);
    assert_eq!(decoded.slave_statuses[0].state, SlaveState::Ok);
    assert_eq!(decoded.slave_statuses[0].fault_code, 0);
}

#[test]
fn claim_handshake_reply_roundtrips_fault_slave() {
    let reply = ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 1,
            state: SlaveState::Fault,
            fault_code: 0x0102,
        }],
    };
    let decoded = roundtrip(&reply);
    assert_eq!(decoded.slave_statuses[0].state, SlaveState::Fault);
    assert_eq!(decoded.slave_statuses[0].fault_code, 0x0102);
}

#[test]
fn unknown_slave_state_byte_is_hard_error() {
    // state=0xFF is not defined; must reject, not default-to-ok.
    let mut buf = Vec::new();
    buf.push(1u8); // slave_count = 1
    buf.push(1u8); // slave_idx = 1
    buf.push(0xFFu8); // state = unknown
    buf.extend_from_slice(&0u16.to_le_bytes()); // fault_code = 0
    let result = ClaimHandshakeReply::decode(&buf);
    assert!(result.is_err(), "unknown state byte must be a hard error");
}

#[test]
fn empty_slave_list_is_hard_error() {
    let mut buf = Vec::new();
    buf.push(0u8); // slave_count = 0 — missing status list
    let result = ClaimHandshakeReply::decode(&buf);
    assert!(
        result.is_err(),
        "empty slave status list must be a hard error"
    );
}

#[test]
fn message_kind_claim_handshake_roundtrips() {
    // Both kinds must survive a from_u16(as_u16()) round-trip.
    let raw_request = MessageKind::ClaimHandshake.as_u16();
    assert_eq!(
        MessageKind::from_u16(raw_request),
        Some(MessageKind::ClaimHandshake)
    );
    let raw_reply = MessageKind::ClaimHandshakeReply.as_u16();
    assert_eq!(
        MessageKind::from_u16(raw_reply),
        Some(MessageKind::ClaimHandshakeReply)
    );
    // Routing contract: neither kind is an unsolicited event — the host
    // dispatcher must route both through the correlation-id pending map.
    assert!(
        !MessageKind::ClaimHandshake.is_event(),
        "ClaimHandshake must not be classified as an event"
    );
    assert!(
        !MessageKind::ClaimHandshakeReply.is_event(),
        "ClaimHandshakeReply must not be classified as an event"
    );
}
