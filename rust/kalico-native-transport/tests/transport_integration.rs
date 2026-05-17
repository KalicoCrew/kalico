//! Integration tests for `KalicoNativeTransport`:
//! * Bootstrap-ABI handshake (success).
//! * Schema-hash mismatch -> Faulted -> subsequent calls fail.
//! * Reset-epoch transition: in-flight call returns `TransportError::Reset`.

use std::time::Duration;

use kalico_native_transport::{
    bootstrap::encode_identify_response, ConnectionState, EpochChange, IdentifyResponse,
    KalicoNativeTransport, MessageKind, MockConnection, Transport, TransportError,
};
use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::encode_message_header;

fn host_schema_hash() -> [u8; 32] {
    let mut h = [0u8; 32];
    for (i, b) in h.iter_mut().enumerate() {
        *b = i as u8;
    }
    h
}

fn make_identify_response_frame(correlation_id: u32, schema_hash: [u8; 32], reset_epoch: u32) -> Vec<u8> {
    let resp = IdentifyResponse {
        proto_version: 0x01,
        firmware_ver: 0x0000_0001,
        build_hash: [0u8; 20],
        schema_hash,
        reset_epoch,
        capabilities: 0,
        mcu_serial: [0u8; 12],
    };
    let payload = encode_identify_response(correlation_id, &resp);
    encode_frame(CHANNEL_CONTROL, &payload)
}

fn make_status_event_frame(reset_epoch: u32) -> Vec<u8> {
    // v2 body: engine_status u8 | queue_depth u8 | current_segment_id u32
    //        | last_fault i32 | fault_detail u32 | reset_epoch u32
    //        | retired_through_segment_id u32 = 22 bytes.
    let mut body = Vec::with_capacity(22);
    body.push(0); // engine_status
    body.push(0); // queue_depth
    body.extend_from_slice(&0u32.to_le_bytes()); // current_segment_id
    body.extend_from_slice(&0i32.to_le_bytes()); // last_fault
    body.extend_from_slice(&0u32.to_le_bytes()); // fault_detail
    body.extend_from_slice(&reset_epoch.to_le_bytes()); // reset_epoch
    body.extend_from_slice(&0u32.to_le_bytes()); // retired_through_segment_id
    let mut payload = Vec::new();
    payload.extend_from_slice(&encode_message_header(MessageKind::StatusEvent, 1, 0));
    payload.extend_from_slice(&body);
    encode_frame(CHANNEL_EVENTS, &payload)
}

#[test]
fn bootstrap_handshake_success() {
    let host_hash = host_schema_hash();
    let (host_half, peer) = MockConnection::pair();
    let transport = KalicoNativeTransport::with_schema_hash(host_half, host_hash, 0x01);
    let epoch_rx = transport.epoch_change_subscribe();

    // Spawn a peer thread that waits for Identify, then responds.
    let peer_thread = {
        let peer = peer.clone();
        std::thread::spawn(move || {
            // Spin briefly until host emits Identify bytes.
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while peer.read_all_pending().is_empty() {
                assert!(std::time::Instant::now() < deadline, "peer never saw Identify");
                std::thread::sleep(Duration::from_millis(1));
            }
            // Use any plausible correlation_id; transport just stashes the epoch.
            let frame = make_identify_response_frame(1, host_hash, 0xCAFE_BABE);
            peer.write(&frame);
        })
    };

    let epoch = transport.identify(Duration::from_secs(2)).unwrap();
    assert_eq!(epoch, 0xCAFE_BABE);
    assert!(matches!(transport.state(), ConnectionState::Identified { reset_epoch } if reset_epoch == 0xCAFE_BABE));

    // Epoch subscriber sees Established.
    let evt = epoch_rx.recv_timeout(Duration::from_millis(50)).unwrap();
    assert!(matches!(evt, EpochChange::Established { reset_epoch } if reset_epoch == 0xCAFE_BABE));
    peer_thread.join().unwrap();
}

#[test]
fn schema_hash_mismatch_faults() {
    let host_hash = host_schema_hash();
    let mut wrong_hash = host_hash;
    wrong_hash[0] ^= 0xFF;
    let (host_half, peer) = MockConnection::pair();
    let transport = KalicoNativeTransport::with_schema_hash(host_half, host_hash, 0x01);

    let peer = peer.clone();
    let _ = std::thread::spawn(move || {
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while peer.read_all_pending().is_empty() {
            if std::time::Instant::now() >= deadline {
                return;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        let frame = make_identify_response_frame(1, wrong_hash, 0xDEAD);
        peer.write(&frame);
    });

    let err = transport.identify(Duration::from_secs(2)).unwrap_err();
    assert!(matches!(err, TransportError::Faulted(_)), "{err:?}");
    assert!(matches!(transport.state(), ConnectionState::Faulted(_)));

    // Subsequent call refuses.
    let err = transport.call(MessageKind::LoadCurve, &[], Duration::from_millis(50)).unwrap_err();
    assert!(matches!(err, TransportError::NotIdentified(_)));
}

#[test]
fn reset_epoch_transition_invalidates_inflight() {
    let host_hash = host_schema_hash();
    let (host_half, peer) = MockConnection::pair();
    let transport = std::sync::Arc::new(KalicoNativeTransport::with_schema_hash(
        host_half, host_hash, 0x01,
    ));
    let epoch_rx = transport.epoch_change_subscribe();

    // Bootstrap with epoch=1.
    {
        let peer = peer.clone();
        let _ = std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while peer.read_all_pending().is_empty() {
                if std::time::Instant::now() >= deadline {
                    return;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            peer.write(&make_identify_response_frame(1, host_hash, 1));
        });
    }
    let _ = transport.identify(Duration::from_secs(2)).unwrap();
    let _ = epoch_rx.recv_timeout(Duration::from_millis(50)).unwrap();

    // Now: peer emits a StatusEvent with epoch=2 BEFORE the in-flight call's
    // response. The transport should detect the change and notify pending
    // callers with TransportError::Reset.
    {
        let peer = peer.clone();
        let _ = std::thread::spawn(move || {
            // Wait for the host's outbound LoadCurve frame to land.
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while peer.read_all_pending().is_empty() {
                if std::time::Instant::now() >= deadline {
                    return;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            // Inject the new-epoch StatusEvent.
            peer.write(&make_status_event_frame(2));
        });
    }

    let err = transport
        .call(MessageKind::LoadCurve, &[0u8; 4], Duration::from_secs(2))
        .unwrap_err();
    assert!(matches!(err, TransportError::Reset), "{err:?}");

    let evt = epoch_rx.recv_timeout(Duration::from_millis(100)).unwrap();
    assert!(matches!(evt, EpochChange::Changed { old: 1, new: 2 }));

    // Transport is now Unidentified; further calls fail until re-identified.
    let err = transport
        .call(MessageKind::LoadCurve, &[0u8; 4], Duration::from_millis(50))
        .unwrap_err();
    assert!(matches!(err, TransportError::NotIdentified(_)));
}
