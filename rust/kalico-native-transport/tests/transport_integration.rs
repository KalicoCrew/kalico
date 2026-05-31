//! Integration tests for `KalicoNativeTransport`:
//! * Bootstrap-ABI handshake (success).
//! * Schema-hash mismatch -> Faulted -> subsequent calls fail.

use std::time::Duration;

use kalico_native_transport::frame::{CHANNEL_CONTROL, encode_frame};
use kalico_native_transport::{
    ConnectionState, EpochChange, IdentifyResponse, KalicoNativeTransport, MessageKind,
    MockConnection, Transport, TransportError, bootstrap::encode_identify_response,
};

fn host_schema_hash() -> [u8; 32] {
    let mut h = [0u8; 32];
    for (i, b) in h.iter_mut().enumerate() {
        *b = i as u8;
    }
    h
}

fn make_identify_response_frame(
    correlation_id: u32,
    schema_hash: [u8; 32],
    reset_epoch: u32,
) -> Vec<u8> {
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
                assert!(
                    std::time::Instant::now() < deadline,
                    "peer never saw Identify"
                );
                std::thread::sleep(Duration::from_millis(1));
            }
            // Use any plausible correlation_id; transport just stashes the epoch.
            let frame = make_identify_response_frame(1, host_hash, 0xCAFE_BABE);
            peer.write(&frame);
        })
    };

    let epoch = transport.identify(Duration::from_secs(2)).unwrap();
    assert_eq!(epoch, 0xCAFE_BABE);
    assert!(
        matches!(transport.state(), ConnectionState::Identified { reset_epoch } if reset_epoch == 0xCAFE_BABE)
    );

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
    let err = transport
        .call(MessageKind::PushPieces, &[], Duration::from_millis(50))
        .unwrap_err();
    assert!(matches!(err, TransportError::NotIdentified(_)));
}
