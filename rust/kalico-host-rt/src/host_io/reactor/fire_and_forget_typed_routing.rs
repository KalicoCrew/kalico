use super::*;
use crate::host_io::test_harness::ReactorHarness;

#[test]
fn fire_and_forget_typed_command_writes_payload_to_wire() {
    let mut h = ReactorHarness::new();
    let payload = vec![0x2A, 0x07, 0x11];

    h.submission_tx
        .send(ReactorCommand::FireAndForgetTyped {
            payload: payload.clone(),
        })
        .expect("submission_tx open");

    h.tick();

    let tx = h.tx_log();
    assert!(
        tx.windows(payload.len()).any(|w| w == payload.as_slice()),
        "payload {payload:?} should appear in tx log {tx:?}",
    );
    assert_eq!(h.unacked_depth(), 1);
}
