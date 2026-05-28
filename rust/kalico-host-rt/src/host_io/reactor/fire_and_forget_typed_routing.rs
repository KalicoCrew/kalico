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

    // The payload must have been wrapped into a frame and written to the
    // wire. We don't recompute the exact frame bytes here (that's the
    // wire layer's contract, exercised elsewhere) — just confirm the
    // payload bytes appear contiguously in the tx log.
    let tx = h.tx_log();
    assert!(
        tx.windows(payload.len()).any(|w| w == payload.as_slice()),
        "payload {payload:?} should appear in tx log {tx:?}",
    );
    // And the unacked window must have grown by exactly one entry (the
    // typed fire-and-forget frame is wire-tracked for NAK/RTO).
    assert_eq!(h.unacked_depth(), 1);
}
