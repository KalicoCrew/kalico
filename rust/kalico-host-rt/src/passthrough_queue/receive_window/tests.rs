use super::*;

#[test]
fn window_default_starts_empty() {
    let w = ReceiveWindow::new();
    assert!(w.can_emit());
    assert_eq!(w.send_seq(), 1);
    assert_eq!(w.receive_seq(), 1);
}

#[test]
fn emit_check_includes_message_max_overhead() {
    // Window of exactly MESSAGE_MAX: one emit should be possible
    let mut w = ReceiveWindow::with_limits(100, MESSAGE_MAX);
    assert!(w.can_emit());

    // After emitting 1 byte, need_ack_bytes=1, check is 1+64 > 64 → fail
    w.record_emit(1);
    assert!(!w.can_emit());
}

#[test]
fn pending_blocks_gate() {
    let mut w = ReceiveWindow::with_limits(2, 10_000);
    assert!(w.can_emit());

    w.record_emit(10);
    assert!(w.can_emit()); // send_seq=2, receive_seq=1, diff=1 < 2

    w.record_emit(10);
    assert!(!w.can_emit()); // send_seq=3, receive_seq=1, diff=2 >= 2

    w.record_ack(10);
    assert!(w.can_emit()); // receive_seq=2, diff=1 < 2
}

#[test]
fn last_ack_bytes_carry_when_acks_lag() {
    // Window is large enough for one message + overhead, but
    // last_ack_bytes carry tips it over.
    let mut w = ReceiveWindow::with_limits(100, MESSAGE_MAX + 20);
    w.record_emit(20);
    // need_ack_bytes=20, last_ack_bytes=0, last_ack_seq=0 < receive_seq=1
    // check: 20 + 64 + 0 = 84 <= 84 ✓
    assert!(w.can_emit());

    w.record_emit(10);
    // need_ack_bytes=30, last_ack_bytes=20, last_ack_seq=1 >= receive_seq=1
    // carry=0 → 30+64+0=94 > 84 → blocked by bytes
    assert!(!w.can_emit());

    // Ack brings receive_seq to 2, last_ack_seq=1 < 2 → carry kicks in
    w.record_ack(20);
    // need_ack_bytes=10, last_ack_bytes=20, last_ack_seq=1 < receive_seq=2
    // check: 10+64+20=94 > 84 → still blocked
    assert!(!w.can_emit());

    // Another ack
    w.record_ack(10);
    // need_ack_bytes=0, last_ack_bytes=20, last_ack_seq=1 < receive_seq=3
    // check: 0+64+20=84 <= 84 ✓
    assert!(w.can_emit());
}
