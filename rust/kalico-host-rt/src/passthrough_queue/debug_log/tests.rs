use super::*;

#[test]
fn entries_are_stored_and_retrievable() {
    let mut log = DebugLog::new();
    log.record_sent(1, vec![0xAA], 1.0);
    log.record_sent(2, vec![0xBB], 2.0);
    log.record_received(10, vec![0xCC], 3.0);

    let (sent, received) = log.extract_old();
    assert_eq!(sent.len(), 2);
    assert_eq!(sent[0].seq, 1);
    assert_eq!(sent[0].bytes, vec![0xAA]);
    assert_eq!(sent[1].seq, 2);
    assert_eq!(received.len(), 1);
    assert_eq!(received[0].seq, 10);
}

#[test]
fn sent_queue_capped_at_100() {
    let mut log = DebugLog::new();
    for i in 0..150 {
        log.record_sent(i, vec![0x01], i as f64);
    }
    assert_eq!(log.sent_count(), DEBUG_QUEUE_SENT);

    let (sent, _) = log.extract_old();
    assert_eq!(sent.len(), DEBUG_QUEUE_SENT);
    assert_eq!(sent[0].seq, 50);
    assert_eq!(sent[99].seq, 149);
}

#[test]
fn received_queue_capped_at_100() {
    let mut log = DebugLog::new();
    for i in 0..150 {
        log.record_received(i, vec![0x02], i as f64);
    }
    assert_eq!(log.received_count(), DEBUG_QUEUE_RECEIVE);

    let (_, received) = log.extract_old();
    assert_eq!(received.len(), DEBUG_QUEUE_RECEIVE);
    assert_eq!(received[0].seq, 50);
}

#[test]
fn extract_old_drains_buffers() {
    let mut log = DebugLog::new();
    log.record_sent(1, vec![0x01], 1.0);
    log.record_received(2, vec![0x02], 2.0);

    let _ = log.extract_old();
    assert_eq!(log.sent_count(), 0);
    assert_eq!(log.received_count(), 0);

    let (sent, received) = log.extract_old();
    assert!(sent.is_empty());
    assert!(received.is_empty());
}
