use super::*;
use std::time::Duration;

fn make_entry(
    call_id: u64,
    name: &str,
    seq: u64,
    deadline: Instant,
) -> (
    AwaitEntry,
    std::sync::mpsc::Receiver<Result<MessageParams, TransportError>>,
) {
    let (tx, rx) = std::sync::mpsc::sync_channel(1);
    (
        AwaitEntry {
            call_id,
            seq,
            expected_response_name: name.to_string(),
            completion: tx,
            submitted_at: Instant::now(),
            deadline,
            abandoned: false,
            sent_time_raw: 0.0,
        },
        rx,
    )
}

#[test]
fn fifo_match_finds_oldest() {
    let mut a = AwaitingResponse::default();
    let (e1, _r1) = make_entry(1, "rsp", 1, Instant::now() + Duration::from_secs(60));
    let (e2, _r2) = make_entry(2, "rsp", 2, Instant::now() + Duration::from_secs(60));
    a.push(e1).unwrap();
    a.push(e2).unwrap();
    assert_eq!(a.find_match("rsp"), Some(0));
}

#[test]
fn fifo_skips_abandoned() {
    let mut a = AwaitingResponse::default();
    let (e1, _r1) = make_entry(1, "rsp", 1, Instant::now() + Duration::from_secs(60));
    let (e2, _r2) = make_entry(2, "rsp", 2, Instant::now() + Duration::from_secs(60));
    a.push(e1).unwrap();
    a.push(e2).unwrap();
    a.mark_abandoned(1);
    assert_eq!(a.find_match("rsp"), Some(1));
}

#[test]
fn evict_expired_returns_past_deadline() {
    let mut a = AwaitingResponse::default();
    let past = Instant::now() - Duration::from_millis(100);
    let future = Instant::now() + Duration::from_secs(60);
    let (e1, _r1) = make_entry(1, "rsp", 1, past);
    let (e2, _r2) = make_entry(2, "rsp", 2, future);
    a.push(e1).unwrap();
    a.push(e2).unwrap();
    let evicted = a.evict_expired(Instant::now());
    assert_eq!(evicted.len(), 1);
    assert_eq!(evicted[0].call_id, 1);
    assert_eq!(a.len(), 1);
}
