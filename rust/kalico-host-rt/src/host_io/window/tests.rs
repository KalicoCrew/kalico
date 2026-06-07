use super::*;

fn entry(seq: u64) -> UnackedEntry {
    UnackedEntry {
        seq,
        frame_bytes: vec![],
        sent_at: Instant::now(),
        retry_count: 0,
    }
}

#[test]
fn pop_acked_strict_less_than() {
    let mut w = UnackedWindow::default();
    w.push(entry(1));
    w.push(entry(2));
    w.push(entry(3));
    let popped = w.pop_acked(2);
    assert_eq!(popped.len(), 1);
    assert_eq!(popped[0].seq, 1);
    assert_eq!(w.len(), 2);
    assert_eq!(w.front().unwrap().seq, 2);
}
