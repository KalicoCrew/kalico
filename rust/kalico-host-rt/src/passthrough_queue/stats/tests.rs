use super::*;

#[test]
fn default_stats_are_zero() {
    let s = PassthroughStats::default();
    assert_eq!(s.bytes_write, 0);
    assert_eq!(s.bytes_read, 0);
    assert_eq!(s.send_seq, 0);
}

#[test]
fn counters_snapshot_round_trips() {
    let mut c = StatsCounters::new();
    c.bytes_write = 100;
    c.bytes_read = 50;
    c.send_seq = 42;

    let snap = c.snapshot();
    assert_eq!(snap.bytes_write, 100);
    assert_eq!(snap.bytes_read, 50);
    assert_eq!(snap.send_seq, 42);
}
