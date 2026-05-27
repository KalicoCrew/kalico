use super::*;

#[test]
fn entry_holds_bytes_and_clocks() {
    let entry = PassthroughEntry::new(vec![0xAA, 0xBB], 100, 200, NotifyId::new(42));
    assert_eq!(entry.bytes(), &[0xAA, 0xBB]);
    assert_eq!(entry.min_clock(), 100);
    assert_eq!(entry.req_clock(), 200);
    assert_eq!(entry.notify_id(), NotifyId::new(42));
}

#[test]
fn notify_id_distinct() {
    let none = NotifyId::none();
    let id1 = NotifyId::new(1);
    let id2 = NotifyId::new(2);

    assert!(none.is_none());
    assert!(!id1.is_none());
    assert_ne!(id1, id2);
    assert_eq!(none.raw(), 0);
    assert_eq!(id1.raw(), 1);
}
