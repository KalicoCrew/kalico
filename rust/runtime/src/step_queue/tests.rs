use super::*;

#[test]
fn clear_empties_queue() {
    let mut q = StepQueue::new();
    q.tail = 5;
    q.head = 2;
    assert_ne!(q.tail, q.head);
    q.clear();
    assert_eq!(q.tail, 0);
    assert_eq!(q.head, 0);
}
