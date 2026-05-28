use super::*;

#[test]
fn fresh_pool_has_full_capacity() {
    let p = SlotPool::new(CURVE_POOL_N);
    assert_eq!(p.free_count(), CURVE_POOL_N);
    assert_eq!(p.in_flight_count(), 0);
}

#[test]
fn alloc_advances_generation_per_slot() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    let (s0, g0) = p.try_alloc().unwrap();
    assert_eq!(g0, 1, "first alloc is gen=1");
    // Free and re-alloc — same slot should bump to gen=2.
    p.release(s0);
    let mut found = None;
    // The free queue is FIFO so after releasing s0 the next alloc may
    // not be s0. Drain until we get it back, advancing generations on
    // intervening slots.
    for _ in 0..CURVE_POOL_N + 1 {
        let (s, g) = p.try_alloc().unwrap();
        if s == s0 {
            found = Some(g);
            break;
        }
    }
    assert_eq!(found, Some(2), "second alloc of same slot must be gen=2");
}

#[test]
fn pool_exhausts_at_capacity() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    for _ in 0..CURVE_POOL_N {
        assert!(p.try_alloc().is_some());
    }
    assert!(p.try_alloc().is_none(), "exhausted pool must return None");
    assert_eq!(p.in_flight_count(), CURVE_POOL_N);
}

#[test]
fn release_is_idempotent() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    let (s, _) = p.try_alloc().unwrap();
    p.release(s);
    p.release(s); // duplicate
    assert_eq!(p.in_flight_count(), 0);
    assert_eq!(p.free_count(), CURVE_POOL_N);
}

#[test]
fn retire_through_segment_releases_eligible_slots() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    let (s1, _) = p.try_alloc().unwrap();
    p.register_segment(s1, 1);
    let (s2, _) = p.try_alloc().unwrap();
    p.register_segment(s2, 2);
    let (s3, _) = p.try_alloc().unwrap();
    p.register_segment(s3, 3);

    assert_eq!(p.in_flight_count(), 3);

    // MCU reports "everything up to seg 2 retired."
    let n = p.retire_through_segment(2);
    assert_eq!(n, 2, "should release 2 slots");
    assert_eq!(p.in_flight_count(), 1);
    // Then a higher retirement releases the rest.
    let n = p.retire_through_segment(10);
    assert_eq!(n, 1);
    assert_eq!(p.in_flight_count(), 0);
}

#[test]
fn retire_through_lower_id_is_noop() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    let (s, _) = p.try_alloc().unwrap();
    p.register_segment(s, 100);
    // Stale event from earlier in the print.
    assert_eq!(p.retire_through_segment(50), 0);
    assert_eq!(p.in_flight_count(), 1);
}

#[test]
fn alloc_without_register_segment_skips_retirement() {
    // An alloc that hasn't yet been pushed (race window) must not be
    // released by a segment-id retirement event — the segment_id is
    // unknown, so the slot stays in-flight until either explicitly
    // released or its eventual segment-id retires.
    let mut p = SlotPool::new(CURVE_POOL_N);
    let _ = p.try_alloc().unwrap();
    // No register_segment call yet.
    assert_eq!(p.retire_through_segment(u32::MAX), 0);
    assert_eq!(p.in_flight_count(), 1);
}

/// B.1 design memo §5: on `push_segment` failure mid-burst, the dispatch
/// loop calls `release(slot)` for every slot in the failed chunk's
/// `allocated_slots`. Verify that this defensive cleanup returns the
/// pool to its prior state.
#[test]
fn release_after_failed_push_does_not_leak() {
    let mut p = SlotPool::new(CURVE_POOL_N);
    let mut allocated: Vec<u16> = Vec::new();
    for i in 0..5 {
        let (s, _) = p.try_alloc().expect("alloc");
        p.register_segment(s, 100 + i);
        allocated.push(s);
    }
    assert_eq!(p.in_flight_count(), 5);
    assert_eq!(p.free_count(), CURVE_POOL_N - 5);

    // Simulate push_segment failure: release every allocated slot.
    for s in &allocated {
        p.release(*s);
    }
    assert_eq!(p.in_flight_count(), 0);
    assert_eq!(p.free_count(), CURVE_POOL_N);
}

#[test]
fn many_alloc_release_cycles_dont_leak() {
    // Cycle through more than CURVE_POOL_N allocations to verify the
    // pool stays balanced — this is the regression for the original
    // u16 rolling-counter bug (would have errored at slot 64).
    let mut p = SlotPool::new(CURVE_POOL_N);
    for i in 0..(CURVE_POOL_N * 5) {
        let (s, _) = p
            .try_alloc()
            .unwrap_or_else(|| panic!("alloc {i} failed — pool starved"));
        p.register_segment(s, i as u32);
        // Retire immediately (simulates a flushed pipeline).
        p.retire_through_segment(i as u32);
    }
    assert_eq!(p.free_count(), CURVE_POOL_N);
}
