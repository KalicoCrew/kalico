use super::*;
use std::sync::Arc;
use std::thread;

/// Pool has free slots — `alloc_blocking` must return without waiting.
#[test]
fn alloc_blocking_immediate_when_free() {
    let pool = SharedSlotPool::new(4);
    let result = pool.alloc_blocking(Duration::from_millis(100));
    assert!(result.is_some(), "expected Some, got None");
    assert_eq!(pool.in_flight_count(), 1);
    assert_eq!(pool.free_count(), 3);
}

/// Exhaust pool, then retire a segment from another thread — the blocked
/// `alloc_blocking` call must wake and succeed.
#[test]
fn alloc_blocking_wakes_on_retire() {
    let pool = Arc::new(SharedSlotPool::new(4));

    // Exhaust all slots and register them under segment ids.
    let mut slots = Vec::new();
    for i in 0..4u32 {
        let (s, _) = pool.try_alloc().expect("alloc");
        pool.register_segment(s, i);
        slots.push(s);
    }
    assert_eq!(pool.free_count(), 0);

    // Spawn a thread that blocks waiting for a slot.
    let pool2 = Arc::clone(&pool);
    let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(5)));

    // Give the waiter time to park in wait_timeout.
    thread::sleep(Duration::from_millis(50));

    // Retire segment 0, freeing its slot.
    pool.retire_through_segment(0);

    let result = handle.join().expect("thread panicked");
    assert!(result.is_some(), "alloc_blocking must succeed after retire");
}

/// Exhaust pool and call `alloc_blocking` with a short timeout — must
/// return `None` and the elapsed time must be at least 40 ms.
#[test]
fn alloc_blocking_times_out() {
    let pool = SharedSlotPool::new(4);
    for _ in 0..4 {
        pool.try_alloc().expect("alloc");
    }
    assert_eq!(pool.free_count(), 0);

    let start = Instant::now();
    let result = pool.alloc_blocking(Duration::from_millis(50));
    let elapsed = start.elapsed();

    assert!(result.is_none(), "expected timeout, got {:?}", result);
    assert!(
        elapsed >= Duration::from_millis(40),
        "should have waited close to the full timeout, elapsed={elapsed:?}"
    );
}

/// Exhaust pool, spawn a blocking-alloc thread, then call `release` from
/// the main thread — the waiter must wake and succeed.
#[test]
fn release_wakes_alloc_blocking() {
    let pool = Arc::new(SharedSlotPool::new(4));

    // Exhaust all slots.
    let mut slots = Vec::new();
    for _ in 0..4 {
        let (s, _) = pool.try_alloc().expect("alloc");
        slots.push(s);
    }
    assert_eq!(pool.free_count(), 0);

    // Spawn a thread that blocks waiting for a slot.
    let pool2 = Arc::clone(&pool);
    let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(5)));

    // Give the waiter time to park.
    thread::sleep(Duration::from_millis(50));

    // Release one slot.
    pool.release(slots[0]);

    let result = handle.join().expect("thread panicked");
    assert!(
        result.is_some(),
        "alloc_blocking must succeed after release"
    );
}

#[test]
fn close_wakes_blocked_alloc() {
    let pool = Arc::new(SharedSlotPool::new(1));
    pool.try_alloc().unwrap();
    assert_eq!(pool.free_count(), 0);

    let pool2 = Arc::clone(&pool);
    let handle = thread::spawn(move || pool2.alloc_blocking(Duration::from_secs(10)));
    thread::sleep(Duration::from_millis(50));

    let start = Instant::now();
    pool.close();
    let result = handle.join().expect("thread panicked");
    assert!(result.is_none(), "closed pool must return None");
    assert!(
        start.elapsed() < Duration::from_secs(1),
        "close must wake waiter promptly, not wait for timeout"
    );
}

#[test]
fn try_alloc_returns_none_when_closed() {
    let pool = SharedSlotPool::new(4);
    assert!(pool.try_alloc().is_some());
    pool.close();
    assert!(pool.try_alloc().is_none());
    assert!(pool.is_closed());
}
