use super::*;
use std::sync::Arc;
use std::thread;

#[test]
fn acquire_blocking_returns_immediately_when_available() {
    let c = CreditCounter::new(4);
    assert!(c.acquire_blocking(Duration::from_millis(10)).is_ok());
    assert_eq!(c.available(), 3);
}

#[test]
fn acquire_blocking_times_out_when_no_credit() {
    let c = CreditCounter::new(1);
    assert!(c.try_acquire().is_some());
    assert_eq!(c.available(), 0);
    let start = Instant::now();
    let result = c.acquire_blocking(Duration::from_millis(20));
    let elapsed = start.elapsed();
    assert!(result.is_err());
    assert!(
        elapsed >= Duration::from_millis(15),
        "blocking acquire should wait nearly the full timeout, got {:?}",
        elapsed
    );
}

#[test]
fn release_unblocks_acquire_blocking() {
    let c = Arc::new(CreditCounter::new(1));
    assert!(c.try_acquire().is_some());
    assert_eq!(c.available(), 0);

    let c2 = Arc::clone(&c);
    let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
    // Give the waiter time to enter wait_timeout.
    thread::sleep(Duration::from_millis(50));
    c.release();
    let result = handle.join().unwrap();
    assert!(result.is_ok(), "release must unblock waiter");
    // The waiter consumed the credit, so available is back to 0.
    assert_eq!(c.available(), 0);
}

#[test]
fn on_credit_freed_unblocks_acquire_blocking() {
    let c = Arc::new(CreditCounter::new(7));
    // Drain to zero.
    for _ in 0..7 {
        assert!(c.try_acquire().is_some());
    }
    assert_eq!(c.available(), 0);

    let c2 = Arc::clone(&c);
    let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
    thread::sleep(Duration::from_millis(50));
    // MCU reports 3 slots free.
    c.on_credit_freed(3);
    assert_eq!(c.credit_freed_events(), 1);
    let result = handle.join().unwrap();
    assert!(result.is_ok());
    assert_eq!(c.available(), 2);
}

#[test]
fn on_epoch_change_unblocks_acquire_blocking() {
    let c = Arc::new(CreditCounter::new(7));
    for _ in 0..7 {
        assert!(c.try_acquire().is_some());
    }
    assert_eq!(c.available(), 0);

    let c2 = Arc::clone(&c);
    let handle = thread::spawn(move || c2.acquire_blocking(Duration::from_secs(2)));
    thread::sleep(Duration::from_millis(50));
    c.on_epoch_change(1);
    let result = handle.join().unwrap();
    assert!(result.is_ok());
    assert_eq!(c.available(), 6);
    assert_eq!(c.credit_epoch(), 1);
}

#[test]
fn multiple_waiters_all_get_served_in_order() {
    // 8 waiters, capacity 4; on_credit_freed grants 4 slots. Each
    // waiter consumes 1 — only 4 of the 8 should succeed.
    let c = Arc::new(CreditCounter::new(4));
    for _ in 0..4 {
        assert!(c.try_acquire().is_some());
    }

    let mut handles = Vec::new();
    for _ in 0..8 {
        let c2 = Arc::clone(&c);
        handles.push(thread::spawn(move || {
            c2.acquire_blocking(Duration::from_millis(200))
        }));
    }
    thread::sleep(Duration::from_millis(50));
    c.on_credit_freed(4);

    let mut ok = 0;
    let mut err = 0;
    for h in handles {
        match h.join().unwrap() {
            Ok(()) => ok += 1,
            Err(()) => err += 1,
        }
    }
    assert_eq!(ok, 4, "exactly 4 waiters should get credit");
    assert_eq!(err, 4, "the other 4 must time out");
}
