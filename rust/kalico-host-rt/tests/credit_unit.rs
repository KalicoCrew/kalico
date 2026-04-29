//! Phase 10 Task 10.4 unit tests: per-MCU `CreditCounter`.

use kalico_host_rt::credit::CreditCounter;

#[test]
fn fresh_counter_starts_at_capacity() {
    let c = CreditCounter::new(8);
    assert_eq!(c.available(), 8);
    assert_eq!(c.capacity(), 8);
}

#[test]
fn try_acquire_decrements() {
    let c = CreditCounter::new(2);
    c.try_acquire().expect("first acquire");
    assert_eq!(c.available(), 1);
    c.try_acquire().expect("second acquire");
    assert_eq!(c.available(), 0);
    assert!(c.try_acquire().is_none(), "third acquire must fail");
}

#[test]
fn release_rolls_back_failed_push() {
    let c = CreditCounter::new(4);
    c.try_acquire().unwrap();
    c.try_acquire().unwrap();
    assert_eq!(c.available(), 2);
    c.release();
    assert_eq!(c.available(), 3);
}

#[test]
fn release_clamped_at_capacity() {
    // Adversarial: release without prior acquire must NOT exceed
    // capacity, otherwise concurrent on_credit_freed events could push
    // available past the queue depth.
    let c = CreditCounter::new(2);
    c.release();
    c.release();
    c.release();
    assert_eq!(c.available(), 2);
}

#[test]
fn on_credit_freed_snaps_to_reported_value() {
    let c = CreditCounter::new(8);
    c.try_acquire().unwrap();
    c.try_acquire().unwrap();
    c.try_acquire().unwrap();
    assert_eq!(c.available(), 5);
    // MCU reports 6 free slots.
    c.on_credit_freed(6);
    assert_eq!(c.available(), 6);
}

#[test]
fn on_credit_freed_clamps_to_capacity() {
    let c = CreditCounter::new(4);
    c.on_credit_freed(255);
    assert_eq!(c.available(), 4, "must clamp to capacity");
}

#[test]
fn on_epoch_change_resets_to_capacity_and_records_epoch() {
    let c = CreditCounter::new(8);
    c.try_acquire().unwrap();
    c.try_acquire().unwrap();
    assert_eq!(c.available(), 6);
    c.on_epoch_change(7);
    assert_eq!(c.available(), 8);
    assert_eq!(c.credit_epoch(), 7);
}
