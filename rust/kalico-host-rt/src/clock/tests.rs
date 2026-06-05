use super::*;

#[test]
fn mock_clock_advances_monotonically() {
    let c = MockClock::new();
    let t0 = c.now();
    c.advance(Duration::from_millis(100));
    let t1 = c.now();
    assert_eq!(t1 - t0, Duration::from_millis(100));
}

#[test]
fn mock_clock_can_be_arc_dyn() {
    let c: Arc<dyn Clock> = MockClock::new();
    let _ = c.now();
}

#[test]
fn real_clock_increases() {
    let c = RealClock;
    let t0 = c.now();
    std::thread::sleep(Duration::from_millis(1));
    let t1 = c.now();
    assert!(t1 > t0);
}

#[test]
fn monotonic_raw_secs_is_non_negative() {
    let t = monotonic_raw_secs();
    assert!(t >= 0.0, "monotonic_raw_secs must be non-negative, got {t}");
}

#[test]
fn monotonic_raw_secs_advances() {
    let t0 = monotonic_raw_secs();
    std::thread::sleep(Duration::from_millis(2));
    let t1 = monotonic_raw_secs();
    assert!(
        t1 > t0,
        "monotonic_raw_secs must increase over time: t0={t0} t1={t1}"
    );
}
