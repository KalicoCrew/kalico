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
