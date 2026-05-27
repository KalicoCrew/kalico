use super::*;

#[test]
fn default_rto_is_min_rto() {
    let e = RttEstimator::default();
    assert_eq!(e.current_rto(), MIN_RTO);
}

#[test]
fn first_sample_initializes() {
    let mut e = RttEstimator::default();
    // Use a sample above MIN_RTO so the clamp doesn't mask the
    // RFC 6298 formula being tested.
    e.update(Duration::from_millis(200));
    // RTO = SRTT + max(G, K * RTTVAR) = 200 + max(1, 4*100) = 600ms.
    assert!(e.current_rto() >= Duration::from_millis(600));
}

#[test]
fn backoff_doubles_with_clamp() {
    let mut e = RttEstimator::default();
    e.backoff();
    assert!(e.current_rto() >= Duration::from_millis(50));
    for _ in 0..20 {
        e.backoff();
    }
    assert_eq!(e.current_rto(), MAX_RTO);
}
