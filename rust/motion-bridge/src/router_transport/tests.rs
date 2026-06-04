use super::*;
use kalico_host_rt::clock::RealClock;

fn router() -> Arc<Mutex<PassthroughRouter>> {
    let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
    Arc::new(Mutex::new(PassthroughRouter::with_clock(clock)))
}

#[test]
fn no_parser_returns_parse_error() {
    let router = router();
    let (mcu, queue) = {
        let mut r = router.lock().unwrap();
        let h = r.claim_mcu("test");
        let q = r.alloc_command_queue(h).unwrap();
        (h, q)
    };
    let parser_slot: Arc<Mutex<Option<Arc<MsgProtoParser>>>> = Arc::new(Mutex::new(None));
    let t = RouterTransport::new(router, mcu, queue, parser_slot);

    match t.call(
        "kalico_load_curve",
        "kalico_load_curve_response",
        Duration::from_millis(10),
    ) {
        Err(TransportError::Parse(s)) => assert!(s.contains("not configured")),
        other => panic!("expected Parse error, got {other:?}"),
    }
}

#[test]
fn host_time_to_mcu_clock_helper_round_trips() {
    let router = router();
    let mcu = {
        let mut r = router.lock().unwrap();
        r.claim_mcu("clk")
    };
    {
        let mut r = router.lock().unwrap();
        r.set_clock_est(mcu, 1_000_000.0, 0.0, 0).unwrap();
    }
    let r = router.lock().unwrap();
    let got = r.host_time_to_mcu_clock(mcu, 1.5).unwrap();
    assert_eq!(got, 1_500_000);
}

#[test]
fn host_time_to_mcu_clock_returns_zero_when_unset() {
    let router = router();
    let mcu = {
        let mut r = router.lock().unwrap();
        r.claim_mcu("clk")
    };
    let r = router.lock().unwrap();
    assert_eq!(r.host_time_to_mcu_clock(mcu, 1.5).unwrap(), 0);
}
