use std::sync::Arc;
use std::time::Instant;

use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_host_rt::clock::{instant_to_f64, Clock, RealClock};
use kalico_host_rt::passthrough_queue::PassthroughRouter;

/// Seed one EC MCU record the same way `init_planner` does, then project
/// `query_ns` (a RAW nanosecond sample) back to the Instant epoch and assert
/// it matches `instant_to_f64(Instant::now())` within 1 ms.
///
/// The test pins the invariant that the endpoint's `CLOCK_MONOTONIC_RAW`
/// nanoseconds and the host's `instant_to_f64` are in the same domain.
/// A 1 ms tolerance covers sample skew between the two independent clock reads;
/// the NTP-slew drift that CLOCK_MONOTONIC would accumulate during an idle
/// session is O(8 ms) — well outside this bound — so the test catches a
/// domain regression.
#[test]
fn ec_seed_projection_same_domain_within_1ms() {
    let real_clock = Arc::new(RealClock) as Arc<dyn Clock + Send + Sync>;
    let mut router = PassthroughRouter::with_clock(real_clock);
    let mcu = router.claim_mcu("ec_mcu");

    let seed_instant = Instant::now();
    let seed_ns = monotonic_ns();

    router
        .set_clock_est_from_sample(mcu, 1_000_000_000.0_f64, seed_instant, seed_ns)
        .unwrap();

    let query_ns = monotonic_ns();
    let expected_host = instant_to_f64(Instant::now());

    // Invert host_time_to_mcu_clock: host_t = clock_offset + (query_ns - last_clock) / freq
    let seed_offset = instant_to_f64(seed_instant);
    #[allow(clippy::cast_precision_loss)]
    let projected_host = seed_offset + (query_ns as f64 - seed_ns as f64) / 1_000_000_000.0_f64;

    let diff_secs = (projected_host - expected_host).abs();
    assert!(
        diff_secs < 0.001,
        "EC projection differs from host Instant by {:.6} s (> 1 ms) — \
         clocks are in different domains; expected both on CLOCK_MONOTONIC_RAW",
        diff_secs
    );
}

/// `monotonic_ns` must advance (sanity check for the RAW clock being live).
#[test]
fn monotonic_ns_advances() {
    let t0 = monotonic_ns();
    std::thread::sleep(std::time::Duration::from_millis(2));
    let t1 = monotonic_ns();
    assert!(t1 > t0, "monotonic_ns must advance: t0={t0} t1={t1}");
}
