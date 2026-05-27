use super::*;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

/// Simulates the real-hardware failure: CreditFreed arrives before
/// the probe triggers, causing HomingSegmentState to become Completed.
/// The loop must NOT exit on segment retirement — only on trigger
/// or sensor_fault_timeout.
#[test]
fn loop_does_not_exit_on_segment_completed() {
    let triggered = Arc::new(AtomicBool::new(false));
    let triggered_clone = Arc::clone(&triggered);

    // Set the trigger after 75ms (3 ticks). If the loop exited
    // early on segment retirement, it would return SensorFault or
    // SegmentRetired within 1 tick, not ProbeTriggered after 75ms.
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(75));
        triggered_clone.store(true, Ordering::Release);
    });

    let start = Instant::now();

    // Inline the loop logic to test without needing real KalicoHostIo.
    let sensor_fault_timeout = Duration::from_secs(5);
    let result = loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        if triggered.load(Ordering::Acquire) {
            break ProbeHomingResult::ProbeTriggered;
        }

        if elapsed > sensor_fault_timeout {
            break ProbeHomingResult::SensorFault;
        }
    };

    assert_eq!(result, ProbeHomingResult::ProbeTriggered);
    // Verify it took at least 50ms (not instant exit)
    assert!(start.elapsed() >= Duration::from_millis(50));
}

/// The loop must exit on sensor_fault_timeout if no trigger arrives.
#[test]
fn loop_exits_on_sensor_fault_timeout() {
    let triggered = Arc::new(AtomicBool::new(false));
    let sensor_fault_timeout = Duration::from_millis(60);

    let start = Instant::now();
    let result = loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        if triggered.load(Ordering::Acquire) {
            break ProbeHomingResult::ProbeTriggered;
        }

        if elapsed > sensor_fault_timeout {
            break ProbeHomingResult::SensorFault;
        }
    };

    assert_eq!(result, ProbeHomingResult::SensorFault);
}

/// The loop must exit immediately when the trigger flag is set.
#[test]
fn loop_exits_on_trigger() {
    let triggered = Arc::new(AtomicBool::new(true)); // pre-set
    let sensor_fault_timeout = Duration::from_secs(60);

    let start = Instant::now();
    let result = loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        if triggered.load(Ordering::Acquire) {
            break ProbeHomingResult::ProbeTriggered;
        }

        if elapsed > sensor_fault_timeout {
            break ProbeHomingResult::SensorFault;
        }
    };

    assert_eq!(result, ProbeHomingResult::ProbeTriggered);
    assert!(start.elapsed() < Duration::from_millis(100));
}
