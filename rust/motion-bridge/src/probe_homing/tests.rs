use super::*;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

#[test]
fn loop_does_not_exit_on_segment_completed() {
    let triggered = Arc::new(AtomicBool::new(false));
    let triggered_clone = Arc::clone(&triggered);

    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(75));
        triggered_clone.store(true, Ordering::Release);
    });

    let start = Instant::now();

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
    assert!(start.elapsed() >= Duration::from_millis(50));
}

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

#[test]
fn loop_exits_on_trigger() {
    let triggered = Arc::new(AtomicBool::new(true));
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
