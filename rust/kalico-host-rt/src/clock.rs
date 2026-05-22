//! Time abstraction. Production wires `RealClock`; tests wire `MockClock`.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub trait Clock: Send + Sync {
    fn now(&self) -> Instant;
}

#[derive(Debug, Default)]
pub struct RealClock;

impl Clock for RealClock {
    fn now(&self) -> Instant {
        Instant::now()
    }
}

/// Hand-driven clock for deterministic tests. Interior-mutable so a single
/// `Arc<MockClock>` can be shared across all consumers and advanced from
/// the test thread.
#[derive(Debug)]
pub struct MockClock {
    inner: Mutex<Instant>,
}

impl MockClock {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: Mutex::new(Instant::now()),
        })
    }

    pub fn advance(&self, by: Duration) {
        let mut g = self.inner.lock().unwrap();
        *g += by;
    }
}

impl Clock for MockClock {
    fn now(&self) -> Instant {
        *self.inner.lock().unwrap()
    }
}

#[cfg(test)]
mod tests {
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
}
