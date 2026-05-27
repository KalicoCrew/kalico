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
mod tests;
