//! Time abstraction. Production wires `RealClock`; tests wire `MockClock`.
//!
//! `monotonic_raw_secs` gives a CLOCK_MONOTONIC_RAW reading as seconds since an
//! arbitrary epoch, used for RTT measurement wherever CLOCK_MONOTONIC and
//! CLOCK_MONOTONIC_RAW diverge (i.e. under NTP slew). On non-Linux platforms it
//! falls back to `Instant` (CLOCK_MONOTONIC).

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Read CLOCK_MONOTONIC_RAW and return the value as seconds since an arbitrary
/// but stable epoch.
///
/// The returned value is only meaningful when compared to other values from the
/// same call — it is not wall time. On platforms that do not support
/// CLOCK_MONOTONIC_RAW (anything other than Linux) it falls back to
/// `Instant::now()` measured against a process-lifetime anchor.
///
/// The unit is seconds (f64) to match klippy's reactor.monotonic() convention.
pub fn monotonic_raw_secs() -> f64 {
    #[cfg(target_os = "linux")]
    {
        // SAFETY: clock_gettime is a safe POSIX syscall; we only read the result.
        #[allow(unsafe_code)]
        unsafe {
            let mut ts: libc::timespec = std::mem::zeroed();
            libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts);
            ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        use std::sync::OnceLock;
        static ANCHOR: OnceLock<Instant> = OnceLock::new();
        let anchor = ANCHOR.get_or_init(Instant::now);
        Instant::now()
            .saturating_duration_since(*anchor)
            .as_secs_f64()
    }
}

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
