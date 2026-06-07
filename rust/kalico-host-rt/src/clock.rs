//! Time abstraction. Production wires `RealClock`; tests wire `MockClock`.
//!
//! `monotonic_raw_secs` gives a CLOCK_MONOTONIC_RAW reading as seconds since an
//! arbitrary epoch, used for RTT measurement wherever CLOCK_MONOTONIC and
//! CLOCK_MONOTONIC_RAW diverge (i.e. under NTP slew). On non-Linux platforms it
//! falls back to `Instant` (CLOCK_MONOTONIC).

use std::sync::{Arc, Mutex, OnceLock};
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
        // non-negative contract: anchor may be seeded slightly after the current
        // instant under parallel test initialisation, making instant_to_f64 negative.
        instant_to_f64(Instant::now()).max(0.0)
    }
}

/// Convert an `Instant` to seconds relative to a stable process-lifetime anchor.
///
/// The anchor is initialised on first call and shared across all callers in the
/// same process, so `instant_to_f64(a) - instant_to_f64(b)` equals
/// `a.duration_since(b)` for any two `Instant`s — including values produced by
/// `monotonic_raw_secs`'s non-Linux fallback, which uses the same anchor.
///
/// On Linux this is the companion to `monotonic_raw_secs`: use
/// `monotonic_raw_secs()` for wire-stamp deltas and `instant_to_f64` for
/// `Instant`-based durations; both share the same zero-point off-Linux so
/// mixed arithmetic in `set_clock_est_rebased` is correct on all platforms.
pub fn instant_to_f64(instant: Instant) -> f64 {
    static ANCHOR: OnceLock<Instant> = OnceLock::new();
    let anchor = ANCHOR.get_or_init(Instant::now);
    if instant >= *anchor {
        instant.duration_since(*anchor).as_secs_f64()
    } else {
        -(anchor.duration_since(instant).as_secs_f64())
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
