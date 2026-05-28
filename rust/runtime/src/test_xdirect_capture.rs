//! Host-test sink for `phase_stepping_write_xdirect` calls.
//!
//! Production firmware builds (`target_os = "none"`) route XDIRECT writes
//! through the C FFI helper (`src/stm32/phase_stepping_spi.c`, Task 3 of the
//! 2026-05-18 phase-stepping plan). Host-build tests instead record each
//! call into the process-global sink below so integration tests can assert
//! on the SPI traffic without a real bus.
//!
//! On `target_os = "none"` the host helpers compile to no-op stubs so the
//! module can stay `pub` without leaking host-only state into firmware.
//!
//! 2026-05-19 — record now carries `motor_idx` (and only motor_idx) for the
//! per-motor-CS dispatch refactor. Bus / CS resolution is the C side's job
//! via the `phase_motors[]` table; host tests assert on motor identity.

/// One captured XDIRECT write — the three parameters of
/// `phase_stepping_write_xdirect`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XDirectRecord {
    pub motor_idx: u8,
    pub coil_a: i16,
    pub coil_b: i16,
}

#[cfg(not(target_os = "none"))]
mod host {
    use super::XDirectRecord;
    use std::sync::{Mutex, MutexGuard, OnceLock};

    fn sink() -> &'static Mutex<Vec<XDirectRecord>> {
        static SINK: OnceLock<Mutex<Vec<XDirectRecord>>> = OnceLock::new();
        SINK.get_or_init(|| Mutex::new(Vec::new()))
    }

    /// Serialization lock for integration tests that own the sink for the
    /// duration of a single test case. Acquire with `lock_for_test()` and
    /// hold the returned guard until the test assertion completes so that
    /// concurrent tests do not interleave `record()` calls into the window
    /// between a test's `clear()` and `drain()`.
    fn test_serial() -> &'static Mutex<()> {
        static SERIAL: OnceLock<Mutex<()>> = OnceLock::new();
        SERIAL.get_or_init(|| Mutex::new(()))
    }

    /// Acquire exclusive ownership of the capture sink for one test.
    ///
    /// The caller **must** hold the returned guard until its own `drain()`
    /// or `clear()` call completes.  Dropping the guard returns the lock to
    /// the next waiting test.
    pub fn lock_for_test() -> MutexGuard<'static, ()> {
        test_serial().lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Append a record. Called by `engine::write_xdirect` in host builds.
    pub fn record(motor_idx: u8, coil_a: i16, coil_b: i16) {
        // Lock poisoning means a prior test panicked while holding the
        // mutex — recover the inner Vec anyway so subsequent tests are
        // not collateral damage.
        let mut g = sink().lock().unwrap_or_else(|p| p.into_inner());
        g.push(XDirectRecord {
            motor_idx,
            coil_a,
            coil_b,
        });
    }

    /// Drain and return all records captured since the last drain. Each
    /// test should drain at the start so prior tests' captures don't leak.
    pub fn drain() -> Vec<XDirectRecord> {
        let mut g = sink().lock().unwrap_or_else(|p| p.into_inner());
        let out = g.clone();
        g.clear();
        out
    }

    /// Drop any pending captures without returning them. Useful when a
    /// test wants a clean slate before exercising the path under
    /// assertion, regardless of what earlier setup may have recorded.
    pub fn clear() {
        let mut g = sink().lock().unwrap_or_else(|p| p.into_inner());
        g.clear();
    }

    pub fn count() -> usize {
        let g = sink().lock().unwrap_or_else(|p| p.into_inner());
        g.len()
    }
}

#[cfg(not(target_os = "none"))]
pub use host::{clear, count, drain, lock_for_test, record};

// On target, the helpers compile to no-ops so the production write_xdirect
// path still type-checks if it ever reaches this module (it shouldn't —
// `engine::write_xdirect`'s `#[cfg]` gates send target builds straight to
// the C FFI).
#[cfg(target_os = "none")]
pub fn record(_motor_idx: u8, _coil_a: i16, _coil_b: i16) {}

#[cfg(target_os = "none")]
pub fn drain() -> &'static [XDirectRecord] {
    &[]
}

#[cfg(target_os = "none")]
pub fn clear() {}

/// No-op on target — there is no parallelism in a bare-metal ISR context.
#[cfg(target_os = "none")]
pub fn lock_for_test() {}
