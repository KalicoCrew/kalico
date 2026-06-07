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

    /// Serialization lock for integration tests. Acquire with `lock_for_test()`
    /// and hold the returned guard until assertion completes to prevent
    /// interleaving `record()` calls between a test's `clear()` and `drain()`.
    fn test_serial() -> &'static Mutex<()> {
        static SERIAL: OnceLock<Mutex<()>> = OnceLock::new();
        SERIAL.get_or_init(|| Mutex::new(()))
    }

    pub fn lock_for_test() -> MutexGuard<'static, ()> {
        test_serial().lock().unwrap_or_else(|p| p.into_inner())
    }

    pub fn record(motor_idx: u8, coil_a: i16, coil_b: i16) {
        let mut g = sink().lock().unwrap_or_else(|p| p.into_inner());
        g.push(XDirectRecord {
            motor_idx,
            coil_a,
            coil_b,
        });
    }

    pub fn drain() -> Vec<XDirectRecord> {
        let mut g = sink().lock().unwrap_or_else(|p| p.into_inner());
        let out = g.clone();
        g.clear();
        out
    }

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

#[cfg(target_os = "none")]
pub fn record(_motor_idx: u8, _coil_a: i16, _coil_b: i16) {}

#[cfg(target_os = "none")]
pub fn drain() -> &'static [XDirectRecord] {
    &[]
}

#[cfg(target_os = "none")]
pub fn clear() {}

#[cfg(target_os = "none")]
pub fn lock_for_test() {}
