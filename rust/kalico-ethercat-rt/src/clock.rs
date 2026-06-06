//! Nanoseconds on the host-wide `CLOCK_MONOTONIC_RAW` timeline.
//!
//! `std::time::Instant` is per-process; a shared POSIX clock is required for
//! piece `start_time` values to be comparable between the host pump and this
//! endpoint. `CLOCK_MONOTONIC_RAW` is used (not `CLOCK_MONOTONIC`) because the
//! host's `instant_to_f64` anchor is also RAW-domain on Linux
//! (`kalico-host-rt` uses `CLOCK_MONOTONIC_RAW` for `monotonic_raw_secs`), so
//! the init-planner seed that pairs `Instant::now()` with this value lands both
//! sides in the same domain and the linear mapping (freq 1e9) stays exact.
//!
//! On macOS (CI / development) `CLOCK_MONOTONIC_RAW` exists and is available
//! via libc. The endpoint binary only ships on Linux; the macOS path runs
//! in tests only.
#![allow(unsafe_code)]

#[must_use]
pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, fully-initialized `timespec`; `clock_gettime`
    // only writes through the pointer and returns 0 on success for a valid
    // clock id. CLOCK_MONOTONIC_RAW is available on Linux and macOS.
    let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts) };
    assert_eq!(rc, 0, "clock_gettime(CLOCK_MONOTONIC_RAW) failed");
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}
