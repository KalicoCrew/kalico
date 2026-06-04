//! Nanoseconds on the host-wide `CLOCK_MONOTONIC` timeline.
//!
//! `std::time::Instant` is per-process; `CLOCK_MONOTONIC` is shared across
//! processes on the same host, which is required for piece `start_time` values
//! to be comparable between the host pump and this endpoint.
#![allow(unsafe_code)]

#[must_use]
pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, fully-initialized `timespec`; `clock_gettime`
    // only writes through the pointer and returns 0 on success for a valid
    // clock id. CLOCK_MONOTONIC is always available on Linux/macOS.
    let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    assert_eq!(rc, 0, "clock_gettime(CLOCK_MONOTONIC) failed");
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}
