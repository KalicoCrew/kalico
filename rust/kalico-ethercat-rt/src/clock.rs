//! System-wide monotonic clock in nanoseconds (`CLOCK_MONOTONIC`).
//!
//! `std::time::Instant` reads `CLOCK_MONOTONIC` but its value is opaque and
//! anchored per-process, so timestamps are NOT comparable between the endpoint
//! and the test client. The raw `CLOCK_MONOTONIC` epoch is shared by every
//! process on the host, so reading it directly gives both processes one common
//! timeline — which is what makes `PushSegment.t_start`/`t_end` (absolute ns)
//! meaningful across the socket.
#![allow(unsafe_code)]

/// Nanoseconds on the host-wide `CLOCK_MONOTONIC` timeline.
#[must_use]
pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: `ts` is a valid, fully-initialized `timespec`; `clock_gettime`
    // only writes through the pointer and returns 0 on success for a valid
    // clock id. CLOCK_MONOTONIC is always available on Linux/macOS.
    let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    assert_eq!(rc, 0, "clock_gettime(CLOCK_MONOTONIC) failed");
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}
