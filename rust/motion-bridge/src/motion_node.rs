pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid writable `timespec`; `CLOCK_MONOTONIC_RAW` is available on Linux/macOS.
    #[allow(unsafe_code)]
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

#[cfg(test)]
mod tests;
