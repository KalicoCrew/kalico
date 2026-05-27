// Parallel dispatch for shaper pipeline. MVP: sequential implementation.
//
// The API is parallel-ready (fan_out_indexed takes a thread count), but for MVP
// we iterate sequentially. Actual thread dispatch can be added later without
// changing any call sites.

/// Execute `f(i)` for `i` in `0..count`, returning `(index, result)` pairs.
///
/// MVP implementation: sequential. The `_n_threads` parameter is accepted for
/// API compatibility with future parallel dispatch.
#[allow(dead_code)]
pub fn fan_out_indexed<F, R>(count: usize, _n_threads: usize, f: F) -> Vec<(usize, R)>
where
    F: Fn(usize) -> R + Sync,
    R: Send,
{
    (0..count).map(|i| (i, f(i))).collect()
}

#[cfg(test)]
mod tests;
