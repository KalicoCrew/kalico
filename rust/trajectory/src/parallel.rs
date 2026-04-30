// Parallel dispatch for shaper pipeline. MVP: sequential implementation.
//
// The API is parallel-ready (fan_out_indexed takes a thread count), but for MVP
// we iterate sequentially. Actual thread dispatch can be added later without
// changing any call sites.

/// Execute `f(i)` for `i` in `0..count`, returning `(index, result)` pairs.
///
/// MVP implementation: sequential. The `_n_threads` parameter is accepted for
/// API compatibility with future parallel dispatch.
pub fn fan_out_indexed<F, R>(count: usize, _n_threads: usize, f: F) -> Vec<(usize, R)>
where
    F: Fn(usize) -> R + Sync,
    R: Send,
{
    (0..count).map(|i| (i, f(i))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fan_out_indexed_empty() {
        let results: Vec<(usize, i32)> = fan_out_indexed(0, 4, |_| 42);
        assert!(results.is_empty());
    }

    #[test]
    fn fan_out_indexed_identity() {
        let results = fan_out_indexed(5, 2, |i| i * 10);
        assert_eq!(results.len(), 5);
        for (idx, val) in &results {
            assert_eq!(*val, idx * 10);
        }
    }

    #[test]
    fn fan_out_indexed_preserves_order() {
        let results = fan_out_indexed(10, 3, |i| i);
        let indices: Vec<usize> = results.iter().map(|(i, _)| *i).collect();
        assert_eq!(indices, (0..10).collect::<Vec<_>>());
    }
}
