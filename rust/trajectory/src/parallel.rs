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
