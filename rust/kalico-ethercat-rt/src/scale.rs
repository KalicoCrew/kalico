#[derive(Debug, Clone, Copy)]
pub struct CountMap {
    pub counts_per_mm: f64,
    pub origin_counts: i32,
    pub origin_mm: f64,
}

impl CountMap {
    pub fn new(counts_per_mm: f64, actual_counts: i32, pos_mm: f64) -> Self {
        Self {
            counts_per_mm,
            origin_counts: actual_counts,
            origin_mm: pos_mm,
        }
    }

    pub fn target_counts(&self, pos_mm: f64) -> i32 {
        let delta = (pos_mm - self.origin_mm) * self.counts_per_mm;
        self.origin_counts + delta.round() as i32
    }
}

#[cfg(test)]
mod tests;
