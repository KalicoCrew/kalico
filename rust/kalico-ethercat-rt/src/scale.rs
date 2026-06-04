//! mm -> encoder-count mapping, relative to a captured origin.

/// Fixed mm->counts gain and the origin captured at first arm.
#[derive(Debug, Clone, Copy)]
pub struct CountMap {
    pub counts_per_mm: f64,
    pub origin_counts: i32,
    pub origin_mm: f64,
}

impl CountMap {
    /// Capture the origin: `actual_counts` is the rotor position now,
    /// `pos_mm` is the trajectory position at the same instant.
    pub fn new(counts_per_mm: f64, actual_counts: i32, pos_mm: f64) -> Self {
        Self {
            counts_per_mm,
            origin_counts: actual_counts,
            origin_mm: pos_mm,
        }
    }

    /// Map a trajectory position (mm) to an absolute target count.
    pub fn target_counts(&self, pos_mm: f64) -> i32 {
        let delta = (pos_mm - self.origin_mm) * self.counts_per_mm;
        self.origin_counts + delta.round() as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_maps_to_itself() {
        let m = CountMap::new(3276.8, 14578, 5.0);
        assert_eq!(m.target_counts(5.0), 14578);
    }

    #[test]
    fn positive_delta_rounds_and_adds() {
        let m = CountMap::new(1000.0, 0, 0.0);
        assert_eq!(m.target_counts(1.0004), 1000); // 1000.4 -> 1000
        assert_eq!(m.target_counts(1.0006), 1001); // 1000.6 -> 1001
    }

    #[test]
    fn negative_delta() {
        let m = CountMap::new(1000.0, 5000, 10.0);
        assert_eq!(m.target_counts(9.0), 4000);
    }

    /// Verify the origin-capture invariant: `target_counts(origin_mm) == actual_counts`
    /// exactly, for the specific gain and rotor position used in the "no-jump" boot
    /// sequence. If this fails the servo will jump by `delta` counts at arm time.
    ///
    /// Historical bug: early versions computed an offset relative to mm=0, not the
    /// captured rotor position, causing a startup position jump proportional to the
    /// origin in mm.
    ///
    /// Analytic values:
    ///   origin_mm = 7.5, counts_per_mm = 3276.8, actual_counts = 14578
    ///   target_counts(7.5) = 14578 + round((7.5 - 7.5) * 3276.8) = 14578 + 0 = 14578
    ///   target_counts(7.5 + 1/3276.8) = 14578 + round(1.0) = 14579
    ///   target_counts(7.5 - 1/3276.8) = 14578 + round(-1.0) = 14577
    #[test]
    fn origin_no_jump() {
        let counts_per_mm = 3276.8_f64;
        let actual_counts = 14578_i32;
        let origin_mm = 7.5_f64;

        let m = CountMap::new(counts_per_mm, actual_counts, origin_mm);

        // The origin must map to the captured rotor position exactly — no startup jump.
        assert_eq!(
            m.target_counts(origin_mm),
            actual_counts,
            "origin_mm must map to actual_counts exactly; a mismatch is a startup jump"
        );

        // One count forward: delta = 1/counts_per_mm mm → round(delta * counts_per_mm) = 1.
        let one_count_fwd = origin_mm + 1.0 / counts_per_mm;
        assert_eq!(
            m.target_counts(one_count_fwd),
            actual_counts + 1,
            "one count forward must be actual_counts + 1"
        );

        // One count back: delta = -1/counts_per_mm mm → round(delta * counts_per_mm) = -1.
        let one_count_back = origin_mm - 1.0 / counts_per_mm;
        assert_eq!(
            m.target_counts(one_count_back),
            actual_counts - 1,
            "one count back must be actual_counts - 1"
        );
    }
}
