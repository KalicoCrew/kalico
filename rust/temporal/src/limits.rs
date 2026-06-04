#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// Per-axis [X, Y, Z] velocity bound, mm/s.
    pub v_max: [f64; 3],
    /// Per-axis [X, Y, Z] acceleration bound, mm/s².
    pub a_max: [f64; 3],
    /// Per-axis [X, Y, Z] jerk bound, mm/s³.
    pub j_max: [f64; 3],
    /// Centripetal-acceleration cap, mm/s² (scalar; per-axis deferred).
    pub a_centripetal_max: f64,
}

impl Limits {
    /// Construct a `Limits` from all required fields. The struct is
    /// `#[non_exhaustive]`; external callers must use this constructor.
    #[must_use]
    pub fn new(v_max: [f64; 3], a_max: [f64; 3], j_max: [f64; 3], a_centripetal_max: f64) -> Self {
        Self {
            v_max,
            a_max,
            j_max,
            a_centripetal_max,
        }
    }
}
