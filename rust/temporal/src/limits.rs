//! Per-axis kinematic limits and centripetal cap. Pure data.
//!
//! Spec §4.4. Per-axis centripetal limits are deferred (§4.4 / §11).

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
