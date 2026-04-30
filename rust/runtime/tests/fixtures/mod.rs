//! Shared test fixtures. Used by Surface A integration tests + Surface B
//! FFI tests + Surface C Python validation. Spec §6.7.
#![allow(clippy::unwrap_used, clippy::expect_used, dead_code)]

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct Fixture {
    pub name: String,
    pub description: String,
    pub control_points: Vec<[f32; 3]>,
    pub knots: Vec<f32>,
    pub weights: Vec<f32>,
    pub degree: u8,
    pub duration_us: u32,
    pub kinematics: String, // "CoreXyAndE" or "CartesianXyzAndE"
}

#[derive(Debug, Deserialize)]
pub struct FixtureSet {
    pub fixtures: Vec<Fixture>,
}

pub fn load() -> FixtureSet {
    // Embedded at compile time so the test runs under Miri, which blocks
    // host filesystem access by default.
    const RAW: &str = include_str!("step5_segments.json");
    serde_json::from_str(RAW).expect("fixture parse failed")
}

// ─── Scalar NURBS helpers (Task 6) ──────────────────────────────────────

/// Create a degree-1 linear scalar NURBS from `start` to `end` on [0, 1].
/// Returns `(degree, knots, control_points)`.
pub fn linear_scalar(start: f32, end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let degree = 1u8;
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let cps = vec![start, end];
    (degree, knots, cps)
}

/// Create a degree-1 constant scalar NURBS holding `value` on [0, 1].
/// Returns `(degree, knots, control_points)`.
pub fn constant_scalar(value: f32) -> (u8, Vec<f32>, Vec<f32>) {
    linear_scalar(value, value)
}

/// Load a scalar NURBS into the curve pool, returning the handle. Scans
/// for a free slot starting from `start_slot`.
pub fn load_scalar(
    pool: &runtime::curve_pool::CurvePool,
    start_slot: u16,
    degree: u8,
    knots: &[f32],
    cps: &[f32],
) -> runtime::curve_pool::CurveHandle {
    for slot_idx in (start_slot as usize)..runtime::curve_pool::CURVE_POOL_N {
        if let Ok(handle) = pool.validate_and_load(slot_idx as u16, degree, knots, cps) {
            return handle;
        }
    }
    panic!("no free curve pool slots starting from {start_slot}");
}
