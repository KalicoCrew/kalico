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
    pub kinematics: String,
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

