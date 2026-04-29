//! Run segmentation types for the compat layer.
//!
//! A `Run` is a contiguous sequence of G1-style waypoints that share a feedrate
//! and form a single candidate for spline-fitting or collinear G5 emission.

/// A single waypoint in a G1 run: absolute position plus the input-file E
/// accumulator at that point and the source line number (for diagnostics).
#[derive(Debug, Clone)]
pub struct Waypoint {
    /// Absolute XYZ position in mm.
    pub pos: [f64; 3],
    /// Input-side E accumulator value at this waypoint (absolute mm).
    pub input_e: f64,
    /// Source G-code line number (1-based), for error messages.
    pub line_no: u32,
}

/// A contiguous sequence of waypoints sharing a feedrate, bounded by corners
/// or feedrate changes.
#[derive(Debug)]
pub struct Run {
    /// Ordered waypoints including the run's start and end points.
    pub waypoints: Vec<Waypoint>,
    /// Feedrate in mm/min for all moves in this run.
    pub feedrate_mm_min: f64,
    /// E-ratio (`E_delta` / `XY_path_length`) for segments in this run, if
    /// established. Used to detect run breaks when the ratio changes.
    pub e_ratio: Option<f64>,
    /// Tangent direction arriving at the start of the run from a previous
    /// segment, if known. Used to set C¹ continuity at the run boundary.
    pub start_tangent: Option<[f64; 2]>,
    /// Tangent direction leaving the end of the run toward the next segment,
    /// if known.
    pub end_tangent: Option<[f64; 2]>,
}

impl Run {
    /// Create a new `Run` starting at `start` with the given feedrate.
    pub fn new(start: Waypoint, feedrate_mm_min: f64) -> Self {
        Self {
            waypoints: vec![start],
            feedrate_mm_min,
            e_ratio: None,
            start_tangent: None,
            end_tangent: None,
        }
    }

    /// Append a waypoint to the end of the run.
    pub fn push(&mut self, wp: Waypoint) {
        self.waypoints.push(wp);
    }

    /// Number of waypoints in the run.
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// True when the run contains no waypoints.
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Total E displacement across the run: `last.input_e − first.input_e`.
    /// Returns 0.0 when the run has fewer than two waypoints.
    pub fn total_e_delta(&self) -> f64 {
        match (self.waypoints.first(), self.waypoints.last()) {
            (Some(first), Some(last)) if self.waypoints.len() > 1 => {
                last.input_e - first.input_e
            }
            _ => 0.0,
        }
    }

    /// Extract just the XYZ positions from the waypoints, in order.
    pub fn positions(&self) -> Vec<[f64; 3]> {
        self.waypoints.iter().map(|wp| wp.pos).collect()
    }
}
