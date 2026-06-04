#[derive(Debug, Clone)]
pub struct Waypoint {
    pub pos: [f64; 3],
    pub input_e: f64,
    pub line_no: u32,
}

#[derive(Debug)]
pub struct Run {
    pub waypoints: Vec<Waypoint>,
    pub feedrate_mm_min: f64,
    pub e_ratio: Option<f64>,
    pub start_tangent: Option<[f64; 2]>,
    pub end_tangent: Option<[f64; 2]>,
}

impl Run {
    pub fn new(start: Waypoint, feedrate_mm_min: f64) -> Self {
        Self {
            waypoints: vec![start],
            feedrate_mm_min,
            e_ratio: None,
            start_tangent: None,
            end_tangent: None,
        }
    }

    pub fn push(&mut self, wp: Waypoint) {
        self.waypoints.push(wp);
    }

    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    pub fn total_e_delta(&self) -> f64 {
        match (self.waypoints.first(), self.waypoints.last()) {
            (Some(first), Some(last)) if self.waypoints.len() > 1 => last.input_e - first.input_e,
            _ => 0.0,
        }
    }

    pub fn positions(&self) -> Vec<[f64; 3]> {
        self.waypoints.iter().map(|wp| wp.pos).collect()
    }
}
