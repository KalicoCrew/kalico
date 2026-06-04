#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}

#[derive(Debug, Clone)]
pub struct ModalState {
    pub position: [f64; 3],
    pub input_e: f64,
    pub output_e: f64,
    pub feedrate_mm_min: Option<f64>,
    pub absolute_xyz: bool,
    pub absolute_e: bool,
    pub active_plane: Plane,
    pub prev_g5_pq: Option<[f64; 2]>,
    pub prev_tangent: Option<[f64; 2]>,
}

impl ModalState {
    pub fn new() -> Self {
        Self {
            position: [0.0; 3],
            input_e: 0.0,
            output_e: 0.0,
            feedrate_mm_min: None,
            absolute_xyz: true,
            absolute_e: true,
            active_plane: Plane::default(),
            prev_g5_pq: None,
            prev_tangent: None,
        }
    }

    pub fn resolve_position(&self, x: Option<f64>, y: Option<f64>, z: Option<f64>) -> [f64; 3] {
        if self.absolute_xyz {
            [
                x.unwrap_or(self.position[0]),
                y.unwrap_or(self.position[1]),
                z.unwrap_or(self.position[2]),
            ]
        } else {
            [
                self.position[0] + x.unwrap_or(0.0),
                self.position[1] + y.unwrap_or(0.0),
                self.position[2] + z.unwrap_or(0.0),
            ]
        }
    }

    pub fn resolve_input_e(&self, e_param: Option<f64>) -> Option<f64> {
        e_param.map(|e| if self.absolute_e { e } else { self.input_e + e })
    }

    pub fn has_xy_motion(&self, end: &[f64; 3]) -> bool {
        let dx = end[0] - self.position[0];
        let dy = end[1] - self.position[1];
        dx * dx + dy * dy > 1e-12
    }
}

impl Default for ModalState {
    fn default() -> Self {
        Self::new()
    }
}
