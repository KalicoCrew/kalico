#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FitterParams {
    pub theta_smooth_deg: f64,
    pub theta_hard_deg: f64,
    pub seg_len_collapse_mm: f64,
    pub degree: u8,
    pub n_init_interior: u32,
    pub eps_chord_mm: f64,
    pub eps_iter_mm: f64,
    pub max_lspia_iter: u32,
    pub max_refine_iter: u32,
    pub n_chord_samples: u32,
    pub max_window_vertices: u32,
    pub blend_tolerance_mm: f64,
}

impl Default for FitterParams {
    fn default() -> Self {
        Self {
            theta_smooth_deg: 15.0,
            theta_hard_deg: 60.0,
            seg_len_collapse_mm: 0.05,
            degree: 3,
            n_init_interior: 4,
            eps_chord_mm: 0.025,
            eps_iter_mm: 1e-9,
            max_lspia_iter: 100,
            max_refine_iter: 20,
            n_chord_samples: 50,
            max_window_vertices: 64,
            blend_tolerance_mm: 0.050,
        }
    }
}
