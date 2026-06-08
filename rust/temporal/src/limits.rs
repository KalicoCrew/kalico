#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub v_max: [f64; 3],
    pub a_max: [f64; 3],
    pub j_max: [f64; 3],
    pub a_centripetal_max: f64,
}

impl Limits {
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
