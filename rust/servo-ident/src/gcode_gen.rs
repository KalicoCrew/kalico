use std::fmt::Write as _;

#[derive(Clone)]
pub struct Excitation {
    pub axis: String,
    pub min_mm: f64,
    pub max_mm: f64,
    pub accels_mm_s2: Vec<f64>,
    pub speeds_mm_s: Vec<f64>,
    pub reps: usize,
}

#[derive(Debug)]
pub enum GenError {
    BadBounds,
    StrokeTooShort {
        accel: f64,
        speed: f64,
        needed_mm: f64,
    },
}

pub fn generate(e: &Excitation) -> Result<String, GenError> {
    let span = e.max_mm - e.min_mm;
    if span <= 0.0
        || e.accels_mm_s2.is_empty()
        || e.speeds_mm_s.is_empty()
        || e.reps == 0
        || e.accels_mm_s2.iter().any(|&a| a <= 0.0)
        || e.speeds_mm_s.iter().any(|&v| v <= 0.0)
    {
        return Err(GenError::BadBounds);
    }
    let mut g = String::new();
    for &a in &e.accels_mm_s2 {
        for &v in &e.speeds_mm_s {
            let needed = v * v / a;
            if needed > span {
                return Err(GenError::StrokeTooShort {
                    accel: a,
                    speed: v,
                    needed_mm: needed,
                });
            }
            let f = (v * 60.0).round();
            let _ = writeln!(g, "SET_VELOCITY_LIMIT ACCEL={a} ACCEL_TO_DECEL={a}");
            for _ in 0..e.reps {
                let _ = writeln!(g, "G1 {}{} F{f}", e.axis, e.max_mm);
                let _ = writeln!(g, "M400");
                let _ = writeln!(g, "G1 {}{} F{f}", e.axis, e.min_mm);
                let _ = writeln!(g, "M400");
            }
        }
    }
    Ok(g)
}
