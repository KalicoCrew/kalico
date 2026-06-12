use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct ProfileFile {
    version: u32,
    axes: Vec<String>,
    mass: Vec<Vec<f64>>,
    viscous: Vec<f64>,
    coulomb_fwd: Vec<f64>,
    coulomb_rev: Vec<f64>,
    coulomb_deadband_mm_s: f64,
    #[allow(dead_code)]
    fit_rms_residual: Vec<f64>,
}

#[derive(Debug)]
pub enum ProfileError {
    Parse(String),
    Version(u32),
    Dim(&'static str),
    NotFinite(&'static str),
    NotSymmetric,
    NotPositiveDefinite,
}

#[derive(Debug)]
pub struct DynamicsModel {
    pub n: usize,
    pub axes: Vec<String>,
    mass: Vec<f32>,
    viscous: Vec<f32>,
    coulomb_fwd: Vec<f32>,
    coulomb_rev: Vec<f32>,
    deadband: f32,
}

impl DynamicsModel {
    pub fn from_toml_str(s: &str) -> Result<Self, ProfileError> {
        let f: ProfileFile = toml::from_str(s).map_err(|e| ProfileError::Parse(e.to_string()))?;
        if f.version != 1 {
            return Err(ProfileError::Version(f.version));
        }
        let n = f.axes.len();
        if n == 0 {
            return Err(ProfileError::Dim("axes is empty"));
        }
        if f.mass.len() != n || f.mass.iter().any(|row| row.len() != n) {
            return Err(ProfileError::Dim("mass must be n x n"));
        }
        if f.viscous.len() != n {
            return Err(ProfileError::Dim("viscous length"));
        }
        if f.coulomb_fwd.len() != n {
            return Err(ProfileError::Dim("coulomb_fwd length"));
        }
        if f.coulomb_rev.len() != n {
            return Err(ProfileError::Dim("coulomb_rev length"));
        }
        let mass: Vec<f64> = f.mass.iter().flatten().copied().collect();
        let all_finite = mass
            .iter()
            .chain(&f.viscous)
            .chain(&f.coulomb_fwd)
            .chain(&f.coulomb_rev)
            .chain(std::iter::once(&f.coulomb_deadband_mm_s))
            .all(|v| v.is_finite());
        if !all_finite {
            return Err(ProfileError::NotFinite("profile contains non-finite value"));
        }
        for i in 0..n {
            for j in (i + 1)..n {
                let (a, b) = (mass[i * n + j], mass[j * n + i]);
                if (a - b).abs() > 1e-9 * a.abs().max(b.abs()).max(1e-12) {
                    return Err(ProfileError::NotSymmetric);
                }
            }
        }
        if !cholesky_is_pd(&mass, n) {
            return Err(ProfileError::NotPositiveDefinite);
        }
        Ok(Self {
            n,
            axes: f.axes,
            mass: mass.iter().map(|&v| v as f32).collect(),
            viscous: f.viscous.iter().map(|&v| v as f32).collect(),
            coulomb_fwd: f.coulomb_fwd.iter().map(|&v| v as f32).collect(),
            coulomb_rev: f.coulomb_rev.iter().map(|&v| v as f32).collect(),
            deadband: f.coulomb_deadband_mm_s as f32,
        })
    }

    pub fn torque_ff(&self, axis: usize, acc_mm_s2: &[f32], vel_mm_s: &[f32]) -> f32 {
        assert_eq!(acc_mm_s2.len(), self.n);
        assert_eq!(vel_mm_s.len(), self.n);
        assert!(axis < self.n);
        let row = &self.mass[axis * self.n..][..self.n];
        let inertial: f32 = row.iter().zip(acc_mm_s2.iter()).map(|(m, a)| m * a).sum();
        let v = vel_mm_s[axis];
        let coulomb = if v > self.deadband {
            self.coulomb_fwd[axis]
        } else if v < -self.deadband {
            self.coulomb_rev[axis]
        } else {
            0.0
        };
        inertial + self.viscous[axis] * v + coulomb
    }
}

fn cholesky_is_pd(m: &[f64], n: usize) -> bool {
    let mut l = m.to_vec();
    for k in 0..n {
        for j in 0..k {
            l[k * n + k] -= l[k * n + j] * l[k * n + j];
        }
        if l[k * n + k] <= 0.0 {
            return false;
        }
        l[k * n + k] = l[k * n + k].sqrt();
        for i in (k + 1)..n {
            for j in 0..k {
                l[i * n + k] -= l[i * n + j] * l[k * n + j];
            }
            l[i * n + k] /= l[k * n + k];
        }
    }
    true
}

pub fn clamp_torque(raw_tenths_pct: f32, limit_tenths_pct: i16, saturation_count: &mut u32) -> i16 {
    assert!(raw_tenths_pct.is_finite(), "non-finite torque FF");
    assert!(limit_tenths_pct > 0, "torque clamp limit must be positive");
    let lim = f32::from(limit_tenths_pct);
    if raw_tenths_pct > lim {
        *saturation_count += 1;
        limit_tenths_pct
    } else if raw_tenths_pct < -lim {
        *saturation_count += 1;
        -limit_tenths_pct
    } else {
        raw_tenths_pct as i16
    }
}

#[cfg(test)]
mod tests;
