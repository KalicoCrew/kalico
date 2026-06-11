use crate::linalg::{solve_spd, sym_eig_extremes};
use crate::model::{PhysicalParams, Structure};

#[derive(Clone)]
pub struct FitInput {
    pub structure: Structure,
    /// Per motor, equal lengths: acc[motor][sample] (mm/s²).
    pub acc: Vec<Vec<f64>>,
    pub vel: Vec<Vec<f64>>,
    /// Measured torque per motor (0.1% rated units).
    pub torque: Vec<Vec<f64>>,
}

pub struct FitOptions {
    /// Refusal threshold on `FitResult::condition` (column-scaled Gram).
    pub max_condition: f64,
    pub saturation_abs: f64,
    pub max_saturated_fraction: f64,
    pub max_rms_residual: f64,
}

impl Default for FitOptions {
    fn default() -> Self {
        Self {
            max_condition: 1.0e8,
            saturation_abs: 3900.0,
            max_saturated_fraction: 0.001,
            max_rms_residual: 50.0,
        }
    }
}

#[derive(Debug)]
pub enum FitError {
    ShapeMismatch(&'static str),
    SaturatedTorque { fraction: f64 },
    InsufficientExcitation { condition: f64 },
    ResidualTooLarge { rms: f64 },
}

#[derive(Debug)]
pub struct FitResult {
    pub params: PhysicalParams,
    /// In-sample RMS (0.1% rated units); optimism bias is negligible for
    /// sample counts far above the parameter count.
    pub rms_residual: f64,
    /// λmax/λmin of the column-scaled Gram matrix — an excitation-quality
    /// score, not cond(AᵀA).
    pub condition: f64,
    /// Time samples per motor; regression rows = samples × motor count.
    pub samples: usize,
}

pub fn fit(input: &FitInput, opts: &FitOptions) -> Result<FitResult, FitError> {
    let s = input.structure;
    let n_motors = s.axis_count();
    if input.acc.len() != n_motors || input.vel.len() != n_motors || input.torque.len() != n_motors
    {
        return Err(FitError::ShapeMismatch("motor count"));
    }
    let n_samples = input.acc[0].len();
    if n_samples == 0 {
        return Err(FitError::ShapeMismatch("no samples"));
    }
    for m in 0..n_motors {
        if input.acc[m].len() != n_samples
            || input.vel[m].len() != n_samples
            || input.torque[m].len() != n_samples
        {
            return Err(FitError::ShapeMismatch("sample count"));
        }
    }

    let saturated = input
        .torque
        .iter()
        .flatten()
        .filter(|t| t.abs() >= opts.saturation_abs)
        .count();
    let fraction = saturated as f64 / (n_motors * n_samples) as f64;
    if fraction > opts.max_saturated_fraction {
        return Err(FitError::SaturatedTorque { fraction });
    }

    let p = s.param_count();
    let mut ata = vec![0.0_f64; p * p];
    let mut aty = vec![0.0_f64; p];
    let mut col_norm2 = vec![0.0_f64; p];

    for k in 0..n_samples {
        let acc_k: Vec<f64> = (0..n_motors).map(|m| input.acc[m][k]).collect();
        let vel_k: Vec<f64> = (0..n_motors).map(|m| input.vel[m][k]).collect();
        for motor in 0..n_motors {
            let row = s.row(motor, &acc_k, &vel_k);
            let y = input.torque[motor][k];
            for i in 0..p {
                aty[i] += row[i] * y;
                col_norm2[i] += row[i] * row[i];
                for j in 0..p {
                    ata[i * p + j] += row[i] * row[j];
                }
            }
        }
    }

    let scale: Vec<f64> = col_norm2
        .iter()
        .map(|&c| if c > 0.0 { c.sqrt() } else { 0.0 })
        .collect();
    if scale.iter().any(|&sc| sc == 0.0) {
        return Err(FitError::InsufficientExcitation {
            condition: f64::INFINITY,
        });
    }

    let mut ata_s = vec![0.0_f64; p * p];
    for i in 0..p {
        for j in 0..p {
            ata_s[i * p + j] = ata[i * p + j] / (scale[i] * scale[j]);
        }
    }

    let (lo, hi) = sym_eig_extremes(&ata_s, p);
    let condition = if lo > 0.0 { hi / lo } else { f64::INFINITY };
    if condition > opts.max_condition {
        return Err(FitError::InsufficientExcitation { condition });
    }

    let aty_s: Vec<f64> = (0..p).map(|i| aty[i] / scale[i]).collect();
    let theta_s =
        solve_spd(&ata_s, &aty_s, p).ok_or(FitError::InsufficientExcitation { condition })?;
    let theta: Vec<f64> = (0..p).map(|i| theta_s[i] / scale[i]).collect();

    let mut sq_sum = 0.0_f64;
    for k in 0..n_samples {
        let acc_k: Vec<f64> = (0..n_motors).map(|m| input.acc[m][k]).collect();
        let vel_k: Vec<f64> = (0..n_motors).map(|m| input.vel[m][k]).collect();
        for motor in 0..n_motors {
            let row = s.row(motor, &acc_k, &vel_k);
            let pred: f64 = row.iter().zip(&theta).map(|(r, t)| r * t).sum();
            let e = input.torque[motor][k] - pred;
            sq_sum += e * e;
        }
    }
    let rms = (sq_sum / (n_motors * n_samples) as f64).sqrt();
    if rms > opts.max_rms_residual {
        return Err(FitError::ResidualTooLarge { rms });
    }

    Ok(FitResult {
        params: s.unpack(&theta),
        rms_residual: rms,
        condition,
        samples: n_samples,
    })
}
