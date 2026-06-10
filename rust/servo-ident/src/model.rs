pub const COULOMB_DEADBAND_MM_S: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Structure {
    CartesianScalar,
    CoreXY,
}

#[derive(Debug, PartialEq)]
pub struct PhysicalParams {
    pub mass: Vec<Vec<f64>>,
    pub viscous: Vec<f64>,
    pub coulomb_fwd: Vec<f64>,
    pub coulomb_rev: Vec<f64>,
}

fn coulomb_cols(v: f64) -> (f64, f64) {
    if v > COULOMB_DEADBAND_MM_S {
        (1.0, 0.0)
    } else if v < -COULOMB_DEADBAND_MM_S {
        (0.0, 1.0)
    } else {
        (0.0, 0.0)
    }
}

impl Structure {
    pub fn axis_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 1,
            Structure::CoreXY => 2,
        }
    }

    /// Scalar: theta = [m, b, c_fwd, c_rev].
    /// CoreXY: theta = [m_diag, m_off, b_a, cf_a, cr_a, b_b, cf_b, cr_b].
    pub fn param_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 4,
            Structure::CoreXY => 8,
        }
    }

    /// Regression row such that tau_motor = row(motor, acc, vel) · theta.
    pub fn row(self, motor: usize, acc: &[f64], vel: &[f64]) -> Vec<f64> {
        match self {
            Structure::CartesianScalar => {
                assert_eq!(motor, 0);
                assert!(!acc.is_empty() && !vel.is_empty(), "scalar row needs 1 acc and 1 vel sample, got {} and {}", acc.len(), vel.len());
                let (cf, cr) = coulomb_cols(vel[0]);
                vec![acc[0], vel[0], cf, cr]
            }
            Structure::CoreXY => {
                assert!(motor < 2);
                assert!(acc.len() >= 2 && vel.len() >= 2, "corexy row needs 2 acc and 2 vel samples, got {} and {}", acc.len(), vel.len());
                let other = 1 - motor;
                let (cf, cr) = coulomb_cols(vel[motor]);
                #[allow(clippy::indexing_slicing)]
                let mut r = vec![acc[motor], acc[other], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                let base = 2 + 3 * motor;
                #[allow(clippy::indexing_slicing)]
                {
                    r[base] = vel[motor];
                    r[base + 1] = cf;
                    r[base + 2] = cr;
                }
                r
            }
        }
    }

    pub fn unpack(self, theta: &[f64]) -> PhysicalParams {
        assert_eq!(theta.len(), self.param_count());
        match self {
            Structure::CartesianScalar => PhysicalParams {
                mass: vec![vec![theta[0]]],
                viscous: vec![theta[1]],
                coulomb_fwd: vec![theta[2]],
                coulomb_rev: vec![theta[3]],
            },
            Structure::CoreXY => PhysicalParams {
                mass: vec![vec![theta[0], theta[1]], vec![theta[1], theta[0]]],
                viscous: vec![theta[2], theta[5]],
                coulomb_fwd: vec![theta[3], theta[6]],
                coulomb_rev: vec![theta[4], theta[7]],
            },
        }
    }
}
