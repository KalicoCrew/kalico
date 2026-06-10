use servo_ident::model::{Structure, COULOMB_DEADBAND_MM_S};

fn physical_torque(
    p: &servo_ident::model::PhysicalParams,
    motor: usize,
    acc: &[f64],
    vel: &[f64],
) -> f64 {
    let inertial: f64 = p.mass[motor]
        .iter()
        .zip(acc)
        .map(|(m, a)| m * a)
        .sum();
    let v = vel[motor];
    let coulomb = if v > COULOMB_DEADBAND_MM_S {
        p.coulomb_fwd[motor]
    } else if v < -COULOMB_DEADBAND_MM_S {
        p.coulomb_rev[motor]
    } else {
        0.0
    };
    inertial + p.viscous[motor] * v + coulomb
}

#[test]
fn row_dot_theta_matches_unpacked_physics() {
    let cases: &[(Structure, Vec<f64>)] = &[
        (
            Structure::CartesianScalar,
            vec![0.0123, 0.0045, 1.2, -1.1],
        ),
        (
            Structure::CoreXY,
            vec![0.030, -0.010, 0.004, 1.0, -1.1, 0.005, 0.9, -0.8],
        ),
    ];
    for (s, theta) in cases {
        let p = s.unpack(theta);
        let n = s.axis_count();
        #[allow(clippy::cast_precision_loss)]
        let probes: &[(Vec<f64>, Vec<f64>)] = &[
            (vec![1000.0; n], vec![100.0; n]),
            (vec![-500.0; n], vec![-30.0; n]),
            (
                (0..n).map(|i| 800.0 - 1600.0 * i as f64).collect(),
                (0..n).map(|i| 0.1 * i as f64).collect(),
            ),
        ];
        for (acc, vel) in probes {
            for motor in 0..n {
                let via_row: f64 = s
                    .row(motor, acc, vel)
                    .iter()
                    .zip(theta)
                    .map(|(r, t)| r * t)
                    .sum();
                let via_physics = physical_torque(&p, motor, acc, vel);
                assert!(
                    (via_row - via_physics).abs() < 1e-12,
                    "{s:?} motor {motor}: row {via_row} vs physics {via_physics}"
                );
            }
        }
    }
}

#[test]
fn scalar_row_layout() {
    let s = Structure::CartesianScalar;
    assert_eq!(s.param_count(), 4);
    let row = s.row(0, &[1000.0], &[100.0]);
    assert_eq!(row, vec![1000.0, 100.0, 1.0, 0.0]);
    let row_rev = s.row(0, &[1000.0], &[-100.0]);
    assert_eq!(row_rev, vec![1000.0, -100.0, 0.0, 1.0]);
    let row_dead = s.row(0, &[1000.0], &[COULOMB_DEADBAND_MM_S / 2.0]);
    assert_eq!(row_dead[2], 0.0);
    assert_eq!(row_dead[3], 0.0);
}

#[test]
fn corexy_rows_share_mass_params() {
    let s = Structure::CoreXY;
    assert_eq!(s.param_count(), 8);
    let ra = s.row(0, &[100.0, 50.0], &[10.0, -10.0]);
    assert_eq!(ra, vec![100.0, 50.0, 10.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    let rb = s.row(1, &[100.0, 50.0], &[10.0, -10.0]);
    assert_eq!(rb, vec![50.0, 100.0, 0.0, 0.0, 0.0, -10.0, 0.0, 1.0]);
}

#[test]
fn params_to_profile_blocks() {
    let s = Structure::CoreXY;
    let theta = vec![0.030, -0.010, 0.004, 1.0, -1.1, 0.005, 0.9, -0.8];
    let p = s.unpack(&theta);
    assert_eq!(p.mass, vec![vec![0.030, -0.010], vec![-0.010, 0.030]]);
    assert_eq!(p.viscous, vec![0.004, 0.005]);
    assert_eq!(p.coulomb_fwd, vec![1.0, 0.9]);
    assert_eq!(p.coulomb_rev, vec![-1.1, -0.8]);
}
