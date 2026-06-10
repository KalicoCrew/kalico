use servo_ident::fit::{fit, FitError, FitInput, FitOptions};
use servo_ident::model::Structure;

fn triangle(a: f64, t1: f64, dt: f64, reps: usize) -> (Vec<f64>, Vec<f64>) {
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut v = 0.0;
    for _ in 0..reps {
        for phase in [a, -a, -a, a] {
            let steps = (t1 / dt) as usize;
            for _ in 0..steps {
                acc.push(phase);
                v += phase * dt;
                vel.push(v);
            }
        }
    }
    (acc, vel)
}

fn noisy(x: f64, k: usize) -> f64 {
    let h = k.wrapping_mul(2654435761) as u32;
    x + (f64::from(h % 1000) / 1000.0 - 0.5)
}

#[test]
fn recovers_scalar_truth() {
    let (m, b, cf, cr) = (0.0123, 0.0045, 1.2, -1.1);
    let (acc, vel) = triangle(2000.0, 0.08, 0.001, 6);
    let torque: Vec<f64> = acc
        .iter()
        .zip(&vel)
        .enumerate()
        .map(|(k, (&a, &v))| {
            let c = if v > 0.5 {
                cf
            } else if v < -0.5 {
                cr
            } else {
                0.0
            };
            noisy(m * a + b * v + c, k).round()
        })
        .collect();
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![acc],
        vel: vec![vel],
        torque: vec![torque],
    };
    let r = fit(&input, &FitOptions::default()).unwrap();
    let p = &r.params;
    assert!((p.mass[0][0] - m).abs() < 0.1 * m, "m: {}", p.mass[0][0]);
    assert!((p.viscous[0] - b).abs() < 0.1 * b, "b: {}", p.viscous[0]);
    assert!((p.coulomb_fwd[0] - cf).abs() < 0.5, "cf: {}", p.coulomb_fwd[0]);
    assert!((p.coulomb_rev[0] - cr).abs() < 0.5, "cr: {}", p.coulomb_rev[0]);
    assert!(r.rms_residual < 2.0);
}

#[test]
fn recovers_corexy_coupling() {
    let (md, mo) = (0.030, -0.010);
    let (acc_x, vel_x) = triangle(1500.0, 0.06, 0.001, 4);
    let (acc_y, vel_y) = triangle(1500.0, 0.06, 0.001, 4);
    let acc_a: Vec<f64> = acc_x.iter().chain(&acc_y).copied().collect();
    let vel_a: Vec<f64> = vel_x.iter().chain(&vel_y).copied().collect();
    let acc_b: Vec<f64> = acc_x
        .iter()
        .copied()
        .chain(acc_y.iter().map(|&v| -v))
        .collect();
    let vel_b: Vec<f64> = vel_x
        .iter()
        .copied()
        .chain(vel_y.iter().map(|&v| -v))
        .collect();
    let tq = |acc_self: f64, acc_other: f64, v: f64, k: usize| {
        let c = if v > 0.5 {
            1.0
        } else if v < -0.5 {
            -1.0
        } else {
            0.0
        };
        noisy(md * acc_self + mo * acc_other + 0.004 * v + c, k).round()
    };
    let torque_a: Vec<f64> = (0..acc_a.len())
        .map(|k| tq(acc_a[k], acc_b[k], vel_a[k], k))
        .collect();
    let torque_b: Vec<f64> = (0..acc_b.len())
        .map(|k| tq(acc_b[k], acc_a[k], vel_b[k], k + 7))
        .collect();
    let input = FitInput {
        structure: Structure::CoreXY,
        acc: vec![acc_a, acc_b],
        vel: vec![vel_a, vel_b],
        torque: vec![torque_a, torque_b],
    };
    let r = fit(&input, &FitOptions::default()).unwrap();
    assert!((r.params.mass[0][0] - md).abs() < 0.1 * md);
    assert!((r.params.mass[0][1] - mo).abs() < 0.1 * mo.abs());
}

#[test]
fn refuses_insufficient_excitation() {
    let n = 2000;
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![vec![0.0; n]],
        vel: vec![vec![100.0; n]],
        torque: vec![vec![1.0; n]],
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::InsufficientExcitation { .. })
    ));
}

#[test]
fn refuses_saturated_torque() {
    let (acc, vel) = triangle(2000.0, 0.08, 0.001, 4);
    let n = acc.len();
    let mut torque = vec![100.0; n];
    for t in torque.iter_mut().take(n / 10) {
        *t = 3995.0;
    }
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![acc],
        vel: vec![vel],
        torque: vec![torque],
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::SaturatedTorque { .. })
    ));
}
