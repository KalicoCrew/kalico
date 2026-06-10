use servo_ident::model::{Structure, COULOMB_DEADBAND_MM_S};

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
