use kalico_ethercat_rt::dynamics::DynamicsModel;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::render_profile;

#[test]
fn fitter_rendered_profile_loads_and_evaluates() {
    let p = PhysicalParams {
        mass: vec![vec![0.030, -0.010], vec![-0.010, 0.030]],
        viscous: vec![0.004, 0.005],
        coulomb_fwd: vec![1.0, 0.9],
        coulomb_rev: vec![-1.1, -0.8],
    };
    let text = render_profile(&p, &["a", "b"], &[0.5, 0.6]);
    let m = DynamicsModel::from_toml_str(&text).expect("fitter output must load");
    assert_eq!(m.n, 2);
    assert_eq!(m.axes, ["a", "b"]);
    let heavy = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0]);
    assert!((heavy - 40.0).abs() < 1e-3, "{heavy}");
    let light = m.torque_ff(0, &[1000.0, 1000.0], &[0.0, 0.0]);
    assert!((light - 20.0).abs() < 1e-3, "{light}");
}

#[test]
fn integer_valued_fit_results_still_load() {
    let p = PhysicalParams {
        mass: vec![vec![2.0]],
        viscous: vec![0.0],
        coulomb_fwd: vec![1.0],
        coulomb_rev: vec![-1.0],
    };
    let text = render_profile(&p, &["x"], &[1.0]);
    let m = DynamicsModel::from_toml_str(&text).expect("integer-valued floats must load");
    assert_eq!(m.n, 1);
}
