use servo_ident::capture::parse_capture_csv;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

#[test]
fn parses_and_differentiates() {
    let mut csv = String::from("t,target_x,torque_x\n");
    for k in 0..100 {
        let t = k as f64 * 0.001;
        csv.push_str(&format!("{t},{},{}\n", 0.5 * 1000.0 * t * t, 12.0));
    }
    let cap = parse_capture_csv(&csv, &["x"]).unwrap();
    assert_eq!(cap.torque[0][50], 12.0);
    assert!((cap.vel[0][50] - 1000.0 * 0.050).abs() < 1.0);
    assert!((cap.acc[0][50] - 1000.0).abs() < 5.0);
    assert_eq!(cap.acc[0].len(), 100);
}

#[test]
fn rejects_missing_column() {
    assert!(parse_capture_csv("t,target_x\n0,0\n", &["x"]).is_err());
}

#[test]
fn renders_loadable_profile() {
    let p = PhysicalParams {
        mass: vec![vec![0.0123]],
        viscous: vec![0.0045],
        coulomb_fwd: vec![1.2],
        coulomb_rev: vec![-1.1],
    };
    let toml_text = render_profile(&p, &["x"], &[0.8]);
    assert!(toml_text.contains("version = 1"));
    assert!(toml_text.contains("coulomb_deadband_mm_s = 0.5"));
    assert!(toml_text.contains("mass = [[0.0123]]"));
}

#[test]
fn c0006_matches_hand_calculation() {
    let j_total = 0.0123 * (1.27 / 1000.0) * 40.0 / (2.0 * std::f64::consts::PI);
    let rotor = 0.269e-4;
    let expect = (j_total - rotor) / rotor * 100.0;
    let got = c0006_recommendation(0.0123, 1.27, 40.0, rotor);
    assert!((got - expect).abs() < 1e-9, "{got} vs {expect}");
    assert!((got - 269.69).abs() < 0.01, "independent pin: {got}");
}

#[test]
fn renders_integer_valued_floats_as_toml_floats() {
    let p = PhysicalParams {
        mass: vec![vec![2.0]],
        viscous: vec![0.0],
        coulomb_fwd: vec![1.0],
        coulomb_rev: vec![-1.0],
    };
    let toml_text = render_profile(&p, &["x"], &[1.0]);
    assert!(toml_text.contains("mass = [[2.0]]"), "{toml_text}");
    assert!(toml_text.contains("viscous = [0.0]"), "{toml_text}");
    assert!(toml_text.contains("fit_rms_residual = [1.0]"), "{toml_text}");
}
