use super::*;

fn textbook_limits() -> Limits {
    Limits {
        v_max: [500.0, 500.0, 500.0],
        a_max: [5_000.0, 5_000.0, 5_000.0],
        j_max: [100_000.0, 100_000.0, 100_000.0],
        a_centripetal_max: 2_500.0,
    }
}

#[test]
fn jd_collinear_no_cap() {
    let t_x = [1.0, 0.0, 0.0];
    let cap = sharp_corner_jd_cap(&t_x, &t_x, &textbook_limits(), 0.05);
    // Collinear should give ∞ (or B_MAX_CENT_CAP.sqrt() = 10000 mm/s).
    assert!(
        cap >= 9999.9,
        "collinear should give ~10000 mm/s cap, got {cap}"
    );
}

#[test]
fn jd_90_degree_corner_matches_klipper() {
    let t_x = [1.0, 0.0, 0.0];
    let t_y = [0.0, 1.0, 0.0];
    let limits = textbook_limits();
    // a · δ = 2500 · 0.05 = 125. v² = 125 · 2.414 = 301.75. v = 17.37 mm/s.
    let cap = sharp_corner_jd_cap(&t_x, &t_y, &limits, 0.05);
    let expected = (limits.a_centripetal_max * 0.05 * 2.414_213_562).sqrt();
    assert!(
        (cap - expected).abs() < 0.05,
        "90° JD: got {cap}, expected ~{expected}",
    );
}

#[test]
fn compute_junction_velocity_g1_to_g1_90deg() {
    let left = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        None,
    )
    .unwrap();
    let right = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]],
        None,
    )
    .unwrap();
    let limits = textbook_limits();
    let result = compute_junction_velocity(&left, &right, &limits, &limits, 0.05);
    let expected = (limits.a_centripetal_max * 0.05 * 2.414_213_562).sqrt();
    assert!(
        (result.v_junction - expected).abs() < 0.05,
        "got {}, expected ~{}",
        result.v_junction,
        expected
    );
    assert!(matches!(
        result.binding_cap,
        JunctionBindingCap::SharpCornerChord
    ));
}
