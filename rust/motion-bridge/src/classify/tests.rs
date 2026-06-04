use super::*;

#[test]
fn xy_travel_classifies_correctly() {
    let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    assert!(matches!(m.class, MoveClass::XyTravel));
    assert_eq!(m.segment.e_mode, EMode::Travel);
    assert_eq!(m.segment.feedrate_mm_s, 100.0);
    let cps = m.segment.xyz.control_points();
    assert_eq!(cps.len(), 4);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]);
    assert!((cps[3][0] - 10.0).abs() < 1e-12);
}

#[test]
fn z_only_classifies_correctly() {
    let m = classify_and_build([0.0, 0.0, 5.0], 0.0, 0.0, 5.0, 0.0, 50.0).unwrap();
    assert!(matches!(m.class, MoveClass::ZOnly));
}

#[test]
fn extrusion_rejected() {
    let r = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 1.0, 100.0);
    assert!(matches!(r, Err(ClassifyError::ExtrusionNotSupported)));
}

#[test]
fn zero_displacement_rejected() {
    let r = classify_and_build([0.0; 3], 0.0, 0.0, 0.0, 0.0, 100.0);
    assert!(matches!(r, Err(ClassifyError::ZeroDisplacement)));
}

#[test]
fn nominal_duration_uses_distance_over_feedrate() {
    let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
    assert!((m.nominal_duration() - 0.1).abs() < 1e-12);
}

#[test]
fn nominal_duration_uses_3d_distance() {
    let m = classify_and_build([0.0; 3], 3.0, 4.0, 0.0, 0.0, 5.0).unwrap();
    assert!((m.nominal_duration() - 1.0).abs() < 1e-12);
}
