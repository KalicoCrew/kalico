use super::*;

#[test]
fn sweep_quarter_ccw() {
    // (1,0) → (0,1), CCW => theta = pi/2
    let theta = compute_sweep(1.0, 0.0, 0.0, 1.0, false);
    assert!((theta - PI / 2.0).abs() < 1e-10);
}

#[test]
fn sweep_quarter_cw() {
    // (0,1) → (1,0), CW => theta = -pi/2
    let theta = compute_sweep(0.0, 1.0, 1.0, 0.0, true);
    assert!((theta - (-PI / 2.0)).abs() < 1e-10);
}

#[test]
fn sweep_full_circle_cw() {
    let theta = compute_sweep(1.0, 0.0, 1.0, 0.0, true);
    assert!((theta - (-TAU)).abs() < 1e-10);
}

#[test]
fn sweep_full_circle_ccw() {
    let theta = compute_sweep(1.0, 0.0, 1.0, 0.0, false);
    assert!((theta - TAU).abs() < 1e-10);
}

#[test]
fn piece_count_quarter_tight() {
    // r=10, 90 deg, tolerance 0.005mm (5um) => expect ~2 pieces
    let n = piece_count(10.0, PI / 2.0, 0.005);
    assert!((1..=4).contains(&n), "expected 1-4 pieces, got {n}");
}
