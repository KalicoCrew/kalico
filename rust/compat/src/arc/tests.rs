use super::*;

#[test]
fn sweep_quarter_ccw() {
    let theta = compute_sweep(1.0, 0.0, 0.0, 1.0, false);
    assert!((theta - PI / 2.0).abs() < 1e-10);
}

#[test]
fn sweep_quarter_cw() {
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
    let n = piece_count(10.0, PI / 2.0, 0.005);
    assert!((1..=4).contains(&n), "expected 1-4 pieces, got {n}");
}
