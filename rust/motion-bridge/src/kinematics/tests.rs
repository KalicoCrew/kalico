use super::*;

const CARTESIAN: u8 = 1;
const COREXY: u8 = 0;

#[test]
fn corexy_forward_is_sum_and_difference() {
    assert_eq!(forward_corexy(3.0, 1.0), (4.0, 2.0));
}
#[test]
fn corexy_inverse_recovers_toolhead() {
    assert_eq!(inverse_corexy(4.0, 2.0), (3.0, 1.0));
}
#[test]
fn corexy_round_trip_identity() {
    for (x, y) in [(0.0, 0.0), (10.0, -7.5), (-3.25, 100.0), (0.1, 0.2)] {
        let (a, b) = forward_corexy(x, y);
        let (rx, ry) = inverse_corexy(a, b);
        assert!((rx - x).abs() < 1e-12 && (ry - y).abs() < 1e-12);
    }
}
#[test]
fn forward_inverse_round_trip_by_tag() {
    for tag in [CARTESIAN, COREXY] {
        let p = [12.0, -4.0, 3.0];
        let motor = forward(tag, p);
        let back = inverse(tag, motor);
        for i in 0..3 {
            assert!((back[i] - p[i]).abs() < 1e-12, "tag {tag} axis {i}");
        }
    }
}
