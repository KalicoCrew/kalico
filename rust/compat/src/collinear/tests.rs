use super::*;

#[test]
fn collinear_bezier_matches_g5line() {
    let start = [10.0, 20.0, 0.0];
    let end = [40.0, 50.0, 0.0];
    let bezier = to_collinear_bezier(start, end);

    assert_eq!(bezier[0], start);
    assert_eq!(bezier[3], end);
    let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    assert!((bezier[1][0] - (start[0] + d[0] / 3.0)).abs() < 1e-12);
    assert!((bezier[1][1] - (start[1] + d[1] / 3.0)).abs() < 1e-12);
    assert!((bezier[1][2] - (start[2] + d[2] / 3.0)).abs() < 1e-12);
    assert!((bezier[2][0] - (start[0] + 2.0 * d[0] / 3.0)).abs() < 1e-12);
    assert!((bezier[2][1] - (start[1] + 2.0 * d[1] / 3.0)).abs() < 1e-12);
    assert!((bezier[2][2] - (start[2] + 2.0 * d[2] / 3.0)).abs() < 1e-12);
}

#[test]
fn collinear_bezier_z_axis_only() {
    let start = [0.0, 0.0, 5.0];
    let end = [0.0, 0.0, 10.0];
    let bezier = to_collinear_bezier(start, end);
    assert_eq!(bezier[0], start);
    assert_eq!(bezier[3], end);
    assert!((bezier[1][2] - (5.0 + 5.0 / 3.0)).abs() < 1e-12);
    assert!((bezier[2][2] - (5.0 + 10.0 / 3.0)).abs() < 1e-12);
}
