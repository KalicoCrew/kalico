use super::*;

#[test]
fn straight_bezier_zero_distance() {
    let p0 = [0.0, 0.0];
    let p1 = [10.0 / 3.0, 0.0];
    let p2 = [20.0 / 3.0, 0.0];
    let p3 = [10.0, 0.0];
    let polyline = vec![[0.0, 0.0], [10.0, 0.0]];
    let d = bezier_to_polyline_hausdorff(p0, p1, p2, p3, &polyline, 1e-9);
    assert!(d < 1e-6, "expected ~0, got {d}");
}

#[test]
fn bulging_bezier_detects_deviation() {
    let p0 = [0.0, 0.0];
    let p1 = [3.0, 5.0];
    let p2 = [7.0, 5.0];
    let p3 = [10.0, 0.0];
    let polyline = vec![[0.0, 0.0], [10.0, 0.0]];
    let d = bezier_to_polyline_hausdorff(p0, p1, p2, p3, &polyline, 1e-6);
    assert!(d > 3.0, "expected bulge >3mm, got {d}");
}

#[test]
fn bezier_on_polyline() {
    let p0 = [0.0, 0.0];
    let p1 = [3.0, 0.0];
    let p2 = [7.0, 0.0];
    let p3 = [10.0, 0.0];
    let polyline = vec![[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]];
    let d = bezier_to_polyline_hausdorff(p0, p1, p2, p3, &polyline, 1e-9);
    assert!(d < 1e-6);
}

#[test]
fn degenerate_polyline_returns_infinity() {
    let p0 = [0.0, 0.0];
    let p1 = [1.0, 0.0];
    let p2 = [2.0, 0.0];
    let p3 = [3.0, 0.0];
    let polyline: Vec<[f64; 2]> = vec![[0.0, 0.0]];
    let d = bezier_to_polyline_hausdorff(p0, p1, p2, p3, &polyline, 1e-6);
    assert!(
        d.is_infinite(),
        "expected infinity for degenerate polyline, got {d}"
    );
}

#[test]
fn point_to_segment_endpoints() {
    let d = point_to_segment_dist([5.0, 0.0], [0.0, 0.0], [3.0, 0.0]);
    assert!((d - 2.0).abs() < 1e-12, "expected 2, got {d}");
}

#[test]
fn bezier_eval_endpoints() {
    let p0 = [1.0, 2.0];
    let p1 = [3.0, 4.0];
    let p2 = [5.0, 6.0];
    let p3 = [7.0, 8.0];
    let start = bezier_eval(p0, p1, p2, p3, 0.0);
    let end = bezier_eval(p0, p1, p2, p3, 1.0);
    assert!((start[0] - p0[0]).abs() < 1e-12);
    assert!((start[1] - p0[1]).abs() < 1e-12);
    assert!((end[0] - p3[0]).abs() < 1e-12);
    assert!((end[1] - p3[1]).abs() < 1e-12);
}

#[test]
fn subdivide_preserves_endpoints() {
    let p0 = [0.0, 0.0];
    let p1 = [1.0, 3.0];
    let p2 = [4.0, 3.0];
    let p3 = [5.0, 0.0];
    let (left, right) = subdivide(p0, p1, p2, p3);
    assert!((left[0][0] - p0[0]).abs() < 1e-12);
    assert!((right[3][0] - p3[0]).abs() < 1e-12);
    let mid = bezier_eval(p0, p1, p2, p3, 0.5);
    assert!((left[3][0] - mid[0]).abs() < 1e-12);
    assert!((right[0][1] - mid[1]).abs() < 1e-12);
}
