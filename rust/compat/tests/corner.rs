use compat::corner::{detect_corners, split_at_corners};

/// Four collinear points along the X axis — no corners expected.
#[test]
fn straight_line_no_corners() {
    let points: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
    ];
    let corners = detect_corners(&points, 0.05);
    assert!(
        corners.is_empty(),
        "collinear points should not produce any corners, got: {corners:?}"
    );
}

/// L-shaped path: move +X then +Y — 90° bend at the junction.
/// With a 1 mm segment and 0.05 mm tolerance the criterion fires.
///
/// L = 1.0, θ = π/2 → deviation = 1.0 * tan(π/8) ≈ 0.414 > 0.05.
#[test]
fn right_angle_corner() {
    let points: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0], // 0
        [1.0, 0.0, 0.0], // 1 ← interior, 90° corner
        [1.0, 1.0, 0.0], // 2
    ];
    let corners = detect_corners(&points, 0.05);
    assert_eq!(
        corners,
        vec![1],
        "expected a corner at index 1 for a 90° bend"
    );
}

/// 20 points along a gentle circular arc (radius = 100 mm, total arc ≈ 10°).
/// The per-segment deviation is well below a 0.05 mm tolerance.
#[test]
fn gentle_curve_no_corners() {
    use std::f64::consts::PI;

    // Arc spans 10° on a circle of radius 100 mm.
    let n = 20usize;
    let radius = 100.0_f64;
    let total_angle = 10.0_f64 * PI / 180.0; // 10° in radians

    let points: Vec<[f64; 3]> = (0..=n)
        .map(|k| {
            let t = k as f64 / n as f64;
            let angle = t * total_angle;
            [radius * angle.cos(), radius * angle.sin(), 0.0]
        })
        .collect();

    let corners = detect_corners(&points, 0.05);
    assert!(
        corners.is_empty(),
        "gentle arc should not produce corners, got indices: {corners:?}"
    );
}

/// Sanity check: `split_at_corners` on a straight run with no corners returns
/// the original points as a single sub-run.
#[test]
fn split_straight_run() {
    let pts: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    let sub_runs = split_at_corners(&pts, &[]);
    assert_eq!(sub_runs.len(), 1);
    assert_eq!(sub_runs[0], pts);
}

/// An L-shaped path: `detect_corners` fires, then `split_at_corners` produces
/// two sub-runs that share the corner point.
#[test]
fn detect_and_split_l_shape() {
    let pts: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0], // interior — 90° corner
        [5.0, 5.0, 0.0],
    ];
    let corners = detect_corners(&pts, 0.05);
    assert_eq!(corners, vec![1]);

    let sub_runs = split_at_corners(&pts, &corners);
    assert_eq!(sub_runs.len(), 2);
    // Both sub-runs share the corner point.
    #[allow(clippy::float_cmp)]
    {
        assert_eq!(sub_runs[0].last().unwrap(), &pts[1]);
        assert_eq!(sub_runs[1].first().unwrap(), &pts[1]);
    }
    // Each sub-run has 2 points.
    assert_eq!(sub_runs[0].len(), 2);
    assert_eq!(sub_runs[1].len(), 2);
}
