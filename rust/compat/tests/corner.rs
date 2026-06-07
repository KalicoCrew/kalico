use compat::corner::{detect_corners, split_at_corners};

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

#[test]
fn right_angle_corner() {
    let points: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
    let corners = detect_corners(&points, 0.05);
    assert_eq!(
        corners,
        vec![1],
        "expected a corner at index 1 for a 90° bend"
    );
}

#[test]
fn gentle_curve_no_corners() {
    use std::f64::consts::PI;

    let n = 20usize;
    let radius = 100.0_f64;
    let total_angle = 10.0_f64 * PI / 180.0;

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

#[test]
fn split_straight_run() {
    let pts: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    let sub_runs = split_at_corners(&pts, &[]);
    assert_eq!(sub_runs.len(), 1);
    assert_eq!(sub_runs[0], pts);
}

#[test]
fn detect_and_split_l_shape() {
    let pts: Vec<[f64; 3]> = vec![[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [5.0, 5.0, 0.0]];
    let corners = detect_corners(&pts, 0.05);
    assert_eq!(corners, vec![1]);

    let sub_runs = split_at_corners(&pts, &corners);
    assert_eq!(sub_runs.len(), 2);
    #[allow(clippy::float_cmp)]
    {
        assert_eq!(sub_runs[0].last().unwrap(), &pts[1]);
        assert_eq!(sub_runs[1].first().unwrap(), &pts[1]);
    }
    assert_eq!(sub_runs[0].len(), 2);
    assert_eq!(sub_runs[1].len(), 2);
}
