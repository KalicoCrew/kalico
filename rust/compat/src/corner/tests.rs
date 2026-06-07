#![allow(clippy::float_cmp)]

use super::*;

fn collinear(xs: &[f64]) -> Vec<[f64; 3]> {
    xs.iter().map(|&x| [x, 0.0, 0.0]).collect()
}

#[test]
fn split_no_corners_returns_original() {
    let pts = collinear(&[0.0, 1.0, 2.0, 3.0]);
    let result = split_at_corners(&pts, &[]);
    assert_eq!(result.len(), 1);
    assert_eq!(result[0], pts);
}

#[test]
fn split_one_corner_produces_two_sub_runs() {
    let pts: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 2.0, 0.0],
        [1.0, 3.0, 0.0],
    ];
    let result = split_at_corners(&pts, &[2]);
    assert_eq!(result.len(), 2);
    assert_eq!(result[0].last().unwrap(), &pts[2]);
    assert_eq!(result[1].first().unwrap(), &pts[2]);
}
