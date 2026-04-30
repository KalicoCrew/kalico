//! Corner detection for G1-run segmentation.
//!
//! Uses the junction-deviation criterion `L * tan(θ/4) > tolerance` to
//! identify interior waypoints where the path direction changes sharply
//! enough to require a velocity reduction in the planner.  Runs are split at
//! those points so that each sub-run can be fitted or emitted independently.

/// Detect corners in a polyline where `L * tan(θ/4) > tolerance`.
///
/// `L` is the shorter of the two adjacent segment lengths and `θ` is the
/// deflection angle at the interior point.  Returns a sorted list of
/// **interior** point indices (i.e. 1 ≤ i ≤ n−2 for n points) where the
/// criterion is met.
///
/// Only the XY plane is considered; Z is ignored for angle computation.
///
/// Degenerate case: if either adjacent segment is shorter than 1 µm the
/// point is always treated as a corner.
pub fn detect_corners(points: &[[f64; 3]], tolerance: f64) -> Vec<usize> {
    if points.len() < 3 {
        return Vec::new();
    }

    let mut corners = Vec::new();

    for i in 1..points.len() - 1 {
        let (x0, y0) = (points[i - 1][0], points[i - 1][1]);
        let (x1, y1) = (points[i][0], points[i][1]);
        let (x2, y2) = (points[i + 1][0], points[i + 1][1]);

        let dx0 = x1 - x0;
        let dy0 = y1 - y0;
        let dx1 = x2 - x1;
        let dy1 = y2 - y1;

        let len0 = (dx0 * dx0 + dy0 * dy0).sqrt();
        let len1 = (dx1 * dx1 + dy1 * dy1).sqrt();
        let shorter = len0.min(len1);

        // Degenerate: zero-length segment → always a corner.
        if shorter < 1e-9 {
            corners.push(i);
            continue;
        }

        // θ = angle between the two direction vectors.
        let cross = dx0 * dy1 - dy0 * dx1;
        let dot = dx0 * dx1 + dy0 * dy1;
        let theta = cross.abs().atan2(dot);

        // Deviation = L * tan(θ/4).
        let deviation = shorter * (theta / 4.0).tan();

        if deviation > tolerance {
            corners.push(i);
        }
    }

    corners
}

/// Split a polyline into sub-runs at the given corner indices.
///
/// Adjacent sub-runs share the corner point (it becomes the last point of one
/// sub-run and the first point of the next).  If `corners` is empty the
/// original slice is returned as a single sub-run.
///
/// `corners` must be sorted in ascending order and contain only valid interior
/// indices (1 ≤ i ≤ n−2).
pub fn split_at_corners(points: &[[f64; 3]], corners: &[usize]) -> Vec<Vec<[f64; 3]>> {
    if corners.is_empty() {
        return vec![points.to_vec()];
    }

    let mut result = Vec::new();
    let mut segment_start = 0;

    for &corner in corners {
        // Include the corner point as the last element of this sub-run.
        result.push(points[segment_start..=corner].to_vec());
        segment_start = corner;
    }

    // Tail: from the last corner to the end.
    result.push(points[segment_start..].to_vec());

    result
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    /// Helper: build points along y = 0 at evenly-spaced x positions.
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
        // Corner at index 2; points 0..=2 and 2..=4 share point 2.
        let pts: Vec<[f64; 3]> = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0], // corner
            [1.0, 2.0, 0.0],
            [1.0, 3.0, 0.0],
        ];
        let result = split_at_corners(&pts, &[2]);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].last().unwrap(), &pts[2]);
        assert_eq!(result[1].first().unwrap(), &pts[2]);
    }
}
