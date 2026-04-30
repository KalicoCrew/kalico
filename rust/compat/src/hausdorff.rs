//! Approximate Hausdorff distance from a cubic Bézier curve to a polyline.
//!
//! Used by the compat layer to verify that arc → multi-piece Bézier reductions
//! stay within the required L∞ tolerance (0.1 µm for G2/G3 → G5 via Goldapp
//! 1991).
//!
//! The algorithm is a recursive subdivision approach: split the Bézier in half
//! until the piece is "flat enough" (control-point deviation from chord < flatness
//! tolerance), then sample uniformly along the flat piece and record the maximum
//! distance from any sample to the nearest point on the polyline.

/// Evaluate a cubic Bézier at parameter `t` via de Casteljau.
fn bezier_eval(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], t: f64) -> [f64; 2] {
    let s = 1.0 - t;
    // Level 1
    let q0 = [s * p0[0] + t * p1[0], s * p0[1] + t * p1[1]];
    let q1 = [s * p1[0] + t * p2[0], s * p1[1] + t * p2[1]];
    let q2 = [s * p2[0] + t * p3[0], s * p2[1] + t * p3[1]];
    // Level 2
    let r0 = [s * q0[0] + t * q1[0], s * q0[1] + t * q1[1]];
    let r1 = [s * q1[0] + t * q2[0], s * q1[1] + t * q2[1]];
    // Level 3
    [s * r0[0] + t * r1[0], s * r0[1] + t * r1[1]]
}

/// Split a cubic Bézier at t = 0.5 using de Casteljau.
///
/// Returns `(left, right)` where each half is `[p0, p1, p2, p3]`.
fn subdivide(
    p0: [f64; 2],
    p1: [f64; 2],
    p2: [f64; 2],
    p3: [f64; 2],
) -> ([[f64; 2]; 4], [[f64; 2]; 4]) {
    let mid = |a: [f64; 2], b: [f64; 2]| -> [f64; 2] { [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5] };
    // Level 1
    let q0 = mid(p0, p1);
    let q1 = mid(p1, p2);
    let q2 = mid(p2, p3);
    // Level 2
    let r0 = mid(q0, q1);
    let r1 = mid(q1, q2);
    // Level 3 — split point
    let s0 = mid(r0, r1);

    ([p0, q0, r0, s0], [s0, r1, q2, p3])
}

/// Distance from point `p` to line segment `[a, b]`.
fn point_to_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let ab_len_sq = ab[0] * ab[0] + ab[1] * ab[1];

    if ab_len_sq == 0.0 {
        // Degenerate segment — a and b are the same point.
        return (ap[0] * ap[0] + ap[1] * ap[1]).sqrt();
    }

    // Project ap onto ab, clamp to [0, 1].
    let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / ab_len_sq).clamp(0.0, 1.0);
    let closest = [a[0] + t * ab[0], a[1] + t * ab[1]];
    let dx = p[0] - closest[0];
    let dy = p[1] - closest[1];
    (dx * dx + dy * dy).sqrt()
}

/// Minimum distance from point `p` to any segment in `polyline`.
///
/// `polyline` is a sequence of vertices; the segments are
/// `polyline[0]→polyline[1]`, `polyline[1]→polyline[2]`, …
///
/// Returns `f64::INFINITY` if `polyline` has fewer than 2 vertices.
pub fn point_to_polyline_dist(p: [f64; 2], polyline: &[[f64; 2]]) -> f64 {
    if polyline.len() < 2 {
        return f64::INFINITY;
    }
    polyline
        .windows(2)
        .map(|seg| point_to_segment_dist(p, seg[0], seg[1]))
        .fold(f64::INFINITY, f64::min)
}

/// Maximum deviation of the interior control points from the chord p0→p3.
///
/// This is the standard "flatness" test for Bézier subdivision: if both p1
/// and p2 are within `flatness_tol` of the chord, the piece is considered flat.
fn flatness(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]) -> f64 {
    let d1 = point_to_segment_dist(p1, p0, p3);
    let d2 = point_to_segment_dist(p2, p0, p3);
    d1.max(d2)
}

/// Number of uniform samples taken per flat leaf piece.
const LEAF_SAMPLES: usize = 16;

/// Maximum recursion depth (guards against degenerate / near-degenerate input).
const MAX_DEPTH: u32 = 20;

/// Recursive subdivision worker.
///
/// Returns the maximum distance from any point on the Bézier piece
/// `[p0, p1, p2, p3]` (in the range `[t_lo, t_hi]` of the original curve)
/// to its nearest point on `polyline`.
fn hausdorff_recurse(
    p0: [f64; 2],
    p1: [f64; 2],
    p2: [f64; 2],
    p3: [f64; 2],
    polyline: &[[f64; 2]],
    flatness_tol: f64,
    depth: u32,
) -> f64 {
    if depth >= MAX_DEPTH || flatness(p0, p1, p2, p3) < flatness_tol {
        // Flat enough (or depth limit reached): sample uniformly along this piece
        // and return the max distance to the polyline.
        let mut max_dist: f64 = 0.0;
        for i in 0..=LEAF_SAMPLES {
            let t = i as f64 / LEAF_SAMPLES as f64;
            let pt = bezier_eval(p0, p1, p2, p3, t);
            let d = point_to_polyline_dist(pt, polyline);
            if d > max_dist {
                max_dist = d;
            }
        }
        max_dist
    } else {
        let (left, right) = subdivide(p0, p1, p2, p3);
        let d_left = hausdorff_recurse(
            left[0],
            left[1],
            left[2],
            left[3],
            polyline,
            flatness_tol,
            depth + 1,
        );
        let d_right = hausdorff_recurse(
            right[0],
            right[1],
            right[2],
            right[3],
            polyline,
            flatness_tol,
            depth + 1,
        );
        d_left.max(d_right)
    }
}

/// Approximate one-sided Hausdorff distance from the cubic Bézier
/// `(p0, p1, p2, p3)` to `polyline`.
///
/// Returns the maximum distance from any point on the Bézier curve to its
/// nearest point on the polyline (max over the curve, min over polyline
/// segments).
///
/// # Parameters
///
/// - `p0 … p3` — control points of the cubic Bézier in the XY plane (mm).
/// - `polyline` — ordered sequence of 2-D vertices forming the reference
///   polyline.  Must have at least 2 vertices; returns `f64::INFINITY`
///   otherwise.
/// - `flatness_tol` — subdivision stops when the deviation of the interior
///   control points from the chord is below this value (mm).  A value of
///   `1e-6` is suitable for the 0.1 µm verification use-case.
pub fn bezier_to_polyline_hausdorff(
    p0: [f64; 2],
    p1: [f64; 2],
    p2: [f64; 2],
    p3: [f64; 2],
    polyline: &[[f64; 2]],
    flatness_tol: f64,
) -> f64 {
    hausdorff_recurse(p0, p1, p2, p3, polyline, flatness_tol, 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_bezier_zero_distance() {
        // Collinear Bézier from (0,0) to (10,0) — distance to that same line = 0.
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
        // Bézier that follows a 2-segment polyline closely.
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
        let polyline: Vec<[f64; 2]> = vec![[0.0, 0.0]]; // single vertex — no segments
        let d = bezier_to_polyline_hausdorff(p0, p1, p2, p3, &polyline, 1e-6);
        assert!(
            d.is_infinite(),
            "expected infinity for degenerate polyline, got {d}"
        );
    }

    #[test]
    fn point_to_segment_endpoints() {
        // Point beyond the end of the segment should clamp to the endpoint.
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
        // Left piece starts at p0, right piece ends at p3.
        assert!((left[0][0] - p0[0]).abs() < 1e-12);
        assert!((right[3][0] - p3[0]).abs() < 1e-12);
        // The join point (left[3] == right[0]) should equal bezier_eval at t=0.5.
        let mid = bezier_eval(p0, p1, p2, p3, 0.5);
        assert!((left[3][0] - mid[0]).abs() < 1e-12);
        assert!((right[0][1] - mid[1]).abs() < 1e-12);
    }
}
