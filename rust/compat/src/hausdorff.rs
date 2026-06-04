fn bezier_eval(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], t: f64) -> [f64; 2] {
    let s = 1.0 - t;
    let q0 = [s * p0[0] + t * p1[0], s * p0[1] + t * p1[1]];
    let q1 = [s * p1[0] + t * p2[0], s * p1[1] + t * p2[1]];
    let q2 = [s * p2[0] + t * p3[0], s * p2[1] + t * p3[1]];
    let r0 = [s * q0[0] + t * q1[0], s * q0[1] + t * q1[1]];
    let r1 = [s * q1[0] + t * q2[0], s * q1[1] + t * q2[1]];
    [s * r0[0] + t * r1[0], s * r0[1] + t * r1[1]]
}

fn subdivide(
    p0: [f64; 2],
    p1: [f64; 2],
    p2: [f64; 2],
    p3: [f64; 2],
) -> ([[f64; 2]; 4], [[f64; 2]; 4]) {
    let mid = |a: [f64; 2], b: [f64; 2]| -> [f64; 2] { [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5] };
    let q0 = mid(p0, p1);
    let q1 = mid(p1, p2);
    let q2 = mid(p2, p3);
    let r0 = mid(q0, q1);
    let r1 = mid(q1, q2);
    let s0 = mid(r0, r1);

    ([p0, q0, r0, s0], [s0, r1, q2, p3])
}

fn point_to_segment_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let ab_len_sq = ab[0] * ab[0] + ab[1] * ab[1];

    if ab_len_sq == 0.0 {
        return (ap[0] * ap[0] + ap[1] * ap[1]).sqrt();
    }

    let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / ab_len_sq).clamp(0.0, 1.0);
    let closest = [a[0] + t * ab[0], a[1] + t * ab[1]];
    let dx = p[0] - closest[0];
    let dy = p[1] - closest[1];
    (dx * dx + dy * dy).sqrt()
}

pub fn point_to_polyline_dist(p: [f64; 2], polyline: &[[f64; 2]]) -> f64 {
    if polyline.len() < 2 {
        return f64::INFINITY;
    }
    polyline
        .windows(2)
        .map(|seg| point_to_segment_dist(p, seg[0], seg[1]))
        .fold(f64::INFINITY, f64::min)
}

fn flatness(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]) -> f64 {
    let d1 = point_to_segment_dist(p1, p0, p3);
    let d2 = point_to_segment_dist(p2, p0, p3);
    d1.max(d2)
}

const LEAF_SAMPLES: usize = 16;
const MAX_DEPTH: u32 = 20;

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
mod tests;
