use crate::collinear::to_collinear_g5;
use crate::emit::G5Line;

const MAX_DEPTH: u32 = 10;

pub fn fit_subrun(
    points: &[[f64; 3]],
    tolerance_mm: f64,
    start_tangent: Option<[f64; 2]>,
    end_tangent: Option<[f64; 2]>,
) -> Vec<G5Line> {
    if points.len() < 2 {
        return Vec::new();
    }
    fit_recursive(points, tolerance_mm, start_tangent, end_tangent, 0)
}

fn fit_recursive(
    points: &[[f64; 3]],
    tolerance_mm: f64,
    start_tangent: Option<[f64; 2]>,
    end_tangent: Option<[f64; 2]>,
    depth: u32,
) -> Vec<G5Line> {
    debug_assert!(points.len() >= 2);

    if points.len() == 2 {
        return vec![to_collinear_g5(points[0], points[1], 0.0, None)];
    }

    if depth >= MAX_DEPTH {
        return emit_collinear(points);
    }

    let polyline_xy: Vec<[f64; 2]> = points.iter().map(|p| [p[0], p[1]]).collect();
    let p0 = polyline_xy[0];
    let p3 = *polyline_xy.last().unwrap();

    let Some((cp1, cp2)) = fit_single_bezier(&polyline_xy, start_tangent, end_tangent) else {
        return emit_collinear(points);
    };

    let (worst_idx, worst_t, err) = find_worst_point(&polyline_xy, p0, cp1, cp2, p3);

    if err <= tolerance_mm {
        let z_end = points.last().unwrap()[2];
        return vec![bezier_to_g5(p0, cp1, cp2, p3, z_end)];
    }

    if worst_idx == 0 || worst_idx >= points.len() - 1 {
        return emit_collinear(points);
    }

    let tangent = bezier_tangent(p0, cp1, cp2, p3, worst_t);
    let tan_len = (tangent[0] * tangent[0] + tangent[1] * tangent[1]).sqrt();
    let split_tangent = if tan_len > 1e-12 {
        Some([tangent[0] / tan_len, tangent[1] / tan_len])
    } else {
        None
    };

    let left = fit_recursive(
        &points[..=worst_idx],
        tolerance_mm,
        start_tangent,
        split_tangent,
        depth + 1,
    );
    let right = fit_recursive(
        &points[worst_idx..],
        tolerance_mm,
        split_tangent,
        end_tangent,
        depth + 1,
    );

    let mut result = left;
    result.extend(right);
    result
}

fn emit_collinear(points: &[[f64; 3]]) -> Vec<G5Line> {
    points
        .windows(2)
        .map(|w| to_collinear_g5(w[0], w[1], 0.0, None))
        .collect()
}

fn chord_length_params(pts: &[[f64; 2]]) -> Vec<f64> {
    let n = pts.len();
    let mut params = Vec::with_capacity(n);
    params.push(0.0);

    for i in 1..n {
        let dx = pts[i][0] - pts[i - 1][0];
        let dy = pts[i][1] - pts[i - 1][1];
        let chord = (dx * dx + dy * dy).sqrt();
        params.push(params[i - 1] + chord);
    }

    let total = *params.last().unwrap();
    if total > 0.0 {
        for p in &mut params {
            *p /= total;
        }
    }
    params
}

fn fit_single_bezier(
    pts: &[[f64; 2]],
    start_tangent: Option<[f64; 2]>,
    end_tangent: Option<[f64; 2]>,
) -> Option<([f64; 2], [f64; 2])> {
    let n = pts.len();
    if n < 2 {
        return None;
    }

    let p0 = pts[0];
    let p3 = pts[n - 1];
    let params = chord_length_params(pts);

    match (start_tangent, end_tangent) {
        (Some(st), Some(et)) => Some(fit_constrained_both(pts, &params, p0, p3, st, et)),
        (Some(st), None) => fit_constrained_start(pts, &params, p0, p3, st),
        (None, Some(et)) => fit_constrained_end(pts, &params, p0, p3, et),
        (None, None) => Some(fit_unconstrained(pts, &params, p0, p3)),
    }
}

fn bernstein3(t: f64) -> [f64; 4] {
    let s = 1.0 - t;
    let s2 = s * s;
    let t2 = t * t;
    [s2 * s, 3.0 * s2 * t, 3.0 * s * t2, t2 * t]
}

fn bezier_eval(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], t: f64) -> [f64; 2] {
    let b = bernstein3(t);
    [
        b[0] * p0[0] + b[1] * p1[0] + b[2] * p2[0] + b[3] * p3[0],
        b[0] * p0[1] + b[1] * p1[1] + b[2] * p2[1] + b[3] * p3[1],
    ]
}

fn fit_unconstrained(
    pts: &[[f64; 2]],
    params: &[f64],
    p0: [f64; 2],
    p3: [f64; 2],
) -> ([f64; 2], [f64; 2]) {
    let mut ata = [[0.0f64; 2]; 2];
    let mut atb_x = [0.0f64; 2];
    let mut atb_y = [0.0f64; 2];

    for i in 1..pts.len() - 1 {
        let t = params[i];
        let b = bernstein3(t);
        let rhs_x = pts[i][0] - b[0] * p0[0] - b[3] * p3[0];
        let rhs_y = pts[i][1] - b[0] * p0[1] - b[3] * p3[1];

        ata[0][0] += b[1] * b[1];
        ata[0][1] += b[1] * b[2];
        ata[1][0] += b[2] * b[1];
        ata[1][1] += b[2] * b[2];

        atb_x[0] += b[1] * rhs_x;
        atb_x[1] += b[2] * rhs_x;
        atb_y[0] += b[1] * rhs_y;
        atb_y[1] += b[2] * rhs_y;
    }

    let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if det.abs() < 1e-20 {
        let dx = p3[0] - p0[0];
        let dy = p3[1] - p0[1];
        return (
            [p0[0] + dx / 3.0, p0[1] + dy / 3.0],
            [p3[0] - dx / 3.0, p3[1] - dy / 3.0],
        );
    }

    let inv_det = 1.0 / det;

    let cp1_x = (ata[1][1] * atb_x[0] - ata[0][1] * atb_x[1]) * inv_det;
    let cp2_x = (ata[0][0] * atb_x[1] - ata[1][0] * atb_x[0]) * inv_det;
    let cp1_y = (ata[1][1] * atb_y[0] - ata[0][1] * atb_y[1]) * inv_det;
    let cp2_y = (ata[0][0] * atb_y[1] - ata[1][0] * atb_y[0]) * inv_det;

    ([cp1_x, cp1_y], [cp2_x, cp2_y])
}

fn fit_constrained_both(
    pts: &[[f64; 2]],
    params: &[f64],
    p0: [f64; 2],
    p3: [f64; 2],
    st: [f64; 2],
    et: [f64; 2],
) -> ([f64; 2], [f64; 2]) {
    let mut ata = [[0.0f64; 2]; 2];
    let mut atb = [0.0f64; 2];

    for i in 1..pts.len() - 1 {
        let t = params[i];
        let b = bernstein3(t);

        let rhs_x = pts[i][0] - (b[0] + b[1]) * p0[0] - (b[2] + b[3]) * p3[0];
        let rhs_y = pts[i][1] - (b[0] + b[1]) * p0[1] - (b[2] + b[3]) * p3[1];

        let a0_x = b[1] * st[0];
        let a1_x = -b[2] * et[0];
        let a0_y = b[1] * st[1];
        let a1_y = -b[2] * et[1];

        ata[0][0] += a0_x * a0_x + a0_y * a0_y;
        ata[0][1] += a0_x * a1_x + a0_y * a1_y;
        ata[1][0] += a1_x * a0_x + a1_y * a0_y;
        ata[1][1] += a1_x * a1_x + a1_y * a1_y;

        atb[0] += a0_x * rhs_x + a0_y * rhs_y;
        atb[1] += a1_x * rhs_x + a1_y * rhs_y;
    }

    let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if det.abs() < 1e-20 {
        return fallback_tangent(p0, p3, Some(st), Some(et));
    }

    let inv_det = 1.0 / det;
    let alpha = ((ata[1][1] * atb[0] - ata[0][1] * atb[1]) * inv_det).max(0.0);
    let beta = ((ata[0][0] * atb[1] - ata[1][0] * atb[0]) * inv_det).max(0.0);

    if alpha < 1e-12 && beta < 1e-12 {
        return fallback_tangent(p0, p3, Some(st), Some(et));
    }

    let cp1 = [p0[0] + alpha * st[0], p0[1] + alpha * st[1]];
    let cp2 = [p3[0] - beta * et[0], p3[1] - beta * et[1]];
    (cp1, cp2)
}

fn fit_constrained_start(
    pts: &[[f64; 2]],
    params: &[f64],
    p0: [f64; 2],
    p3: [f64; 2],
    st: [f64; 2],
) -> Option<([f64; 2], [f64; 2])> {
    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];

    for i in 1..pts.len() - 1 {
        let t = params[i];
        let b = bernstein3(t);

        let rhs_x = pts[i][0] - (b[0] + b[1]) * p0[0] - b[3] * p3[0];
        let rhs_y = pts[i][1] - (b[0] + b[1]) * p0[1] - b[3] * p3[1];

        let a_x = [b[1] * st[0], b[2], 0.0];
        let a_y = [b[1] * st[1], 0.0, b[2]];

        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += a_x[r] * a_x[c] + a_y[r] * a_y[c];
            }
            atb[r] += a_x[r] * rhs_x + a_y[r] * rhs_y;
        }
    }

    let sol = solve_3x3(ata, atb)?;
    let alpha = sol[0].max(0.0);
    let cp1 = [p0[0] + alpha * st[0], p0[1] + alpha * st[1]];
    let cp2 = [sol[1], sol[2]];
    Some((cp1, cp2))
}

fn fit_constrained_end(
    pts: &[[f64; 2]],
    params: &[f64],
    p0: [f64; 2],
    p3: [f64; 2],
    et: [f64; 2],
) -> Option<([f64; 2], [f64; 2])> {
    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];

    for i in 1..pts.len() - 1 {
        let t = params[i];
        let b = bernstein3(t);

        let rhs_x = pts[i][0] - b[0] * p0[0] - (b[2] + b[3]) * p3[0];
        let rhs_y = pts[i][1] - b[0] * p0[1] - (b[2] + b[3]) * p3[1];

        let a_x = [b[1], 0.0, -b[2] * et[0]];
        let a_y = [0.0, b[1], -b[2] * et[1]];

        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += a_x[r] * a_x[c] + a_y[r] * a_y[c];
            }
            atb[r] += a_x[r] * rhs_x + a_y[r] * rhs_y;
        }
    }

    let sol = solve_3x3(ata, atb)?;
    let cp1 = [sol[0], sol[1]];
    let beta = sol[2].max(0.0);
    let cp2 = [p3[0] - beta * et[0], p3[1] - beta * et[1]];
    Some((cp1, cp2))
}

fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-20 {
        return None;
    }

    let inv = 1.0 / det;

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        * inv;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        * inv;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        * inv;

    Some([x0, x1, x2])
}

fn fallback_tangent(
    p0: [f64; 2],
    p3: [f64; 2],
    st: Option<[f64; 2]>,
    et: Option<[f64; 2]>,
) -> ([f64; 2], [f64; 2]) {
    let dx = p3[0] - p0[0];
    let dy = p3[1] - p0[1];
    let chord = (dx * dx + dy * dy).sqrt();
    let scale = chord / 3.0;

    let cp1 = match st {
        Some(t) => [p0[0] + scale * t[0], p0[1] + scale * t[1]],
        None => [p0[0] + dx / 3.0, p0[1] + dy / 3.0],
    };
    let cp2 = match et {
        Some(t) => [p3[0] - scale * t[0], p3[1] - scale * t[1]],
        None => [p3[0] - dx / 3.0, p3[1] - dy / 3.0],
    };
    (cp1, cp2)
}

fn find_worst_point(
    pts: &[[f64; 2]],
    p0: [f64; 2],
    cp1: [f64; 2],
    cp2: [f64; 2],
    p3: [f64; 2],
) -> (usize, f64, f64) {
    let params = chord_length_params(pts);
    let mut worst_idx = 1;
    let mut worst_dist: f64 = 0.0;

    for i in 1..pts.len() - 1 {
        let t = params[i];
        let bpt = bezier_eval(p0, cp1, cp2, p3, t);
        let dx = pts[i][0] - bpt[0];
        let dy = pts[i][1] - bpt[1];
        let d = (dx * dx + dy * dy).sqrt();
        if d > worst_dist {
            worst_dist = d;
            worst_idx = i;
        }
    }

    (worst_idx, params[worst_idx], worst_dist)
}

fn bezier_tangent(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], t: f64) -> [f64; 2] {
    let s = 1.0 - t;
    let c0 = 3.0 * s * s;
    let c1 = 6.0 * s * t;
    let c2 = 3.0 * t * t;
    [
        c0 * (p1[0] - p0[0]) + c1 * (p2[0] - p1[0]) + c2 * (p3[0] - p2[0]),
        c0 * (p1[1] - p0[1]) + c1 * (p2[1] - p1[1]) + c2 * (p3[1] - p2[1]),
    ]
}

fn bezier_to_g5(p0: [f64; 2], cp1: [f64; 2], cp2: [f64; 2], p3: [f64; 2], z_end: f64) -> G5Line {
    G5Line {
        x: p3[0],
        y: p3[1],
        z: z_end,
        i: cp1[0] - p0[0],
        j: cp1[1] - p0[1],
        p: cp2[0] - p3[0],
        q: cp2[1] - p3[1],
        e: 0.0,
        f: None,
    }
}
