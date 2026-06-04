use crate::emit::G5Line;
use std::f64::consts::{PI, TAU};

#[derive(Debug, Clone)]
pub struct ArcParams {
    pub start: [f64; 3],
    pub end: [f64; 3],
    pub center: [f64; 2],
    pub clockwise: bool,
    pub tolerance_mm: f64,
}

pub fn arc_to_g5(params: &ArcParams) -> Vec<G5Line> {
    let cx = params.center[0];
    let cy = params.center[1];

    let sx = params.start[0] - cx;
    let sy = params.start[1] - cy;
    let ex = params.end[0] - cx;
    let ey = params.end[1] - cy;

    let r = sx.hypot(sy);

    let start_angle = sy.atan2(sx);
    let theta = compute_sweep(sx, sy, ex, ey, params.clockwise);

    let n = piece_count(r, theta, params.tolerance_mm);

    let piece_angle = theta / n as f64;
    let dz = params.end[2] - params.start[2];

    let mut pieces = Vec::with_capacity(n);

    for i in 0..n {
        let a0 = start_angle + piece_angle * i as f64;
        let a1 = start_angle + piece_angle * (i + 1) as f64;

        let p0x = cx + r * a0.cos();
        let p0y = cy + r * a0.sin();

        let (p3x, p3y) = if i == n - 1 {
            (params.end[0], params.end[1])
        } else {
            (cx + r * a1.cos(), cy + r * a1.sin())
        };

        let z = if i == n - 1 {
            params.end[2]
        } else {
            params.start[2] + dz * (i + 1) as f64 / n as f64
        };

        let k = (4.0 / 3.0) * (piece_angle.abs() / 4.0).tan();
        let t0 = [-a0.sin(), a0.cos()];
        let t1 = [-a1.sin(), a1.cos()];
        let sign = if piece_angle >= 0.0 { 1.0 } else { -1.0 };
        let cp1x = p0x + sign * k * r * t0[0];
        let cp1y = p0y + sign * k * r * t0[1];
        let cp2x = p3x - sign * k * r * t1[0];
        let cp2y = p3y - sign * k * r * t1[1];

        pieces.push(G5Line {
            x: p3x,
            y: p3y,
            z,
            i: cp1x - p0x,
            j: cp1y - p0y,
            p: cp2x - p3x,
            q: cp2y - p3y,
            e: 0.0,
            f: None,
        });
    }

    pieces
}

pub fn arc_start_tangent(params: &ArcParams) -> [f64; 2] {
    let sx = params.start[0] - params.center[0];
    let sy = params.start[1] - params.center[1];
    let r = sx.hypot(sy);
    if r < 1e-12 {
        return [1.0, 0.0];
    }
    if params.clockwise {
        [sy / r, -sx / r]
    } else {
        [-sy / r, sx / r]
    }
}

pub fn arc_endpoint_tangent(params: &ArcParams) -> [f64; 2] {
    let ex = params.end[0] - params.center[0];
    let ey = params.end[1] - params.center[1];
    let r = ex.hypot(ey);
    if r < 1e-12 {
        return [1.0, 0.0];
    }
    if params.clockwise {
        [ey / r, -ex / r]
    } else {
        [-ey / r, ex / r]
    }
}

fn compute_sweep(sx: f64, sy: f64, ex: f64, ey: f64, clockwise: bool) -> f64 {
    let cross = sx * ey - sy * ex;
    let dot = sx * ex + sy * ey;
    let mut theta = cross.atan2(dot);

    if theta < 0.0 {
        theta += TAU;
    }

    if clockwise {
        theta -= TAU;
    }

    if theta.abs() < 1e-10 {
        let dx = ex - sx;
        let dy = ey - sy;
        if dx.hypot(dy) < 1e-10 {
            theta = if clockwise { -TAU } else { TAU };
        }
    }

    theta
}

fn piece_count(r: f64, theta: f64, tolerance: f64) -> usize {
    if r < 1e-15 || theta.abs() < 1e-15 {
        return 1;
    }

    let abs_theta = theta.abs();
    let mut n = 1usize;
    loop {
        let alpha = abs_theta / n as f64;
        if alpha > PI {
            n += 1;
            continue;
        }
        let half = alpha / 2.0;
        let cos_half = half.cos();
        if cos_half.abs() < 1e-15 {
            n += 1;
            continue;
        }
        let err = r * (1.0 - cos_half).powi(2) / cos_half;
        if err <= tolerance {
            break;
        }
        n += 1;
    }
    n
}

#[cfg(test)]
mod tests;
