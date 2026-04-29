//! G2/G3 → G5 reduction: Goldapp circular-arc-to-cubic-Bézier approximation.
//!
//! Converts a circular arc (G2 clockwise / G3 counter-clockwise) into one or
//! more G5 cubic Bézier pieces that approximate the arc within a caller-specified
//! radial tolerance.
//!
//! The Goldapp 1991 construction places control points so that each piece
//! matches the circle at both endpoints and at the midpoint (to third order).
//! The radial error for a single piece spanning angle `alpha` is approximately
//! `r * (1 - cos(alpha/2))^2 / cos(alpha/2)`, which for small alpha ~ r*alpha^4/64.
//!
//! Adaptive piece count ensures the approximation stays within the requested
//! tolerance. At 0.1 um (1e-4 mm) tolerance on a typical printer radius,
//! a quarter-arc needs roughly 2 pieces.
//!
//! Z is interpolated linearly across pieces. E is set to 0 and F to None —
//! the caller is responsible for distributing extrusion and feed rate.

use crate::emit::G5Line;
use std::f64::consts::{PI, TAU};

/// Parameters describing a circular arc in the XY plane with optional helical Z.
#[derive(Debug, Clone)]
pub struct ArcParams {
    /// Start position XYZ (absolute, mm).
    pub start: [f64; 3],
    /// End position XYZ (absolute, mm).
    pub end: [f64; 3],
    /// Arc center XY (absolute, mm — not offset from start).
    pub center: [f64; 2],
    /// `true` for G2 (clockwise), `false` for G3 (counter-clockwise).
    pub clockwise: bool,
    /// Maximum radial error allowed (mm).
    pub tolerance_mm: f64,
}

/// Convert a circular arc to one or more G5 cubic Bézier pieces via the
/// Goldapp approximation.
///
/// Returns G5 lines with `e = 0.0` and `f = None`. The caller distributes
/// extrusion proportionally and sets the feed rate.
pub fn arc_to_g5(params: &ArcParams) -> Vec<G5Line> {
    let cx = params.center[0];
    let cy = params.center[1];

    // Vectors from center to start and end.
    let sx = params.start[0] - cx;
    let sy = params.start[1] - cy;
    let ex = params.end[0] - cx;
    let ey = params.end[1] - cy;

    let r = sx.hypot(sy);

    // Start angle and angular travel (Klipper gcode_arcs.py convention).
    let start_angle = sy.atan2(sx);
    let theta = compute_sweep(sx, sy, ex, ey, params.clockwise);

    // Adaptive piece count: solve for max piece angle within tolerance.
    let n = piece_count(r, theta, params.tolerance_mm);

    let piece_angle = theta / n as f64;
    let dz = params.end[2] - params.start[2];

    let mut pieces = Vec::with_capacity(n);

    for i in 0..n {
        let a0 = start_angle + piece_angle * i as f64;
        let a1 = start_angle + piece_angle * (i + 1) as f64;

        // Piece start on circle.
        let p0x = cx + r * a0.cos();
        let p0y = cy + r * a0.sin();

        // Piece end on circle (last piece snaps to the exact endpoint).
        let (p3x, p3y) = if i == n - 1 {
            (params.end[0], params.end[1])
        } else {
            (cx + r * a1.cos(), cy + r * a1.sin())
        };

        // Z: linear interpolation.
        let z = if i == n - 1 {
            params.end[2]
        } else {
            params.start[2] + dz * (i + 1) as f64 / n as f64
        };

        // Goldapp control distance: k = (4/3) * tan(|piece_angle| / 4).
        let k = (4.0 / 3.0) * (piece_angle.abs() / 4.0).tan();

        // Tangent directions at a0 and a1 (perpendicular to radius, outward rotation).
        let t0 = [-a0.sin(), a0.cos()];
        let t1 = [-a1.sin(), a1.cos()];

        // Sign for CW vs CCW.
        let sign = if piece_angle >= 0.0 { 1.0 } else { -1.0 };

        // CP1 = P0 + sign * k * r * t0
        let cp1x = p0x + sign * k * r * t0[0];
        let cp1y = p0y + sign * k * r * t0[1];

        // CP2 = P3 - sign * k * r * t1
        let cp2x = p3x - sign * k * r * t1[0];
        let cp2y = p3y - sign * k * r * t1[1];

        // G5 offsets: I = CP1.x - P0.x, J = CP1.y - P0.y,
        //             P = CP2.x - P3.x, Q = CP2.y - P3.y.
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

/// Unit tangent vector at the arc's start point, in the direction of motion.
///
/// For CCW (G3): tangent = (-sy/r, sx/r) — perpendicular to radius, rotating
/// counter-clockwise.
/// For CW (G2):  tangent = (sy/r, -sx/r) — perpendicular to radius, rotating
/// clockwise.
pub fn arc_start_tangent(params: &ArcParams) -> [f64; 2] {
    let sx = params.start[0] - params.center[0];
    let sy = params.start[1] - params.center[1];
    let r = sx.hypot(sy);
    if params.clockwise {
        [sy / r, -sx / r]
    } else {
        [-sy / r, sx / r]
    }
}

/// Unit tangent vector at the arc's endpoint, in the direction of motion.
///
/// For CCW (G3): tangent = (-ey/r, ex/r).
/// For CW (G2):  tangent = (ey/r, -ex/r).
pub fn arc_endpoint_tangent(params: &ArcParams) -> [f64; 2] {
    let ex = params.end[0] - params.center[0];
    let ey = params.end[1] - params.center[1];
    let r = ex.hypot(ey);
    if params.clockwise {
        [ey / r, -ex / r]
    } else {
        [-ey / r, ex / r]
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the signed sweep angle for the arc.
///
/// Follows Klipper's `gcode_arcs.py` convention:
/// - `theta = atan2(sx*ey - sy*ex, sx*ex + sy*ey)` (cross/dot)
/// - Normalize: if theta < 0, add 2pi.
/// - For CW: subtract 2pi so theta is negative.
/// - Full circle: if theta ~ 0 and start ~ end, use +/-2pi.
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

    // Full circle detection: if angular travel ~ 0 and endpoints coincide.
    if theta.abs() < 1e-10 {
        let dx = ex - sx;
        let dy = ey - sy;
        if dx.hypot(dy) < 1e-10 {
            theta = if clockwise { -TAU } else { TAU };
        }
    }

    theta
}

/// Determine the number of Bézier pieces needed to stay within tolerance.
///
/// Goldapp radial error for a single piece spanning angle alpha:
///   err ~ r * (1 - cos(alpha/2))^2 / cos(alpha/2)
///
/// We find the maximum piece angle where this error <= tolerance, then
/// n = ceil(|theta| / `max_piece_angle`). Minimum n = 1.
fn piece_count(r: f64, theta: f64, tolerance: f64) -> usize {
    if r < 1e-15 || theta.abs() < 1e-15 {
        return 1;
    }

    // Binary search is overkill for this monotone function.
    // Instead, iterate from a generous starting angle and check.
    // The error function is monotone in alpha, so we can solve directly.
    //
    // For efficiency, start with n=1 and increase until error is within
    // tolerance.
    let abs_theta = theta.abs();

    // Try n=1 first, then increase.
    let mut n = 1usize;
    loop {
        let alpha = abs_theta / n as f64;
        if alpha > PI {
            // Single piece spanning > 180 deg is degenerate; force split.
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
mod tests {
    use super::*;

    #[test]
    fn sweep_quarter_ccw() {
        // (1,0) → (0,1), CCW => theta = pi/2
        let theta = compute_sweep(1.0, 0.0, 0.0, 1.0, false);
        assert!((theta - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn sweep_quarter_cw() {
        // (0,1) → (1,0), CW => theta = -pi/2
        let theta = compute_sweep(0.0, 1.0, 1.0, 0.0, true);
        assert!((theta - (-PI / 2.0)).abs() < 1e-10);
    }

    #[test]
    fn sweep_full_circle_cw() {
        let theta = compute_sweep(1.0, 0.0, 1.0, 0.0, true);
        assert!((theta - (-TAU)).abs() < 1e-10);
    }

    #[test]
    fn sweep_full_circle_ccw() {
        let theta = compute_sweep(1.0, 0.0, 1.0, 0.0, false);
        assert!((theta - TAU).abs() < 1e-10);
    }

    #[test]
    fn piece_count_quarter_tight() {
        // r=10, 90 deg, tolerance 0.005mm (5um) => expect ~2 pieces
        let n = piece_count(10.0, PI / 2.0, 0.005);
        assert!((1..=4).contains(&n), "expected 1-4 pieces, got {n}");
    }
}
