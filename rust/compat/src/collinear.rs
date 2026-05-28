//! G1 → G5 reduction: collinear cubic Bézier.
//!
//! A G1 linear move is converted to a single-piece cubic Bézier (G5) with
//! collinear control points at 1/3 and 2/3 lerp along the segment. This is
//! an exact degree-elevation — zero fit error.
//!
//! Control-point layout for G5 (`LinuxCNC` convention):
//! - P0 = start (current position, implicit)
//! - P1 = start + (I, J)   [CP1]
//! - P2 = end   + (P, Q)   [CP2]
//! - P3 = end (X, Y target)
//!
//! For a collinear cubic from `start` to `end`:
//! - CP1 = start + (dx/3, dy/3)  → I = dx/3, J = dy/3
//! - CP2 = end   - (dx/3, dy/3)  → P = -dx/3, Q = -dy/3
//!
//! where dx = end[0] - start[0], dy = end[1] - start[1].

use crate::emit::G5Line;

/// Convert a single linear segment (G0/G1) to a collinear cubic Bézier G5 line.
///
/// `start` and `end` are absolute XYZ coordinates in mm.
/// `e_absolute` is the output-side absolute E position (already resolved by the
/// caller from whatever E mode the source G-code used).
/// `f` is the feed rate in mm/min, passed through only when it has changed.
pub fn to_collinear_g5(start: [f64; 3], end: [f64; 3], e_absolute: f64, f: Option<f64>) -> G5Line {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];

    G5Line {
        x: end[0],
        y: end[1],
        z: end[2],
        i: dx / 3.0,
        j: dy / 3.0,
        p: -dx / 3.0,
        q: -dy / 3.0,
        e: e_absolute,
        f,
    }
}

/// Structured variant of `to_collinear_g5` for bridge callers.
///
/// Returns the 4 cubic Bézier control points [P0, P1, P2, P3] directly
/// as `[[f64; 3]; 4]`. Same 1/3-2/3 lerp math as `to_collinear_g5`,
/// without the `G5Line` text intermediary.
pub fn to_collinear_bezier(start: [f64; 3], end: [f64; 3]) -> [[f64; 3]; 4] {
    let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    let p1 = [
        start[0] + d[0] / 3.0,
        start[1] + d[1] / 3.0,
        start[2] + d[2] / 3.0,
    ];
    let p2 = [
        start[0] + 2.0 * d[0] / 3.0,
        start[1] + 2.0 * d[1] / 3.0,
        start[2] + 2.0 * d[2] / 3.0,
    ];
    [start, p1, p2, end]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collinear_bezier_matches_g5line() {
        let start = [10.0, 20.0, 0.0];
        let end = [40.0, 50.0, 0.0];
        let bezier = to_collinear_bezier(start, end);

        #[allow(clippy::float_cmp)]
        {
            assert_eq!(bezier[0], start);
            assert_eq!(bezier[3], end);
        }
        let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
        assert!((bezier[1][0] - (start[0] + d[0] / 3.0)).abs() < 1e-12);
        assert!((bezier[1][1] - (start[1] + d[1] / 3.0)).abs() < 1e-12);
        assert!((bezier[1][2] - (start[2] + d[2] / 3.0)).abs() < 1e-12);
        assert!((bezier[2][0] - (start[0] + 2.0 * d[0] / 3.0)).abs() < 1e-12);
        assert!((bezier[2][1] - (start[1] + 2.0 * d[1] / 3.0)).abs() < 1e-12);
        assert!((bezier[2][2] - (start[2] + 2.0 * d[2] / 3.0)).abs() < 1e-12);
    }

    #[test]
    fn collinear_bezier_z_axis_only() {
        let start = [0.0, 0.0, 5.0];
        let end = [0.0, 0.0, 10.0];
        let bezier = to_collinear_bezier(start, end);
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(bezier[0], start);
            assert_eq!(bezier[3], end);
        }
        assert!((bezier[1][2] - (5.0 + 5.0 / 3.0)).abs() < 1e-12);
        assert!((bezier[2][2] - (5.0 + 10.0 / 3.0)).abs() < 1e-12);
    }
}
