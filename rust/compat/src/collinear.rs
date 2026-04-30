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
