//! G5.1 → G5 reduction: exact degree elevation.
//!
//! A G5.1 quadratic Bézier (3 control points) is converted to a cubic Bézier
//! (G5, 4 control points) via the standard degree-elevation formula. This is
//! exact — zero fit error.
//!
//! Control-point layout for the quadratic input (G5.1):
//! - P0 = start (current position, implicit)
//! - P1 = start + (I, J)   [quadratic control point]
//! - P2 = end (X, Y target)
//!
//! Degree elevation to cubic adds a fourth control point:
//! - `CP0_cubic` = P0                        (unchanged)
//! - `CP1_cubic` = (1/3)\*P0 + (2/3)\*P1
//! - `CP2_cubic` = (2/3)\*P1 + (1/3)\*P2
//! - `CP3_cubic` = P2                        (unchanged)
//!
//! G5 `LinuxCNC` convention encodes the cubic control points as offsets:
//! - I = `CP1_cubic.x` − P0.x
//! - J = `CP1_cubic.y` − P0.y
//! - P = `CP2_cubic.x` − P2.x
//! - Q = `CP2_cubic.y` − P2.y
//! - X, Y = P2.x, P2.y (target endpoint)
//! - Z = P2.z (target Z, carried through)

use crate::emit::G5Line;

/// Convert a quadratic Bézier (G5.1) to a cubic Bézier (G5) via exact degree
/// elevation.
///
/// `p0`, `p1`, `p2` are the three XYZ control points of the quadratic:
/// - `p0` = start (current machine position, absolute mm)
/// - `p1` = quadratic control point (absolute mm)
/// - `p2` = end (target position, absolute mm)
///
/// `e_absolute` is the output-side absolute E position (already resolved by
/// the caller from whatever E mode the source G-code used).
///
/// `f` is the feed rate in mm/min, passed through only when it has changed.
pub fn elevate_g51_to_g5(
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    e_absolute: f64,
    f: Option<f64>,
) -> G5Line {
    // Degree-elevation formula (XY only — G5.1 is restricted to the active
    // plane; Z is carried through from P2 unchanged):
    //   CP1_cubic = (1/3)*P0 + (2/3)*P1
    //   CP2_cubic = (2/3)*P1 + (1/3)*P2
    let cp1x = p0[0] / 3.0 + 2.0 * p1[0] / 3.0;
    let cp1y = p0[1] / 3.0 + 2.0 * p1[1] / 3.0;

    let cp2x = 2.0 * p1[0] / 3.0 + p2[0] / 3.0;
    let cp2y = 2.0 * p1[1] / 3.0 + p2[1] / 3.0;

    G5Line {
        x: p2[0],
        y: p2[1],
        z: p2[2],
        i: cp1x - p0[0],
        j: cp1y - p0[1],
        p: cp2x - p2[0],
        q: cp2y - p2[1],
        e: e_absolute,
        f,
    }
}
