use crate::emit::G5Line;

pub fn elevate_g51_to_g5(
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    e_absolute: f64,
    f: Option<f64>,
) -> G5Line {
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
