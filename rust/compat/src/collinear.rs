use crate::emit::G5Line;

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
mod tests;
