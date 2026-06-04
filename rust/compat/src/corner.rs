pub fn detect_corners(points: &[[f64; 3]], tolerance: f64) -> Vec<usize> {
    if points.len() < 3 {
        return Vec::new();
    }

    let mut corners = Vec::new();

    for i in 1..points.len() - 1 {
        let (x0, y0) = (points[i - 1][0], points[i - 1][1]);
        let (x1, y1) = (points[i][0], points[i][1]);
        let (x2, y2) = (points[i + 1][0], points[i + 1][1]);

        let dx0 = x1 - x0;
        let dy0 = y1 - y0;
        let dx1 = x2 - x1;
        let dy1 = y2 - y1;

        let len0 = (dx0 * dx0 + dy0 * dy0).sqrt();
        let len1 = (dx1 * dx1 + dy1 * dy1).sqrt();
        let shorter = len0.min(len1);

        if shorter < 1e-9 {
            corners.push(i);
            continue;
        }

        let cross = dx0 * dy1 - dy0 * dx1;
        let dot = dx0 * dx1 + dy0 * dy1;
        let theta = cross.abs().atan2(dot);

        let deviation = shorter * (theta / 4.0).tan();

        if deviation > tolerance {
            corners.push(i);
        }
    }

    corners
}

pub fn split_at_corners(points: &[[f64; 3]], corners: &[usize]) -> Vec<Vec<[f64; 3]>> {
    if corners.is_empty() {
        return vec![points.to_vec()];
    }

    let mut result = Vec::new();
    let mut segment_start = 0;

    for &corner in corners {
        result.push(points[segment_start..=corner].to_vec());
        segment_start = corner;
    }

    result.push(points[segment_start..].to_vec());

    result
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
