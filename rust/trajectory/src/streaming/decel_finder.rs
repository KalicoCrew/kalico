use crate::fit::FittedSegment;

pub(super) fn find_decel_start_time(fitted: &[FittedSegment]) -> f64 {
    const SAMPLES_PER_PIECE: usize = 32;
    const V_SQ_EPSILON: f64 = 1e-6;

    let mut samples: Vec<(f64, f64)> = Vec::new();
    for f in fitted {
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[0]);
        let y_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[1]);
        for (xp, yp) in x_pieces.iter().zip(y_pieces.iter()) {
            let dx = xp.differentiate();
            let dy = yp.differentiate();
            let u0 = xp.u_start;
            let u1 = xp.u_end;
            let start_s = usize::from(!samples.is_empty());
            for s in start_s..=SAMPLES_PER_PIECE {
                let t = u0 + (u1 - u0) * (s as f64) / (SAMPLES_PER_PIECE as f64);
                let vx = dx.evaluate(t);
                let vy = dy.evaluate(t);
                let v_sq = vx * vx + vy * vy;
                samples.push((t, v_sq));
            }
        }
    }

    if samples.len() < 2 {
        return fitted[0].t_start;
    }

    let mut i = samples.len() - 1;
    while i > 0 {
        let v_later = samples[i].1;
        let v_earlier = samples[i - 1].1;
        if v_earlier > v_later + V_SQ_EPSILON {
            i -= 1;
        } else {
            break;
        }
    }
    samples[i].0
}
