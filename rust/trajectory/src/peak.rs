use nurbs::bezier::extract_bezier_pieces;
use nurbs::ScalarNurbs;

pub fn peak_accel(curve: &ScalarNurbs<f64>) -> f64 {
    let pieces = extract_bezier_pieces(curve);
    if pieces.is_empty() {
        return 0.0;
    }

    let dt = 25e-6; // 40 kHz sample rate, matching MCU
    let mut peak = 0.0_f64;

    for piece in &pieces {
        let duration = piece.u_end - piece.u_start;
        if duration < 3.0 * dt {
            continue;
        }

        let interior = duration - 2.0 * dt;
        #[allow(clippy::cast_sign_loss)] // interior is guaranteed positive (duration >= 3*dt)
        let n_samples = (interior / dt).ceil() as usize;
        let n_samples = n_samples.max(1);

        for i in 0..=n_samples {
            let t = piece.u_start + dt + interior * i as f64 / n_samples as f64;
            let t = t.min(piece.u_end - dt);

            let x_prev = piece.evaluate(t - dt);
            let x_curr = piece.evaluate(t);
            let x_next = piece.evaluate(t + dt);
            let accel = (x_next - 2.0 * x_curr + x_prev) / (dt * dt);
            peak = peak.max(accel.abs());
        }
    }

    peak
}

#[cfg(test)]
mod tests;
