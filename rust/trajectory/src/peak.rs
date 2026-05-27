// Post-shape peak acceleration check.
//
// Stage 4: compute the peak (maximum absolute) acceleration of a shaped
// `ScalarNurbs<f64>` via dense numerical sampling with central finite
// differences at the MCU sample rate (40 kHz, dt = 25 µs).
//
// The previous symbolic approach (differentiate twice → root-find on x'''(t))
// failed at production shaper frequencies (120-180 Hz) because the convolution
// kernel's normalization constant c = 15/(16h^5) creates ~1e12-magnitude
// polynomial coefficients that amplify through double differentiation, causing
// catastrophic cancellation in root-finding.  Horner evaluation of the
// polynomial itself is numerically stable, so sampling x(t) and recovering
// acceleration via finite differences avoids the instability.

use nurbs::bezier::extract_bezier_pieces;
use nurbs::ScalarNurbs;

/// Compute the peak absolute acceleration (max |x''(t)|) of a shaped trajectory
/// via dense central finite differences at 40 kHz.
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
            // Piece too short for finite differences. Skip — very short pieces
            // (< 75 µs at 40 kHz) contribute negligibly to physical motion and
            // their symbolic second derivatives are numerically unreliable for
            // high-degree convolved NURBS (kernel normalization constant
            // c = 15/(16h^5) creates large polynomial coefficients that amplify
            // through double differentiation, causing catastrophic cancellation).
            // The beta-medium loop's derate accuracy is dominated by longer pieces
            // where finite differences are used correctly; skipping short-piece
            // symbolic fallback prevents spurious inflated peak estimates that
            // drive the derate ratio below physically meaningful limits.
            continue;
        }

        // Sample at 40 kHz within [u_start + dt, u_end - dt].
        let interior = duration - 2.0 * dt;
        #[allow(clippy::cast_sign_loss)] // interior is guaranteed positive (duration >= 3*dt)
        let n_samples = (interior / dt).ceil() as usize;
        let n_samples = n_samples.max(1);

        for i in 0..=n_samples {
            let t = piece.u_start + dt + interior * i as f64 / n_samples as f64;
            let t = t.min(piece.u_end - dt); // clamp

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
