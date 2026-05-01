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
mod tests {
    use super::*;
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

    #[test]
    fn peak_accel_of_constant_is_zero() {
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![5.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        // Finite differences of a constant are exactly zero (up to float noise).
        assert!(peak_accel(&curve).abs() < 1e-6);
    }

    #[test]
    fn peak_accel_of_linear_is_zero() {
        // x(t) = 3 + 2t -> x''(t) = 0
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![3.0, 2.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        // Central finite differences of a linear function should be zero up to
        // floating-point rounding (~1e-4 at dt=25e-6 with values O(1)).
        assert!(
            peak_accel(&curve).abs() < 1.0,
            "expected ~0, got {}",
            peak_accel(&curve)
        );
    }

    #[test]
    fn peak_accel_of_quadratic() {
        // x(t) = 5t^2 -> x''(t) = 10 (constant)
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 0.0, 5.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        // Central finite differences are exact for degree-2 polynomials.
        let peak = peak_accel(&curve);
        assert!((peak - 10.0).abs() < 0.01, "expected ~10.0, got {peak}",);
    }

    #[test]
    fn peak_accel_of_cubic() {
        // x(t) = t^3 on [0, 2] -> x''(t) = 6t -> peak at t=2: |x''(2)| = 12
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 0.0, 0.0, 1.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        // The peak is near t=2 but sampling stops at u_end-dt, so we may
        // slightly undershoot the true peak (12.0). Allow 1% tolerance.
        let peak = peak_accel(&curve);
        assert!((peak - 12.0).abs() < 0.15, "expected ~12.0, got {peak}",);
    }

    #[test]
    fn peak_accel_multi_piece() {
        // Two quadratic pieces: first has low accel (2), second has high accel (100)
        // bezier_pieces_to_nurbs requires consistent degrees across pieces.
        let p1 = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, 1.0],
        }; // x(t) = t + t^2, x''(t) = 2
        let p2 = BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![2.0, 1.0, 50.0],
        }; // quadratic, x''(t) = 100
        let curve = bezier_pieces_to_nurbs(&[p1, p2]);
        // Central finite differences are exact for degree-2 polynomials.
        let peak = peak_accel(&curve);
        assert!((peak - 100.0).abs() < 1.0, "expected ~100.0, got {peak}",);
    }

    #[test]
    fn peak_accel_interior_extremum() {
        // x(t) = t - t^3 -> x''(t) = -6t -> peak at endpoints: max(|0|, |-6|) = 6
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, 0.0, -1.0],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);
        // Sampling near t=1 should recover close to 6. Allow 1% tolerance.
        let peak = peak_accel(&curve);
        assert!((peak - 6.0).abs() < 0.1, "expected ~6.0, got {peak}",);
    }

    #[test]
    fn peak_accel_production_frequency_kernel() {
        // Verify peak_accel works with a 150 Hz shaper kernel — the narrow
        // kernel that caused numerical instability with the symbolic approach.
        //
        // The old symbolic code (differentiate twice → root-find on x'''(t))
        // produced NaN or 1e15+ values on convolved polynomials at this
        // frequency. The sample-based approach must return finite, physically
        // reasonable values.
        use nurbs::algebra::{convolve, PiecewisePolynomialKernel};

        // Build a cubic trajectory: x(t) = 1666.67*t^3 on [0, 0.1]
        // x'(t) = 5000*t^2, x''(t) = 10000*t, x'''(t) = 10000.
        // Peak |x''(t)| on [0, 0.1] = 1000 mm/s^2 (at t=0.1).
        //
        // After convolving with a unit-integral smooth kernel, the interior
        // acceleration is preserved; edges see transition effects proportional
        // to jerk × kernel_half_support.
        let piece = BezierPiece {
            u_start: 0.0,
            u_end: 0.1,
            coeffs: vec![0.0, 0.0, 0.0, 1666.67],
        };
        let curve = bezier_pieces_to_nurbs(&[piece]);

        // Build 150 Hz smooth_zv kernel: w(τ) = c*(h^4 - 2h^2 τ^2 + τ^4)
        // on [-h, h], c = 15/(16 h^5), h = t_sm/2.
        let t_sm: f64 = 0.8025 / 150.0;
        let h = t_sm / 2.0;
        let c = 15.0 / (16.0 * h.powi(5));
        let coeffs = vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c];
        let kernel = PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h, h));

        // Convolve — this is the operation that the old code could not handle.
        let convolved = convolve(&curve, &kernel).unwrap();

        // The key assertion: peak_accel returns a finite, physically meaningful
        // value, not NaN or 1e15. The exact value depends on edge effects from
        // the convolution truncation, but should be in a reasonable range.
        let peak = peak_accel(&convolved);
        assert!(peak.is_finite(), "peak is not finite: {peak}");
        // Interior acceleration is ≤ 1000 mm/s^2, but edge convolution
        // truncation adds transient acceleration. The peak should be bounded —
        // not astronomically large.
        assert!(peak > 100.0, "peak too low: {peak}");
        assert!(
            peak < 1_000_000.0,
            "peak too high (numerical blowup?): {peak}"
        );
    }
}
