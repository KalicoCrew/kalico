// Stage 3b-c: per-axis convolution + trim.
//
// Convolves a padded per-axis curve with the smooth-shaper kernel, then trims
// the result back to the segment's time domain.

use nurbs::algebra::{convolve, restrict_to_domain, PiecewisePolynomialKernel};
use nurbs::ScalarNurbs;

/// Convolve a padded per-axis curve with the shaper kernel, then trim to the
/// segment's `[t_start, t_end]` domain.
///
/// The input `padded` must extend at least `t_sm/2` beyond `[t_start, t_end]`
/// on each side (produced by `pad::pad_segment_axis`).
///
/// For passthrough axes (Z by default), skip this function and return the
/// fitted axis NURBS directly.
pub fn shape_axis(
    padded: &ScalarNurbs<f64>,
    kernel: &PiecewisePolynomialKernel<f64>,
    t_start: f64,
    t_end: f64,
) -> Result<ScalarNurbs<f64>, nurbs::AlgebraError> {
    let convolved = convolve(padded, kernel)?;
    restrict_to_domain(&convolved, t_start, t_end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::FittedSegment;
    use crate::kernel::build_smooth_zv_kernel;
    use crate::pad::pad_segment_axis;
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

    /// Build a `FittedSegment` with constant position on all axes.
    fn constant_segment(x: f64, y: f64, z: f64, t_start: f64, t_end: f64) -> FittedSegment {
        let make_axis = |val: f64| {
            bezier_pieces_to_nurbs(&[BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![val],
            }])
        };
        FittedSegment {
            axes: [make_axis(x), make_axis(y), make_axis(z)],
            t_start,
            t_end,
        }
    }

    /// Build a `FittedSegment` with linear X motion, constant Y and Z.
    fn linear_segment(
        x_start: f64,
        x_end: f64,
        t_start: f64,
        t_end: f64,
    ) -> FittedSegment {
        let dt = t_end - t_start;
        let slope = (x_end - x_start) / dt;
        let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![x_start, slope],
        }]);
        let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![0.0],
        }]);
        let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![0.0],
        }]);
        FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start,
            t_end,
        }
    }

    // ------------------------------------------------------------------
    // Test 1: Convolution of constant produces constant
    // ------------------------------------------------------------------

    #[test]
    fn shape_constant_is_constant() {
        // A constant position curve, after convolution with a normalized kernel
        // (DC gain = 1), should remain constant.
        let freq = 150.0;
        let t_sm = 0.8025 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_zv_kernel(t_sm);

        let x_val = 42.0;
        let fitted = vec![constant_segment(x_val, 0.0, 0.0, 0.0, 1.0)];

        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
        let shaped = shape_axis(&padded, &kernel, 0.0, 1.0).unwrap();

        // Sample at multiple points — all should be close to x_val.
        let pieces = extract_bezier_pieces(&shaped);
        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            // Find the piece containing t.
            let val = eval_at(&pieces, t);
            assert!(
                (val - x_val).abs() < 1e-6,
                "at t={t}: expected {x_val}, got {val}"
            );
        }
    }

    // ------------------------------------------------------------------
    // Test 2: Pad-and-trim matches global convolve on 3 segments
    // ------------------------------------------------------------------

    #[test]
    fn pad_trim_matches_global_convolve() {
        // Use a low frequency (wide kernel) to avoid numerical ill-conditioning
        // from very large kernel coefficients. 10 Hz gives t_sm ≈ 0.08s, kernel
        // coefficients of manageable magnitude.
        let freq = 10.0;
        let t_sm = 0.8025 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_zv_kernel(t_sm);

        // 3 segments with linear motion on X, each 1s long.
        let fitted = vec![
            linear_segment(0.0, 10.0, 0.0, 1.0),
            linear_segment(10.0, 30.0, 1.0, 2.0),
            linear_segment(30.0, 35.0, 2.0, 3.0),
        ];
        let batch_t_start = 0.0;
        let batch_t_end = 3.0;

        // Method A: pad each segment, convolve each, trim each.
        let mut shaped_per_seg: Vec<ScalarNurbs<f64>> = Vec::new();
        for seg_idx in 0..3 {
            let padded = pad_segment_axis(
                seg_idx,
                0,
                &fitted,
                &[],
                t_sm_half,
                batch_t_start,
                batch_t_end,
            );
            let shaped = shape_axis(
                &padded,
                &kernel,
                fitted[seg_idx].t_start,
                fitted[seg_idx].t_end,
            )
            .unwrap();
            shaped_per_seg.push(shaped);
        }

        // Method B: build one global padded curve and convolve once.
        let mut global_pieces: Vec<BezierPiece<f64>> = Vec::new();

        // Left constant extension.
        global_pieces.push(BezierPiece {
            u_start: -t_sm_half,
            u_end: 0.0,
            coeffs: vec![0.0, 0.0], // constant 0 at degree 1
        });

        // All 3 segments' X-axis pieces.
        for seg in &fitted {
            global_pieces.extend(extract_bezier_pieces(&seg.axes[0]));
        }

        // Right constant extension.
        global_pieces.push(BezierPiece {
            u_start: 3.0,
            u_end: 3.0 + t_sm_half,
            coeffs: vec![35.0, 0.0], // constant 35 at degree 1
        });

        let global_nurbs = bezier_pieces_to_nurbs(&global_pieces);
        let global_convolved = convolve(&global_nurbs, &kernel).unwrap();

        // Compare at interior sample points within each segment's domain.
        // The per-segment approach introduces trimmed piece boundaries that
        // the global approach doesn't have. These extra Minkowski-sum breakpoints
        // produce the same mathematical polynomial; only floating-point rounding
        // differs. Tolerance 1e-6 for this moderate kernel width.
        for seg_idx in 0..3 {
            let seg = &fitted[seg_idx];
            let per_seg_pieces = extract_bezier_pieces(&shaped_per_seg[seg_idx]);
            let global_pieces = extract_bezier_pieces(&global_convolved);

            let n_samples = 10;
            for i in 1..n_samples {
                let t = seg.t_start + (seg.t_end - seg.t_start) * (i as f64 / n_samples as f64);
                let val_per_seg = eval_at(&per_seg_pieces, t);
                let val_global = eval_at(&global_pieces, t);
                assert!(
                    (val_per_seg - val_global).abs() < 1e-6,
                    "seg {seg_idx}, t={t}: per_seg={val_per_seg}, global={val_global}, diff={}",
                    (val_per_seg - val_global).abs()
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Test 3: Boundary extension at batch edges
    // ------------------------------------------------------------------

    #[test]
    fn batch_edge_constant_extension() {
        let freq = 150.0;
        let t_sm = 0.8025 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_zv_kernel(t_sm);

        // Single segment — batch edges get constant-position extension.
        let x_start = 5.0;
        let x_end = 15.0;
        let fitted = vec![linear_segment(x_start, x_end, 0.0, 1.0)];

        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
        let pieces = extract_bezier_pieces(&padded);

        // Verify the padded curve extends beyond [0, 1].
        assert!(
            pieces[0].u_start < 0.0,
            "padding should extend before t=0"
        );
        assert!(
            pieces.last().unwrap().u_end > 1.0,
            "padding should extend past t=1"
        );

        // The shaped result should be valid on [0, 1].
        let shaped = shape_axis(&padded, &kernel, 0.0, 1.0).unwrap();
        let shaped_pieces = extract_bezier_pieces(&shaped);

        // At t=0, the shaped value should be close to x_start (constant extension
        // means the shaper "sees" x_start to the left).
        let val_at_0 = eval_at(&shaped_pieces, 0.0);
        assert!(
            (val_at_0 - x_start).abs() < 0.5,
            "at t=0: expected ~{x_start}, got {val_at_0}"
        );

        // At t=1, the shaped value should be close to x_end.
        let val_at_1 = eval_at(&shaped_pieces, 1.0);
        assert!(
            (val_at_1 - x_end).abs() < 0.5,
            "at t=1: expected ~{x_end}, got {val_at_1}"
        );

        // The shaped output should be monotonically increasing (linear + constant
        // extension with a symmetric kernel).
        let n_samples = 50;
        let mut prev = f64::NEG_INFINITY;
        for i in 0..=n_samples {
            let t = i as f64 / n_samples as f64;
            let val = eval_at(&shaped_pieces, t);
            assert!(
                val >= prev - 1e-10,
                "not monotone at t={t}: prev={prev}, val={val}"
            );
            prev = val;
        }
    }

    // ------------------------------------------------------------------
    // Helper: evaluate piecewise Bezier at a parameter value
    // ------------------------------------------------------------------

    fn eval_at(pieces: &[BezierPiece<f64>], t: f64) -> f64 {
        for p in pieces {
            if t >= p.u_start - 1e-15 && t <= p.u_end + 1e-15 {
                return p.evaluate(t);
            }
        }
        panic!("t={t} not in any piece");
    }
}
