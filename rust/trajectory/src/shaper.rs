// Stage 3b-c: per-axis convolution + trim.
//
// Convolves a padded per-axis curve with the smooth-shaper kernel, then trims
// the result back to the segment's time domain.

use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

/// Convolve a padded per-axis curve with the shaper kernel, then trim to the
/// segment's `[t_start, t_end]` domain.
///
/// The input `padded` must extend at least `t_sm/2` beyond `[t_start, t_end]`
/// on each side (produced by `pad::pad_segment_axis`).
///
/// For passthrough axes (Z by default), skip this function and return the
/// fitted axis NURBS directly.
const INPUT_SAMPLES_PER_KERNEL_WIDTH: usize = 40;
const OUTPUT_SAMPLES_PER_KERNEL_WIDTH: usize = 12;

pub fn shape_axis(
    padded: &ScalarNurbs<f64>,
    kernel: &PiecewisePolynomialKernel<f64>,
    t_start: f64,
    t_end: f64,
) -> Result<ScalarNurbs<f64>, nurbs::AlgebraError> {
    Ok(convolve_discrete(
        padded,
        kernel,
        t_start,
        t_end,
        INPUT_SAMPLES_PER_KERNEL_WIDTH,
        OUTPUT_SAMPLES_PER_KERNEL_WIDTH,
    ))
}

fn eval_clamped(curve: &ScalarNurbs<f64>, t: f64) -> f64 {
    let knots = curve.knots();
    let lo = knots[0];
    let hi = knots[knots.len() - 1];
    nurbs::eval::eval(curve, t.clamp(lo, hi))
}

fn eval_kernel(kernel: &PiecewisePolynomialKernel<f64>, z: f64) -> f64 {
    let (k_lo, k_hi) = kernel.support();
    if z < k_lo || z > k_hi {
        return 0.0;
    }
    for p in &kernel.pieces {
        if z >= p.u_start - 1e-15 && z <= p.u_end + 1e-15 {
            return p.evaluate(z);
        }
    }
    0.0
}

fn convolve_discrete(
    padded: &ScalarNurbs<f64>,
    kernel: &PiecewisePolynomialKernel<f64>,
    t_start: f64,
    t_end: f64,
    input_samples_per_kw: usize,
    output_samples_per_kw: usize,
) -> ScalarNurbs<f64> {
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

    let (k_lo, k_hi) = kernel.support();
    let kernel_width = k_hi - k_lo;
    let dt_in = kernel_width / (input_samples_per_kw as f64);
    let dt_out = kernel_width / (output_samples_per_kw as f64);

    let input_lo = t_start + k_lo;
    let input_hi = t_end + k_hi;
    let n_input = ((input_hi - input_lo) / dt_in).ceil() as usize + 1;

    let input_samples: Vec<f64> = (0..n_input)
        .map(|i| {
            let t = input_lo + (i as f64) * dt_in;
            eval_clamped(padded, t)
        })
        .collect();

    let n_output = (((t_end - t_start) / dt_out).ceil() as usize) + 1;
    let mut output_times: Vec<f64> = Vec::with_capacity(n_output + 1);
    let mut output_values: Vec<f64> = Vec::with_capacity(n_output + 1);

    let fir_at = |t_out: f64| -> f64 {
        let j_lo_f = (t_out - k_hi - input_lo) / dt_in;
        let j_hi_f = (t_out - k_lo - input_lo) / dt_in;
        let j_lo = (j_lo_f.floor() as isize).max(0) as usize;
        let j_hi = ((j_hi_f.ceil() as isize) + 1).min(n_input as isize) as usize;
        let mut acc = 0.0_f64;
        for j in j_lo..j_hi {
            let t_j = input_lo + (j as f64) * dt_in;
            let w = eval_kernel(kernel, t_out - t_j);
            acc += input_samples[j] * w * dt_in;
        }
        acc
    };

    for i in 0..n_output {
        let t_out = (t_start + (i as f64) * dt_out).min(t_end);
        output_times.push(t_out);
        output_values.push(fir_at(t_out));
    }

    // Ensure the last sample is exactly at t_end
    if let Some(last_t) = output_times.last() {
        if (*last_t - t_end).abs() > dt_out * 0.01 {
            output_times.push(t_end);
            output_values.push(fir_at(t_end));
        }
    }

    let n_out = output_times.len();
    assert!(n_out >= 2, "need at least 2 output samples");

    let pieces: Vec<BezierPiece<f64>> = (0..n_out - 1)
        .map(|i| {
            let t0 = output_times[i];
            let t1 = output_times[i + 1];
            let v0 = output_values[i];
            let v1 = output_values[i + 1];
            let dt_piece = t1 - t0;
            let slope = if dt_piece > 0.0 { (v1 - v0) / dt_piece } else { 0.0 };
            BezierPiece {
                u_start: t0,
                u_end: t1,
                coeffs: vec![v0, slope],
            }
        })
        .collect();

    bezier_pieces_to_nurbs(&pieces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::FittedSegment;
    use crate::kernel::build_smooth_zv_kernel;
    use crate::pad::pad_segment_axis;
    use nurbs::algebra::convolve;
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
    fn linear_segment(x_start: f64, x_end: f64, t_start: f64, t_end: f64) -> FittedSegment {
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
        // Tolerance 1e-4 mm (100 nm): the discrete FIR's kernel normalization
        // introduces ~16 nm error on a constant; 100 nm gives 6× margin while
        // staying well below the 5 µm refit budget.
        let pieces = extract_bezier_pieces(&shaped);
        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let val = eval_at(&pieces, t);
            assert!(
                (val - x_val).abs() < 1e-4,
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
        // The per-segment discrete FIR and the global NURBS convolve use
        // fundamentally different algorithms; the FIR introduces
        // discretization error proportional to (kernel_width / N_samples)².
        // At 10 Hz (wide kernel, dt_in ≈ 2ms) the peak error is ~10 nm.
        // Tolerance 1e-4 mm (100 nm) gives 10× margin while staying well
        // below the 5 µm refit budget.
        for seg_idx in 0..3 {
            let seg = &fitted[seg_idx];
            let per_seg_pieces = extract_bezier_pieces(&shaped_per_seg[seg_idx]);
            let global_pieces = extract_bezier_pieces(&global_convolved);

            let n_samples = 10;
            for i in 1..n_samples {
                let t =
                    seg.t_start + (seg.t_end - seg.t_start) * (f64::from(i) / f64::from(n_samples));
                let val_per_seg = eval_at(&per_seg_pieces, t);
                let val_global = eval_at(&global_pieces, t);
                assert!(
                    (val_per_seg - val_global).abs() < 1e-4,
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
        assert!(pieces[0].u_start < 0.0, "padding should extend before t=0");
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
            let t = f64::from(i) / f64::from(n_samples);
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

#[cfg(test)]
mod long_segment_stability {
    use crate::fit::FittedSegment;
    use crate::kernel::build_smooth_mzv_kernel;
    use crate::pad::pad_segment_axis;
    use crate::shaper::shape_axis;
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

    fn constant_segment_69s(x_val: f64) -> FittedSegment {
        FittedSegment {
            axes: [
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: 69.0,
                    coeffs: vec![x_val],
                }]),
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: 69.0,
                    coeffs: vec![0.0],
                }]),
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: 69.0,
                    coeffs: vec![0.0],
                }]),
            ],
            t_start: 0.0,
            t_end: 69.0,
        }
    }

    fn eval_at(pieces: &[BezierPiece<f64>], t: f64) -> f64 {
        for p in pieces {
            if t >= p.u_start - 1e-15 && t <= p.u_end + 1e-15 {
                return p.evaluate(t);
            }
        }
        panic!("t={t} not in any piece");
    }

    #[test]
    fn constant_69s_near_zero_deviation() {
        let freq = 186.0;
        let t_sm = 0.95625 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_mzv_kernel(t_sm);

        let x_val = 150.0;
        let fitted = vec![constant_segment_69s(x_val)];
        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

        let shaped = shape_axis(&padded, &kernel, 0.0, 69.0).unwrap();
        let pieces = extract_bezier_pieces(&shaped);

        let mut max_dev = 0.0_f64;
        for i in 0..=20 {
            let t = 69.0 * (i as f64) / 20.0;
            let val = eval_at(&pieces, t.clamp(0.0, 69.0));
            max_dev = max_dev.max((val - x_val).abs());
        }

        assert!(
            max_dev < 1e-3,
            "max deviation from {x_val} = {max_dev:.6} mm; expected < 1µm"
        );
    }

    #[test]
    fn stable_where_nurbs_convolve_fails() {
        let freq = 186.0;
        let t_sm = 0.95625 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_mzv_kernel(t_sm);

        let x_val = 150.0;
        let fitted = vec![constant_segment_69s(x_val)];
        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

        let shaped = shape_axis(&padded, &kernel, 0.0, 69.0).unwrap();
        let pieces = extract_bezier_pieces(&shaped);

        let mut max_dev = 0.0_f64;
        for i in 0..=50 {
            let t = 69.0 * (i as f64) / 50.0;
            let val = eval_at(&pieces, t.clamp(0.0, 69.0));
            max_dev = max_dev.max((val - x_val).abs());
        }

        assert!(
            max_dev < 1e-3,
            "max dev = {max_dev:.6} mm on 69s constant input"
        );
    }
}
