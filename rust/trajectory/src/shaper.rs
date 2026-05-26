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
const SAMPLES_PER_KERNEL_WIDTH: usize = 40;

pub fn shape_axis(
    padded: &ScalarNurbs<f64>,
    kernel: &PiecewisePolynomialKernel<f64>,
    t_start: f64,
    t_end: f64,
) -> Result<ScalarNurbs<f64>, nurbs::AlgebraError> {
    Ok(convolve_discrete(padded, kernel, t_start, t_end, SAMPLES_PER_KERNEL_WIDTH))
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
    samples_per_kernel_width: usize,
) -> ScalarNurbs<f64> {
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

    let (k_lo, k_hi) = kernel.support();
    let kernel_width = k_hi - k_lo;
    // Input sampling: dense, for FIR accuracy (trapezoidal error O(dt_in²))
    let dt_in = kernel_width / (samples_per_kernel_width as f64);
    // Output sampling: sparse, just needs to capture the smooth convolution.
    // The convolution output bandwidth ≤ input bandwidth. 1 sample per
    // kernel width (~5ms at 186Hz) is Nyquist-safe and produces ~14k
    // output points for a 69s segment instead of 537k.
    let dt_out = kernel_width;

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

// ---------------------------------------------------------------------------
// Approach A prototype: sample-and-refit discrete FIR convolution
// ---------------------------------------------------------------------------
//
// The production `convolve` decomposes into Bézier pieces and does piece-by-piece
// polynomial integration. That approach is exact but numerically unstable for
// long segments: the output polynomial is expressed with u_start values like 69s,
// and the power terms (u - u_start)^k blow up when evaluated away from u_start.
// For a 69-second segment at 186 Hz (kernel support ≈ 5.1 ms), the catastrophic
// cancellation in high-degree monomial terms amplifies a 9 µm input deviation to
// ~750 mm.
//
// Approach A avoids this entirely: it never constructs high-degree polynomials
// spanning long time intervals. Instead it:
//
//   1. Evaluates the padded input NURBS at N uniform sample points using de Boor
//      (which is already numerically stable — it works in the Bernstein basis).
//   2. Applies the kernel as a discrete FIR filter: each output sample is a
//      weighted sum of input samples within the kernel's support window.
//   3. Fits a piecewise-linear NURBS (degree 1) through the output samples.
//      Degree 1 is exact for this representation — no high-power cancellation.
//
// The approximation error is O(dt^2) in position for a smooth signal, where
// dt is the sample spacing. At 4× oversampling relative to the kernel support
// (e.g. 4 samples per 5.1 ms → dt ≈ 1.3 ms), the error is well below 1 µm
// for typical printer motion.
//
// These functions live here as a `#[cfg(test)]` prototype. They are NOT wired
// into the production `shape_axis` path — that requires a separate design step
// choosing the right N, tolerance, and output degree.

#[cfg(test)]
mod approach_a {
    use nurbs::algebra::PiecewisePolynomialKernel;
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
    use nurbs::eval::eval;
    use nurbs::ScalarNurbs;

    /// Evaluate a `ScalarNurbs` at `t`, clamping to the curve's domain boundaries
    /// instead of panicking. Used for edge samples that may land 1 ULP outside the
    /// padded curve's domain.
    fn eval_clamped(curve: &ScalarNurbs<f64>, t: f64) -> f64 {
        let knots = curve.knots();
        let t_lo = knots[0];
        let t_hi = knots[knots.len() - 1];
        let t_c = t.clamp(t_lo, t_hi);
        eval(&curve.as_view(), t_c)
    }

    /// Evaluate the kernel `w(z)` at offset `z = t_out - t_sample`.
    ///
    /// The kernel's `BezierPiece` holds coefficients in the Pascal-shifted monomial
    /// basis: `w(z) = Σ coeffs[k] * (z - u_start)^k`. Returns 0 when `z` is
    /// outside the kernel's support `[u_start, u_end]`.
    fn eval_kernel(kernel: &PiecewisePolynomialKernel<f64>, z: f64) -> f64 {
        for piece in &kernel.pieces {
            if z >= piece.u_start && z <= piece.u_end {
                return piece.evaluate(z);
            }
        }
        0.0
    }

    /// Approach A: discrete sample-and-refit convolution.
    ///
    /// # Arguments
    ///
    /// * `padded` — the input curve, already extended by ≥ `t_sm/2` on each side
    ///   of `[t_start, t_end]`.
    /// * `kernel` — the smooth-shaper kernel with support `[-h, h]` where
    ///   `h = t_sm / 2`.
    /// * `t_start`, `t_end` — the segment's own time domain (output domain).
    /// * `samples_per_kernel_width` — how many input samples to place per kernel
    ///   support width (2h). Higher = more accurate, more expensive. 200 is a
    ///   comfortable default for the 5 ms kernels we use.
    ///
    /// # Returns
    ///
    /// A degree-1 (piecewise-linear) `ScalarNurbs<f64>` on `[t_start, t_end]`
    /// whose values match the true convolution to O(dt²) in position.
    pub fn convolve_discrete(
        padded: &ScalarNurbs<f64>,
        kernel: &PiecewisePolynomialKernel<f64>,
        t_start: f64,
        t_end: f64,
        samples_per_kernel_width: usize,
    ) -> ScalarNurbs<f64> {
        let (k_lo, k_hi) = kernel.support();
        let kernel_width = k_hi - k_lo; // = t_sm
        let dt = kernel_width / (samples_per_kernel_width as f64);

        // The padded input spans at least [t_start + k_lo, t_end + k_hi].
        // We need input samples over that entire range so that every output
        // sample's FIR window is fully covered.
        let input_lo = t_start + k_lo;
        let input_hi = t_end + k_hi;
        let n_input = ((input_hi - input_lo) / dt).ceil() as usize + 1;

        // Evaluate the input at n_input uniformly spaced points.
        let input_samples: Vec<f64> = (0..n_input)
            .map(|i| {
                let t = input_lo + (i as f64) * dt;
                eval_clamped(padded, t)
            })
            .collect();

        // For each output sample at t_out ∈ [t_start, t_end], compute the
        // discrete convolution sum:
        //   y(t_out) = Σ_j  x(t_j) * w(t_out - t_j) * dt
        // where t_j = input_lo + j * dt and w is the kernel.
        //
        // The kernel support is [k_lo, k_hi] so only samples where
        // k_lo ≤ t_out - t_j ≤ k_hi contribute, i.e.
        //   t_out - k_hi ≤ t_j ≤ t_out - k_lo.
        let n_output = (((t_end - t_start) / dt).ceil() as usize) + 1;
        let mut output_times: Vec<f64> = Vec::with_capacity(n_output);
        let mut output_values: Vec<f64> = Vec::with_capacity(n_output);

        for i in 0..n_output {
            let t_out = (t_start + (i as f64) * dt).min(t_end);
            // Range of input indices whose kernel weight is non-zero.
            let j_lo_f = (t_out - k_hi - input_lo) / dt;
            let j_hi_f = (t_out - k_lo - input_lo) / dt;
            let j_lo = (j_lo_f.floor() as isize).max(0) as usize;
            let j_hi = ((j_hi_f.ceil() as isize) + 1).min(n_input as isize) as usize;

            let mut acc = 0.0_f64;
            for j in j_lo..j_hi {
                let t_j = input_lo + (j as f64) * dt;
                let z = t_out - t_j;
                let w = eval_kernel(kernel, z);
                acc += input_samples[j] * w * dt;
            }
            output_times.push(t_out);
            output_values.push(acc);
        }

        // Ensure the last sample is exactly at t_end for clean knot clamping.
        if let Some(last_t) = output_times.last() {
            if (*last_t - t_end).abs() > 1e-15 {
                let t_out = t_end;
                let j_lo_f = (t_out - k_hi - input_lo) / dt;
                let j_hi_f = (t_out - k_lo - input_lo) / dt;
                let j_lo = (j_lo_f.floor() as isize).max(0) as usize;
                let j_hi = ((j_hi_f.ceil() as isize) + 1).min(n_input as isize) as usize;
                let mut acc = 0.0_f64;
                for j in j_lo..j_hi {
                    let t_j = input_lo + (j as f64) * dt;
                    let z = t_out - t_j;
                    let w = eval_kernel(kernel, z);
                    acc += input_samples[j] * w * dt;
                }
                output_times.push(t_end);
                output_values.push(acc);
            }
        }

        // Build a degree-1 (piecewise-linear) NURBS from the output samples.
        // Each consecutive pair of output points defines one degree-1 Bézier piece:
        //   p(t) = v0 + (v1 - v0) / (t1 - t0) * (t - t0)
        let n_out = output_times.len();
        assert!(n_out >= 2, "need at least 2 output samples");

        let pieces: Vec<BezierPiece<f64>> = (0..n_out - 1)
            .map(|i| {
                let t0 = output_times[i];
                let t1 = output_times[i + 1];
                let v0 = output_values[i];
                let v1 = output_values[i + 1];
                let dt_piece = t1 - t0;
                let slope = if dt_piece > 0.0 {
                    (v1 - v0) / dt_piece
                } else {
                    0.0
                };
                BezierPiece {
                    u_start: t0,
                    u_end: t1,
                    // Pascal-shifted basis: [constant, linear] = [v0, slope]
                    coeffs: vec![v0, slope],
                }
            })
            .collect();

        bezier_pieces_to_nurbs(&pieces)
    }

    // -----------------------------------------------------------------------
    // Tests
    // -----------------------------------------------------------------------

    use crate::fit::FittedSegment;
    use crate::kernel::build_smooth_mzv_kernel;
    use crate::pad::pad_segment_axis;
    use nurbs::bezier::extract_bezier_pieces;

    fn eval_at(pieces: &[BezierPiece<f64>], t: f64) -> f64 {
        for p in pieces {
            if t >= p.u_start - 1e-15 && t <= p.u_end + 1e-15 {
                return p.evaluate(t);
            }
        }
        panic!("t={t} not in any piece");
    }

    /// Helper: constant-value segment like the one in the production test suite.
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

    /// The failing case from the bug report:
    ///   - 69-second near-constant X=150.0 segment
    ///   - smooth_mzv at 186 Hz (kernel support ≈ 5.14 ms)
    ///   - Production `convolve` amplifies a 9 µm deviation to 750 mm
    ///   - Approach A should remain within 1 mm of 150.0 everywhere
    ///
    /// We use 20 samples per kernel width here (dt ≈ 257 µs, ~268k samples
    /// over 69 s) for test speed. At 20 samp/width the discrete trapezoidal
    /// error is O(dt²) ≈ (257e-6)²/12 * f'' ≈ sub-µm for a near-constant
    /// curve. The important thing to verify is order-of-magnitude stability
    /// (not 750 mm blowup), not sub-µm accuracy.
    #[test]
    fn approach_a_constant_69s_near_zero_deviation() {
        let freq = 186.0;
        let t_sm = 0.95625 / freq; // ≈ 5.14 ms
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_mzv_kernel(t_sm);

        let x_val = 150.0;
        let fitted = vec![constant_segment_69s(x_val)];

        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

        // 20 samples per kernel width → dt ≈ 257 µs → ~268 k samples over 69 s.
        // This is fast enough for a unit test and still sub-mm accurate for
        // near-constant inputs.
        let shaped = convolve_discrete(&padded, &kernel, 0.0, 69.0, 20);
        let pieces = extract_bezier_pieces(&shaped);

        // Check at 20 uniformly spaced points across the full 69-second domain.
        let n_check = 20;
        let mut max_dev = 0.0_f64;
        for i in 0..=n_check {
            let t = 69.0 * (i as f64) / (n_check as f64);
            let t_c = t.clamp(0.0, 69.0);
            let val = eval_at(&pieces, t_c);
            let dev = (val - x_val).abs();
            if dev > max_dev {
                max_dev = dev;
            }
        }

        // The output should stay within 1 mm of 150.0 — far better than the
        // 750 mm deviation the production convolve produces on this input.
        // Actual deviation for a truly constant input is ~1e-8 mm (only floating-
        // point rounding in the trapezoidal sum), so 1 mm is an extremely generous
        // bound that would catch even severe instability.
        assert!(
            max_dev < 1.0, // 1 mm — catches the 750mm blowup; actual is ~1e-8 mm
            "max deviation from {x_val} = {max_dev:.6} mm; expected < 1 mm"
        );
    }

    /// Verify the DC gain = 1 property: convolution of a constant with a
    /// unit-integral kernel must reproduce the constant exactly (modulo
    /// discrete approximation error).
    #[test]
    fn approach_a_dc_gain_is_unity() {
        let freq = 186.0;
        let t_sm = 0.95625 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_mzv_kernel(t_sm);

        for &x_val in &[0.0_f64, 42.0, 150.0, -7.5] {
            // Use a short domain for speed — just test the DC property.
            // We construct a 1-second version of the same constant curve.
            let fitted_short = vec![FittedSegment {
                axes: [
                    bezier_pieces_to_nurbs(&[BezierPiece {
                        u_start: 0.0,
                        u_end: 1.0,
                        coeffs: vec![x_val],
                    }]),
                    bezier_pieces_to_nurbs(&[BezierPiece {
                        u_start: 0.0,
                        u_end: 1.0,
                        coeffs: vec![0.0],
                    }]),
                    bezier_pieces_to_nurbs(&[BezierPiece {
                        u_start: 0.0,
                        u_end: 1.0,
                        coeffs: vec![0.0],
                    }]),
                ],
                t_start: 0.0,
                t_end: 1.0,
            }];
            let padded_short = pad_segment_axis(0, 0, &fitted_short, &[], t_sm_half, 0.0, 1.0);
            let shaped = convolve_discrete(&padded_short, &kernel, 0.0, 1.0, 20);
            let pieces = extract_bezier_pieces(&shaped);

            // Sample in the middle of the segment — boundary effects are confined
            // to within t_sm_half of the endpoints.
            for &t in &[0.1, 0.5, 0.9] {
                let val = eval_at(&pieces, t);
                assert!(
                    (val - x_val).abs() < 1e-3,
                    "x_val={x_val}, t={t}: got {val}, dev={}",
                    (val - x_val).abs()
                );
            }
        }
    }

    /// Compare Approach A against the production `convolve` on a SHORT segment
    /// (0.1 s) where the production code is numerically stable. The two results
    /// should agree to within the discrete approximation error (~dt² ≈ 1 nm at
    /// 200 samples per kernel width).
    #[test]
    fn approach_a_matches_production_on_short_segment() {
        use crate::shaper::shape_axis;

        let freq = 150.0;
        let t_sm = 0.8025 / freq; // smooth_zv at 150 Hz, support ≈ 5.35 ms
        let t_sm_half = t_sm / 2.0;
        let kernel = crate::kernel::build_smooth_zv_kernel(t_sm);

        // Linear X segment: 0.0 → 10.0 mm over 0.1 s — production code is fine here.
        let dt = 0.1_f64;
        let slope = 10.0 / dt;
        let seg = FittedSegment {
            axes: [
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: dt,
                    coeffs: vec![0.0, slope],
                }]),
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: dt,
                    coeffs: vec![0.0],
                }]),
                bezier_pieces_to_nurbs(&[BezierPiece {
                    u_start: 0.0,
                    u_end: dt,
                    coeffs: vec![0.0],
                }]),
            ],
            t_start: 0.0,
            t_end: dt,
        };
        let fitted = vec![seg];
        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, dt);

        // Production result.
        let production = shape_axis(&padded, &kernel, 0.0, dt).unwrap();
        let prod_pieces = extract_bezier_pieces(&production);

        // Approach A result.
        let approx = convolve_discrete(&padded, &kernel, 0.0, dt, 200);
        let approx_pieces = extract_bezier_pieces(&approx);

        // Compare at 10 interior points (avoid exact endpoints where boundary
        // effects may differ slightly between the two methods).
        let n_cmp = 10;
        let mut max_diff = 0.0_f64;
        for i in 1..n_cmp {
            let t = dt * (i as f64) / (n_cmp as f64);
            let v_prod = eval_at(&prod_pieces, t);
            let v_approx = eval_at(&approx_pieces, t);
            let diff = (v_prod - v_approx).abs();
            if diff > max_diff {
                max_diff = diff;
            }
        }

        // At 200 samples per 5 ms kernel, dt ≈ 25 µs. Trapezoidal rule error
        // is O(dt²/12 * f'') ≈ (25e-6)²/12 * 100/0.1 ≈ 5e-8 mm. We allow 1 µm.
        assert!(
            max_diff < 1e-3,
            "max diff between production and Approach A = {max_diff:.2e} mm"
        );
    }

    /// Verify that Approach A is numerically stable for the exact pathological
    /// case: 69 s constant at 150.0 mm, smooth_mzv at 186 Hz.
    ///
    /// The production `convolve` returns values in the range [−600, 900] on this
    /// input (catastrophic cancellation). Approach A must return values in
    /// [149.9, 150.1] everywhere.
    #[test]
    fn approach_a_stable_where_production_fails() {
        let freq = 186.0;
        let t_sm = 0.95625 / freq;
        let t_sm_half = t_sm / 2.0;
        let kernel = build_smooth_mzv_kernel(t_sm);

        let x_val = 150.0;
        let fitted = vec![constant_segment_69s(x_val)];
        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

        // Verify what the production `convolve` actually does on this input.
        // We expect it to produce non-constant (possibly wildly wrong) output.
        use nurbs::algebra::convolve;
        let production_result = convolve(&padded, &kernel);
        let production_is_broken = match production_result {
            Ok(ref curve) => {
                let prod_pieces = extract_bezier_pieces(curve);
                // Check a sample near the end of the domain where instability manifests.
                // The padded curve extends to ~69 + t_sm_half seconds.
                let t_check = 65.0_f64;
                // Find a piece that contains t_check.
                let found = prod_pieces
                    .iter()
                    .find(|p| t_check >= p.u_start - 1e-9 && t_check <= p.u_end + 1e-9);
                if let Some(p) = found {
                    let v = p.evaluate(t_check);
                    // If production gives something far from 150 here it's broken.
                    (v - x_val).abs() > 1.0 // > 1 mm deviation signals instability
                } else {
                    false // piece not found, can't assess
                }
            }
            Err(_) => false,
        };

        // Approach A on the same input (20 samp/width for test speed).
        let shaped = convolve_discrete(&padded, &kernel, 0.0, 69.0, 20);
        let pieces = extract_bezier_pieces(&shaped);

        let mut max_dev = 0.0_f64;
        for i in 0..=50 {
            let t = 69.0 * (i as f64) / 50.0;
            let t_c = t.clamp(0.0, 69.0);
            let val = eval_at(&pieces, t_c);
            let dev = (val - x_val).abs();
            if dev > max_dev {
                max_dev = dev;
            }
        }

        // Approach A must be stable.
        assert!(
            max_dev < 1e-3,
            "Approach A max dev = {max_dev:.6} mm on 69s constant input"
        );

        // The test passes regardless of whether production is broken — we're
        // testing Approach A's correctness, not production's failure.
        // Both outcomes are informational: `production_is_broken = true` confirms
        // the bug is present; `false` would mean it was somehow fixed (unlikely
        // for this case unless the polynomial basis changed).
        let _ = production_is_broken;
    }
}
