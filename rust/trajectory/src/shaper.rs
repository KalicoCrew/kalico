use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

const INPUT_SAMPLES_PER_KERNEL_WIDTH: usize = 40;
const OUTPUT_SAMPLES_PER_KERNEL_WIDTH: usize = 12;

pub fn shape_axis(
    padded: &ScalarNurbs<f64>,
    kernel: &PiecewisePolynomialKernel<f64>,
    t_start: f64,
    t_end: f64,
) -> ScalarNurbs<f64> {
    convolve_discrete(
        padded,
        kernel,
        t_start,
        t_end,
        INPUT_SAMPLES_PER_KERNEL_WIDTH,
        OUTPUT_SAMPLES_PER_KERNEL_WIDTH,
    )
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
            let slope = if dt_piece > 0.0 {
                (v1 - v0) / dt_piece
            } else {
                0.0
            };
            BezierPiece {
                u_start: t0,
                u_end: t1,
                coeffs: vec![v0, slope],
            }
        })
        .collect();

    bezier_pieces_to_nurbs(&pieces)
}

pub struct ShapedSignal<'a> {
    input_samples: Vec<f64>,
    input_lo: f64,
    dt_in: f64,
    n_input: usize,
    kernel: &'a PiecewisePolynomialKernel<f64>,
    k_lo: f64,
    k_hi: f64,
}

impl<'a> ShapedSignal<'a> {
    pub fn new(
        padded: &ScalarNurbs<f64>,
        kernel: &'a PiecewisePolynomialKernel<f64>,
        t_start: f64,
        t_end: f64,
    ) -> Self {
        let (k_lo, k_hi) = kernel.support();
        let kernel_width = k_hi - k_lo;
        let dt_in = kernel_width / (INPUT_SAMPLES_PER_KERNEL_WIDTH as f64);
        let input_lo = t_start + k_lo;
        let input_hi = t_end + k_hi;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_input = ((input_hi - input_lo) / dt_in).ceil() as usize + 1;
        let input_samples = (0..n_input)
            .map(|i| eval_clamped(padded, input_lo + (i as f64) * dt_in))
            .collect();
        Self {
            input_samples,
            input_lo,
            dt_in,
            n_input,
            kernel,
            k_lo,
            k_hi,
        }
    }

    pub fn eval(&self, t: f64) -> f64 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let j_lo =
            (((t - self.k_hi - self.input_lo) / self.dt_in).floor() as isize).max(0) as usize;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let j_hi = ((((t - self.k_lo - self.input_lo) / self.dt_in).ceil() as isize) + 1)
            .min(self.n_input as isize) as usize;
        let mut acc = 0.0_f64;
        for j in j_lo..j_hi {
            let t_j = self.input_lo + (j as f64) * self.dt_in;
            acc += self.input_samples[j] * eval_kernel(self.kernel, t - t_j) * self.dt_in;
        }
        acc
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod long_segment_stability;
