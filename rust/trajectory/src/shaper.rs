use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

const INPUT_SAMPLES_PER_KERNEL_WIDTH: usize = 40;

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
