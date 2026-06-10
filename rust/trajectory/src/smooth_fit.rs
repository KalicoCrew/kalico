use nurbs::bezier::BezierPiece;

/// Clamped interpolating cubic spline through `knots` (strictly increasing,
/// len m+1 >= 2) with values `y`, matching first derivative `yp0` at the start
/// and `ypn` at the end. Returns `m` cubic `BezierPiece`s in local monomial
/// basis. C2-continuous across interior joints by construction.
fn build_clamped_spline(knots: &[f64], y: &[f64], yp0: f64, ypn: f64) -> Vec<BezierPiece<f64>> {
    let m = knots.len() - 1;
    debug_assert!(m >= 1 && y.len() == knots.len());

    let h: Vec<f64> = (0..m).map(|i| knots[i + 1] - knots[i]).collect();

    // Solve for second derivatives M[0..=m] (clamped boundary conditions).
    let n = m + 1;
    let mut a = vec![0.0; n]; // sub-diagonal
    let mut b = vec![0.0; n]; // diagonal
    let mut c = vec![0.0; n]; // super-diagonal
    let mut d = vec![0.0; n]; // rhs

    // Start clamped: 2 h0 M0 + h0 M1 = 6((y1-y0)/h0 - yp0)
    b[0] = 2.0 * h[0];
    c[0] = h[0];
    d[0] = 6.0 * ((y[1] - y[0]) / h[0] - yp0);

    // Interior i=1..m-1: h[i-1] M[i-1] + 2(h[i-1]+h[i]) M[i] + h[i] M[i+1] = rhs
    for i in 1..m {
        a[i] = h[i - 1];
        b[i] = 2.0 * (h[i - 1] + h[i]);
        c[i] = h[i];
        d[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    // End clamped: h[m-1] M[m-1] + 2 h[m-1] M[m] = 6(ypn - (ym - y[m-1])/h[m-1])
    a[m] = h[m - 1];
    b[m] = 2.0 * h[m - 1];
    d[m] = 6.0 * (ypn - (y[m] - y[m - 1]) / h[m - 1]);

    let mm = solve_tridiagonal(&a, &b, &c, &d);

    // Build each cubic piece in local monomial basis (x = t - knots[i]):
    //   S_i(x) = y_i + b_i x + (M_i/2) x^2 + ((M_{i+1}-M_i)/(6 h_i)) x^3
    //   b_i = (y_{i+1}-y_i)/h_i - h_i (2 M_i + M_{i+1})/6
    (0..m)
        .map(|i| {
            let bi = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * mm[i] + mm[i + 1]) / 6.0;
            BezierPiece {
                u_start: knots[i],
                u_end: knots[i + 1],
                coeffs: vec![y[i], bi, mm[i] / 2.0, (mm[i + 1] - mm[i]) / (6.0 * h[i])],
            }
        })
        .collect()
}

/// Thomas algorithm for a tridiagonal system. `a` is the sub-diagonal
/// (a[0] ignored), `b` the diagonal, `c` the super-diagonal (c[n-1] ignored),
/// `d` the right-hand side. Returns the solution vector.
fn solve_tridiagonal(a: &[f64], b: &[f64], c: &[f64], d: &[f64]) -> Vec<f64> {
    let n = b.len();
    debug_assert!(n > 0 && a.len() == n && c.len() == n && d.len() == n);
    let mut cp = vec![0.0; n];
    let mut dp = vec![0.0; n];
    cp[0] = c[0] / b[0];
    dp[0] = d[0] / b[0];
    for i in 1..n {
        let m = b[i] - a[i] * cp[i - 1];
        cp[i] = c[i] / m;
        dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
    }
    let mut x = vec![0.0; n];
    x[n - 1] = dp[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = dp[i] - cp[i] * x[i + 1];
    }
    x
}

use nurbs::bezier::bezier_pieces_to_nurbs;
use nurbs::ScalarNurbs;

const MAX_KNOTS: usize = 4096;
const SAMPLES_PER_INTERVAL: usize = 16;

#[derive(Debug)]
pub struct FitError {
    pub achieved_mm: f64,
}

/// Adaptive clamped C2 cubic-spline fit of `f` on `[t_start, t_end]` to
/// `tolerance`. Knots are inserted at the worst-error location until the max
/// deviation (sampled within intervals) is within tolerance. End slopes are
/// taken from finite differences of `f`. Fails loudly if `MAX_KNOTS` is
/// exhausted before tolerance is met.
pub fn fit_c2_cubic<F: Fn(f64) -> f64>(
    f: &F,
    t_start: f64,
    t_end: f64,
    tolerance: f64,
) -> Result<ScalarNurbs<f64>, FitError> {
    let span = t_end - t_start;
    debug_assert!(span > 0.0 && tolerance > 0.0);

    let fd = (span * 1e-4).max(f64::MIN_POSITIVE);
    let yp0 = (f(t_start + fd) - f(t_start)) / fd;
    let ypn = (f(t_end) - f(t_end - fd)) / fd;

    // Start with start, midpoint, end so the first spline is non-degenerate.
    let mut knots = vec![t_start, t_start + 0.5 * span, t_end];

    loop {
        let values: Vec<f64> = knots.iter().map(|&t| f(t)).collect();
        let pieces = build_clamped_spline(&knots, &values, yp0, ypn);

        // Find the interval with the worst sampled error and the t of that max.
        let mut worst_err = 0.0_f64;
        let mut worst_t = f64::NAN;
        let mut worst_interval = 0usize;
        for (i, p) in pieces.iter().enumerate() {
            let (a, b) = (knots[i], knots[i + 1]);
            for s in 1..SAMPLES_PER_INTERVAL {
                let t = a + (b - a) * (s as f64 / SAMPLES_PER_INTERVAL as f64);
                let e = (p.evaluate(t) - f(t)).abs();
                if e > worst_err {
                    worst_err = e;
                    worst_t = t;
                    worst_interval = i;
                }
            }
        }

        if worst_err <= tolerance {
            return Ok(bezier_pieces_to_nurbs(&pieces));
        }
        if knots.len() >= MAX_KNOTS || !worst_t.is_finite() {
            return Err(FitError {
                achieved_mm: worst_err,
            });
        }

        let interval_lo = knots[worst_interval];
        let interval_hi = knots[worst_interval + 1];
        let interval_h = interval_hi - interval_lo;
        let left_gap = worst_t - interval_lo;
        let right_gap = interval_hi - worst_t;
        if left_gap < interval_h * 1e-9 || right_gap < interval_h * 1e-9 {
            let mid = interval_lo + 0.5 * interval_h;
            if mid <= interval_lo || mid >= interval_hi {
                return Err(FitError {
                    achieved_mm: worst_err,
                });
            }
            knots.insert(worst_interval + 1, mid);
        } else {
            knots.insert(worst_interval + 1, worst_t);
        }
    }
}

#[cfg(test)]
mod tests;
