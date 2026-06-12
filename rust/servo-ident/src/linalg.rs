/// Cholesky solve of A·x = y for symmetric positive-definite A (row-major
/// n×n). None when A is not PD.
#[allow(clippy::indexing_slicing)]
pub fn solve_spd(a: &[f64], y: &[f64], n: usize) -> Option<Vec<f64>> {
    assert_eq!(a.len(), n * n);
    assert_eq!(y.len(), n);
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = a[i * n + j];
            for k in 0..j {
                s -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if s <= 0.0 {
                    return None;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    let mut z = vec![0.0; n];
    for i in 0..n {
        let mut s = y[i];
        for k in 0..i {
            s -= l[i * n + k] * z[k];
        }
        z[i] = s / l[i * n + i];
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = z[i];
        for k in (i + 1)..n {
            s -= l[k * n + i] * x[k];
        }
        x[i] = s / l[i * n + i];
    }
    Some(x)
}

/// Smallest and largest eigenvalue of a symmetric matrix via cyclic Jacobi.
#[allow(clippy::indexing_slicing)]
pub fn sym_eig_extremes(a: &[f64], n: usize) -> (f64, f64) {
    assert_eq!(a.len(), n * n);
    let mut m = a.to_vec();
    for _sweep in 0..64 {
        let mut off = 0.0;
        for p in 0..n {
            for q in (p + 1)..n {
                off += m[p * n + q] * m[p * n + q];
            }
        }
        if off < 1e-24 {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                let apq = m[p * n + q];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (m[q * n + q] - m[p * n + p]) / (2.0 * apq);
                let sign = if theta >= 0.0 { 1.0_f64 } else { -1.0_f64 };
                let t = sign / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let akp = m[k * n + p];
                    let akq = m[k * n + q];
                    m[k * n + p] = c * akp - s * akq;
                    m[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let apk = m[p * n + k];
                    let aqk = m[q * n + k];
                    m[p * n + k] = c * apk - s * aqk;
                    m[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    #[cfg(debug_assertions)]
    {
        let mut residual_off = 0.0;
        for p in 0..n {
            for q in (p + 1)..n {
                residual_off += m[p * n + q] * m[p * n + q];
            }
        }
        assert!(
            residual_off < 1e-24,
            "sym_eig_extremes: Jacobi did not converge in 64 sweeps (n={n})"
        );
    }
    for i in 0..n {
        lo = lo.min(m[i * n + i]);
        hi = hi.max(m[i * n + i]);
    }
    (lo, hi)
}
