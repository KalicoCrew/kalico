use temporal::topp::stencil::s_dddot_at;

const H: f64 = 0.4;
const N_GRID: usize = 10;

fn j_axis_at_iterate(
    b_bars: &[f64],
    a_bars: &[f64],
    i: usize,
    cp: f64,
    cpp: f64,
    cppp: f64,
) -> f64 {
    let s_dddot = s_dddot_at(b_bars, i, H);
    let b_i = b_bars[i].max(0.0);
    let s_i = b_i.sqrt();
    let s3 = b_i * s_i;
    cppp * s3 + 3.0 * cpp * a_bars[i] * s_i + cp * s_dddot
}

fn interior_cut_coeffs(
    b_bars: &[f64],
    a_bars: &[f64],
    i: usize,
    cp: f64,
    cpp: f64,
    cppp: f64,
) -> (f64, f64, f64, f64, f64) {
    let b_i = b_bars[i];
    let s = b_i.sqrt();
    let s3 = b_i * s;
    let d2 = b_bars[i - 1] - 2.0 * b_i + b_bars[i + 1];
    let alpha_b_im1 = cp * s / (2.0 * H * H);
    let alpha_b_ip1 = cp * s / (2.0 * H * H);
    let alpha_a_i = 3.0 * cpp * s;
    let alpha_b_i = 1.5 * cppp * s + 3.0 * cpp * a_bars[i] / (2.0 * s) - cp * s / (H * H)
        + cp * d2 / (4.0 * H * H * s);
    let k = -0.5 * cppp * s3 - 1.5 * cpp * a_bars[i] * s - cp * d2 * s / (4.0 * H * H);
    (alpha_b_im1, alpha_b_i, alpha_b_ip1, alpha_a_i, k)
}

fn start_boundary_cut_coeffs(
    b_bars: &[f64],
    a_bars: &[f64],
    cp: f64,
    cpp: f64,
    cppp: f64,
) -> (f64, f64, f64, f64, f64) {
    let b_0 = b_bars[0];
    let s = b_0.sqrt();
    let s3 = b_0 * s;
    let d2 = b_0 - 2.0 * b_bars[1] + b_bars[2];
    let alpha_b_0 = 1.5 * cppp * s
        + 3.0 * cpp * a_bars[0] / (2.0 * s)
        + cp * s / (2.0 * H * H)
        + cp * d2 / (4.0 * H * H * s);
    let alpha_b_1 = -cp * s / (H * H);
    let alpha_b_2 = cp * s / (2.0 * H * H);
    let alpha_a_0 = 3.0 * cpp * s;
    let k = -0.5 * cppp * s3 - 1.5 * cpp * a_bars[0] * s - cp * d2 * s / (4.0 * H * H);
    (alpha_b_0, alpha_b_1, alpha_b_2, alpha_a_0, k)
}

fn end_boundary_cut_coeffs(
    b_bars: &[f64],
    a_bars: &[f64],
    cp: f64,
    cpp: f64,
    cppp: f64,
) -> (f64, f64, f64, f64, f64) {
    let n = b_bars.len();
    let b_last = b_bars[n - 1];
    let s = b_last.sqrt();
    let s3 = b_last * s;
    let d2 = b_bars[n - 3] - 2.0 * b_bars[n - 2] + b_last;
    let alpha_b_nm3 = cp * s / (2.0 * H * H);
    let alpha_b_nm2 = -cp * s / (H * H);
    let alpha_b_nm1 = 1.5 * cppp * s
        + 3.0 * cpp * a_bars[n - 1] / (2.0 * s)
        + cp * s / (2.0 * H * H)
        + cp * d2 / (4.0 * H * H * s);
    let alpha_a_nm1 = 3.0 * cpp * s;
    let k = -0.5 * cppp * s3 - 1.5 * cpp * a_bars[n - 1] * s - cp * d2 * s / (4.0 * H * H);
    (alpha_b_nm3, alpha_b_nm2, alpha_b_nm1, alpha_a_nm1, k)
}

fn check_identity_at(
    b_bars: &[f64],
    a_bars: &[f64],
    i: usize,
    cp: f64,
    cpp: f64,
    cppp: f64,
    label: &str,
) {
    let n = b_bars.len();
    let j_actual = j_axis_at_iterate(b_bars, a_bars, i, cp, cpp, cppp);

    let j_from_cut = if i == 0 {
        let (a_b_0, a_b_1, a_b_2, a_a_0, k) =
            start_boundary_cut_coeffs(b_bars, a_bars, cp, cpp, cppp);
        a_b_0 * b_bars[0] + a_b_1 * b_bars[1] + a_b_2 * b_bars[2] + a_a_0 * a_bars[0] + k
    } else if i == n - 1 {
        let (a_b_nm3, a_b_nm2, a_b_nm1, a_a_nm1, k) =
            end_boundary_cut_coeffs(b_bars, a_bars, cp, cpp, cppp);
        a_b_nm3 * b_bars[n - 3]
            + a_b_nm2 * b_bars[n - 2]
            + a_b_nm1 * b_bars[n - 1]
            + a_a_nm1 * a_bars[n - 1]
            + k
    } else {
        let (a_b_im1, a_b_i, a_b_ip1, a_a_i, k) =
            interior_cut_coeffs(b_bars, a_bars, i, cp, cpp, cppp);
        a_b_im1 * b_bars[i - 1]
            + a_b_i * b_bars[i]
            + a_b_ip1 * b_bars[i + 1]
            + a_a_i * a_bars[i]
            + k
    };

    let diff = (j_actual - j_from_cut).abs();
    assert!(
        diff < 1e-9,
        "{label} identity failed at i={i}: j_actual={j_actual}, j_from_cut={j_from_cut}, diff={diff}"
    );
}

fn synthetic_iterate() -> (Vec<f64>, Vec<f64>) {
    let b: Vec<f64> = (0..N_GRID)
        .map(|i| {
            let s = i as f64 * H;
            10.0 + 5.0 * s + 0.1 * s * s
        })
        .collect();
    let a: Vec<f64> = (0..N_GRID).map(|i| 2.0 + 0.3 * (i as f64)).collect();
    (b, a)
}

#[test]
fn row_sum_identity_collinear_paths() {
    let (b, a) = synthetic_iterate();
    for &i in &[0usize, 1, 5, 8, 9] {
        check_identity_at(&b, &a, i, 1.0, 0.0, 0.0, "collinear");
    }
}

#[test]
fn row_sum_identity_curved_paths() {
    let (b, a) = synthetic_iterate();
    for &i in &[0usize, 1, 5, 8, 9] {
        check_identity_at(&b, &a, i, 0.7, 0.4, 0.0, "curved");
    }
}

#[test]
fn row_sum_identity_pathological_paths() {
    let (b, a) = synthetic_iterate();
    for &i in &[0usize, 1, 5, 8, 9] {
        check_identity_at(&b, &a, i, 0.6, -0.3, 0.2, "pathological");
    }
}

#[test]
fn row_sum_identity_holds_at_slp_b_floor() {
    // The cut helper floors b̄[i] at SLP_B_FLOOR=1.0 before computing coefficients.
    // The identity must hold at the FLOORED value — compute ground-truth with the
    // same floor applied.
    let mut b = synthetic_iterate().0;
    let a = synthetic_iterate().1;
    b[5] = 0.5;
    let i = 5;
    let cp = 0.6;
    let cpp = -0.3;
    let cppp = 0.2;

    let mut b_floored = b.clone();
    b_floored[i] = b[i].max(1.0);
    check_identity_at(&b_floored, &a, i, cp, cpp, cppp, "slp_b_floor");
}
