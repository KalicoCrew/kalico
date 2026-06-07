use super::*;
use crate::Limits;
use crate::topp::constraints::{BuildOutcome, EndpointVelocities, build};
use crate::topp::path::ArclengthGrid;

/// Verify that `append_axis_jerk_cut_to_clarabel` emits ∞-norm-normalized rows.
///
/// With cp=1.0, b_bars=[6.0; 3], h=1e-3 the stencil coefficients include
/// cp·√b/h² ≈ 2.449e6 — a scale that historically wrecked QDLDL conditioning.
/// After the fix every pushed coefficient must be ≤ 1.0 in absolute value (the
/// row has been divided through by its ∞-norm), and the RHS values must equal
/// the unscaled values divided by that same scale.
#[test]
fn axis_jerk_cut_row_norm_is_one() {
    let n_grid = 5_usize;
    let off_a = n_grid;
    let n_vars = 2 * n_grid;

    let h = 1e-3_f64;
    let b_val = 6.0_f64;
    let cp = 1.0_f64;
    // cpp and cppp set to zero so the dominant term is cp·√b/h², which is
    // the O(N²) coefficient that caused conditioning failures.
    let h_uniform = h;
    let w = crate::topp::stencil::b_dd_weights(h_uniform, h_uniform);
    let cut = AxisJerkCut {
        i: 2,
        axis: 0,
        idx: [1, 2, 3],
        w,
        b_bars: [b_val, b_val, b_val],
        a_bar_i: 0.0,
        cp,
        cpp: 0.0,
        cppp: 0.0,
        j_lim_inflated: 1_000.0,
    };

    // Compute the expected unscaled row_scale.  Interior stencil with cpp=cppp=0,
    // a_bar=0, d2=0:
    //   alpha_b_im1 = cp·√b / (2h²)
    //   alpha_b_ip1 = cp·√b / (2h²)
    //   alpha_b_i   = -cp·√b / h² + 0  (d2 = 0)
    //   alpha_a_i   = 0
    // So |alpha_b_i| = cp·√b/h² and the two side coefficients are half that.
    // row_scale = cp·√b/h².
    let s = b_val.sqrt();
    let expected_scale = cp * s / (h * h);
    assert!(
        expected_scale > 1e5,
        "test is only meaningful with large unscaled coefficients; got {expected_scale}"
    );

    let mut rowval: Vec<Vec<usize>> = vec![Vec::new(); n_vars];
    let mut nzval: Vec<Vec<f64>> = vec![Vec::new(); n_vars];
    let mut b_rhs: Vec<f64> = Vec::new();
    let mut n_rows = 0_usize;

    let b_floor = 0.0_f64;
    append_axis_jerk_cut_to_clarabel(
        &cut,
        b_floor,
        &mut n_rows,
        &mut rowval,
        &mut nzval,
        &mut b_rhs,
        n_grid,
    );

    assert_eq!(n_rows, 2, "expected two rows (± pair)");
    assert_eq!(b_rhs.len(), 2);

    // Collect all non-zero coefficient magnitudes across both rows.
    let max_coeff: f64 = nzval
        .iter()
        .flat_map(|col| col.iter().copied())
        .map(f64::abs)
        .fold(0.0_f64, f64::max);

    assert!(
        (max_coeff - 1.0).abs() < 1e-10,
        "∞-norm of emitted rows should be 1.0, got {max_coeff}"
    );

    // RHS pair: the cut has k_const = 0 (cpp=cppp=0, d2=0, a_bar=0), so
    // rhs_pos = j / row_scale and rhs_neg = j / row_scale — both equal.
    let j = cut.j_lim_inflated;
    let expected_rhs = j / expected_scale;
    assert!(
        (b_rhs[0] - expected_rhs).abs() < 1e-10 * expected_rhs.abs(),
        "rhs[0] = {}, expected {expected_rhs}",
        b_rhs[0]
    );
    assert!(
        (b_rhs[1] - expected_rhs).abs() < 1e-10 * expected_rhs.abs(),
        "rhs[1] = {}, expected {expected_rhs}",
        b_rhs[1]
    );

    // The ± rows must have identical coefficient magnitudes (symmetry preserved).
    let coeff_pos: Vec<f64> = nzval
        .iter()
        .enumerate()
        .filter_map(|(col, entries)| {
            let idx = entries
                .iter()
                .zip(rowval[col].iter())
                .position(|(_, &r)| r == 0)?;
            Some(entries[idx])
        })
        .collect();
    let coeff_neg: Vec<f64> = nzval
        .iter()
        .enumerate()
        .filter_map(|(col, entries)| {
            let idx = entries
                .iter()
                .zip(rowval[col].iter())
                .position(|(_, &r)| r == 1)?;
            Some(entries[idx])
        })
        .collect();
    assert_eq!(
        coeff_pos.len(),
        coeff_neg.len(),
        "± rows must touch the same number of columns"
    );
    for (p, n) in coeff_pos.iter().zip(coeff_neg.iter()) {
        assert!(
            (p.abs() - n.abs()).abs() < 1e-14,
            "coefficient magnitudes must match between ± rows: {p} vs {n}"
        );
    }

    // off_a column (alpha_a_i = 0) must not appear in the output.
    assert!(
        rowval[off_a + cut.i].is_empty(),
        "a_i column should be absent when cpp = 0"
    );
}

fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let u = s.clone();
    let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
    let kappa = vec![0.0; n];
    ArclengthGrid {
        s,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        kappa,
        total_length: length,
    }
}

#[test]
fn straight_line_solves_to_nontrivial_profile() {
    let grid = dummy_straight_grid(50, 100.0);
    let limits = Limits {
        v_max: [500.0, 500.0, 500.0],
        a_max: [5_000.0, 5_000.0, 5_000.0],
        j_max: [100_000.0, 100_000.0, 100_000.0],
        a_centripetal_max: 2_500.0,
    };
    let bundle = match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 0.0,
            v_end: 0.0,
        },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(b) => panic!("expected Ok, got Boundary({b:?})"),
    };
    let result = solve(&bundle).expect("solver setup");
    assert!(
        matches!(
            result.status,
            SolverStatus::Solved | SolverStatus::SolvedInexact { .. }
        ),
        "expected Solved or SolvedInexact, got {:?}",
        result.status
    );
    assert_eq!(result.b.len(), 50);

    // Endpoints clamped to zero (v_start = v_end = 0).
    assert!(
        result.b[0].abs() < 1e-6,
        "b[0] should be ~0, got {}",
        result.b[0]
    );
    assert!(
        result.b[49].abs() < 1e-6,
        "b[49] should be ~0, got {}",
        result.b[49]
    );

    // For length=100mm, zero endpoints, v_max=500 mm/s, a_max=5000 mm/s²:
    //   - If accel-bound throughout: b_max ≈ 2·a·s_peak where s_peak = 50mm,
    //     so b_max ≈ 2·5000·50 = 500_000 (mm/s)².
    //   - If velocity-bound: b_max = v_max² = 250_000 (mm/s)².
    //   - Actual answer is min of the two regimes.
    // Bracket the midpoint: must be substantially > 0 (not just barely-feasible)
    // and below v_max² (the global cap).
    let b_mid = result.b[25];
    assert!(
        b_mid > 1e4,
        "b[25] = {b_mid}, expected > 1e4 (substantially accelerating)"
    );
    assert!(
        b_mid <= 250_000.0 * 1.01,
        "b[25] = {b_mid}, expected ≤ v_max² + tolerance"
    );

    // Sign check: from rest, the path must be ACCELERATING in the first half
    // and DECELERATING in the second half. A sign-flip in the constraint
    // matrix could produce a profile where b is monotonically increasing or
    // decreasing, which we'd miss without these checks.
    assert!(
        result.b[10] > result.b[1],
        "must accelerate from rest: b[1]={}, b[10]={}",
        result.b[1],
        result.b[10]
    );
    assert!(
        result.b[40] < result.b[25],
        "must decelerate toward end: b[25]={}, b[40]={}",
        result.b[25],
        result.b[40]
    );

    // Path acceleration sign: a > 0 in first half, a < 0 in second.
    assert!(
        result.a[5] > 0.0,
        "a[5] = {} should be positive (accelerating)",
        result.a[5]
    );
    assert!(
        result.a[44] < 0.0,
        "a[44] = {} should be negative (decelerating)",
        result.a[44]
    );
}
