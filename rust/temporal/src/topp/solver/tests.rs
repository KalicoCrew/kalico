use super::*;
use crate::Limits;
use crate::topp::constraints::{BuildOutcome, EndpointVelocities, build};
use crate::topp::path::ArclengthGrid;

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
