use super::*;
use nurbs::VectorNurbs;

#[test]
fn straight_line_x_aligned_returns_unit_tangent_and_zero_curvature() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
    )
    .unwrap();

    let grid = sample_arclength_grid(&curve, 5).unwrap();
    assert_eq!(grid.s.len(), 5);
    assert!((grid.total_length - 10.0).abs() < 1e-6);
    assert!((grid.s[0] - 0.0).abs() < 1e-9);
    assert!((grid.s[4] - 10.0).abs() < 1e-6);
    for tan in &grid.c_prime {
        assert!((tan[0] - 1.0).abs() < 1e-6);
        assert!(tan[1].abs() < 1e-6);
        assert!(tan[2].abs() < 1e-6);
    }
    for k in &grid.kappa {
        assert!(k.abs() < 1e-6);
    }
}

#[test]
fn rejects_grid_size_below_two() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
    )
    .unwrap();
    assert!(matches!(
        sample_arclength_grid(&curve, 1),
        Err(PathSampleError::GridTooSmall(1))
    ));
}

#[test]
fn cubic_bezier_pins_third_derivative_at_start() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 1.0, 0.0],
        ],
    )
    .unwrap();

    let grid = sample_arclength_grid(&curve, 5).unwrap();

    let triple_at_start = grid.c_triple_prime[0];
    let expected = [0.0_f64, 2.0 / 9.0, 0.0];

    let scale = expected[1].abs();
    let err = (triple_at_start[0] - expected[0]).abs()
        + (triple_at_start[1] - expected[1]).abs()
        + (triple_at_start[2] - expected[2]).abs();
    assert!(
        err / scale < 0.01,
        "c_triple_prime[0] = {triple_at_start:?}, expected ≈ {expected:?}, \
         relative err = {:.4} (limit 0.01)",
        err / scale
    );
}

#[test]
fn cubic_bezier_c3_at_endpoints_matches_closed_form() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [3.0, 3.0, 0.0],
            [7.0, 3.0, 0.0],
            [10.0, 0.0, 0.0],
        ],
    )
    .unwrap();

    let grid = sample_arclength_grid(&curve, 200).unwrap();
    let triple_start = grid.c_triple_prime[0];
    let triple_end = *grid.c_triple_prime.last().unwrap();

    let expected_start = [0.000_970_f64, -0.016_489_f64, 0.0];
    let expected_end = [0.000_970_f64, 0.016_489_f64, 0.0];
    let tol = 1e-4_f64;

    for (label, got, exp) in [
        ("start", triple_start, expected_start),
        ("end", triple_end, expected_end),
    ] {
        assert!(
            (got[0] - exp[0]).abs() < tol,
            "{label}: c'''_x = {} vs expected {} (tol {})",
            got[0],
            exp[0],
            tol
        );
        assert!(
            (got[1] - exp[1]).abs() < tol,
            "{label}: c'''_y = {} vs expected {} (tol {})",
            got[1],
            exp[1],
            tol
        );
        assert!(
            got[2].abs() < tol,
            "{label}: c'''_z = {} vs expected 0 (tol {})",
            got[2],
            tol
        );
    }
}

#[test]
fn degenerate_g1_curve_does_not_panic() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
    )
    .unwrap();

    let grid = sample_arclength_grid(&curve, 5).unwrap();

    for (i, c3) in grid.c_triple_prime.iter().enumerate() {
        assert!(
            c3[0].abs() + c3[1].abs() + c3[2].abs() < 1e-9,
            "c_triple_prime[{i}] = {c3:?} should be ~0 on a straight line",
        );
    }
    for (i, c2) in grid.c_double_prime.iter().enumerate() {
        assert!(
            c2[0].abs() + c2[1].abs() + c2[2].abs() < 1e-9,
            "c_double_prime[{i}] = {c2:?} should be ~0 on a straight line",
        );
    }
}
