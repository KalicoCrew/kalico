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

/// Pin `c_triple_prime` to a known closed-form value on a non-trivial cubic Bezier.
///
/// # Fixture
///
/// Degree-3 non-rational NURBS, knots `[0,0,0,0,1,1,1,1]`, control points:
///   P0=(0,0,0), P1=(1,0,0), P2=(2,0,0), P3=(3,1,0).
///
/// # Closed-form algebra at u=0
///
/// For a cubic Bezier C(u) = (1-u)³P0 + 3(1-u)²u P1 + 3(1-u)u² P2 + u³ P3:
///
///   dC/du  = 3[(1-u)²(P1-P0) + 2(1-u)u(P2-P1) + u²(P3-P2)]
///            At u=0: 3·(1,0,0) = (3,0,0)    → f = |dC/du| = 3
///
///   d²C/du² = 6[(1-u)(P2-2P1+P0) + u(P3-2P2+P1)]
///             P2-2P1+P0 = (2,0,0)-(2,0,0)+(0,0,0) = (0,0,0)
///             P3-2P2+P1 = (3,1,0)-(4,0,0)+(1,0,0) = (0,1,0)
///             At u=0: 6·[(1)·(0,0,0) + 0·(0,1,0)] = (0,0,0)
///
///   d³C/du³ = 6(P3-3P2+3P1-P0) = 6·((3,1,0)-(6,0,0)+(3,0,0)-(0,0,0))
///           = 6·(0,1,0) = (0,6,0)   (constant in u for a cubic Bezier)
///
///   df/du     = dot(d²C/du², dC/du) / f = dot((0,0,0),(3,0,0)) / 3 = 0
///   d²f/du²   = (|d²C/du²|² + dot(dC/du, d³C/du³)) / f - (df/du)²/f
///             = (0 + dot((3,0,0),(0,6,0))) / 3 - 0 = 0
///
///   du/ds = 1/f = 1/3
///   d²u/ds² = -df/du / f³ = 0
///   d³u/ds³ = -d²f/du² / f⁴ + 3(df/du)² / f⁵ = 0
///
///   d³C/ds³ = d³C/du³ · (du/ds)³  +  3·d²C/du²·(du/ds)·d²u/ds²  +  dC/du·d³u/ds³
///           = (0,6,0) · (1/3)³     +  3·(0,0,0)·(1/3)·0           +  (3,0,0)·0
///           = (0,6,0) / 27
///           = (0, 2/9, 0)  ≈  (0, 0.22222…, 0)
///
/// # Why this fixture catches chain-rule bugs
///
/// At u=0 all the "speed-variation" terms (df/du, d²f/du², d²u/ds², d³u/ds³) vanish,
/// so `d³C/ds³` reduces to the cleanest possible form: `d³C/du³ / f³`. Any
/// implementation error in those terms would go undetected here — but that is
/// precisely the value: the surviving term `(0,6,0)/27` directly checks that the
/// `d³C/du³ · (du/ds)³` branch is wired correctly. The vanishing of the other terms
/// also guarantees a correct zero contribution from each of them; a sign error
/// or wrong coefficient in those branches that produces a non-zero contribution at
/// this point would corrupt the result and fail the test.
#[test]
fn cubic_bezier_pins_third_derivative_at_start() {
    // Degree-3 non-rational NURBS, knots [0,0,0,0,1,1,1,1].
    // At u=0: dC/du=(3,0,0), d²C/du²=(0,0,0), d³C/du³=(0,6,0) (constant).
    // All speed-variation terms vanish → d³C/ds³ = (0,6,0)/27 = (0, 2/9, 0).
    //
    // Post-fix (vector_derivative replaces FD for non-rational): the result
    // is exact to floating-point round-off (analytical degree-lowering plus
    // the u(s) inversion). Tolerance tightened from 5 % to 1 % accordingly;
    // the previous 5 % was permissive of catastrophic-cancellation noise
    // that has been removed (see /tmp/path_diag.json, /tmp/path_verifier.json).
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

    // n=5 is sufficient; we only assert on index 0 (s=0, u=0).
    let grid = sample_arclength_grid(&curve, 5).unwrap();

    let triple_at_start = grid.c_triple_prime[0];
    let expected = [0.0_f64, 2.0 / 9.0, 0.0];

    let scale = expected[1].abs(); // 2/9
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

/// Pin `c_triple_prime` at *both* endpoints on the asymmetric G5 cubic
/// (used by Step 9 fixture 4). This is the regression guard for the
/// catastrophic-cancellation FD bug: with the old `eval_kth_deriv` k=3
/// stencil floored at `h*0.01 = 1e-7`, the endpoint values were
/// round-off coin-flips (~1e5 raw / O(50) after chain-rule scaling);
/// after switching to analytical `vector_derivative`, they match closed
/// form to ~1e-6.
///
/// # Closed-form derivation (independently verified, NOT taken from the
/// diagnosis whose `predictedValue` x-component was arithmetic-wrong)
///
/// CPs P0=(0,0,0), P1=(3,3,0), P2=(7,3,0), P3=(10,0,0). Cubic Bezier:
///   dC/du = 3[(1−u)²(P1−P0) + 2(1−u)u(P2−P1) + u²(P3−P2)]
///   d²C/du² = 6[(1−u)(P2−2P1+P0) + u(P3−2P2+P1)]
///   d³C/du³ = 6(P3−3P2+3P1−P0) = 6(10−21+9−0, 0−6+9−0, 0) = 6(−2,3,0)·... wait
///
/// Let me recompute: P3−3P2+3P1−P0 = (10,0,0)−3(7,3,0)+3(3,3,0)−(0,0,0)
///                                  = (10−21+9, 0−9+9, 0) = (−2, 0, 0).
/// So d³C/du³ = 6·(−2, 0, 0) = (−12, 0, 0). Constant in u.
///
/// At u=1: dC/du = 3·(P3−P2) = 3·(3,−3,0) = (9, −9, 0). f=|dC/du|=9√2.
/// d²C/du² at u=1: 6·(P3−2P2+P1) = 6·((10,0,0)−(14,6,0)+(3,3,0)) = 6·(−1,−3,0) = (−6,−18,0).
///
/// At u=0: dC/du = 3·(P1−P0) = (9,9,0). f=9√2.
/// d²C/du² at u=0: 6·(P2−2P1+P0) = 6·((7,3,0)−(6,6,0)+(0,0,0)) = 6·(1,−3,0) = (6,−18,0).
///
/// Chain rule (see module docstring) with f=9√2, f²=162, f³=1458√2,
/// f⁴=26244, f⁵=26244·9√2:
///
/// At u=1:
///   df/du = dot((−6,−18,0),(9,−9,0))/9√2 = (−54+162)/9√2 = 12/√2 = 6√2
///   d²u/ds² = −(6√2)/(1458√2) = −1/243
///   |d²C/du²|² = 36+324 = 360
///   dot(dC/du, d³C/du³) = 9·(−12) = −108
///   d²f/du² = (360 − 108)/9√2 − (6√2)²/9√2 = 252/9√2 − 72/9√2 = 180/9√2 = 10√2
///   d³u/ds³ = −10√2/26244 + 3·(6√2)²/(26244·9√2) = −10√2/26244 + 12√2/26244 = 2√2/26244
///
///   c'''(u=1) = (−12,0,0)/(1458√2) + 3·(−6,−18,0)/(9√2)·(−1/243) + (9,−9,0)·(2√2/26244)
///   c'''_x   = −12/(1458√2) + (18·9√2)/(9√2·243·9√2)·... — easier numerically:
///             ≈ −0.005820 + 0.005820 + 0.000970 = +0.000970
///   c'''_y   = 0 + 3·(−18)/(9√2)·(−1/243) + (−9)·(2√2/26244)
///             = 54/(9√2·243) − 18√2/26244
///             ≈ 0.017459 − 0.000970 = +0.016489
///
/// At u=0 (mirror by sign-flips of the y-tangent components):
///   c'''_x(u=0) = +0.000970 (same as u=1; symmetry of the chain-rule x algebra)
///   c'''_y(u=0) = −0.016489 (sign-flipped relative to u=1)
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

    // n=200 matches the Step 9 fixture 4 grid resolution.
    let grid = sample_arclength_grid(&curve, 200).unwrap();
    let triple_start = grid.c_triple_prime[0];
    let triple_end = *grid.c_triple_prime.last().unwrap();

    // Closed-form values (re-derived from scratch above, NOT from
    // /tmp/path_diag.json's `predictedValue` field which has an
    // arithmetic error in the x-component).
    let expected_start = [0.000_970_f64, -0.016_489_f64, 0.0];
    let expected_end = [0.000_970_f64, 0.016_489_f64, 0.0];
    let tol = 1e-4_f64; // generous; analytical degree-lowering is ~1e-12

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

/// Degree-1 (G1 line) NURBS must NOT panic when the chain rule asks for
/// the 2nd or 3rd parametric derivative. Mathematically: a polynomial of
/// degree p has identically zero (p+1)-th and higher derivatives. The
/// patch guards against `vector_derivative`'s `assert!(p >= 1)` panic by
/// returning [0,0,0] when the degree-lowering chain bottoms out.
#[test]
fn degenerate_g1_curve_does_not_panic() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
    )
    .unwrap();

    // n=5 is enough — we only need to confirm the call returns rather
    // than panicking, and that c''' is zero at every grid point (a
    // straight line has zero second and third arclength derivatives).
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
