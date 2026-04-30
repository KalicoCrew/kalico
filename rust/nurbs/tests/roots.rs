use nurbs::bezier::BezierPiece;

#[test]
fn roots_of_linear() {
    // p(x) = -1 + 2x  on [0, 2] -> root at x=0.5 -> u=0.5
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 2.0,
        coeffs: vec![-1.0, 2.0],
    };
    let roots = p.real_roots_in_domain();
    assert_eq!(roots.len(), 1);
    assert!((roots[0] - 0.5).abs() < 1e-10);
}

#[test]
fn roots_of_quadratic_two_roots() {
    // p(x) = x^2 - x + 0.21 = (x-0.3)(x-0.7) on [0, 1]
    // coeffs in Pascal-shifted at u_start=0: [0.21, -1.0, 1.0]
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.21, -1.0, 1.0],
    };
    let mut roots = p.real_roots_in_domain();
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(roots.len(), 2);
    assert!((roots[0] - 0.3).abs() < 1e-8);
    assert!((roots[1] - 0.7).abs() < 1e-8);
}

#[test]
fn roots_outside_domain_excluded() {
    // p(x) = x - 5  on [0, 1] -> root at x=5, outside
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![-5.0, 1.0],
    };
    let roots = p.real_roots_in_domain();
    assert!(roots.is_empty());
}

#[test]
fn roots_with_nonzero_u_start() {
    // p(u) = (u-2.5) on [2, 3], Pascal-shifted at u_start=2: coeffs = [-0.5, 1.0]
    let p = BezierPiece {
        u_start: 2.0,
        u_end: 3.0,
        coeffs: vec![-0.5, 1.0],
    };
    let roots = p.real_roots_in_domain();
    assert_eq!(roots.len(), 1);
    assert!((roots[0] - 2.5).abs() < 1e-10);
}

#[test]
fn roots_of_cubic() {
    // p(x) = x(x-0.5)(x-1) = x^3 - 1.5x^2 + 0.5x on [0, 1]
    // coeffs at u_start=0: [0.0, 0.5, -1.5, 1.0]
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.5, -1.5, 1.0],
    };
    let mut roots = p.real_roots_in_domain();
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(roots.len(), 3);
    assert!((roots[0] - 0.0).abs() < 1e-8);
    assert!((roots[1] - 0.5).abs() < 1e-8);
    assert!((roots[2] - 1.0).abs() < 1e-8);
}

#[test]
fn no_real_roots() {
    // p(x) = x^2 + 1 on [-2, 2] -> no real roots
    let p = BezierPiece {
        u_start: -2.0,
        u_end: 2.0,
        coeffs: vec![1.0, 0.0, 1.0],
    };
    let roots = p.real_roots_in_domain();
    assert!(roots.is_empty());
}

#[test]
fn degree_6_roots_evaluate_near_zero() {
    // Build a degree-5 polynomial with known roots at 0.1, 0.3, 0.5, 0.7, 0.9
    // p(x) = (x-0.1)(x-0.3)(x-0.5)(x-0.7)(x-0.9)
    let mut coeffs = vec![1.0]; // start with 1
    for &root in &[0.1, 0.3, 0.5, 0.7, 0.9] {
        let mut new_coeffs = vec![0.0; coeffs.len() + 1];
        for (i, &c) in coeffs.iter().enumerate() {
            new_coeffs[i] += c * (-root);
            new_coeffs[i + 1] += c;
        }
        coeffs = new_coeffs;
    }
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs,
    };
    let roots = p.real_roots_in_domain();
    // Should find 5 roots, all evaluating near zero
    assert!(
        roots.len() >= 5,
        "found {} roots, expected 5",
        roots.len()
    );
    for r in &roots {
        assert!(
            p.evaluate(*r).abs() < 1e-6,
            "root {r} evaluates to {}",
            p.evaluate(*r)
        );
        assert!(
            *r >= -1e-10 && *r <= 1.0 + 1e-10,
            "root {r} outside domain"
        );
    }
}
