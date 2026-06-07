use nurbs::bezier::BezierPiece;

#[test]
fn roots_of_linear() {
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
    let mut coeffs = vec![1.0];
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
    assert!(roots.len() >= 5, "found {} roots, expected 5", roots.len());
    for r in &roots {
        assert!(
            p.evaluate(*r).abs() < 1e-6,
            "root {r} evaluates to {}",
            p.evaluate(*r)
        );
        assert!(*r >= -1e-10 && *r <= 1.0 + 1e-10, "root {r} outside domain");
    }
}
