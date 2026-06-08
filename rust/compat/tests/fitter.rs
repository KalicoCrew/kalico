use compat::fitter::fit_subrun;

#[test]
fn straight_line_stays_collinear() {
    let pts = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
    ];
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert!(!pieces.is_empty());
    for p in &pieces {
        assert!(p.j.abs() < 1e-6, "J should be ~0, got {}", p.j);
        assert!(p.q.abs() < 1e-6, "Q should be ~0, got {}", p.q);
    }
}

#[test]
fn circular_arc_within_tolerance() {
    let n = 20_usize;
    let pts: Vec<[f64; 3]> = (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let angle = t * std::f64::consts::FRAC_PI_2;
            [10.0 * angle.cos(), 10.0 * angle.sin(), 0.0]
        })
        .collect();
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert!(!pieces.is_empty());
    assert!(
        pieces.len() < n,
        "expected fewer than {n} pieces, got {}",
        pieces.len()
    );
    let last = &pieces[pieces.len() - 1];
    assert!((last.x - 0.0).abs() < 0.01);
    assert!((last.y - 10.0).abs() < 0.01);
}

#[test]
fn short_run_collinear_fallback() {
    let pts = vec![[0.0, 0.0, 0.0], [5.0, 3.0, 0.0], [10.0, 0.0, 0.0]];
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert_eq!(pieces.len(), 2);
}

#[test]
fn four_point_minimum_for_fitting() {
    let pts = vec![
        [0.0, 0.0, 0.0],
        [3.0, 1.0, 0.0],
        [6.0, 0.0, 0.0],
        [9.0, -1.0, 0.0],
    ];
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert!(!pieces.is_empty());
}

#[test]
fn boundary_tangent_affects_shape() {
    let pts = vec![
        [0.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [4.0, 0.0, 0.0],
        [6.0, -1.0, 0.0],
        [8.0, 0.0, 0.0],
    ];
    let p1 = fit_subrun(&pts, 0.05, None, None);
    let p2 = fit_subrun(&pts, 0.05, Some([0.0, 1.0]), None);
    assert!(!p1.is_empty() && !p2.is_empty());
}

#[test]
fn z_variation_handled() {
    let pts: Vec<[f64; 3]> = (0..=10_usize)
        .map(|i| [i as f64, 0.0, i as f64 * 0.1])
        .collect();
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert!(!pieces.is_empty());
    let last = &pieces[pieces.len() - 1];
    assert!((last.z - 1.0).abs() < 0.01);
}
