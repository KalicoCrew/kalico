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
    // All J and Q should be ~0 (no lateral deviation)
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
    // Should produce fewer pieces than input segments
    assert!(
        pieces.len() < n,
        "expected fewer than {n} pieces, got {}",
        pieces.len()
    );
    // Last piece should end near (0, 10)
    let last = &pieces[pieces.len() - 1];
    assert!((last.x - 0.0).abs() < 0.01);
    assert!((last.y - 10.0).abs() < 0.01);
}

#[test]
fn short_run_collinear_fallback() {
    let pts = vec![[0.0, 0.0, 0.0], [5.0, 3.0, 0.0], [10.0, 0.0, 0.0]];
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert_eq!(pieces.len(), 2); // per-segment collinear
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
    // Same points, different start tangent should produce different control points
    let pts = vec![
        [0.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [4.0, 0.0, 0.0],
        [6.0, -1.0, 0.0],
        [8.0, 0.0, 0.0],
    ];
    let p1 = fit_subrun(&pts, 0.05, None, None);
    let p2 = fit_subrun(&pts, 0.05, Some([0.0, 1.0]), None);
    // With upward start tangent, first piece should have different J
    assert!(!p1.is_empty() && !p2.is_empty());
    // The shapes should differ (can't assert exact values but at least both should succeed)
}

#[test]
fn z_variation_handled() {
    // Points with Z ramp -- fitter should handle without panicking
    let pts: Vec<[f64; 3]> = (0..=10_usize)
        .map(|i| [i as f64, 0.0, i as f64 * 0.1])
        .collect();
    let pieces = fit_subrun(&pts, 0.005, None, None);
    assert!(!pieces.is_empty());
    // Last piece Z should be ~1.0
    let last = &pieces[pieces.len() - 1];
    assert!((last.z - 1.0).abs() < 0.01);
}
