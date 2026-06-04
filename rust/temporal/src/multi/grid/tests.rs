use super::*;

fn straight_100mm() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
    )
    .unwrap()
}

#[test]
fn fixed_strategy_returns_n_unchanged() {
    let curve = straight_100mm();
    assert_eq!(compute_n(&GridStrategy::Fixed(50), &curve), 50);
    assert_eq!(compute_n(&GridStrategy::Fixed(200), &curve), 200);
}

#[test]
fn adaptive_short_segment_floors_to_min_n() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], // 1 mm
    )
    .unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 1mm / 0.5mm = 2; clamped to min_n = 10.
    assert_eq!(compute_n(&strategy, &curve), 10);
}

#[test]
fn adaptive_typical_segment_scales_with_arclength() {
    let curve_50 = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 50mm / 0.5mm = 100.
    assert_eq!(compute_n(&strategy, &curve_50), 100);
}

#[test]
fn adaptive_long_segment_caps_to_max_n() {
    let curve_200mm = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
    )
    .unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 200mm / 0.5mm = 400; clamped to max_n = 200.
    assert_eq!(compute_n(&strategy, &curve_200mm), 200);
}

#[test]
fn adaptive_zero_length_segment_floors_to_min_n() {
    // Degenerate G1 with two identical control points — no path length.
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[10.0, 0.0, 0.0], [10.0, 0.0, 0.0]], // zero-length
    )
    .unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 0 / 0.5 = 0 → ceil 0 → clamp to min_n = 10.
    assert_eq!(compute_n(&strategy, &curve), 10);
}
