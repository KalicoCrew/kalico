use compat::collinear::to_collinear_g5;

/// I = 10/3, J = 0, P = -10/3, Q = 0.
#[test]
fn collinear_simple_xy() {
    let line = to_collinear_g5([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.5, Some(3000.0));

    assert!((line.x - 10.0).abs() < 1e-12);
    assert!((line.y - 0.0).abs() < 1e-12);
    assert!((line.z - 0.0).abs() < 1e-12);
    assert!((line.i - 10.0 / 3.0).abs() < 1e-12);
    assert!((line.j - 0.0).abs() < 1e-12);
    assert!((line.p - (-10.0 / 3.0)).abs() < 1e-12);
    assert!((line.q - 0.0).abs() < 1e-12);
    assert!((line.e - 1.5).abs() < 1e-12);
    assert_eq!(line.f, Some(3000.0));
}

#[test]
fn collinear_with_z() {
    let line = to_collinear_g5([0.0, 0.0, 2.5], [0.0, 0.0, 5.0], 0.0, None);

    assert!((line.i).abs() < 1e-12);
    assert!((line.j).abs() < 1e-12);
    assert!((line.p).abs() < 1e-12);
    assert!((line.q).abs() < 1e-12);
    assert!((line.z - 5.0).abs() < 1e-12);
    assert_eq!(line.f, None);
}

/// dx = 3, dy = 4 → I = 1, J = 4/3, P = -1, Q = -4/3.
#[test]
fn collinear_diagonal() {
    let line = to_collinear_g5([5.0, 5.0, 1.0], [8.0, 9.0, 1.0], 2.0, None);

    assert!((line.x - 8.0).abs() < 1e-12);
    assert!((line.y - 9.0).abs() < 1e-12);
    assert!((line.z - 1.0).abs() < 1e-12);
    assert!((line.i - 1.0).abs() < 1e-12);
    assert!((line.j - 4.0 / 3.0).abs() < 1e-12);
    assert!((line.p - (-1.0)).abs() < 1e-12);
    assert!((line.q - (-4.0 / 3.0)).abs() < 1e-12);
}

#[test]
fn collinear_zero_length() {
    let line = to_collinear_g5([3.0, 7.0, 0.0], [3.0, 7.0, 0.0], 0.0, None);

    assert!(line.i.abs() < 1e-12, "I should be ~0, got {}", line.i);
    assert!(line.j.abs() < 1e-12, "J should be ~0, got {}", line.j);
    assert!(line.p.abs() < 1e-12, "P should be ~0, got {}", line.p);
    assert!(line.q.abs() < 1e-12, "Q should be ~0, got {}", line.q);
    assert!((line.x - 3.0).abs() < 1e-12);
    assert!((line.y - 7.0).abs() < 1e-12);
}
