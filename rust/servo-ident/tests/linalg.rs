use servo_ident::linalg::{solve_spd, sym_eig_extremes};

#[test]
fn solves_known_spd_system() {
    let a = vec![4.0, 1.0, 1.0, 3.0];
    let x = solve_spd(&a, &[1.0, 2.0], 2).unwrap();
    assert!((x[0] - 1.0 / 11.0).abs() < 1e-12);
    assert!((x[1] - 7.0 / 11.0).abs() < 1e-12);
}

#[test]
fn rejects_non_pd() {
    let a = vec![1.0, 2.0, 2.0, 1.0];
    assert!(solve_spd(&a, &[1.0, 1.0], 2).is_none());
}

#[test]
fn eig_extremes_of_diagonal() {
    let a = vec![9.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.0];
    let (lo, hi) = sym_eig_extremes(&a, 3);
    assert!((lo - 1.0).abs() < 1e-9 && (hi - 9.0).abs() < 1e-9);
}

#[test]
fn eig_extremes_of_rotated_matrix() {
    let a = vec![2.0, 1.0, 1.0, 2.0];
    let (lo, hi) = sym_eig_extremes(&a, 2);
    assert!((lo - 1.0).abs() < 1e-9 && (hi - 3.0).abs() < 1e-9);
}
