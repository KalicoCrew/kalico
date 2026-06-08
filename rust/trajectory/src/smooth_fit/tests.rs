use super::*;

#[test]
fn thomas_solves_known_system() {
    // Tridiagonal system:
    // [ 2 1 0 ] [x0]   [3]
    // [ 1 2 1 ] [x1] = [4]   -> solution x = [1, 1, 1]
    // [ 0 1 2 ] [x2]   [3]
    let a = [0.0, 1.0, 1.0]; // sub-diagonal (a[0] unused)
    let b = [2.0, 2.0, 2.0]; // diagonal
    let c = [1.0, 1.0, 0.0]; // super-diagonal (c[n-1] unused)
    let d = [3.0, 4.0, 3.0];
    let x = solve_tridiagonal(&a, &b, &c, &d);
    for xi in &x {
        assert!((xi - 1.0).abs() < 1e-12, "x = {x:?}");
    }
}
