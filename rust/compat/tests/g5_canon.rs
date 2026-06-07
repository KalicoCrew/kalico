use compat::g5_canon::canonicalize_g5;

#[test]
fn explicit_ij_passthrough() {
    let mut params = gcode::Params::default();
    params.set(b'I', 3.0);
    params.set(b'J', 3.0);
    params.set(b'P', -3.0);
    params.set(b'Q', 3.0);

    let result = canonicalize_g5(&params, Some([2.0, 1.0]));
    let (i, j, p, q) = result.expect("should succeed");
    assert!((i - 3.0).abs() < 1e-12, "i={i}");
    assert!((j - 3.0).abs() < 1e-12, "j={j}");
    assert!((p - (-3.0)).abs() < 1e-12, "p={p}");
    assert!((q - 3.0).abs() < 1e-12, "q={q}");
}

#[test]
fn implicit_ij_from_chain() {
    let mut params = gcode::Params::default();
    params.set(b'P', 5.0);
    params.set(b'Q', -4.0);

    let result = canonicalize_g5(&params, Some([2.0, 1.0]));
    let (i, j, p, q) = result.expect("should succeed");
    assert!((i - (-2.0)).abs() < 1e-12, "i={i} expected -2");
    assert!((j - (-1.0)).abs() < 1e-12, "j={j} expected -1");
    assert!((p - 5.0).abs() < 1e-12, "p={p}");
    assert!((q - (-4.0)).abs() < 1e-12, "q={q}");
}

#[test]
fn implicit_ij_no_chain_errors() {
    let mut params = gcode::Params::default();
    params.set(b'P', 5.0);
    params.set(b'Q', -4.0);

    let result = canonicalize_g5(&params, None);
    assert!(result.is_err(), "expected Err when prev_pq is None");
}

#[test]
fn missing_pq_errors() {
    let mut params = gcode::Params::default();
    params.set(b'I', 1.0);
    params.set(b'J', 2.0);

    let result = canonicalize_g5(&params, None);
    assert!(result.is_err(), "expected Err when P/Q are missing");
}
