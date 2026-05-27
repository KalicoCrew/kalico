use super::*;

#[test]
fn unknown_id_returns_none() {
    let mut cps = [0.0f32; FIXTURE_CPS_MAX];
    let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
    let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
    assert!(lookup(99, &mut cps, &mut knots, &mut weights).is_none());
}

#[test]
fn straight_line_shape() {
    let mut cps = [0.0f32; FIXTURE_CPS_MAX];
    let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
    let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
    let (degree, n_cp, n_knots, n_weights) =
        lookup(0, &mut cps, &mut knots, &mut weights).expect("fixture 0");
    assert_eq!((degree, n_cp, n_knots, n_weights), (1, 2, 4, 2));
    // Clamped degree-1: knots == [0, 0, 1, 1].
    assert_eq!(&knots[..4], &[0.0, 0.0, 1.0, 1.0]);
    assert_eq!(cps[3], 10.0);
}

#[test]
fn quarter_arc_shape() {
    let mut cps = [0.0f32; FIXTURE_CPS_MAX];
    let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
    let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
    let (degree, n_cp, n_knots, n_weights) =
        lookup(1, &mut cps, &mut knots, &mut weights).expect("fixture 1");
    assert_eq!((degree, n_cp, n_knots, n_weights), (2, 3, 6, 3));
    assert_eq!(weights[0], 1.0);
    assert_eq!(weights[2], 1.0);
    // Middle weight is cos(pi/4) ≈ 0.7071...
    assert!((weights[1] - 0.707_106_77).abs() < 1e-6);
}

#[test]
fn cubic_bezier_shape() {
    let mut cps = [0.0f32; FIXTURE_CPS_MAX];
    let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
    let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
    let (degree, n_cp, n_knots, n_weights) =
        lookup(2, &mut cps, &mut knots, &mut weights).expect("fixture 2");
    assert_eq!((degree, n_cp, n_knots, n_weights), (3, 4, 8, 4));
    // Clamped degree-3: 4 zeros + 4 ones.
    assert_eq!(&knots[..4], &[0.0, 0.0, 0.0, 0.0]);
    assert_eq!(&knots[4..8], &[1.0, 1.0, 1.0, 1.0]);
}

/// Extract scalar (first component) from 3D fixture CPs.
fn extract_scalar_cps(
    cps_3d: &[f32],
    n_cp: usize,
) -> [f32; crate::curve_pool::MAX_CONTROL_POINTS] {
    let mut scalar = [0.0f32; crate::curve_pool::MAX_CONTROL_POINTS];
    for i in 0..n_cp {
        scalar[i] = cps_3d[i * 3];
    }
    scalar
}

#[test]
fn loads_into_curve_pool_via_validate_and_load() {
    // End-to-end: fixture 0 must validate as a NURBS through the regular
    // (production) `validate_and_load` path. Step 7-B: fixtures emit
    // 3D data; we extract the X component as scalar.
    use crate::curve_pool::CurvePool;
    let mut cps = [0.0f32; FIXTURE_CPS_MAX];
    let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
    let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
    let (degree, n_cp, n_knots, _n_weights) =
        lookup(0, &mut cps, &mut knots, &mut weights).expect("fixture 0");
    let scalar = extract_scalar_cps(&cps, n_cp);
    let pool = CurvePool::new();
    let r = pool.validate_and_load(0, degree, &knots[..n_knots], &scalar[..n_cp]);
    assert!(r.is_ok(), "fixture 0 must validate as a NURBS: {r:?}");
}

#[test]
fn load_unchecked_round_trips() {
    // The FFI path: `load_unchecked` should accept fixture data and
    // produce a resolvable view.
    use crate::curve_pool::CurvePool;
    let pool = CurvePool::new();
    for fid in [0u16, 1u16, 2u16] {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, _n_weights) =
            lookup(fid, &mut cps, &mut knots, &mut weights).expect("fixture");
        let scalar = extract_scalar_cps(&cps, n_cp);
        let handle = pool
            .load_unchecked(fid, degree, &knots[..n_knots], &scalar[..n_cp])
            .unwrap_or_else(|e| panic!("fixture {fid} must load_unchecked: {e:?}"));
        assert!(pool.lookup(handle).is_ok());
        // After confirm_retired we can re-load the same slot — exercises
        // the SEGMENT_END reclaim path indirectly.
        pool.confirm_retired(handle);
    }
}

#[test]
fn loads_quarter_arc_and_cubic() {
    use crate::curve_pool::CurvePool;
    let pool = CurvePool::new();
    for fid in [1u16, 2u16] {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, _n_weights) =
            lookup(fid, &mut cps, &mut knots, &mut weights).expect("fixture");
        let scalar = extract_scalar_cps(&cps, n_cp);
        let r = pool.validate_and_load(fid, degree, &knots[..n_knots], &scalar[..n_cp]);
        assert!(r.is_ok(), "fixture {fid} must validate: {r:?}");
    }
}
