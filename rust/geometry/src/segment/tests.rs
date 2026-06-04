use super::*;
use nurbs::VectorNurbs;

#[test]
fn cubic_variant_constructs() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("valid cubic");
    let cs = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .expect("valid travel");
    let seg: Segment = Segment::Cubic(cs);
    assert!(matches!(seg, Segment::Cubic(_)));
}

/// **Position continuity invariant** (the headline `split_cubic_bezier`
/// guarantee): for any `s ∈ (0, 1)` the left half evaluated at `u = 1` and
/// the right half evaluated at `u = 0` both reproduce the original curve's
/// value at `s`, and the global curve traversed via left-then-right
/// matches the original at every sampled point.
#[test]
fn split_cubic_bezier_preserves_position_at_split_and_along_curve() {
    use nurbs::eval::vector_eval;

    // A non-degenerate, non-collinear cubic Bézier in 3D. Z varies too so
    // we exercise all three coordinates.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0, 20.0, 1.0],
            [30.0, 5.0, 2.0],
            [40.0, 25.0, 3.0],
        ],
    )
    .unwrap();

    for &s in &[0.1_f64, 0.25, 0.4, 0.5, 0.6, 0.75, 0.9] {
        let (left, right) = split_cubic_bezier(&xyz, s);

        // Continuity at the split: left(1) == right(0) == xyz(s).
        let p_orig = vector_eval(&xyz, s);
        let p_left_end = vector_eval(&left, 1.0);
        let p_right_start = vector_eval(&right, 0.0);
        for axis in 0..3 {
            assert!(
                (p_left_end[axis] - p_orig[axis]).abs() < 1e-12,
                "s = {s}, axis {axis}: left(1) = {} vs xyz(s) = {}",
                p_left_end[axis],
                p_orig[axis],
            );
            assert!(
                (p_right_start[axis] - p_orig[axis]).abs() < 1e-12,
                "s = {s}, axis {axis}: right(0) = {} vs xyz(s) = {}",
                p_right_start[axis],
                p_orig[axis],
            );
        }

        // Endpoint continuity: left(0) == xyz(0), right(1) == xyz(1).
        let p0 = vector_eval(&xyz, 0.0);
        let p1 = vector_eval(&xyz, 1.0);
        let p_left_start = vector_eval(&left, 0.0);
        let p_right_end = vector_eval(&right, 1.0);
        for axis in 0..3 {
            assert!((p_left_start[axis] - p0[axis]).abs() < 1e-12);
            assert!((p_right_end[axis] - p1[axis]).abs() < 1e-12);
        }

        // Along-curve fidelity: sample both halves at a sweep of internal
        // u values and confirm the reconstructed traversal matches the
        // original curve at the matching s-domain location.
        //
        // Left covers original [0, s], right covers [s, 1]. Sample 21 u
        // values in [0, 1] for each half and remap to the original s-domain.
        for k in 0..=20 {
            let u_local = (k as f64) / 20.0;
            let u_left = u_local * s;
            let u_right = s + u_local * (1.0 - s);
            let lhs_left = vector_eval(&left, u_local);
            let lhs_right = vector_eval(&right, u_local);
            let rhs_left = vector_eval(&xyz, u_left);
            let rhs_right = vector_eval(&xyz, u_right);
            for axis in 0..3 {
                assert!(
                    (lhs_left[axis] - rhs_left[axis]).abs() < 1e-10,
                    "s = {s}, u_local = {u_local}, axis {axis}: left mismatch",
                );
                assert!(
                    (lhs_right[axis] - rhs_right[axis]).abs() < 1e-10,
                    "s = {s}, u_local = {u_local}, axis {axis}: right mismatch",
                );
            }
        }

        // Both halves are valid single-piece cubic Béziers — same
        // invariants `CubicSegment::try_new` enforces.
        assert_eq!(left.degree(), 3);
        assert_eq!(right.degree(), 3);
        assert_eq!(left.control_points().len(), 4);
        assert_eq!(right.control_points().len(), 4);
        let expected: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        assert_eq!(left.knots(), expected.as_slice());
        assert_eq!(right.knots(), expected.as_slice());
    }
}

/// `split_cubic_bezier` panics if `s` is not strictly interior.
#[test]
#[should_panic(expected = "strictly interior")]
fn split_cubic_bezier_panics_on_boundary_s() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let _ = split_cubic_bezier(&xyz, 0.0);
}
