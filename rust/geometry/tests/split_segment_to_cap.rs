// `metadata_propagates` asserts byte-equal pass-through of the f64 fields the
// splitter copies verbatim (no arithmetic, no conversion). Comparing those with
// `assert_eq!` is the precise check for that contract — `float_cmp` does not
// apply to "this field is unchanged".
#![allow(clippy::float_cmp)]

use geometry::{CubicSegment, EMode, SourceRange, split_segment_to_cap};
use nurbs::VectorNurbs;

fn straight_cubic(length_mm: f64) -> CubicSegment {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [length_mm / 3.0, 0.0, 0.0],
            [2.0 * length_mm / 3.0, 0.0, 0.0],
            [length_mm, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    CubicSegment::try_new(
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
    .unwrap()
}

#[test]
fn passthrough_when_below_cap() {
    let seg = straight_cubic(5.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn passthrough_at_exact_cap() {
    let seg = straight_cubic(12.5);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn splits_into_two_at_25mm() {
    let seg = straight_cubic(25.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 2);
    for (i, child) in out.iter().enumerate() {
        let info = child.split_info.expect("split_info populated");
        assert_eq!(info.sub_index, i as u32);
        assert_eq!(info.sub_count, 2);
    }
}

#[test]
fn splits_into_eight_at_100mm() {
    let seg = straight_cubic(100.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 8);
}

#[test]
fn metadata_propagates() {
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for child in &out {
        assert_eq!(child.feedrate_mm_s, seg.feedrate_mm_s);
        assert_eq!(child.e_mode, seg.e_mode);
        assert_eq!(child.extrusion_per_xy_mm, seg.extrusion_per_xy_mm);
        assert_eq!(child.source, seg.source);
    }
}

#[test]
fn boundary_continuity_within_round_off() {
    // Round-1-review fix: split_piece_at re-shifts coefficients, and
    // to_bernstein/from_bernstein adds another floating-point pass on the
    // way through vector_nurbs_from_pieces. Use a tolerance bound, not
    // assert_eq!.
    const BOUNDARY_TOL: f64 = 1e-12;
    use nurbs::eval::vector_eval;
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for window in out.windows(2) {
        let left_end = vector_eval(&window[0].xyz, 1.0);
        let right_start = vector_eval(&window[1].xyz, 0.0);
        for axis in 0..3 {
            let diff = (left_end[axis] - right_start[axis]).abs();
            assert!(
                diff < BOUNDARY_TOL,
                "boundary mismatch axis {axis}: {left_end:?} vs {right_start:?}, diff={diff}"
            );
        }
    }
}

#[test]
fn pure_e_only_independent_passthrough() {
    use nurbs::ScalarNurbs;
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0; 3]; 4], // all four CPs at origin → cp_polygon_length == 0
        None,
    )
    .unwrap();
    let e_curve = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, -2.0], // retraction
        None,
    )
    .unwrap();
    let seg = CubicSegment::try_new(
        xyz,
        EMode::Independent,
        0.0,
        Some(e_curve),
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .unwrap();
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn closed_loop_chord_zero_splits_by_arc_length() {
    // Cubic Bézier returning to its start point but with real arc length.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [50.0, 50.0, 0.0],
            [-50.0, 50.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap();
    let seg = CubicSegment::try_new(
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
    .unwrap();

    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert!(out.len() > 1, "closed loop should split, not passthrough");
}

#[test]
fn invalid_cap_rejects_zero() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, 0.0).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_negative() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, -1.0).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_nan() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, f64::NAN).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_infinity() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, f64::INFINITY).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn rejects_independent_with_non_trivial_xyz() {
    use nurbs::ScalarNurbs;

    // Construct an Independent CubicSegment with non-trivial xyz motion
    // (long Z move). Pre-fix this would silently produce N× over-extrusion;
    // post-fix the splitter rejects it at runtime with CannotSplitIndependent.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 10.0],
            [0.0, 0.0, 20.0],
            [0.0, 0.0, 30.0],
        ],
        None,
    )
    .unwrap();
    let e_curve =
        ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 5.0], None).unwrap();
    let seg = CubicSegment::try_new(
        xyz,
        EMode::Independent,
        0.0,
        Some(e_curve),
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .expect(
        "CubicSegment::try_new accepts Independent + xyz (no per-segment invariant prevents it)",
    );

    let err = split_segment_to_cap(&seg, 12.5).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::CannotSplitIndependent),
        "expected CannotSplitIndependent, got {err:?}"
    );
}
