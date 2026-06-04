use super::*;
use crate::ScalarNurbs;
use crate::eval::eval;

#[test]
fn try_new_accepts_monotone_knots() {
    let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap();
    assert_eq!(kv.len(), 5);
    assert_eq!(kv.as_slice(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
}

#[test]
fn try_new_rejects_non_monotone() {
    let result = KnotVector::<f64>::try_new(vec![0.0, 0.5, 0.3, 1.0]);
    assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
}

#[test]
fn try_new_rejects_too_short() {
    let result = KnotVector::<f64>::try_new(vec![0.0]);
    assert!(matches!(
        result,
        Err(ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn find_knot_span_returns_correct_span() {
    let knots = [0.0_f64, 0.0, 0.5, 1.0, 1.0];
    // degree 1, n = 3 cps. Span at u=0.25 is 1 (between knots[1]=0.0 and knots[2]=0.5).
    assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
    // u >= knots[n] returns n-1.
    assert_eq!(find_knot_span(&knots, 1, 3, 1.0), 2);
    // u <= knots[p] returns p.
    assert_eq!(find_knot_span(&knots, 1, 3, 0.0), 1);
}

#[test]
fn knot_vector_find_span_delegates() {
    let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap();
    assert_eq!(kv.find_span(0.25, 1, 3), 1);
}

#[test]
fn multiplicity_at_counts_repeated_knots() {
    let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 0.5, 1.0, 1.0]).unwrap();
    assert_eq!(kv.multiplicity_at(0.0), 2);
    assert_eq!(kv.multiplicity_at(0.5), 2);
    assert_eq!(kv.multiplicity_at(1.0), 2);
    assert_eq!(kv.multiplicity_at(0.25), 0);
}

#[test]
fn remove_knot_returns_zero_when_tolerance_not_met() {
    // A real C^0 corner at u=0.5: knot at multiplicity 2 (== degree), and
    // CPs chosen so removal would visibly displace.
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 5.0, 0.0, 1.0], // sharp jump at the corner
    )
    .unwrap();

    let (result, removed) = remove_knot(&curve, 0.5, 1, 1e-9);
    assert_eq!(removed, 0);
    // Curve unchanged.
    assert_eq!(result.knots(), curve.knots());
}

#[test]
fn remove_knot_undoes_insertion_within_tolerance() {
    let curve =
        ScalarNurbs::<f64>::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0])
            .unwrap();

    let inserted = insert_knot(&curve, 0.5, 1).unwrap();
    let (removed, count) = remove_knot(&inserted, 0.5, 1, 1e-10);

    assert_eq!(count, 1);
    assert_eq!(removed.knots(), curve.knots());
    for (a, b) in removed.control_points().iter().zip(curve.control_points()) {
        assert!((a - b).abs() < 1e-10, "cp mismatch: {a} vs {b}");
    }
}

#[test]
fn remove_knot_undoes_insertion_for_cubic_with_irregular_cps() {
    // Cubic curve, irregular non-symmetric, non-linear CPs so any indexing
    // bug yields a numerically distinct wrong answer. Inner loop runs more
    // than once for p=3. With 5 cps and degree 3 we need one interior knot
    // (n + p + 1 = 9 knots), placed away from the insertion point so the
    // removal target is a fresh single-multiplicity knot.
    let curve = ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 4.0, 9.0, 16.0],
    )
    .unwrap();

    let inserted = insert_knot(&curve, 0.4, 1).unwrap();
    let (recovered, count) = remove_knot(&inserted, 0.4, 1, 1e-10);

    assert_eq!(count, 1);
    assert_eq!(recovered.knots().len(), curve.knots().len());
    for (a, b) in recovered.knots().iter().zip(curve.knots()) {
        assert!((a - b).abs() < 1e-9, "knot mismatch: {a} vs {b}");
    }
    assert_eq!(
        recovered.control_points().len(),
        curve.control_points().len()
    );
    for (a, b) in recovered
        .control_points()
        .iter()
        .zip(curve.control_points())
    {
        assert!((a - b).abs() < 1e-9, "cp mismatch: {a} vs {b}");
    }
}

#[test]
fn remove_knot_two_round_trips_for_cubic_with_irregular_cps() {
    // Degree-4 curve with no interior knots, irregular non-symmetric CPs.
    // Insert at u=0.4 twice to lift multiplicity to 2, then attempt to
    // remove both. With p=4 and s=2, iteration t=1 of remove_knot exits its
    // inner loop with j == i (i.e. j < i + t strictly), exercising the
    // convergence-branch predicate. With the buggy `j + t < i` predicate
    // this routes to the else branch, reads outside the temp window, and
    // either panics or returns count=1 with displaced cps.
    let curve = ScalarNurbs::<f64>::try_new(
        4,
        vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 2.5, -1.0, 3.0, 0.5],
    )
    .unwrap();

    let inserted_once = insert_knot(&curve, 0.4, 1).unwrap();
    let inserted_twice = insert_knot(&inserted_once, 0.4, 1).unwrap();
    let (recovered, count) = remove_knot(&inserted_twice, 0.4, 2, 1e-10);

    assert_eq!(count, 2);
    // Knot vector should be byte-identical post round-trip.
    assert_eq!(recovered.knots(), curve.knots());
    assert_eq!(
        recovered.control_points().len(),
        curve.control_points().len()
    );
    for (a, b) in recovered
        .control_points()
        .iter()
        .zip(curve.control_points())
    {
        assert!((a - b).abs() < 1e-9, "cp mismatch: {a} vs {b}");
    }
}

#[test]
fn refined_to_full_multiplicity_raises_interior_knots() {
    // Cubic with one interior knot at 0.5 (multiplicity 1).
    let curve = ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
    )
    .unwrap();

    let refined = refined_to_full_multiplicity(&curve);

    // Interior knot 0.5 should now have multiplicity = degree = 3.
    assert_eq!(
        refined.knots(),
        &[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]
    );
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let before = eval(&curve.as_view(), u);
        let after = eval(&refined.as_view(), u);
        assert!(
            (before - after).abs() < 1e-10,
            "u={u}: before={before}, after={after}"
        );
    }
}

#[test]
fn insert_knot_multifold_at_existing_preserves_evaluation_for_failing_case() {
    // From the algebra_proptest shrunk failure: cubic with interior knot at 0.1
    // (multiplicity 1), inserted twice — Boehm A5.3 multi-fold + existing path
    // produced wrong control points pre-fix.
    use crate::eval::eval;
    let curve = ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.1, 0.55, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.181_828_016_839_598_23, 0.0, 0.0],
    )
    .unwrap();

    // r=1 path (path A: two single insertions).
    let path_a = insert_knot(&insert_knot(&curve, 0.1, 1).unwrap(), 0.1, 1).unwrap();

    // r=2 path (path B: one double insertion).
    let path_b = insert_knot(&curve, 0.1, 2).unwrap();

    // Knot vectors must match.
    assert_eq!(path_a.knots(), path_b.knots());

    // Control points must match (the regression).
    for (i, (a, b)) in path_a
        .control_points()
        .iter()
        .zip(path_b.control_points())
        .enumerate()
    {
        assert!(
            (a - b).abs() < 1e-12,
            "cp[{i}]: r=1+1 path = {a}, r=2 path = {b}"
        );
    }

    // And eval must match the baseline.
    let baseline = eval(&curve.as_view(), 0.1);
    assert!((eval(&path_a.as_view(), 0.1) - baseline).abs() < 1e-12);
    assert!((eval(&path_b.as_view(), 0.1) - baseline).abs() < 1e-12);
}

#[test]
fn insert_knot_at_existing_multiplicity_preserves_evaluation() {
    // Quadratic curve with interior knot at 0.5 (multiplicity 1).
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    // Insert one more at u=0.5: existing=1 + 1 = 2 == degree, allowed.
    let inserted = insert_knot(&curve, 0.5, 1).unwrap();
    assert_eq!(inserted.knots(), &[0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]);

    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let before = eval(&curve.as_view(), u);
        let after = eval(&inserted.as_view(), u);
        assert!(
            (before - after).abs() < 1e-12,
            "u={u}: before={before}, after={after}"
        );
    }
}

#[test]
fn insert_knot_rejects_multiplicity_exceeded() {
    // Quadratic curve with interior knot at 0.5 (multiplicity 1, so we can add 1 more).
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    // Insert 2 more at u=0.5: existing=1 + 2 = 3 > degree 2.
    let result = insert_knot(&curve, 0.5, 2);
    assert!(matches!(
        result,
        Err(KnotError::MultiplicityExceeded {
            existing: 1,
            requested: 2,
            max: 2
        })
    ));
}

#[test]
fn insert_knot_rejects_clamped_boundary() {
    let curve = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();

    assert!(matches!(
        insert_knot(&curve, 0.0, 1),
        Err(KnotError::BoundaryInsertion)
    ));
    assert!(matches!(
        insert_knot(&curve, 1.0, 1),
        Err(KnotError::BoundaryInsertion)
    ));
}

#[test]
fn insert_knot_into_simple_curve_preserves_evaluation() {
    // Linear curve from 0 to 2 over [0, 1]. Insert knot at u=0.5.
    let curve = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 2.0]).unwrap();

    let inserted = insert_knot(&curve, 0.5, 1).unwrap();

    assert_eq!(inserted.knots(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
    assert_eq!(inserted.control_points().len(), 3); // was 2, now 3
    // Geometric invariance: eval at sample points unchanged.
    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let before = eval(&curve.as_view(), u);
        let after = eval(&inserted.as_view(), u);
        assert!(
            (before - after).abs() < 1e-12,
            "u={u}: before={before}, after={after}"
        );
    }
}
