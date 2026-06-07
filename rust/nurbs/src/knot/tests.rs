use super::*;
use crate::ScalarNurbs;
use crate::eval::eval;

fn reference_refined_to_full_multiplicity(curve: &ScalarNurbs<f64>) -> ScalarNurbs<f64> {
    let p = curve.degree() as usize;
    let mut current = curve.clone();
    let knots_snapshot: Vec<f64> = current.knots().to_vec();
    let mut seen: Vec<f64> = Vec::new();
    let mut i = p + 1;
    while i < knots_snapshot.len() - p - 1 {
        let u = knots_snapshot[i];
        if !seen.contains(&u) {
            seen.push(u);
        }
        i += 1;
    }
    for u in seen {
        let existing = current.knots().iter().filter(|k| **k == u).count();
        if existing < p {
            current = insert_knot(&current, u, p - existing).unwrap();
        }
    }
    current
}

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
    assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
    assert_eq!(find_knot_span(&knots, 1, 3, 1.0), 2);
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
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 5.0, 0.0, 1.0],
    )
    .unwrap();

    let (result, removed) = remove_knot(&curve, 0.5, 1, 1e-9);
    assert_eq!(removed, 0);
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
    let curve = ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
    )
    .unwrap();

    let refined = refined_to_full_multiplicity(&curve);

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
fn refined_to_full_multiplicity_matches_reference_on_mixed_multiplicity_curve() {
    let p: u8 = 3;
    let n_interior = 50usize;

    let mut knots: Vec<f64> = vec![0.0; p as usize + 1];
    for k in 0..n_interior {
        let u = (k + 1) as f64 / (n_interior + 1) as f64;
        let mult = 1 + (k % (p as usize));
        for _ in 0..mult {
            knots.push(u);
        }
    }
    knots.extend(vec![1.0; p as usize + 1]);

    let n_cps = knots.len() - p as usize - 1;
    let cps: Vec<f64> = (0..n_cps)
        .map(|i| {
            let t = i as f64 / (n_cps - 1) as f64;
            t * t - 0.5 * t + 0.1 * (i as f64 * 1.3).sin()
        })
        .collect();

    let curve = ScalarNurbs::<f64>::try_new(p, knots, cps).unwrap();

    let fast = refined_to_full_multiplicity(&curve);
    let reference = reference_refined_to_full_multiplicity(&curve);

    assert_eq!(fast.knots(), reference.knots(), "knot vectors differ");
    assert_eq!(
        fast.control_points().len(),
        reference.control_points().len(),
        "control point count differs"
    );

    for (idx, (a, b)) in fast
        .control_points()
        .iter()
        .zip(reference.control_points())
        .enumerate()
    {
        let rel_tol = 1e-12 * (a.abs().max(b.abs()) + 1.0);
        assert!(
            (a - b).abs() <= rel_tol,
            "cp[{idx}]: fast={a:.15e} reference={b:.15e} diff={:.3e}",
            (a - b).abs()
        );
    }

    let sample_params: Vec<f64> = (0..=20).map(|i| i as f64 / 20.0).collect();
    for u in sample_params {
        let v_fast = eval(&fast.as_view(), u);
        let v_ref = eval(&reference.as_view(), u);
        assert!(
            (v_fast - v_ref).abs() < 1e-12,
            "eval mismatch at u={u}: fast={v_fast:.15e} ref={v_ref:.15e}"
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

    let path_a = insert_knot(&insert_knot(&curve, 0.1, 1).unwrap(), 0.1, 1).unwrap();
    let path_b = insert_knot(&curve, 0.1, 2).unwrap();

    assert_eq!(path_a.knots(), path_b.knots());

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

    let baseline = eval(&curve.as_view(), 0.1);
    assert!((eval(&path_a.as_view(), 0.1) - baseline).abs() < 1e-12);
    assert!((eval(&path_b.as_view(), 0.1) - baseline).abs() < 1e-12);
}

#[test]
fn insert_knot_at_existing_multiplicity_preserves_evaluation() {
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

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
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

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
    let curve = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 2.0]).unwrap();

    let inserted = insert_knot(&curve, 0.5, 1).unwrap();

    assert_eq!(inserted.knots(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
    assert_eq!(inserted.control_points().len(), 3);
    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let before = eval(&curve.as_view(), u);
        let after = eval(&inserted.as_view(), u);
        assert!(
            (before - after).abs() < 1e-12,
            "u={u}: before={before}, after={after}"
        );
    }
}
