//! Knot vector type and host-only knot operations (insertion, removal, span queries).
//! See `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` §4–§6.
//!
//! Module-level host-only gating is applied at the `pub mod knot;` site in
//! `lib.rs`; an inner `#![cfg(feature = "host")]` here would be redundant.

use crate::{ConstructError, Float, KnotError, ScalarNurbs};

/// Owned knot vector. Validates `non-decreasing` invariant on construction.
/// Clamping and length-vs-degree invariants are enforced by `ScalarNurbs::try_new`
/// where applicable; this type holds knots independent of any single curve.
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector<T: Float> {
    knots: Vec<T>,
}

impl<T: Float> KnotVector<T> {
    pub fn try_new(knots: Vec<T>) -> Result<Self, ConstructError> {
        if knots.len() < 2 {
            return Err(ConstructError::KnotCountMismatch {
                expected: 2,
                got: knots.len(),
            });
        }
        for window in knots.windows(2) {
            if window[1] < window[0] {
                return Err(ConstructError::KnotsNotMonotone);
            }
        }
        Ok(Self { knots })
    }

    pub fn as_slice(&self) -> &[T] {
        &self.knots
    }

    pub fn len(&self) -> usize {
        self.knots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.knots.is_empty()
    }

    /// Consume the wrapper, returning the underlying `Vec<T>`.
    pub fn into_inner(self) -> Vec<T> {
        self.knots
    }
}

/// Find the knot span `k` such that `knots[k] <= u < knots[k+1]`, with the
/// clamped-end special case mapping `u >= knots[n]` to the last span.
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A2.1.
///
/// Free function form for callers that have raw `&[T]`. See also
/// `KnotVector::find_span` for owned-type callers.
pub fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] {
        return n - 1;
    }
    if u <= knots[p] {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

impl<T: Float> KnotVector<T> {
    /// Find the knot span containing `u` for a curve of given degree `p` with
    /// `n` control points. Delegates to the free function `find_knot_span`.
    pub fn find_span(&self, u: T, p: usize, n: usize) -> usize {
        find_knot_span(&self.knots, p, n, u)
    }

    /// Count consecutive equal knots at value `u`. Returns 0 if `u` is not present.
    pub fn multiplicity_at(&self, u: T) -> usize {
        self.knots.iter().filter(|k| **k == u).count()
    }
}

/// Insert ū into a curve with the given multiplicity (number of repeated insertions).
///
/// Boehm's algorithm (Piegl & Tiller §5.2, Algorithm A5.1 / A5.3). The inserted
/// knot does not change the curve geometrically — eval is invariant. The
/// number of control points grows by `multiplicity`.
///
/// Errors:
/// - `BoundaryInsertion` if ū equals a clamped endpoint.
/// - `MultiplicityExceeded` if `existing + multiplicity > degree`.
/// - `OutOfRange` if ū is outside the knot vector range.
pub fn insert_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    multiplicity: usize,
) -> Result<ScalarNurbs<T>, KnotError> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let weights = curve.weights();

    // Validate u is in (knots[0], knots[last]) — strictly interior.
    if u <= knots[0] || u >= knots[knots.len() - 1] {
        return Err(KnotError::BoundaryInsertion);
    }
    if u < knots[0] || u > knots[knots.len() - 1] {
        return Err(KnotError::OutOfRange);
    }

    // Existing multiplicity at u.
    let existing = curve.knots().iter().filter(|k| **k == u).count();
    if existing + multiplicity > p {
        return Err(KnotError::MultiplicityExceeded {
            existing: existing as u8,
            requested: multiplicity as u8,
            max: p as u8,
        });
    }

    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    // Build new knot vector: insert `multiplicity` copies of u at position k+1.
    let mut new_knots = Vec::with_capacity(knots.len() + multiplicity);
    new_knots.extend_from_slice(&knots[..=k]);
    for _ in 0..multiplicity {
        new_knots.push(u);
    }
    new_knots.extend_from_slice(&knots[k + 1..]);

    // Apply A5.3 fused multi-insertion to control points.
    let new_cps = if let Some(w) = weights {
        // Homogeneous lift: (cp * w, w), insert, project.
        let homo: Vec<(T, T)> = cps.iter().zip(w.iter()).map(|(c, w)| (*c * *w, *w)).collect();
        let new_homo = boehm_insert_homogeneous(&homo, knots, p, k, u, existing, multiplicity);
        new_homo.into_iter().map(|(num, w)| num / w).collect::<Vec<T>>()
    } else {
        boehm_insert_unweighted(cps, knots, p, k, u, existing, multiplicity)
    };

    let new_weights = if let Some(w) = weights {
        Some(boehm_insert_unweighted(w, knots, p, k, u, existing, multiplicity))
    } else {
        None
    };

    ScalarNurbs::try_new(curve.degree(), new_knots, new_cps, new_weights)
        .map_err(|_| KnotError::Invalid)
}

/// Single-insertion fused as r-fold (P&T A5.3) for unweighted control points.
fn boehm_insert_unweighted<T: Float>(
    cps: &[T],
    knots: &[T],
    p: usize,
    k: usize,
    u: T,
    existing: usize,
    r: usize,  // number of insertions
) -> Vec<T> {
    let n = cps.len();
    let new_n = n + r;
    let mut new_cps = vec![T::ZERO; new_n];

    // Unaffected CPs pass through.
    for i in 0..=k - p {
        new_cps[i] = cps[i];
    }
    for i in (k - existing)..n {
        new_cps[i + r] = cps[i];
    }

    // Working buffer for the r-fold blend.
    let mut work: Vec<T> = (0..=p - existing).map(|i| cps[k - p + i]).collect();

    // r-fold insertion (A5.3).
    for j in 1..=r {
        let l = k - p + j;
        for i in 0..=p - j - existing {
            let denom = knots[l + i + p] - knots[l + i];
            let alpha = if denom > T::ZERO {
                (u - knots[l + i]) / denom
            } else {
                T::ZERO
            };
            work[i] = (T::ONE - alpha) * work[i] + alpha * work[i + 1];
        }
        new_cps[l] = work[0];
        new_cps[k + r - j - existing] = work[p - j - existing];
    }

    // Remaining middle CPs.
    for i in (k - p + r)..(k - existing) {
        new_cps[i] = work[i - (k - p + r)];
    }

    new_cps
}

/// Raise every interior knot's multiplicity to `degree`, producing a curve
/// whose representation decomposes cleanly into Bézier pieces. Geometric
/// invariance preserved.
pub fn refined_to_full_multiplicity<T: Float>(curve: &ScalarNurbs<T>) -> ScalarNurbs<T> {
    let p = curve.degree() as usize;
    let mut current = curve.clone();

    // Collect unique interior knot values.
    let knots_snapshot: Vec<T> = current.knots().to_vec();
    let mut interior: Vec<T> = Vec::new();
    let mut i = p + 1;
    while i < knots_snapshot.len() - p - 1 {
        let u = knots_snapshot[i];
        if !interior.contains(&u) {
            interior.push(u);
        }
        i += 1;
    }

    for u in interior {
        let existing = current.knots().iter().filter(|k| **k == u).count();
        if existing < p {
            current = insert_knot(&current, u, p - existing)
                .expect("refined_to_full_multiplicity: insertion should be valid");
        }
    }

    current
}

/// Tiller knot removal (P&T §5.4, Algorithm A5.8). Removes knot ū up to
/// `count` times if removal preserves the curve within chord-error `tol` in
/// control-point space. Returns the new curve and the number of removals
/// actually performed (may be less than `count`).
///
/// For unweighted curves only in v1; weighted (rational) curves return the
/// input unchanged with count 0 (no error — caller can detect via the count).
pub fn remove_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    count: usize,
    tol: T,
) -> (ScalarNurbs<T>, usize) {
    if curve.weights().is_some() {
        // v1: rational removal not supported; return input unchanged.
        return (curve.clone(), 0);
    }
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();

    // Find span and existing multiplicity.
    let s = knots.iter().filter(|k| **k == u).count();
    if s == 0 {
        return (curve.clone(), 0);  // u not in knot vector
    }
    let r = find_knot_span(knots, p, n, u);

    let mut new_cps = cps.to_vec();
    let mut new_knots = knots.to_vec();
    let mut removed = 0;
    let mut current_s = s;

    while removed < count && current_s > 0 {
        // Try one removal (A5.8).
        let first = r - p;
        let last = r - current_s;
        let mut temp = vec![T::ZERO; (last - first + 2).max(2)];

        temp[0] = new_cps[first - 1];
        temp[last - first + 1] = new_cps[last + 1];

        let mut i = first;
        let mut j = last;
        let mut ii = 1;
        let mut jj = last - first;
        let mut converged = true;

        // `j - i > 0` on usize underflows when boundaries cross; use `j > i`.
        while j > i {
            let alpha_i = (u - new_knots[i]) / (new_knots[i + p + 1] - new_knots[i]);
            let alpha_j = (u - new_knots[j]) / (new_knots[j + p + 1] - new_knots[j]);

            temp[ii] = (new_cps[i] - (T::ONE - alpha_i) * temp[ii - 1]) / alpha_i;
            temp[jj] = (new_cps[j] - alpha_j * temp[jj + 1]) / (T::ONE - alpha_j);

            i += 1; ii += 1; j -= 1; jj -= 1;
        }

        // Convergence check: chord-error tolerance.
        // After loop, i may exceed j by 1; treat that as "boundaries met".
        if i >= j {
            let err = (temp[ii - 1] - temp[jj + 1]).abs();
            if err > tol {
                converged = false;
            }
        }

        if !converged {
            break;
        }

        // Apply: shift CPs down, drop one knot.
        let mut i2 = first;
        let mut j2 = last;
        while j2 > i2 {
            new_cps[i2] = temp[i2 - first + 1];
            new_cps[j2] = temp[j2 - first + 1];
            i2 += 1; j2 -= 1;
        }
        // Remove one cp (the duplicate at center) and one knot.
        new_cps.remove((first + last) / 2 + 1);
        new_knots.remove(r);

        removed += 1;
        current_s -= 1;
    }

    let new_curve = ScalarNurbs::try_new(curve.degree(), new_knots, new_cps, None)
        .expect("remove_knot: result invariants should hold");
    (new_curve, removed)
}

/// Homogeneous variant: blends (num, w) tuples.
fn boehm_insert_homogeneous<T: Float>(
    homo: &[(T, T)],
    knots: &[T],
    p: usize,
    k: usize,
    u: T,
    existing: usize,
    r: usize,
) -> Vec<(T, T)> {
    let nums: Vec<T> = homo.iter().map(|(n, _)| *n).collect();
    let ws: Vec<T> = homo.iter().map(|(_, w)| *w).collect();
    let new_nums = boehm_insert_unweighted(&nums, knots, p, k, u, existing, r);
    let new_ws = boehm_insert_unweighted(&ws, knots, p, k, u, existing, r);
    new_nums.into_iter().zip(new_ws).collect()
}

#[cfg(test)]
mod tests {
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
        assert!(matches!(result, Err(ConstructError::KnotCountMismatch { .. })));
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
    fn remove_knot_undoes_insertion_within_tolerance() {
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0], None,
        ).unwrap();

        let inserted = insert_knot(&curve, 0.5, 1).unwrap();
        let (removed, count) = remove_knot(&inserted, 0.5, 1, 1e-10);

        assert_eq!(count, 1);
        assert_eq!(removed.knots(), curve.knots());
        for (a, b) in removed.control_points().iter().zip(curve.control_points()) {
            assert!((a - b).abs() < 1e-10, "cp mismatch: {a} vs {b}");
        }
    }

    #[test]
    fn refined_to_full_multiplicity_raises_interior_knots() {
        // Cubic with one interior knot at 0.5 (multiplicity 1).
        let curve = ScalarNurbs::<f64>::try_new(
            3, vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0, 4.0], None,
        ).unwrap();

        let refined = refined_to_full_multiplicity(&curve);

        // Interior knot 0.5 should now have multiplicity = degree = 3.
        assert_eq!(refined.knots(), &[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&refined.as_view(), u);
            assert!((before - after).abs() < 1e-10, "u={u}: before={before}, after={after}");
        }
    }

    #[test]
    fn insert_knot_at_existing_multiplicity_preserves_evaluation() {
        // Quadratic curve with interior knot at 0.5 (multiplicity 1).
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0, 3.0], None,
        ).unwrap();

        // Insert one more at u=0.5: existing=1 + 1 = 2 == degree, allowed.
        let inserted = insert_knot(&curve, 0.5, 1).unwrap();
        assert_eq!(inserted.knots(), &[0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]);

        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&inserted.as_view(), u);
            assert!((before - after).abs() < 1e-12, "u={u}: before={before}, after={after}");
        }
    }

    #[test]
    fn insert_knot_rejects_multiplicity_exceeded() {
        // Quadratic curve with interior knot at 0.5 (multiplicity 1, so we can add 1 more).
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0, 3.0], None,
        ).unwrap();

        // Insert 2 more at u=0.5: existing=1 + 2 = 3 > degree 2.
        let result = insert_knot(&curve, 0.5, 2);
        assert!(matches!(
            result,
            Err(KnotError::MultiplicityExceeded { existing: 1, requested: 2, max: 2 })
        ));
    }

    #[test]
    fn insert_knot_rejects_clamped_boundary() {
        let curve = ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();

        assert!(matches!(insert_knot(&curve, 0.0, 1), Err(KnotError::BoundaryInsertion)));
        assert!(matches!(insert_knot(&curve, 1.0, 1), Err(KnotError::BoundaryInsertion)));
    }

    #[test]
    fn insert_knot_into_simple_curve_preserves_evaluation() {
        // Linear curve from 0 to 2 over [0, 1]. Insert knot at u=0.5.
        let curve = ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 2.0], None,
        ).unwrap();

        let inserted = insert_knot(&curve, 0.5, 1).unwrap();

        assert_eq!(inserted.knots(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
        assert_eq!(inserted.control_points().len(), 3);  // was 2, now 3
        // Geometric invariance: eval at sample points unchanged.
        for u in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&inserted.as_view(), u);
            assert!((before - after).abs() < 1e-12, "u={u}: before={before}, after={after}");
        }
    }
}
