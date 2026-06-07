use crate::{ConstructError, Float, KnotError, ScalarNurbs};

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

    pub fn into_inner(self) -> Vec<T> {
        self.knots
    }

    pub fn find_span(&self, u: T, p: usize, n: usize) -> usize {
        find_knot_span(&self.knots, p, n, u)
    }

    pub fn multiplicity_at(&self, u: T) -> usize {
        self.knots.iter().filter(|k| **k == u).count()
    }
}

// Piegl & Tiller Algorithm A2.1.
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
    let mut mid = usize::midpoint(low, high);
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = usize::midpoint(low, high);
    }
    mid
}

pub fn insert_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    multiplicity: usize,
) -> Result<ScalarNurbs<T>, KnotError> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();

    if u <= knots[0] || u >= knots[knots.len() - 1] {
        return Err(KnotError::BoundaryInsertion);
    }
    if u < knots[0] || u > knots[knots.len() - 1] {
        return Err(KnotError::OutOfRange);
    }

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

    let mut new_knots = Vec::with_capacity(knots.len() + multiplicity);
    new_knots.extend_from_slice(&knots[..=k]);
    for _ in 0..multiplicity {
        new_knots.push(u);
    }
    new_knots.extend_from_slice(&knots[k + 1..]);

    let new_cps = boehm_insert_unweighted(cps, knots, p, k, u, existing, multiplicity);

    ScalarNurbs::try_new(curve.degree(), new_knots, new_cps).map_err(|_| KnotError::Invalid)
}

fn boehm_insert_unweighted<T: Float>(
    cps: &[T],
    knots: &[T],
    p: usize,
    _k: usize,
    u: T,
    existing: usize,
    r: usize,
) -> Vec<T> {
    debug_assert!(
        existing + r <= p,
        "Boehm: existing + r must not exceed degree"
    );

    let mut current_cps: Vec<T> = cps.to_vec();
    let mut current_knots: Vec<T> = knots.to_vec();
    let mut current_existing = existing;

    for _ in 0..r {
        let n = current_cps.len();
        let k = find_knot_span(&current_knots, p, n, u);
        let new_cps =
            boehm_insert_unweighted_single(&current_cps, &current_knots, p, k, u, current_existing);
        let mut new_knots = Vec::with_capacity(current_knots.len() + 1);
        new_knots.extend_from_slice(&current_knots[..=k]);
        new_knots.push(u);
        new_knots.extend_from_slice(&current_knots[k + 1..]);

        current_cps = new_cps;
        current_knots = new_knots;
        current_existing += 1;
    }
    current_cps
}

fn boehm_insert_unweighted_single<T: Float>(
    cps: &[T],
    knots: &[T],
    p: usize,
    k: usize,
    u: T,
    existing: usize,
) -> Vec<T> {
    let n = cps.len();
    let new_n = n + 1;
    let mut new_cps = vec![T::ZERO; new_n];

    let lead = k - p + 1;
    new_cps[..lead].copy_from_slice(&cps[..lead]);
    let tail_start = k - existing;
    new_cps[(tail_start + 1)..=n].copy_from_slice(&cps[tail_start..n]);

    let l = k - p + 1;
    for i in 0..=p - 1 - existing {
        let denom = knots[l + i + p] - knots[l + i];
        let alpha = if denom > T::ZERO {
            (u - knots[l + i]) / denom
        } else {
            T::ZERO
        };
        new_cps[l + i] = (T::ONE - alpha) * cps[k - p + i] + alpha * cps[k - p + i + 1];
    }

    new_cps
}

pub fn refined_to_full_multiplicity<T: Float>(curve: &ScalarNurbs<T>) -> ScalarNurbs<T> {
    let p = curve.degree() as usize;
    let mut current = curve.clone();

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

// Tiller knot removal: Piegl & Tiller Algorithm A5.8.
pub fn remove_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    count: usize,
    tol: T,
) -> (ScalarNurbs<T>, usize) {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();

    let s = knots.iter().filter(|k| **k == u).count();
    if s == 0 {
        return (curve.clone(), 0);
    }
    let r = find_knot_span(knots, p, n, u);

    let num = count.min(s);

    let mut pw = cps.to_vec();
    let knots_ref = knots;

    let ord = p + 1;
    let fout = (2 * r).saturating_sub(s + p) / 2;
    let mut first = r - p;
    let mut last = r - s;

    let mut temp: Vec<T> = vec![T::ZERO; 2 * p + 2 * num + 2];

    let mut t: usize = 0;
    while t < num {
        let off = first - 1;
        temp[0] = pw[off];
        temp[last + 1 - off] = pw[last + 1];

        let mut i = first;
        let mut j = last;
        let mut ii: usize = 1;
        let mut jj: usize = last - off;

        while j > i + t {
            let alfi = (u - knots_ref[i]) / (knots_ref[i + ord + t] - knots_ref[i]);
            let alfj = (u - knots_ref[j - t]) / (knots_ref[j + ord] - knots_ref[j - t]);

            temp[ii] = (pw[i] - (T::ONE - alfi) * temp[ii - 1]) / alfi;
            temp[jj] = (pw[j] - alfj * temp[jj + 1]) / (T::ONE - alfj);

            i += 1;
            ii += 1;
            j -= 1;
            jj -= 1;
        }

        let remflag = if j < i + t {
            (temp[ii - 1] - temp[jj + 1]).abs() <= tol
        } else {
            let alfi = (u - knots_ref[i]) / (knots_ref[i + ord + t] - knots_ref[i]);
            let blended = alfi * temp[ii + t + 1] + (T::ONE - alfi) * temp[ii - 1];
            (pw[i] - blended).abs() <= tol
        };

        if !remflag {
            break;
        }

        let mut i2 = first;
        let mut j2 = last;
        while j2 > i2 + t {
            pw[i2] = temp[i2 - off];
            pw[j2] = temp[j2 - off];
            i2 += 1;
            j2 -= 1;
        }

        first -= 1;
        last += 1;
        t += 1;
    }

    if t == 0 {
        return (curve.clone(), 0);
    }

    let mut new_knots = Vec::with_capacity(knots_ref.len() - t);
    new_knots.extend_from_slice(&knots_ref[..=(r - t)]);
    new_knots.extend_from_slice(&knots_ref[(r + 1)..]);

    let (drop_lo, drop_hi) = {
        let mut j_idx = fout;
        let mut i_idx = fout;
        for k in 1..t {
            if k % 2 == 1 {
                i_idx += 1;
            } else {
                j_idx -= 1;
            }
        }
        (j_idx, i_idx)
    };

    let mut new_cps = Vec::with_capacity(pw.len() - t);
    new_cps.extend_from_slice(&pw[..drop_lo]);
    new_cps.extend_from_slice(&pw[(drop_hi + 1)..]);

    debug_assert_eq!(new_cps.len(), pw.len() - t);
    debug_assert_eq!(new_knots.len(), knots_ref.len() - t);

    let new_curve = ScalarNurbs::try_new(curve.degree(), new_knots, new_cps)
        .expect("remove_knot: result invariants should hold");
    (new_curve, t)
}

#[cfg(test)]
mod tests;
