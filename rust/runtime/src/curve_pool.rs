//! `CurvePool` — static slab of NURBS curve data referenced by `CurveHandle`.
//! Spec §3.1. Step 5: no-overwrite-after-load. Step 6+ adds refcount / epoch.

use crate::segment::CurveHandle;

/// Slab capacity. Spec §7 open question 1 — revisited at Step 7 MVP.
pub const CURVE_POOL_N: usize = 16;

/// Per-curve storage capacity. Sized for degree-3 NURBS with up to 8 control
/// points in 3D — typical Step 5 fixture range. Larger curves are rejected.
pub const MAX_CONTROL_POINTS: usize = 8;
pub const MAX_DIM: usize = 3;
pub const MAX_KNOT_VECTOR_LEN: usize = MAX_CONTROL_POINTS + 4; // n_cp_max + degree_max + 1
pub const MAX_DEGREE: u8 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurvePoolError {
    OutOfBounds,
    SlotAlreadyLoaded,
    DegreeTooHigh,
    InvalidLengths,
    NonFiniteData,
    /// Catch-all for non-monotone knots, non-positive weights, too few CPs, etc.
    InvalidCurve,
}

#[derive(Debug, Clone, Copy)]
struct LoadedCurve {
    control_points: [[f32; MAX_DIM]; MAX_CONTROL_POINTS],
    weights: [f32; MAX_CONTROL_POINTS],
    knots: [f32; MAX_KNOT_VECTOR_LEN],
    n_cp: u8,
    n_knots: u8,
    degree: u8,
}

#[derive(Debug)]
pub struct CurvePool {
    slots: [Option<LoadedCurve>; CURVE_POOL_N],
}

impl Default for CurvePool {
    fn default() -> Self {
        Self::new()
    }
}

impl CurvePool {
    pub const fn new() -> Self {
        Self {
            slots: [const { None }; CURVE_POOL_N],
        }
    }

    /// Load curve data into a slot. Step-5 policy: no-overwrite-after-load.
    ///
    /// `control_points_flat` must be length `n_cp * MAX_DIM` (3 floats per CP).
    /// `knots` must be length `n_cp + degree + 1`.
    /// `weights` must be length `n_cp`.
    pub fn load(
        &mut self,
        handle: CurveHandle,
        control_points_flat: &[f32],
        knots: &[f32],
        weights: &[f32],
        degree: u8,
    ) -> Result<(), CurvePoolError> {
        let idx = handle.0 as usize;

        // Bounds check first.
        if idx >= CURVE_POOL_N {
            return Err(CurvePoolError::OutOfBounds);
        }

        // No-overwrite policy (Step 5). Step 6+ adds refcount/epoch.
        // Use get() rather than indexing to satisfy clippy::indexing_slicing.
        if self.slots.get(idx).is_some_and(Option::is_some) {
            return Err(CurvePoolError::SlotAlreadyLoaded);
        }

        if degree > MAX_DEGREE {
            return Err(CurvePoolError::DegreeTooHigh);
        }

        let n_cp = weights.len();
        if n_cp == 0 || n_cp > MAX_CONTROL_POINTS {
            return Err(CurvePoolError::InvalidLengths);
        }
        if control_points_flat.len() != n_cp * MAX_DIM {
            return Err(CurvePoolError::InvalidLengths);
        }
        let expected_knots = n_cp + degree as usize + 1;
        if knots.len() > MAX_KNOT_VECTOR_LEN || knots.len() != expected_knots {
            return Err(CurvePoolError::InvalidLengths);
        }

        // Finite check — NaN/Inf in any field is a hard error.
        if !control_points_flat
            .iter()
            .chain(knots.iter())
            .chain(weights.iter())
            .all(|x| x.is_finite())
        {
            return Err(CurvePoolError::NonFiniteData);
        }

        // Mirror `nurbs::validate()`'s preconditions at load time so producer-
        // side rejection is definitive — the ISR must never see an invalid view.
        // Use windows() / iter() to avoid clippy::indexing_slicing.
        for w in knots.windows(2) {
            if w.first().copied().unwrap_or(0.0) > w.last().copied().unwrap_or(0.0) {
                return Err(CurvePoolError::InvalidCurve); // non-monotone
            }
        }

        let first_knot = knots.first().copied().unwrap_or(0.0);
        let last_knot = knots.last().copied().unwrap_or(0.0);
        if !(last_knot > first_knot) {
            return Err(CurvePoolError::InvalidCurve); // degenerate / zero-length range
        }

        if !weights.iter().all(|&w| w > 0.0) {
            return Err(CurvePoolError::InvalidCurve); // non-positive weight
        }

        // n_cp ≥ p+1 required for de Boor's algorithm to have any basis function.
        let p = degree as usize;
        if n_cp < p + 1 {
            return Err(CurvePoolError::InvalidCurve); // too few CPs for degree
        }

        // Clamped at start: knots[0..=p] all equal to first_knot.
        // Exact equality is intentional and matches `nurbs::validate()` — clamping
        // is a structural property of the knot vector, not a numerical tolerance.
        #[allow(clippy::float_cmp)]
        let start_clamped = knots.iter().take(p + 1).all(|&k| k == first_knot);
        if !start_clamped {
            return Err(CurvePoolError::InvalidCurve); // start not clamped
        }
        // Clamped at end: last p+1 knots all equal to last_knot.
        #[allow(clippy::float_cmp)]
        let end_clamped = knots.iter().rev().take(p + 1).all(|&k| k == last_knot);
        if !end_clamped {
            return Err(CurvePoolError::InvalidCurve); // end not clamped
        }

        // Copy data into fixed-size arrays.  All bounds are proven above:
        //   - n_cp  ≤ MAX_CONTROL_POINTS
        //   - n_cp * MAX_DIM == control_points_flat.len()  (verified above)
        //   - knots.len() ≤ MAX_KNOT_VECTOR_LEN            (verified above)
        // Use iterators + enumerate to avoid clippy::indexing_slicing.
        let mut cps = [[0.0f32; MAX_DIM]; MAX_CONTROL_POINTS];
        control_points_flat
            .chunks_exact(MAX_DIM)
            .take(n_cp)
            .zip(cps.iter_mut())
            .for_each(|(src, dst)| {
                dst.iter_mut().zip(src.iter()).for_each(|(d, s)| *d = *s);
            });

        let mut wts = [0.0f32; MAX_CONTROL_POINTS];
        weights
            .iter()
            .zip(wts.iter_mut())
            .for_each(|(src, dst)| *dst = *src);

        let mut knots_buf = [0.0f32; MAX_KNOT_VECTOR_LEN];
        knots
            .iter()
            .zip(knots_buf.iter_mut())
            .for_each(|(src, dst)| *dst = *src);

        // All checks passed. Write the slot.  idx < CURVE_POOL_N == slots.len()
        // was verified at the top, so get_mut() will always yield Some here.
        if let Some(slot) = self.slots.get_mut(idx) {
            *slot = Some(LoadedCurve {
                control_points: cps,
                weights: wts,
                knots: knots_buf,
                n_cp: n_cp as u8,
                n_knots: knots.len() as u8,
                degree,
            });
        }

        Ok(())
    }

    /// Resolve a handle to a curve view suitable for `nurbs::vector_eval`.
    ///
    /// Returns `None` if the handle is out of bounds or the slot is unloaded.
    pub fn resolve(&self, handle: CurveHandle) -> Option<CurveView<'_>> {
        let idx = handle.0 as usize;
        let slot = self.slots.get(idx)?.as_ref()?;
        // n_cp and n_knots were bounds-checked on load; .get() returns Some here.
        // The ?-chain converts silently-None to None for the outer Option.
        let n_cp = slot.n_cp as usize;
        let n_knots = slot.n_knots as usize;
        let control_points = slot.control_points.get(..n_cp)?;
        let weights = slot.weights.get(..n_cp)?;
        let knots = slot.knots.get(..n_knots)?;
        Some(CurveView {
            control_points,
            weights,
            knots,
            degree: slot.degree,
        })
    }
}

/// Borrowed view of a loaded curve. Adapter for `nurbs::eval` consumed in Engine.
#[derive(Debug)]
pub struct CurveView<'a> {
    pub control_points: &'a [[f32; MAX_DIM]],
    pub weights: &'a [f32],
    pub knots: &'a [f32],
    pub degree: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    // Returns the (cps, knots, weights) backing arrays. The caller slices
    // cps[..n_cp*MAX_DIM], knots[..n_cp+4], weights[..n_cp]. Knot vector is
    // a clamped degree-3 open uniform vector: p+1 zeros at the start, p+1 ones
    // at the end, and n_cp - p - 1 evenly-spaced interior knots.
    // Requires n_cp >= 4 (== p + 1 for degree 3).
    #[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
    fn dummy_curve_data(n_cp: usize) -> ([f32; 32], [f32; 32], [f32; 32]) {
        let mut cps = [0.0f32; 32];
        let mut knots = [0.0f32; 32];
        let mut weights = [0.0f32; 32];
        for i in 0..n_cp {
            cps[i * 3] = i as f32;
            cps[i * 3 + 1] = 0.0;
            cps[i * 3 + 2] = 0.0;
            weights[i] = 1.0;
        }
        // Clamped degree-3 knot vector of length n_cp + 4 (== n_cp + degree + 1).
        let knot_len = n_cp + 4;
        for i in 0..4 {
            knots[i] = 0.0;
        }
        for i in (knot_len - 4)..knot_len {
            knots[i] = 1.0;
        }
        let interior = knot_len.saturating_sub(8);
        for k in 0..interior {
            knots[4 + k] = (k + 1) as f32 / (interior + 1) as f32;
        }
        (cps, knots, weights)
    }

    #[test]
    fn fresh_pool_handles_unloaded() {
        let pool = CurvePool::new();
        assert!(pool.resolve(CurveHandle(0)).is_none());
        assert!(pool.resolve(CurveHandle(15)).is_none());
    }

    #[test]
    fn out_of_bounds_handle_returns_none() {
        let pool = CurvePool::new();
        assert!(pool.resolve(CurveHandle(16)).is_none());
        assert!(pool.resolve(CurveHandle(u16::MAX)).is_none());
    }

    #[test]
    fn load_then_resolve_returns_curve() {
        let mut pool = CurvePool::new();
        let (cps, knots, weights) = dummy_curve_data(4);
        let result = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert!(result.is_ok());
        assert!(pool.resolve(CurveHandle(0)).is_some());
    }

    #[test]
    fn load_twice_into_same_slot_is_rejected() {
        let mut pool = CurvePool::new();
        let (cps, knots, weights) = dummy_curve_data(4);
        let first = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert!(first.is_ok());
        let second = pool.load(CurveHandle(0), &cps[..12], &knots[..8], &weights[..4], 3);
        assert_eq!(second, Err(CurvePoolError::SlotAlreadyLoaded));
    }

    #[test]
    fn invalid_curve_data_rejected() {
        let mut pool = CurvePool::new();
        let mut cps = [0.0f32; 12];
        cps[5] = f32::NAN;
        let knots = [0.0f32; 8];
        let weights = [1.0f32; 4];
        let result = pool.load(CurveHandle(0), &cps, &knots, &weights, 3);
        assert_eq!(result, Err(CurvePoolError::NonFiniteData));
    }
}
