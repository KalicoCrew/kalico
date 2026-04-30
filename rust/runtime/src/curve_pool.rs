//! `CurvePool` — static slab of NURBS curve data referenced by `CurveHandle`.
//!
//! Step-6 §10 rewrite: per-slot `(current_gen, last_retired_gen)` `AtomicU16`
//! pair guards a foreground-writer / ISR-reader contract. The foreground
//! reserves a slot AND loads the curve atomically via `try_alloc_and_load`
//! (the alloc predicate is `current_gen == last_retired_gen` modulo `u16`
//! wrap; load-then-bump-gen ordering — Round-1 Codex #4). The ISR resolves
//! a handle via `lookup` which validates the generation match. Foreground
//! drains `SEGMENT_END` trace events and calls `confirm_retired` on each;
//! FIFO ordering of single-writer/single-reader `heapless::spsc` preserves
//! the per-slot retirement sequence so all earlier generations are retired
//! by the time `gen=G` is observed (§10.4).

#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU16, Ordering};

use crate::error::FaultCode;

/// Per-curve storage capacity. Sized for post-convolution degree-9 NURBS
/// with up to 80 scalar control points — the per-axis scalar architecture
/// stores one slot per axis (X, Y, Z, E).
pub const MAX_CONTROL_POINTS: usize = 80;
pub const MAX_KNOT_VECTOR_LEN: usize = 91; // MAX_CONTROL_POINTS + MAX_DEGREE as usize + 1
pub const MAX_DEGREE: u8 = 10;

/// Slab capacity. 64 slots for the per-axis scalar curve pool. Each slot
/// holds a 1D scalar NURBS.
pub const CURVE_POOL_N: usize = 64;

/// Deprecated — kept for `kalico-c-api` compilation until Task 8 updates
/// the FFI. The scalar architecture uses 1D control points, not 3D vectors.
#[deprecated(note = "scalar curve pool — use 1D control points; removed in Task 8")]
pub const MAX_DIM: usize = 1;

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

/// Handle to a loaded curve. 32-bit packed `(slot_idx, generation)` per
/// spec §10.1. ABA-defeating: at `Q_N_MAX = 256` the u16 gen wraps over
/// `65536 - 256 = 65280` allocations, which exceeds any realistic in-flight
/// stale-handle window.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle {
    pub slot_idx: u16,
    pub generation: u16,
}

const _: () = assert!(core::mem::size_of::<CurveHandle>() == 4);

impl CurveHandle {
    pub const fn new(slot_idx: u16, generation: u16) -> Self {
        Self {
            slot_idx,
            generation,
        }
    }

    /// Sentinel for hold segments (§6.5). The ISR short-circuits on the
    /// `SEGMENT_FLAG_HOLD_SEGMENT` flag bit BEFORE looking up this handle, so
    /// the sentinel is never resolved through `CurvePool::lookup`.
    pub const HOLD_SEGMENT_SENTINEL: Self = Self {
        slot_idx: u16::MAX,
        generation: u16::MAX,
    };

    /// Sentinel for unused curve handle slots in multi-handle segment structs.
    /// Distinct from `HOLD_SEGMENT_SENTINEL` to avoid confusion.
    pub const UNUSED_SENTINEL: Self = Self {
        slot_idx: u16::MAX - 1,
        generation: u16::MAX - 1,
    };

    /// Returns `true` if this handle is the unused sentinel.
    pub fn is_unused_sentinel(self) -> bool {
        self == Self::UNUSED_SENTINEL
    }

    /// Pack into a u32 for the wire schema. Layout:
    /// `(generation << 16) | slot_idx`. Mirror in C with `(uint32_t)gen <<
    /// 16 | slot`.
    pub const fn pack(self) -> u32 {
        ((self.generation as u32) << 16) | (self.slot_idx as u32)
    }

    /// Inverse of `pack`.
    pub const fn unpack(packed: u32) -> Self {
        Self {
            slot_idx: (packed & 0xFFFF) as u16,
            generation: ((packed >> 16) & 0xFFFF) as u16,
        }
    }
}

/// Scalar curve data laid out for direct ISR consumption. Per-axis 1D NURBS
/// — no rational weights (all live pipeline NURBS are polynomial).
#[derive(Clone, Debug)]
pub struct LoadedScalarCurve {
    pub control_points: [f32; MAX_CONTROL_POINTS],
    pub knots: [f32; MAX_KNOT_VECTOR_LEN],
    pub n_cp: u8,
    pub n_knots: u8,
    pub degree: u8,
}

impl LoadedScalarCurve {
    /// Empty placeholder used to initialize fresh `PoolSlot`s. The slot's
    /// generation counter is what marks "loaded vs not"; this curve is never
    /// resolved through `lookup` because no handle of `gen=0` is ever issued
    /// (the first `try_alloc_and_load` returns `gen=1`).
    pub const fn empty() -> Self {
        Self {
            control_points: [0.0; MAX_CONTROL_POINTS],
            knots: [0.0; MAX_KNOT_VECTOR_LEN],
            n_cp: 0,
            n_knots: 0,
            degree: 0,
        }
    }
}

/// One slab slot. `current_gen` and `last_retired_gen` are foreground-
/// written / ISR-read `AtomicU16`s; `curve` is the data store.
///
/// Synchronization discipline (§10.2 + Round-1 Codex #4):
/// - Foreground writes `curve` BEFORE `current_gen` (release).
/// - ISR loads `current_gen` (acquire) BEFORE dereferencing `curve`.
/// - Foreground writes `last_retired_gen` (release) on observing
///   `SEGMENT_END(handle)` from the trace ring.
#[allow(missing_debug_implementations)]
pub struct PoolSlot {
    pub current_gen: AtomicU16,
    pub last_retired_gen: AtomicU16,
    pub curve: UnsafeCell<LoadedScalarCurve>,
}

impl Default for PoolSlot {
    fn default() -> Self {
        Self::new()
    }
}

impl PoolSlot {
    pub const fn new() -> Self {
        Self {
            current_gen: AtomicU16::new(0),
            last_retired_gen: AtomicU16::new(0),
            curve: UnsafeCell::new(LoadedScalarCurve::empty()),
        }
    }
}

// SAFETY: `PoolSlot` carries `UnsafeCell<LoadedScalarCurve>` which is `!Sync` by
// default. Synchronization is achieved via per-slot `AtomicU16` generation
// counters (foreground writes `curve` before publishing the new `current_gen`
// with a release store; the ISR acquire-loads `current_gen` before
// dereferencing `slot.curve`). Discipline contract is documented above and
// enforced by code review (no `&mut PoolSlot` ever forms; the FFI shim only
// touches the slot through `&CurvePool` shared borrows that drive the atomic
// API).
unsafe impl Sync for PoolSlot {}

#[allow(missing_debug_implementations)]
pub struct CurvePool {
    pub slots: [PoolSlot; CURVE_POOL_N],
}

// SAFETY: see `PoolSlot`'s discipline contract. `CurvePool` is a top-level
// field on `RuntimeContext`; the FFI shim borrows it via `&CurvePool` only,
// and per-slot atomics bridge the foreground-writer / ISR-reader split.
unsafe impl Sync for CurvePool {}

impl Default for CurvePool {
    fn default() -> Self {
        Self::new()
    }
}

impl CurvePool {
    pub const fn new() -> Self {
        Self {
            slots: [const { PoolSlot::new() }; CURVE_POOL_N],
        }
    }

    /// Foreground reserves a slot AND loads the curve atomically. Returns
    /// `Some(handle)` if `current_gen == last_retired_gen` (modulo u16),
    /// `None` otherwise.
    ///
    /// Validates: `degree <= MAX_DEGREE`, `cps.len() <= MAX_CONTROL_POINTS`,
    /// `knots.len() == cps.len() + degree as usize + 1`.
    ///
    /// Ordering (Round-1 Codex #4): the new curve MUST be written before
    /// `current_gen` is bumped. Otherwise the ISR's lookup could observe
    /// `current_gen == new_gen` while the curve memory is still stale,
    /// dereferencing the previous gen's data through the new handle.
    pub fn try_alloc_and_load(
        &self,
        slot_idx: usize,
        degree: u8,
        knots: &[f32],
        cps: &[f32],
    ) -> Option<CurveHandle> {
        if slot_idx >= CURVE_POOL_N {
            return None;
        }
        if degree > MAX_DEGREE {
            return None;
        }
        let n_cp = cps.len();
        if n_cp == 0 || n_cp > MAX_CONTROL_POINTS {
            return None;
        }
        let expected_knots = n_cp + degree as usize + 1;
        if knots.len() != expected_knots || knots.len() > MAX_KNOT_VECTOR_LEN {
            return None;
        }

        // SAFETY (indexing): bounds-checked above against the array length.
        let slot = self.slots.get(slot_idx)?;
        let cur = slot.current_gen.load(Ordering::Acquire);
        let last = slot.last_retired_gen.load(Ordering::Acquire);
        if cur != last {
            return None;
        }

        // Build the LoadedScalarCurve from validated inputs.
        let mut loaded = LoadedScalarCurve::empty();
        loaded.control_points[..n_cp].copy_from_slice(cps);
        loaded.knots[..knots.len()].copy_from_slice(knots);
        loaded.n_cp = n_cp as u8;
        loaded.n_knots = knots.len() as u8;
        loaded.degree = degree;

        // 1. Write the new curve. The predicate above guarantees no
        //    concurrent ISR access — ISR's lookup checks `current_gen` first
        //    and would see the previous (matched) gen if it raced with us.
        // SAFETY: foreground is the sole writer of `slot.curve`; no `&mut
        // PoolSlot` ever forms, so the `UnsafeCell::get()` raw pointer is
        // valid for an exclusive write while we hold the `cur == last`
        // invariant.
        unsafe {
            *slot.curve.get() = loaded;
        }
        // 2. Bump generation with Release. Wraps on u16 modulo. The ISR's
        //    Acquire-load synchronizes with this Release store, ensuring
        //    the curve write is visible iff the new gen is.
        let new_gen = cur.wrapping_add(1);
        slot.current_gen.store(new_gen, Ordering::Release);
        Some(CurveHandle {
            slot_idx: slot_idx as u16,
            generation: new_gen,
        })
    }

    /// ISR-only lookup; validates handle generation matches `current_gen`.
    /// Returns `Err(FaultCode::InvalidCurveHandle)` on mismatch (stale or
    /// out-of-range).
    pub fn lookup(&self, handle: CurveHandle) -> Result<&LoadedScalarCurve, FaultCode> {
        let slot_idx = handle.slot_idx as usize;
        if slot_idx >= CURVE_POOL_N {
            return Err(FaultCode::InvalidCurveHandle);
        }
        let slot = self
            .slots
            .get(slot_idx)
            .ok_or(FaultCode::InvalidCurveHandle)?;
        if slot.current_gen.load(Ordering::Acquire) != handle.generation {
            return Err(FaultCode::InvalidCurveHandle);
        }
        // SAFETY: handle.generation matches `current_gen`; per the load-
        // before-bump-gen contract, the curve write is visible. The ISR is
        // the sole reader; no `&mut LoadedScalarCurve` ever forms.
        Ok(unsafe { &*slot.curve.get() })
    }

    /// §8.5 flush helper. Resets every slot's `last_retired_gen` to its
    /// `current_gen` so post-flush allocations succeed immediately (the
    /// alloc predicate is `current_gen == last_retired_gen`). This is
    /// safe under the flush contract: by the time foreground reaches
    /// step 5, the queue has been drained, the engine's in-flight segment
    /// is cleared, and no `Segment` in flight references any slot — so we
    /// can declare every slot reclaimed without waiting for `SEGMENT_END`
    /// trace events that will never come.
    pub fn reset_all_retired_to_current(&self) {
        for slot in &self.slots {
            let cur = slot.current_gen.load(Ordering::Acquire);
            slot.last_retired_gen.store(cur, Ordering::Release);
        }
    }

    /// Foreground reclaim. Called from the trace-drain pipeline on observing
    /// `SEGMENT_END(handle)`. FIFO ordering of trace events guarantees all
    /// prior generations for this slot have already retired.
    pub fn confirm_retired(&self, handle: CurveHandle) {
        let slot_idx = handle.slot_idx as usize;
        if slot_idx >= CURVE_POOL_N {
            return;
        }
        if let Some(slot) = self.slots.get(slot_idx) {
            slot.last_retired_gen
                .store(handle.generation, Ordering::Release);
        }
    }

    /// Resolve a handle to a scalar curve view. Returns `None` on stale
    /// handle or out-of-range. Convenience wrapper around `lookup` that
    /// converts to a borrowed-slice view.
    pub fn resolve(&self, handle: CurveHandle) -> Option<CurveView<'_>> {
        let curve = self.lookup(handle).ok()?;
        let n_cp = curve.n_cp as usize;
        let n_knots = curve.n_knots as usize;
        let control_points = curve.control_points.get(..n_cp)?;
        let knots = curve.knots.get(..n_knots)?;
        Some(CurveView {
            control_points,
            knots,
            degree: curve.degree,
        })
    }

    /// Producer-side validation + alloc. Full NURBS precondition validation
    /// (NaN/Inf rejection, knot vector monotonicity and clamping checks) on
    /// top of `try_alloc_and_load`'s length/degree validation. On success,
    /// returns the freshly issued `CurveHandle`.
    pub fn validate_and_load(
        &self,
        slot_idx: u16,
        degree: u8,
        knots: &[f32],
        cps: &[f32],
    ) -> Result<CurveHandle, CurvePoolError> {
        let idx = slot_idx as usize;
        if idx >= CURVE_POOL_N {
            return Err(CurvePoolError::OutOfBounds);
        }
        if degree > MAX_DEGREE {
            return Err(CurvePoolError::DegreeTooHigh);
        }

        let n_cp = cps.len();
        if n_cp == 0 || n_cp > MAX_CONTROL_POINTS {
            return Err(CurvePoolError::InvalidLengths);
        }
        let expected_knots = n_cp + degree as usize + 1;
        if knots.len() > MAX_KNOT_VECTOR_LEN || knots.len() != expected_knots {
            return Err(CurvePoolError::InvalidLengths);
        }

        if !cps.iter().chain(knots.iter()).all(|x| x.is_finite()) {
            return Err(CurvePoolError::NonFiniteData);
        }

        for w in knots.windows(2) {
            if w.first().copied().unwrap_or(0.0) > w.last().copied().unwrap_or(0.0) {
                return Err(CurvePoolError::InvalidCurve);
            }
        }

        let first_knot = knots.first().copied().unwrap_or(0.0);
        let last_knot = knots.last().copied().unwrap_or(0.0);
        if !(last_knot > first_knot) {
            return Err(CurvePoolError::InvalidCurve);
        }

        let p = degree as usize;
        if n_cp < p + 1 {
            return Err(CurvePoolError::InvalidCurve);
        }

        #[allow(clippy::float_cmp)]
        let start_clamped = knots.iter().take(p + 1).all(|&k| k == first_knot);
        if !start_clamped {
            return Err(CurvePoolError::InvalidCurve);
        }
        #[allow(clippy::float_cmp)]
        let end_clamped = knots.iter().rev().take(p + 1).all(|&k| k == last_knot);
        if !end_clamped {
            return Err(CurvePoolError::InvalidCurve);
        }

        self.try_alloc_and_load(idx, degree, knots, cps)
            .ok_or(CurvePoolError::SlotAlreadyLoaded)
    }

    /// Sim-only escape hatch: load pre-validated scalar curve data without
    /// running the FPU-using validation in `validate_and_load`. See module
    /// docs for the Renode-FPU-disabled rationale.
    #[cfg(feature = "kalico-sim")]
    pub fn load_unchecked(
        &self,
        slot_idx: u16,
        degree: u8,
        knots: &[f32],
        cps: &[f32],
    ) -> Result<CurveHandle, CurvePoolError> {
        let idx = slot_idx as usize;
        if idx >= CURVE_POOL_N {
            return Err(CurvePoolError::OutOfBounds);
        }

        // Delegate to try_alloc_and_load which performs length/degree
        // validation but no NaN/Inf or knot-monotonicity checks (those
        // are the FPU-using parts in validate_and_load).
        self.try_alloc_and_load(idx, degree, knots, cps)
            .ok_or(CurvePoolError::SlotAlreadyLoaded)
    }
}

/// Borrowed view of a loaded scalar curve. One view per axis.
#[derive(Debug)]
pub struct CurveView<'a> {
    pub control_points: &'a [f32],
    pub knots: &'a [f32],
    pub degree: u8,
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    /// Build a clamped scalar NURBS test fixture: degree-3, `n_cp` control
    /// points. Returns `(cps, knots)` slices sized to fit.
    fn scalar_curve_data(n_cp: usize) -> (Vec<f32>, Vec<f32>) {
        let mut cps = vec![0.0f32; n_cp];
        for i in 0..n_cp {
            cps[i] = i as f32;
        }
        let knot_len = n_cp + 4; // degree-3
        let mut knots = vec![0.0f32; knot_len];
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
        (cps, knots)
    }

    #[test]
    fn fresh_pool_lookup_unloaded_returns_invalid_handle() {
        let pool = CurvePool::new();
        assert!(
            pool.lookup(CurveHandle::new(0, 1)).is_err(),
            "stale handle gen=1 must reject when current_gen=0"
        );
        assert!(pool.lookup(CurveHandle::new(15, 1)).is_err());
    }

    #[test]
    fn out_of_bounds_handle_returns_err() {
        let pool = CurvePool::new();
        assert!(
            pool.lookup(CurveHandle::new(CURVE_POOL_N as u16, 1))
                .is_err()
        );
        assert!(pool.lookup(CurveHandle::new(u16::MAX, 1)).is_err());
    }

    #[test]
    fn validate_and_load_then_lookup_returns_curve() {
        let pool = CurvePool::new();
        let (cps, knots) = scalar_curve_data(4);
        let handle = pool
            .validate_and_load(0, 3, &knots, &cps)
            .expect("load");
        assert_eq!(handle.slot_idx, 0);
        assert_eq!(handle.generation, 1);
        assert!(pool.lookup(handle).is_ok());
        let view = pool.resolve(handle).expect("resolve");
        assert_eq!(view.control_points, &cps[..]);
        assert_eq!(view.knots, &knots[..]);
        assert_eq!(view.degree, 3);
    }

    #[test]
    fn validate_and_load_twice_into_same_slot_blocks_until_retired() {
        let pool = CurvePool::new();
        let (cps, knots) = scalar_curve_data(4);
        let h1 = pool
            .validate_and_load(0, 3, &knots, &cps)
            .expect("first");
        let second = pool.validate_and_load(0, 3, &knots, &cps);
        assert_eq!(second, Err(CurvePoolError::SlotAlreadyLoaded));
        pool.confirm_retired(h1);
        let h2 = pool
            .validate_and_load(0, 3, &knots, &cps)
            .expect("second");
        assert_eq!(h2.generation, 2);
    }

    #[test]
    fn invalid_curve_data_rejected() {
        let pool = CurvePool::new();
        let mut cps = [0.0f32; 4];
        cps[2] = f32::NAN;
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let result = pool.validate_and_load(0, 3, &knots, &cps);
        assert_eq!(result, Err(CurvePoolError::NonFiniteData));
    }

    #[test]
    fn pack_unpack_round_trips() {
        let h = CurveHandle::new(7, 0xCAFE);
        let packed = h.pack();
        assert_eq!(packed, (0xCAFE_u32 << 16) | 7);
        let h2 = CurveHandle::unpack(packed);
        assert_eq!(h, h2);
    }
}
