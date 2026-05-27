//! `CurvePool` — static slab of cubic-Bezier curve data referenced by `CurveHandle`.
//!
//! Step-6 §10 rewrite (carried forward through 2026-05-20 stepping redesign):
//! per-slot `(current_gen, last_retired_gen)` `AtomicU16` pair guards a
//! foreground-writer / ISR-reader contract. The foreground reserves a slot AND
//! loads the curve atomically via `try_alloc_and_load` (the alloc predicate is
//! `current_gen == last_retired_gen` modulo `u16` wrap; load-then-bump-gen
//! ordering — Round-1 Codex #4). The ISR resolves a handle via `lookup_active`
//! which validates the generation match. Foreground drains `SEGMENT_END` trace
//! events and calls `confirm_retired` on each; FIFO ordering of single-writer/
//! single-reader `heapless::spsc` preserves the per-slot retirement sequence
//! so all earlier generations are retired by the time `gen=G` is observed
//! (§10.4).
//!
//! Payload was NURBS in the original design; as of the 2026-05-20 stepping
//! redesign the slot stores `LoadedCubicCurve` (monomial-form cubic Bezier
//! pieces) instead. The slot+generation discipline is unchanged.

#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU16, Ordering};

use crate::cubic_curve::{populate_from_wire, CubicLoadError, LoadedCubicCurve, WirePiece};

// Build-time-configurable sizing constants. The `pub const`s (see
// `runtime/build.rs`) are emitted from Klipper's Kconfig values. Defaults
// (no Klipper Makefile in the loop) match the `large` profile per
// `docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md`.
include!(concat!(env!("OUT_DIR"), "/sizing.rs"));

/// Deprecated — kept for `kalico-c-api` compilation until Task 8 updates
/// the FFI. The scalar architecture uses 1D control points, not 3D vectors.
#[deprecated(note = "scalar curve pool — use 1D control points; removed in Task 8")]
pub const MAX_DIM: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurvePoolError {
    OutOfBounds,
    SlotAlreadyLoaded,
    NonFiniteData,
    /// Catch-all for piece-count out of range, non-positive durations, etc.
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
    /// the sentinel is never resolved through `CurvePool::lookup_active`.
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
    pub curve: UnsafeCell<LoadedCubicCurve>,
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
            curve: UnsafeCell::new(LoadedCubicCurve::empty()),
        }
    }
}

// SAFETY: `PoolSlot` carries `UnsafeCell<LoadedCubicCurve>` which is `!Sync` by
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

    /// Foreground reserves a slot AND loads the cubic-piece curve atomically.
    /// Returns `Some(handle)` on success, `None` on slot busy or validation failure.
    ///
    /// Slot+generation discipline (unchanged from NURBS variant):
    /// - Predicate: `current_gen == last_retired_gen` (slot is free).
    /// - The new curve is written DIRECTLY into the slot (`populate_from_wire`
    ///   touches only the slot's backing memory; no stack intermediate).
    /// - `current_gen` bumps with Release after the curve write completes. The
    ///   ISR's `lookup_active` Acquire-loads `current_gen` to validate handle
    ///   gen, ensuring the curve write is visible iff the new gen is.
    ///
    /// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §3.2.
    pub fn try_alloc_and_load(
        &self,
        slot_idx: usize,
        wire: &[WirePiece],
    ) -> Option<CurveHandle> {
        self.try_alloc_and_load_diagnostic(slot_idx, wire).ok()
    }

    /// Same as [`Self::try_alloc_and_load`], but returns a compact diagnostic
    /// word on rejection for the `LoadCurveResponse.curve_handle_packed` error
    /// path.
    ///
    /// Encoding:
    /// - kind 1: slot busy, low bits are `(current_gen << 16) | last_retired_gen`
    /// - kind 2: payload validation failed, low bits are a small reason code
    /// - kind 3: slot index out of bounds, low bits are the requested slot
    pub fn try_alloc_and_load_diagnostic(
        &self,
        slot_idx: usize,
        wire: &[WirePiece],
    ) -> Result<CurveHandle, u32> {
        if slot_idx >= CURVE_POOL_N {
            return Err((3u32 << 30) | ((slot_idx as u32) & 0xFFFF));
        }
        let slot = self
            .slots
            .get(slot_idx)
            .ok_or((3u32 << 30) | ((slot_idx as u32) & 0xFFFF))?;
        let cur = slot.current_gen.load(Ordering::Acquire);
        let last = slot.last_retired_gen.load(Ordering::Acquire);
        if cur != last {
            return Err((1u32 << 30) | (u32::from(cur) << 16) | u32::from(last));
        }
        // SAFETY: foreground is the sole writer of `slot.curve`; no `&mut
        // PoolSlot` ever forms, so the `UnsafeCell::get()` raw pointer is
        // valid for an exclusive write while we hold the `cur == last`
        // invariant. `populate_from_wire` validates ALL pieces before any
        // mutation — failure leaves the slot's previous contents intact.
        let result = unsafe {
            let dst: *mut LoadedCubicCurve = slot.curve.get();
            populate_from_wire(&mut *dst, wire)
        };
        if let Err(err) = result {
            // Validation failed; slot still has its previous contents.
            // The generation is unchanged so existing readers see the prior curve.
            let reason = match err {
                CubicLoadError::PieceCountOutOfRange => 1u32,
                CubicLoadError::NonFiniteBernstein => 2u32,
                CubicLoadError::NonPositiveDuration => 3u32,
            };
            return Err((2u32 << 30) | reason);
        }
        // 2. Bump generation with Release. Wraps on u16 modulo. The ISR's
        //    Acquire-load synchronizes with this Release store, ensuring
        //    the curve write is visible iff the new gen is.
        let new_gen = cur.wrapping_add(1);
        slot.current_gen.store(new_gen, Ordering::Release);
        Ok(CurveHandle {
            slot_idx: slot_idx as u16,
            generation: new_gen,
        })
    }

    /// ISR-only lookup; validates handle generation matches `current_gen`.
    /// Returns a const pointer (NOT `&LoadedCubicCurve`) so the ISR can hold
    /// this address across piece-advance calls without conflicting with the
    /// next load's `&mut` projection on the same slot.
    ///
    /// Returns `None` on out-of-range slot or generation mismatch.
    ///
    /// Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §4.4.
    pub fn lookup_active(&self, handle: CurveHandle) -> Option<*const LoadedCubicCurve> {
        let slot_idx = handle.slot_idx as usize;
        // `slots` is `[Slot; CURVE_POOL_N]`; `.get(slot_idx)` is the
        // panic-free form of indexing past the array length.
        let slot = self.slots.get(slot_idx)?;
        let cur = slot.current_gen.load(Ordering::Acquire);
        if cur != handle.generation {
            return None;
        }
        Some(slot.curve.get().cast_const())
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
    /// `SEGMENT_END(handle)` AND from `abort_for_homing_trip` (ISR context).
    ///
    /// Monotonic: only advances `last_retired_gen`, never rolls it back.
    /// `abort_for_homing_trip` may retire a LATER generation before the
    /// foreground drains the trace ring's EARLIER `SEGMENT_END` events.
    /// Without the CAS guard, the stale foreground drain would overwrite
    /// `last_retired_gen` backwards, leaving the slot permanently "busy."
    pub fn confirm_retired(&self, handle: CurveHandle) {
        let slot_idx = handle.slot_idx as usize;
        if slot_idx >= CURVE_POOL_N {
            return;
        }
        if let Some(slot) = self.slots.get(slot_idx) {
            loop {
                let current_last = slot.last_retired_gen.load(Ordering::Acquire);
                let delta = handle.generation.wrapping_sub(current_last);
                if delta == 0 || delta >= 32768 {
                    break;
                }
                match slot.last_retired_gen.compare_exchange_weak(
                    current_last,
                    handle.generation,
                    Ordering::Release,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(_) => continue,
                }
            }
        }
    }

    /// Returns `true` if `slot_idx`'s `last_retired_gen` matches its
    /// `current_gen` — i.e. the slot is back in the "free for reuse"
    /// state. Used by integration tests to assert that retirement fired.
    /// Sentinel slot indices (`>= CURVE_POOL_N`) return `true` (a sentinel
    /// is never "in use"). Foreground-only convenience accessor; the ISR
    /// reads the same atomic in `try_alloc_and_load`.
    pub fn is_slot_free(&self, slot_idx: u16) -> bool {
        let idx = slot_idx as usize;
        if idx >= CURVE_POOL_N {
            return true;
        }
        match self.slots.get(idx) {
            Some(slot) => {
                let cur = slot.current_gen.load(Ordering::Acquire);
                let last = slot.last_retired_gen.load(Ordering::Acquire);
                cur == last
            }
            None => true,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests;
