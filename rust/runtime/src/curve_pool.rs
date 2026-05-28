//! Curve pool stub — Task 5 placeholder.
//!
//! The full curve pool (slot allocator, generation guards, LoadedCubicCurve)
//! has been removed. This module retains only the types and constants that
//! are still referenced by `trace.rs`, `state.rs`, and `kalico-c-api` until
//! Task 6 rewrites the engine layer.

use core::sync::atomic::{AtomicU16, Ordering};

use heapless::Vec;

/// Wire-encoded `(generation << 16) | slot_idx` handle to a curve-pool slot.
/// `#[repr(C)]` so `TraceSample` stays ABI-compatible with the C consumer.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurveHandle {
    /// High 16 bits: generation. Low 16 bits: slot index.
    packed: u32,
}

impl CurveHandle {
    /// Sentinel value meaning "this axis has no curve" (all-ones pattern).
    pub const UNUSED_SENTINEL: Self = Self {
        packed: 0xFFFE_FFFE,
    };

    /// Sentinel for hold segments (differs from UNUSED so the ISR can
    /// distinguish "no work" from "hold position").
    pub const HOLD_SEGMENT_SENTINEL: Self = Self {
        packed: 0xFFFF_FFFF,
    };

    /// Construct from `slot_idx` and `generation`.
    #[inline]
    pub const fn new(slot_idx: u16, generation: u16) -> Self {
        Self {
            packed: ((generation as u32) << 16) | (slot_idx as u32),
        }
    }

    /// Pack to a `u32` for wire transmission.
    #[inline]
    pub const fn pack(self) -> u32 {
        self.packed
    }

    /// Unpack from a wire `u32`.
    #[inline]
    pub const fn unpack(v: u32) -> Self {
        Self { packed: v }
    }

    #[inline]
    pub const fn slot_idx(self) -> u16 {
        self.packed as u16
    }

    #[inline]
    pub const fn generation(self) -> u16 {
        (self.packed >> 16) as u16
    }

    /// Returns true if this handle is the UNUSED sentinel.
    #[inline]
    pub fn is_unused_sentinel(self) -> bool {
        self == Self::UNUSED_SENTINEL
    }
}

// Build-time sizing constants — same source as the original curve_pool.rs.
// `RT_STORAGE_SIZE`, `CURVE_POOL_N`, and `MAX_PIECES_PER_CURVE` are generated
// by `runtime/build.rs` into `$OUT_DIR/sizing.rs`.
include!(concat!(env!("OUT_DIR"), "/sizing.rs"));

/// Minimal per-slot state retained for diagnostic FFI (`runtime_handle_query_pool_state`).
///
/// Task 6 replaces this with the real slot struct containing piece data.
#[allow(missing_debug_implementations)]
pub struct PoolSlot {
    pub current_gen: AtomicU16,
    pub last_retired_gen: AtomicU16,
}

impl PoolSlot {
    const fn new() -> Self {
        Self {
            current_gen: AtomicU16::new(0),
            last_retired_gen: AtomicU16::new(0),
        }
    }

    /// Set `last_retired_gen = current_gen` so the alloc predicate is satisfied.
    pub fn reset_retired_to_current(&self) {
        let current = self.current_gen.load(Ordering::Acquire);
        self.last_retired_gen.store(current, Ordering::Release);
    }
}

/// Stub curve pool — Task 6 replaces with `PieceRing`-based architecture.
#[allow(missing_debug_implementations)]
pub struct CurvePool {
    /// Per-slot diagnostic generation counters, retained for
    /// `runtime_handle_query_pool_state` FFI compatibility.
    pub slots: Vec<PoolSlot, CURVE_POOL_N>,
}

impl CurvePool {
    pub fn new() -> Self {
        let mut slots: Vec<PoolSlot, CURVE_POOL_N> = Vec::new();
        for _ in 0..CURVE_POOL_N {
            // SAFETY: the loop runs exactly CURVE_POOL_N times and the Vec
            // capacity is CURVE_POOL_N, so push never fails.
            let _ = slots.push(PoolSlot::new());
        }
        Self { slots }
    }

    /// Stub: always returns `None`.
    #[allow(clippy::unused_self)]
    pub fn lookup_active(&self, _handle: CurveHandle) -> Option<*const ()> {
        None
    }

    /// Stub: no-op retire.
    #[allow(clippy::unused_self)]
    pub fn confirm_retired(&self, _handle: CurveHandle) {}

    /// Stub: always returns `Err(0)` (no-op — Task 6 wires real loading).
    ///
    /// Returns `Ok(CurveHandle)` on success or `Err(diag_u32)` on failure.
    /// The stub unconditionally returns the UNUSED_SENTINEL handle encoded
    /// as an error so the host observes `KALICO_ERR_INVALID_CURVE`.
    #[allow(clippy::unused_self)]
    pub fn try_alloc_and_load_diagnostic(
        &self,
        _slot_idx: usize,
        _pieces: &[crate::cubic_curve::WirePiece],
    ) -> Result<CurveHandle, u32> {
        Err(CurveHandle::UNUSED_SENTINEL.pack())
    }

    /// Set `last_retired_gen = current_gen` for every slot, satisfying the
    /// "slot is free" alloc predicate. Called on klippy reconnect so that
    /// stale generation counters don't block subsequent curve loads.
    pub fn reset_all_retired_to_current(&self) {
        for slot in &self.slots {
            slot.reset_retired_to_current();
        }
    }
}

impl Default for CurvePool {
    fn default() -> Self {
        Self::new()
    }
}
