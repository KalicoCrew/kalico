//! Rust mirror of the C-side `StepQueue` (`src/step_queue.h`).
//!
//! Storage is owned by C — on the MCU, the C translation unit declares a
//! `[StepQueue; N_AXIS_STEP_QUEUES]` placed in `.axi_bss` (or equivalent),
//! and this module provides typed accessors via the `step_queues` extern.
//! On host / test builds, callers construct `StepQueue::new()` instances
//! on the stack or in a `Box` and pass `*mut StepQueue` into the free
//! functions below.
//!
//! The queue is a single-producer / single-consumer ring buffer of power-of-two
//! depth (`STEP_QUEUE_DEPTH = 32`) using free-running `u16` counters; the slot
//! index is `counter & STEP_QUEUE_DEPTH_MASK`. Counter wraparound is correct
//! by construction because `tail.wrapping_sub(head)` returns the populated
//! length even when `tail` has wrapped past `head` — both unsigned values
//! advance monotonically modulo `2^16`, and the difference modulo `2^16`
//! equals the true outstanding count whenever it is `< 2^16` (here it is
//! bounded by `STEP_QUEUE_DEPTH = 32`).
//!
//! Cross-core ordering on the H7 follows the standard SPSC release/acquire
//! discipline: the producer fills `buf[slot]` then publishes via a Release
//! fence + volatile write to `tail`; the consumer reads `tail` first, takes
//! an Acquire fence, then consumes `buf[slot]`. Volatile counter accesses
//! prevent the compiler from caching them across the fence.

#![allow(unsafe_code)]

use core::ptr;
use core::sync::atomic::{Ordering, fence};

/// Power-of-two ring depth shared with the C side; see `src/step_queue.h`.
pub const STEP_QUEUE_DEPTH: usize = 32;
/// Index mask derived from the depth — `counter & MASK` is the slot index.
pub const STEP_QUEUE_DEPTH_MASK: u16 = (STEP_QUEUE_DEPTH as u16) - 1;
/// Number of per-axis step queues (X, Y, Z, E).
pub const N_AXIS_STEP_QUEUES: usize = 4;

/// One pending step pulse: an absolute MCU cycle time and a direction.
///
/// Layout must match the C struct exactly — `#[repr(C)]` + the same field
/// order + the explicit 3-byte tail pad gives an 8-byte entry on every
/// target we care about (ABI-stable across H7 / F4 / host).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StepEntry {
    pub cycle_abs: u32,
    pub dir: i8,
    // ABI tail padding so each entry is 8 bytes. Public for FFI layout;
    // never read from Rust.
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 3],
}

/// SPSC ring of step entries. Storage is C-owned on the MCU; on host the
/// `new()` constructor lets tests build instances directly.
#[repr(C)]
#[derive(Debug)]
pub struct StepQueue {
    pub tail: u16,
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [StepEntry; STEP_QUEUE_DEPTH],
}

impl StepQueue {
    /// Construct an empty `StepQueue` on the host / in tests. Not available
    /// on the MCU because the storage there lives in a fixed C array.
    #[cfg(any(test, feature = "host"))]
    #[must_use]
    pub fn new() -> Self {
        StepQueue {
            tail: 0,
            head: 0,
            _pad: [0; 4],
            buf: [StepEntry {
                cycle_abs: 0,
                dir: 0,
                _pad: [0; 3],
            }; STEP_QUEUE_DEPTH],
        }
    }
}

#[cfg(any(test, feature = "host"))]
impl Default for StepQueue {
    fn default() -> Self {
        Self::new()
    }
}

// Layout invariants — these must match the C-side struct exactly. If a
// future refactor changes field order or padding the build will fail here
// rather than silently corrupting cross-language reads.
const _: () = {
    assert!(core::mem::size_of::<StepEntry>() == 8);
    assert!(core::mem::size_of::<StepQueue>() == 264);
    assert!(STEP_QUEUE_DEPTH.is_power_of_two());
};

// On MCU builds, storage is the C-declared `step_queues` symbol; the
// `UnsafeCell` wrapper is purely a marker that interior mutation is
// expected. Host / test builds construct queues directly via `new()`.
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    pub static step_queues: core::cell::UnsafeCell<[StepQueue; N_AXIS_STEP_QUEUES]>;
}

/// Returned by `push` when the ring is full.
#[derive(Debug, PartialEq, Eq)]
pub struct StepQueueFull;

/// Push one entry into the ring.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* producer for `q`; calling `push` from
///   two threads / cores / ISRs against the same queue is UB. The
///   consumer (`pop` / `peek`) is allowed to run concurrently on the
///   opposite core.
pub unsafe fn push(q: *mut StepQueue, entry: StepEntry) -> Result<(), StepQueueFull> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail.wrapping_sub(head) >= STEP_QUEUE_DEPTH as u16 {
        return Err(StepQueueFull);
    }
    let slot = (tail & STEP_QUEUE_DEPTH_MASK) as usize;
    unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::write_volatile(&mut (*q).buf[slot], entry);
    }
    fence(Ordering::Release);
    unsafe { ptr::write_volatile(&mut (*q).tail, tail.wrapping_add(1)) };
    Ok(())
}

/// Pop the oldest entry from the ring, or return `None` if empty.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* consumer for `q`; calling `pop` from
///   two threads / cores / ISRs against the same queue is UB. The
///   producer (`push`) is allowed to run concurrently on the opposite core.
pub unsafe fn pop(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    let entry = unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::read_volatile(&(*q).buf[slot])
    };
    fence(Ordering::Release);
    unsafe { ptr::write_volatile(&mut (*q).head, head.wrapping_add(1)) };
    Some(entry)
}

/// Look at the oldest entry without consuming it.
///
/// # Safety
///
/// Same constraints as [`pop`] — `q` must be live and the caller must be
/// the sole consumer (peek advances no counters but must still see a
/// consistent snapshot of `buf[head]`).
pub unsafe fn peek(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    Some(unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::read_volatile(&(*q).buf[slot])
    })
}

/// Current populated length. Racy by design — both endpoints may read
/// this for monitoring without coordination. The result is always a valid
/// `u16` in `0..=STEP_QUEUE_DEPTH`, but may be stale by the time the
/// caller inspects it.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - Safe to call from any context (producer, consumer, or a third
///   observer); does not advance counters and so cannot violate SPSC
///   discipline.
pub unsafe fn len(q: *mut StepQueue) -> u16 {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    tail.wrapping_sub(head)
}
