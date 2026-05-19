//! Rust mirror of the C-side `SpiQueue` (`src/spi_queue.h`).
//!
//! Storage is owned by C — on the MCU the C translation unit declares a
//! `[SpiQueue; N_SPI_BUSES]` and this module provides typed accessors via
//! the `spi_queues` extern. On host / test builds, callers construct
//! `SpiQueue::new()` instances directly and pass `*mut SpiQueue` into the
//! free functions below.
//!
//! The queue is a single-producer / single-consumer ring buffer of
//! power-of-two depth (`SPI_QUEUE_DEPTH = 16`) using free-running `u16`
//! counters; the slot index is `counter & SPI_QUEUE_DEPTH_MASK`.
//! Wraparound is correct by construction because `tail.wrapping_sub(head)`
//! returns the populated length even when `tail` has wrapped past `head` —
//! both unsigned values advance monotonically modulo `2^16`, and the
//! difference modulo `2^16` equals the true outstanding count whenever it
//! is `< 2^16` (here it is bounded by `SPI_QUEUE_DEPTH = 16`).
//!
//! Cross-core ordering follows the same release/acquire discipline as
//! `step_queue.rs`: the producer fills `buf[slot]` then publishes via a
//! Release fence + volatile write to `tail`; the consumer reads `tail`
//! first, takes an Acquire fence, then consumes `buf[slot]`. Volatile
//! counter accesses prevent the compiler from caching them across the
//! fence.

#![allow(unsafe_code)]

use core::ptr;
use core::sync::atomic::{fence, Ordering};

/// Power-of-two ring depth shared with the C side; see `src/spi_queue.h`.
pub const SPI_QUEUE_DEPTH: usize = 16;
/// Index mask derived from the depth — `counter & MASK` is the slot index.
pub const SPI_QUEUE_DEPTH_MASK: u16 = (SPI_QUEUE_DEPTH as u16) - 1;
/// Number of independent SPI buses (SPI1 / SPI3 + headroom).
pub const N_SPI_BUSES: usize = 3;

/// One pending SPI write: chip-select GPIO handle, TMC register address,
/// and a 32-bit payload (packed `(coil_A << 16) | (coil_B & 0xFFFF)` for
/// the XDIRECT register).
///
/// Layout must match the C struct exactly — `#[repr(C)]` + the same field
/// order + the explicit 3-byte pad gives a 12-byte entry on every target
/// we care about (ABI-stable across H7 / F4 / host).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpiWrite {
    pub cs_pin: u32,
    pub reg: u8,
    pub _pad: [u8; 3],
    pub value: i32,
}

/// SPSC ring of SPI writes. Storage is C-owned on the MCU; on host the
/// `new()` constructor lets tests build instances directly.
#[repr(C)]
#[derive(Debug)]
pub struct SpiQueue {
    pub tail: u16,
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [SpiWrite; SPI_QUEUE_DEPTH],
}

impl SpiQueue {
    /// Construct an empty `SpiQueue` on the host / in tests. Not available
    /// on the MCU because the storage there lives in a fixed C array.
    #[cfg(any(test, feature = "host"))]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tail: 0,
            head: 0,
            _pad: [0; 4],
            buf: [SpiWrite {
                cs_pin: 0,
                reg: 0,
                _pad: [0; 3],
                value: 0,
            }; SPI_QUEUE_DEPTH],
        }
    }
}

#[cfg(any(test, feature = "host"))]
impl Default for SpiQueue {
    fn default() -> Self {
        Self::new()
    }
}

// Layout invariants — these must match the C-side struct exactly. If a
// future refactor changes field order or padding the build will fail here
// rather than silently corrupting cross-language reads.
const _: () = {
    assert!(core::mem::size_of::<SpiWrite>() == 12);
    assert!(core::mem::size_of::<SpiQueue>() == 200);
    assert!(SPI_QUEUE_DEPTH.is_power_of_two());
};

// On MCU builds, storage is the C-declared `spi_queues` symbol; the
// `UnsafeCell` wrapper is purely a marker that interior mutation is
// expected. Host / test builds construct queues directly via `new()`.
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    pub static spi_queues: core::cell::UnsafeCell<[SpiQueue; N_SPI_BUSES]>;
}

/// Returned by `push` when the ring is full.
#[derive(Debug, PartialEq, Eq)]
pub struct SpiQueueFull;

/// Push one SPI write into the ring.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `SpiQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* producer for `q`; calling `push` from
///   two threads / cores / ISRs against the same queue is UB. The
///   consumer (`pop` / `peek`) is allowed to run concurrently on the
///   opposite core.
pub unsafe fn push(q: *mut SpiQueue, entry: SpiWrite) -> Result<(), SpiQueueFull> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail.wrapping_sub(head) >= SPI_QUEUE_DEPTH as u16 {
        return Err(SpiQueueFull);
    }
    let slot = (tail & SPI_QUEUE_DEPTH_MASK) as usize;
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
/// - `q` must be a non-null, properly aligned pointer to a live `SpiQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* consumer for `q`; calling `pop` from
///   two threads / cores / ISRs against the same queue is UB. The
///   producer (`push`) is allowed to run concurrently on the opposite core.
pub unsafe fn pop(q: *mut SpiQueue) -> Option<SpiWrite> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & SPI_QUEUE_DEPTH_MASK) as usize;
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
pub unsafe fn peek(q: *mut SpiQueue) -> Option<SpiWrite> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & SPI_QUEUE_DEPTH_MASK) as usize;
    Some(unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::read_volatile(&(*q).buf[slot])
    })
}

/// Current populated length. Racy by design — both endpoints may read
/// this for monitoring without coordination.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `SpiQueue`
///   whose storage outlives this call.
/// - Safe to call from any context (producer, consumer, or a third
///   observer); does not advance counters and so cannot violate SPSC
///   discipline.
pub unsafe fn len(q: *mut SpiQueue) -> u16 {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    tail.wrapping_sub(head)
}
