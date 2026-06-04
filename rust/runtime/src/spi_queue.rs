// Rust mirror of the C-side `SpiQueue` (`src/spi_queue.h`).
//
// Storage is owned by C on the MCU; on host/test builds callers construct
// `SpiQueue::new()` instances directly.
//
// SPSC ring of power-of-two depth (`SPI_QUEUE_DEPTH = 16`) using free-running
// `u16` counters. Cross-core ordering follows the same release/acquire
// discipline as `step_queue.rs`: producer fills `buf[slot]` then publishes
// via Release fence + volatile write to `tail`; consumer reads `tail` first,
// takes Acquire fence, then consumes `buf[slot]`.

#![allow(unsafe_code)]

use core::ptr;
use core::sync::atomic::{Ordering, fence};

/// Power-of-two ring depth shared with the C side; see `src/spi_queue.h`.
pub const SPI_QUEUE_DEPTH: usize = 16;
/// Index mask derived from the depth — `counter & MASK` is the slot index.
pub const SPI_QUEUE_DEPTH_MASK: u16 = (SPI_QUEUE_DEPTH as u16) - 1;
pub const N_SPI_BUSES: usize = 3;

/// One pending SPI write: motor slot index, signed coil-A and coil-B
/// currents for the TMC5160 XDIRECT register.
///
/// Layout must match the C struct exactly — `#[repr(C)]` + explicit pads
/// give an 8-byte entry on every target we care about.
///
/// `motor_idx` is the index into the C-side `phase_motors[]` table.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpiWrite {
    pub motor_idx: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: u8,
    pub coil_a: i16,
    pub coil_b: i16,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad2: [u8; 2],
}
const _: () = assert!(core::mem::size_of::<SpiWrite>() == 8);

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
    #[cfg(any(test, feature = "host"))]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tail: 0,
            head: 0,
            _pad: [0; 4],
            buf: [SpiWrite {
                motor_idx: 0xFF,
                _pad: 0,
                coil_a: 0,
                coil_b: 0,
                _pad2: [0; 2],
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

// Layout invariants — must match the C-side struct exactly.
const _: () = {
    assert!(core::mem::size_of::<SpiWrite>() == 8);
    assert!(core::mem::size_of::<SpiQueue>() == 136);
    assert!(SPI_QUEUE_DEPTH.is_power_of_two());
};

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
/// the sole consumer.
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
/// - Safe to call from any context; does not advance counters.
pub unsafe fn len(q: *mut SpiQueue) -> u16 {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    tail.wrapping_sub(head)
}
