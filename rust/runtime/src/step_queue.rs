// NON-RACING, not lock-free: single-core safety requires the producer (TIM5
// ISR) and consumer (step-output timer ISR) to share one NVIC priority so they
// never interleave. If that ever splits, the volatile-u16 + fence discipline is
// insufficient (torn slot/counter) — upgrade to a true-atomic SPSC. Invariant
// + priority map: `src/generic/kalico_nvic_prio.h`.

#![allow(unsafe_code)]

use core::ptr;
use core::sync::atomic::{Ordering, fence};

/// Power-of-two ring depth shared with the C side; see `src/step_queue.h`.
pub const STEP_QUEUE_DEPTH: usize = 32;
pub const STEP_QUEUE_DEPTH_MASK: u16 = (STEP_QUEUE_DEPTH as u16) - 1;
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
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 3],
}

#[repr(C)]
#[derive(Debug)]
pub struct StepQueue {
    pub tail: u16,
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [StepEntry; STEP_QUEUE_DEPTH],
}

impl StepQueue {
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

    /// Empty the queue by resetting both SPSC counters to 0.
    ///
    /// The caller must hold exclusive access (an IRQ guard): both producer
    /// (writes `tail`) and consumer (writes `head`) must be quiescent.
    #[inline]
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
    }
}

#[cfg(any(test, feature = "host"))]
impl Default for StepQueue {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = {
    assert!(core::mem::size_of::<StepEntry>() == 8);
    assert!(core::mem::size_of::<StepQueue>() == 264);
    assert!(STEP_QUEUE_DEPTH.is_power_of_two());
};

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    pub static step_queues: core::cell::UnsafeCell<[StepQueue; N_AXIS_STEP_QUEUES]>;
}

#[cfg(not(any(test, feature = "host")))]
#[must_use]
pub fn queue_for_axis(i: usize) -> *mut StepQueue {
    if i >= N_AXIS_STEP_QUEUES {
        return ptr::null_mut();
    }
    // SAFETY: `i < N_AXIS_STEP_QUEUES` is checked above, and `step_queues` is
    // the C-declared array of exactly `N_AXIS_STEP_QUEUES` queues, so `add(i)`
    // stays in-bounds and yields a pointer to a live, aligned `StepQueue`.
    unsafe { step_queues.get().cast::<StepQueue>().add(i) }
}

/// Clear all per-axis step queues. MCU-only.
///
/// The caller (`kalico_runtime_reset`) holds the IRQ guard, so no producer
/// ISR or consumer timer runs concurrently with these writes.
#[cfg(not(any(test, feature = "host")))]
pub fn reset_all_queues() {
    for i in 0..N_AXIS_STEP_QUEUES {
        let q = queue_for_axis(i);
        // SAFETY: `i < N_AXIS_STEP_QUEUES` so `q` is non-null and points at a
        // live `StepQueue`; the IRQ guard guarantees exclusive access.
        unsafe {
            (*q).clear();
        }
    }
}

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
/// the sole consumer.
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
/// this for monitoring without coordination.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - Safe to call from any context; does not advance counters and cannot
///   violate SPSC discipline.
pub unsafe fn len(q: *mut StepQueue) -> u16 {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    tail.wrapping_sub(head)
}

#[cfg(test)]
mod tests;
