//! C-backed SPSC segment queue. Replaces `heapless::spsc::Queue<Segment, 8>`
//! for the MCU build because LLVM was miscompiling the borrow-projected
//! `&mut IsrState` → `&mut Consumer<&Queue>` access pattern (2026-05-18
//! bench: tag 0xCC showed `qlen_sd=6, qlen_ps=1` on the SAME Consumer at
//! near-simultaneous moments, indicating per-call-site divergence in
//! Relaxed atomic loads).
//!
//! The C implementation (`src/kalico_segment_queue.c`) is a plain ring
//! buffer with `_Atomic uint32_t head, tail`. No borrow projection, no
//! Producer/Consumer wrapper types — just memory and atomic instructions.
//! Both `Producer` and `Consumer` here are zero-sized markers that route
//! through the same C functions.
//!
//! ## Host builds
//!
//! When `target_os != "none"` (host tests + sim), the queue is backed by a
//! `Mutex<VecDeque<Segment>>` so unit tests don't need the C linker
//! symbols. The semantics match (FIFO, capacity Q_N - 1 = 7).

use crate::queue::Q_N;
use crate::segment::Segment;
use core::marker::PhantomData;

/// Effective capacity. Matches `heapless::spsc::Queue<T, N>` (capacity =
/// `N - 1` due to the slot reserved for full-vs-empty discrimination).
pub const CAPACITY: usize = Q_N - 1;

#[cfg(target_os = "none")]
mod ffi {
    #[allow(unsafe_code)]
    unsafe extern "C" {
        pub fn kalico_native_queue_enqueue(seg_bytes: *const u8) -> i32;
        pub fn kalico_native_queue_dequeue(out_seg_bytes: *mut u8) -> i32;
        pub fn kalico_native_queue_len() -> u32;
        pub fn kalico_native_queue_reset();
    }
}

/// Producer half. Foreground-only. The `&mut self` discipline is preserved
/// for API compatibility with the prior `heapless::spsc::Producer`, but
/// internally all operations route through the C-side singleton — no
/// state lives in this struct.
#[allow(missing_debug_implementations)]
pub struct Producer<T> {
    _phantom: PhantomData<*mut T>,
}

/// Consumer half. Same disclaimer as `Producer`.
#[allow(missing_debug_implementations)]
pub struct Consumer<T> {
    _phantom: PhantomData<*mut T>,
}

// `*mut T` is `!Send + !Sync` by default; force Send for the markers so
// they can move between threads as the heapless types did.
#[allow(unsafe_code)]
unsafe impl<T: Send> Send for Producer<T> {}
#[allow(unsafe_code)]
unsafe impl<T: Send> Send for Consumer<T> {}

impl<T> Producer<T> {
    /// Construct a new Producer marker. The underlying C-side queue is a
    /// static singleton — calling this multiple times will mint markers
    /// that all route through the same backing storage.
    pub const fn new() -> Self {
        Self { _phantom: PhantomData }
    }
}

impl<T> Default for Producer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Consumer<T> {
    pub const fn new() -> Self {
        Self { _phantom: PhantomData }
    }
}

impl<T> Default for Consumer<T> {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Segment-specific impls ──────────────────────────────────────────────
//
// The C queue is hard-coded to Segment-sized slots. Generic over T would
// require runtime size checks; for now we only need Segment.

impl Producer<Segment> {
    /// Enqueue a segment. Returns `Err(seg)` if the queue is full.
    pub fn enqueue(&mut self, seg: Segment) -> Result<(), Segment> {
        #[cfg(target_os = "none")]
        {
            #[allow(unsafe_code)]
            // SAFETY: `&seg as *const Segment` is valid for KALICO_SEGMENT_SIZE
            // bytes (Segment is `#[repr(C)]` and 56 bytes; matches the C side's
            // KALICO_SEGMENT_SIZE constant). The C function performs a memcpy
            // from this pointer, no aliasing, no further use of the pointer
            // after the call returns.
            let r = unsafe {
                ffi::kalico_native_queue_enqueue(
                    core::ptr::from_ref(&seg).cast::<u8>(),
                )
            };
            if r == 0 { Ok(()) } else { Err(seg) }
        }
        #[cfg(not(target_os = "none"))]
        {
            host_backend::enqueue(seg)
        }
    }
}

impl Consumer<Segment> {
    /// Dequeue the next segment. Returns `None` if the queue is empty.
    pub fn dequeue(&mut self) -> Option<Segment> {
        #[cfg(target_os = "none")]
        {
            use core::mem::MaybeUninit;
            let mut slot = MaybeUninit::<Segment>::uninit();
            #[allow(unsafe_code)]
            // SAFETY: `slot.as_mut_ptr()` is valid for KALICO_SEGMENT_SIZE
            // bytes (Segment is 56 bytes, matches C side). On `r == 0` the
            // C function memcpy'd a valid Segment into the slot, so
            // `assume_init` is sound. On `r != 0` we discard the
            // uninitialised slot.
            let r = unsafe {
                ffi::kalico_native_queue_dequeue(
                    slot.as_mut_ptr().cast::<u8>(),
                )
            };
            if r == 0 {
                #[allow(unsafe_code)]
                Some(unsafe { slot.assume_init() })
            } else {
                None
            }
        }
        #[cfg(not(target_os = "none"))]
        {
            host_backend::dequeue()
        }
    }

    /// Current number of segments in the queue (0..CAPACITY).
    pub fn len(&self) -> usize {
        #[cfg(target_os = "none")]
        {
            #[allow(unsafe_code)]
            // SAFETY: no preconditions; reads atomic counters.
            unsafe { ffi::kalico_native_queue_len() as usize }
        }
        #[cfg(not(target_os = "none"))]
        {
            host_backend::len()
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Reset the queue to empty. Used by `runtime_force_idle` during a
/// foreground flush. Caller must guarantee no concurrent producer /
/// consumer access (spec §11 ownership).
pub fn reset() {
    #[cfg(target_os = "none")]
    {
        #[allow(unsafe_code)]
        // SAFETY: no preconditions on the C side; foreground caller's
        // §11 contract serialises access.
        unsafe { ffi::kalico_native_queue_reset() }
    }
    #[cfg(not(target_os = "none"))]
    {
        host_backend::reset();
    }
}

// ─── Host backend (Mutex<VecDeque<Segment>>) ─────────────────────────────

#[cfg(not(target_os = "none"))]
mod host_backend {
    use crate::segment::Segment;
    use std::collections::VecDeque;
    use std::sync::Mutex;
    use std::sync::OnceLock;

    fn queue() -> &'static Mutex<VecDeque<Segment>> {
        static Q: OnceLock<Mutex<VecDeque<Segment>>> = OnceLock::new();
        Q.get_or_init(|| Mutex::new(VecDeque::with_capacity(super::CAPACITY)))
    }

    pub(super) fn enqueue(seg: Segment) -> Result<(), Segment> {
        let mut q = queue().lock().unwrap_or_else(|p| p.into_inner());
        if q.len() >= super::CAPACITY {
            return Err(seg);
        }
        q.push_back(seg);
        Ok(())
    }

    pub(super) fn dequeue() -> Option<Segment> {
        let mut q = queue().lock().unwrap_or_else(|p| p.into_inner());
        q.pop_front()
    }

    pub(super) fn len() -> usize {
        let q = queue().lock().unwrap_or_else(|p| p.into_inner());
        q.len()
    }

    pub(super) fn reset() {
        let mut q = queue().lock().unwrap_or_else(|p| p.into_inner());
        q.clear();
    }
}

// Compile-time size check: the C side hard-codes KALICO_SEGMENT_SIZE = 56;
// if Segment grows beyond that the memcpy will overflow the per-slot
// buffer and we'll either corrupt adjacent state or trap.
const _: () = {
    assert!(core::mem::size_of::<Segment>() == 56);
};
