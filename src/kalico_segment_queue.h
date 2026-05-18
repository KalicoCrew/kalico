// C-side SPSC segment queue. See kalico_segment_queue.c for the design
// rationale (LLVM miscompilation of Rust borrow-projected heapless::spsc::
// Consumer on H7 — bench evidence commit 4bdb14848 tag 0xCC).
//
// ABI: callers pass / receive opaque KALICO_SEGMENT_SIZE-byte blocks.
// The Rust `Segment` is `#[repr(C)]` and the size is asserted on both
// sides (Rust: `const_assert!(size_of::<Segment>() == KALICO_SEGMENT_SIZE)`;
// C: implied by memcpy with the matching size constant).

#ifndef __KALICO_SEGMENT_QUEUE_H
#define __KALICO_SEGMENT_QUEUE_H

#include <stdint.h>

// Rust-side Segment is 56 bytes (rust/runtime/src/segment.rs::tests
// segment_size_under_64_bytes_with_consumers_mask). Keep these in lock-step.
#define KALICO_SEGMENT_SIZE 56

// Enqueue a segment. `seg_bytes` must point at KALICO_SEGMENT_SIZE bytes
// holding a valid `Segment` (Rust-side repr(C) layout). Returns 0 on
// success, -1 if the queue is full.
int kalico_native_queue_enqueue(const void *seg_bytes);

// Dequeue the next segment. Writes KALICO_SEGMENT_SIZE bytes to
// `out_seg_bytes`. Returns 0 on success, -1 if the queue is empty.
int kalico_native_queue_dequeue(void *out_seg_bytes);

// Current count of segments in the queue (0..KALICO_SEG_QUEUE_N-1).
unsigned kalico_native_queue_len(void);

// Reset the queue to empty. Caller must serialise against concurrent
// enqueue/dequeue (foreground-only, ISR gated).
void kalico_native_queue_reset(void);

// Diagnostic counters. Cross-check enqueue_total against host's
// accepted_segment_id and dequeue_total against producer_step's
// observed-none-vs-dequeued ratio.
unsigned kalico_native_queue_enqueue_total(void);
unsigned kalico_native_queue_dequeue_total(void);

#endif // __KALICO_SEGMENT_QUEUE_H
