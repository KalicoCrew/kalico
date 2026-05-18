// C-side SPSC segment queue. Replaces heapless::spsc::Queue for the MCU
// build because LLVM was miscompiling the Rust borrow-projected
// `&mut IsrState` → `&mut Consumer<&Queue>` pattern: bench evidence
// 2026-05-18 (tag 0xCC) showed the same Consumer returning queue.len()=6
// from one call site and queue.len()=1 from another, simultaneously,
// despite identical Relaxed atomic loads of head/tail. C avoids the
// borrow-projection by using plain pointers — there is no compiler
// abstraction that could decide head/tail are aliased or unaliased
// differently between call sites.
//
// Capacity: KALICO_SEG_QUEUE_N = 8 slots, effective capacity 7 (one slot
// reserved so a full queue is `(tail+1) % N == head`). Matches the prior
// heapless::spsc::Queue<Segment, 8> behaviour the Rust side was built
// against.
//
// Segment ABI: the queue treats slots as opaque KALICO_SEGMENT_SIZE-byte
// blocks and uses memcpy for enqueue/dequeue. Rust's `Segment` is
// `#[repr(C)]` and 56 bytes (verified by segment.rs::tests
// segment_size_under_64_bytes_with_consumers_mask). If the Rust struct
// ever changes size the static_assert below catches it at compile time
// (Rust-side: `static_assertions::const_assert!(...)`).

#include "kalico_segment_queue.h"

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#define KALICO_SEG_QUEUE_N 8

// The queue lives in regular .bss (DTCM on H7, normal SRAM on F4 / Linux).
// DTCM is not cached by the M7 core, eliminating any possibility of
// cache-coherency contributing to the wedge. Even on AXI SRAM the M7
// is single-core so cache wouldn't be a real issue — the relevant fix
// here is avoiding the Rust borrow-projection, not memory placement.
static struct {
    atomic_uint head;
    atomic_uint tail;
    uint8_t buf[KALICO_SEG_QUEUE_N][KALICO_SEGMENT_SIZE];
} kalico_seg_queue;

// Diag counter. Incremented on every successful enqueue. Exposed via
// `kalico_native_queue_enqueue_total()` for the runtime_tick.c fault_detail
// rotation (post-migration replacement for the heapless-specific
// `producer_enqueue_success_total` Rust atomic).
static atomic_uint kalico_seg_queue_enqueue_total;
// Incremented on every successful dequeue.
static atomic_uint kalico_seg_queue_dequeue_total;

// All entry points marked `used + externally_visible` so LTO doesn't
// eliminate them — the only callers are inside the Rust staticlib
// (rust/runtime/src/c_segment_queue.rs), which the C compiler can't see
// during whole-program LTO of the firmware build.
__attribute__((used, externally_visible))
int
kalico_native_queue_enqueue(const void *seg_bytes)
{
    unsigned tail = atomic_load_explicit(&kalico_seg_queue.tail,
                                         memory_order_relaxed);
    unsigned next_tail = (tail + 1u) % KALICO_SEG_QUEUE_N;
    unsigned head = atomic_load_explicit(&kalico_seg_queue.head,
                                         memory_order_acquire);
    if (next_tail == head) {
        return -1; // full
    }
    memcpy(kalico_seg_queue.buf[tail], seg_bytes, KALICO_SEGMENT_SIZE);
    atomic_store_explicit(&kalico_seg_queue.tail, next_tail,
                          memory_order_release);
    atomic_fetch_add_explicit(&kalico_seg_queue_enqueue_total, 1u,
                              memory_order_relaxed);
    return 0;
}

__attribute__((used, externally_visible))
int
kalico_native_queue_dequeue(void *out_seg_bytes)
{
    unsigned head = atomic_load_explicit(&kalico_seg_queue.head,
                                         memory_order_relaxed);
    unsigned tail = atomic_load_explicit(&kalico_seg_queue.tail,
                                         memory_order_acquire);
    if (head == tail) {
        return -1; // empty
    }
    memcpy(out_seg_bytes, kalico_seg_queue.buf[head], KALICO_SEGMENT_SIZE);
    unsigned next_head = (head + 1u) % KALICO_SEG_QUEUE_N;
    atomic_store_explicit(&kalico_seg_queue.head, next_head,
                          memory_order_release);
    atomic_fetch_add_explicit(&kalico_seg_queue_dequeue_total, 1u,
                              memory_order_relaxed);
    return 0;
}

__attribute__((used, externally_visible))
unsigned
kalico_native_queue_len(void)
{
    unsigned head = atomic_load_explicit(&kalico_seg_queue.head,
                                         memory_order_relaxed);
    unsigned tail = atomic_load_explicit(&kalico_seg_queue.tail,
                                         memory_order_relaxed);
    return (tail - head + KALICO_SEG_QUEUE_N) % KALICO_SEG_QUEUE_N;
}

__attribute__((used, externally_visible))
void
kalico_native_queue_reset(void)
{
    atomic_store_explicit(&kalico_seg_queue.head, 0u, memory_order_release);
    atomic_store_explicit(&kalico_seg_queue.tail, 0u, memory_order_release);
}

__attribute__((used, externally_visible))
unsigned
kalico_native_queue_enqueue_total(void)
{
    return atomic_load_explicit(&kalico_seg_queue_enqueue_total,
                                memory_order_relaxed);
}

__attribute__((used, externally_visible))
unsigned
kalico_native_queue_dequeue_total(void)
{
    return atomic_load_explicit(&kalico_seg_queue_dequeue_total,
                                memory_order_relaxed);
}

// 2026-05-18 wedge fix: gate flag for `engine.producer_current` visibility
// across the &mut Engine borrow boundary. Rust's AtomicBool through the
// `&SharedState` access path was observed to give inconsistent reads
// (producer_step's atomic load = true while modulated_tick had written
// false). Use a plain C global that Rust accesses via volatile reads /
// writes — no Rust abstraction can decide to cache or reorder these.
//
// 0 = producer_current is None (empty / retired)
// 1 = producer_current is Some(seg)
volatile uint8_t kalico_producer_current_present
    __attribute__((used, externally_visible));
