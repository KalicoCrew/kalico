// rt_storage — backing buffer for the Kalico runtime engine.
//
// Section placement:
//   H7: __attribute__((section(".axi_bss"))) — AXI SRAM at 0x24000000,
//       picked up by the linker rule in src/generic/armcm_link.lds.S
//       (gated #if CONFIG_MACH_STM32H7, targets > axi_ram region).
//   F4: no section attribute — lands in default .bss via the linker's
//       default rule, zeroed by armcm_boot.c::boot_memset.
//
// Alignment: _Alignas(16) is conservative. RuntimeContext contains
// AtomicU64 (8-byte aligned on ARMv7-M) and f32 arrays inside CurvePool;
// 16-byte alignment over-aligns for safety. The Rust side compile-asserts
// align_of::<RuntimeContext>() <= 16 so any future field that requires
// >16-byte alignment fails the build, prompting a bump here.
//
// Size contract: RT_STORAGE_SIZE comes from Kconfig (see
// runtime_storage.h). Rust-side const_assert ensures
// size_of::<RuntimeContext>() <= RT_STORAGE_SIZE; C-side _Static_assert
// below ensures the H7 AXI SRAM doesn't overflow when all .axi_bss
// occupants are summed.
//
// Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md.

#include "runtime_storage.h"

#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss"), used, externally_visible))
#else
__attribute__((used, externally_visible))
#endif
_Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE];

// Belt-and-suspenders sanity check: catches a Kconfig misconfiguration
// (e.g., RT_STORAGE_SIZE accidentally set to 0 or 100) before runtime.
// The real lower-bound enforcement is the Rust-side const_assert.
_Static_assert(RT_STORAGE_SIZE >= 1024,
               "RT_STORAGE_SIZE absurdly small — Kconfig profile broken");

// H7-only AXI SRAM overflow assert. Sum the bytes claimed by all .axi_bss
// occupants and verify the total fits with headroom in the 320 KB AXI
// region. Update this list when adding new .axi_bss statics.
//
// Other .axi_bss occupants on H7 today:
//   - kalico_buf (src/kalico_demux.c)
//   - runtime_bench_samples_buf (src/generic/runtime_bench.c)
//   - receive_buf (src/generic/serial_irq.c, RX_BUFFER_SIZE = 2 KB)
//
// (Segment SPSC queue is NOT in .axi_bss — lives in DTCM/regular .bss
// deliberately, per kalico_segment_queue.c:31-40.)
#if CONFIG_MACH_STM32H7
#define AXI_BSS_KALICO_BUF_BYTES        14752 /* 4 * (1830 + 1850) + 32 */
#define AXI_BSS_RUNTIME_BENCH_BYTES     1024  /* 256 * sizeof(uint32_t) */
#define AXI_BSS_SERIAL_IRQ_RX_BYTES     2048  /* RX_BUFFER_SIZE in serial_irq.c */
#define AXI_BSS_HEADROOM                2048  /* 2 KB margin — AXI is tight on H7 with the LARGE-profile RuntimeContext (~298 KB measured) */
#define AXI_SRAM_SIZE                   (320 * 1024)

_Static_assert(
    RT_STORAGE_SIZE
        + AXI_BSS_KALICO_BUF_BYTES
        + AXI_BSS_RUNTIME_BENCH_BYTES
        + AXI_BSS_SERIAL_IRQ_RX_BYTES
        + AXI_BSS_HEADROOM
        <= AXI_SRAM_SIZE,
    "AXI SRAM overflow: RT_STORAGE_SIZE too large for AXI region "
    "(after summing other .axi_bss occupants + headroom)"
);
#endif // CONFIG_MACH_STM32H7
