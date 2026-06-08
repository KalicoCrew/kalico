// rt_storage — backing buffer for the Kalico runtime engine.
//
// On H7 DTCM is saturated, so rt_storage goes in AXI SRAM via `.axi_bss`; on
// F4 it lands in default .bss. _Alignas(16) matches the Rust-side
// align_of <= 16 assertion (a field needing more fails the build, forcing a
// bump here).

#include "runtime_storage.h"

#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss"), used, externally_visible))
#else
__attribute__((used, externally_visible))
#endif
_Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE];

_Static_assert(RT_STORAGE_SIZE >= 1024,
               "RT_STORAGE_SIZE absurdly small — Kconfig profile broken");

// H7-only AXI SRAM overflow guard — sum every .axi_bss occupant and verify it
// fits the 320 KB AXI region with headroom. Update when adding/removing an
// .axi_bss static. Current occupants:
//   - kalico_buf       (src/kalico_demux.c, KALICO_DEMUX_KALICO_BUF_SIZE)
//   - receive_buf      (src/generic/serial_irq.c, RX_BUFFER_SIZE)
#if CONFIG_MACH_STM32H7
#define AXI_BSS_KALICO_BUF_BYTES        512   /* matches KALICO_DEMUX_KALICO_BUF_SIZE */
#define AXI_BSS_SERIAL_IRQ_RX_BYTES     2048  /* RX_BUFFER_SIZE in serial_irq.c */
#define AXI_BSS_HEADROOM                2048  /* 2 KB margin */
#define AXI_SRAM_SIZE                   (320 * 1024)

_Static_assert(
    RT_STORAGE_SIZE
        + AXI_BSS_KALICO_BUF_BYTES
        + AXI_BSS_SERIAL_IRQ_RX_BYTES
        + AXI_BSS_HEADROOM
        <= AXI_SRAM_SIZE,
    "AXI SRAM overflow: RT_STORAGE_SIZE too large for AXI region "
    "(after summing other .axi_bss occupants + headroom)"
);
#endif
