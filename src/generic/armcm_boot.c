// ARM Cortex-M vector table and initial bootup handling
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "armcm_boot.h" // DECL_ARMCM_IRQ
#include "autoconf.h" // CONFIG_MCU
#include "board/internal.h" // SysTick
#include "command.h" // DECL_CONSTANT_STR
#include "misc.h" // dynmem_start

// Export MCU type
DECL_CONSTANT_STR("MCU", CONFIG_MCU);

// Symbols created by armcm_link.lds.S linker script
extern uint32_t _data_start, _data_end, _data_flash;
extern uint32_t _bss_start, _bss_end, _stack_start;
extern uint32_t _stack_end;

/****************************************************************
 * Basic interrupt handlers
 ****************************************************************/

// Inlined version of memset (to avoid function calls during intial boot code)
static void __always_inline
boot_memset(void *s, int c, size_t n)
{
    volatile uint32_t *p = s;
    while (n) {
        *p++ = c;
        n -= sizeof(*p);
    }
}

// Inlined version of memcpy (to avoid function calls during intial boot code)
static void __always_inline
boot_memcpy(void *dest, const void *src, size_t n)
{
    const uint32_t *s = src;
    volatile uint32_t *d = dest;
    while (n) {
        *d++ = *s++;
        n -= sizeof(*d);
    }
}

// Main initialization code (called from ResetHandler below)
static void __noreturn __section(".text.armcm_boot.stage_two")
reset_handler_stage_two(void)
{
    int i;

#if __CORTEX_M >= 7
    // A bootloader (e.g. Katapult) may leave D-cache and I-cache enabled.
    // CMSIS SCB_EnableDCache/ICache skip invalidation when already on,
    // so stale cache lines survive into the application.  Disable and
    // invalidate both caches before touching RAM.
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB->CCR &= ~SCB_CCR_DC_Msk;
        __DSB(); __ISB();
    }
    SCB->CSSELR = 0U;
    __DSB();
    {
        uint32_t ccsidr = SCB->CCSIDR;
        uint32_t sets = (ccsidr >> 13) & 0x7FFF;
        uint32_t ways = (ccsidr >> 3) & 0x3FF;
        uint32_t wshift = __CLZ(ways) & 0x1FU;
        do {
            uint32_t w = ways;
            do {
                SCB->DCISW = (sets << SCB_DCISW_SET_Pos)
                           | (w << wshift);
            } while (w--);
        } while (sets--);
    }
    __DSB(); __ISB();
    if (SCB->CCR & SCB_CCR_IC_Msk) {
        SCB->CCR &= ~SCB_CCR_IC_Msk;
        __DSB(); __ISB();
    }
    SCB->ICIALLU = 0U;
    __DSB(); __ISB();
#endif

    // Clear all enabled user interrupts and user pending interrupts
    for (i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Reset all user interrupt priorities
#if __CORTEX_M == 33
    for (i = 0; i < ARRAY_SIZE(NVIC->IPR); i++)
        NVIC->IPR[i] = 0;
#else
    for (i = 0; i < ARRAY_SIZE(NVIC->IP); i++)
        NVIC->IP[i] = 0;
#endif

    // Disable SysTick interrupt
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    __DSB();

    // Clear pending pendsv and systick interrupts
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

    // Reset all system interrupt priorities
#if __CORTEX_M == 7 || __CORTEX_M == 33
    for (i = 0; i < ARRAY_SIZE(SCB->SHPR); i++)
        SCB->SHPR[i] = 0;
#else
    for (i = 0; i < ARRAY_SIZE(SCB->SHP); i++)
        SCB->SHP[i] = 0;
#endif

    __DSB();
    __ISB();
    __enable_irq();

    // Copy global variables from flash to ram
    uint32_t count = (&_data_end - &_data_start) * 4;
    boot_memcpy(&_data_start, &_data_flash, count);

    // Clear the bss segment
    boot_memset(&_bss_start, 0, (&_bss_end - &_bss_start) * 4);

    barrier();

    // Initializing the C library isn't needed...
    //__libc_init_array();

    // Run the main board specific code
    armcm_main();

    // The armcm_main() call should not return
    for (;;)
        ;
}

// Initial code entry point - invoked by the processor after a reset
// Reset interrupts and stack to take control from bootloaders
void __section(".text.armcm_boot.stage_one")
ResetHandler(void)
{
    __disable_irq();

    // Explicitly load the stack pointer, jump to stage two
    asm volatile("mov sp, %0\n bx %1"
                 : : "r"(&_stack_end), "r"(reset_handler_stage_two));
}
DECL_ARMCM_IRQ(ResetHandler, -15);

// Code called for any undefined interrupts
void
DefaultHandler(void)
{
    for (;;)
        ;
}


/****************************************************************
 * Dynamic memory range
 ****************************************************************/

// Return the start of memory available for dynamic allocations.
// `.persistent_diag` (NOLOAD, sits at `_bss_end` in the linker script's `ram`
// region) is RAM we deliberately keep OUTSIDE the boot-time bss-zero pass so
// it can preserve a diag word across `NVIC_SystemReset`. Without this skip,
// alloc_init would set alloc_end = &_bss_end — overlapping `.persistent_diag`
// — and the first `alloc_chunk` (e.g. for `oids[]`) would memset over it,
// then subsequent oid_alloc/oid_lookup writes would corrupt
// `rt_diag_persistent.{magic,last_packed,last_us,fault_count}`. Bench-2026-05-12
// symptom: H7 hit `Invalid oid type` on `tmcuart_send` after dozens of
// successful per-oid writes — `oids[N].type` was being clobbered by a
// later runtime_diag_progress write to `rt_diag_persistent.last_packed`.
extern uint32_t _persistent_diag_end;
void *
dynmem_start(void)
{
    return &_persistent_diag_end;
}

// Return the end of memory available for dynamic allocations
void *
dynmem_end(void)
{
    return &_stack_start;
}
