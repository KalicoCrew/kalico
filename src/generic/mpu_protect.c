#include "autoconf.h"
#include "armcm_boot.h"
#include "board/internal.h"
#include "board/irq.h"
#include "command.h"
#include "sched.h"

extern uint32_t _sched_protected_start;
extern uint32_t _sched_protected_end;

#define RASR_SIZE_128B   (6u << 1)
#define RASR_AP_RO       (7u << 24)
#define RASR_AP_RW_OPEN  (3u << 24)
#define RASR_XN          (1u << 28)
#define RASR_TEX0_C_B0_S ((0u << 19) | (1u << 17) | (0u << 16) | (1u << 18))
#define RASR_ENABLE      (1u << 0)

static inline void
mpu_set_region0_ap(uint32_t ap_mask)
{
#if (__CORTEX_M >= 3)
    MPU->RNR = 0;
    uint32_t rasr = MPU->RASR;
    rasr &= ~(7u << 24);
    rasr |= ap_mask;
    MPU->RASR = rasr;
    __DSB();
    __ISB();
#else
    (void)ap_mask;
#endif
}

// Nested begin/end must count, not toggle: the outer SysTick dispatcher holds
// the window open while a dispatched callback's own begin/end pair runs inside
// it, so a non-counting inner end() would clamp the region RO and fault the
// next outer write. volatile + irq_save brackets because both IRQ and task
// context update this.
static volatile uint32_t sched_writable_depth;

void
sched_writable_begin(void)
{
    irqstatus_t flag = irq_save();
    if (sched_writable_depth == 0)
        mpu_set_region0_ap(RASR_AP_RW_OPEN);
    sched_writable_depth++;
    irq_restore(flag);
}

void
sched_writable_end(void)
{
    irqstatus_t flag = irq_save();
    if (sched_writable_depth > 0) {
        sched_writable_depth--;
        if (sched_writable_depth == 0)
            mpu_set_region0_ap(RASR_AP_RO);
    }
    irq_restore(flag);
}

// A try_shutdown longjmp can bypass the matching end() and leave a stale
// non-zero depth, so protection never re-engages; sched_main calls this after
// its setjmp returns non-zero to force the region back to read-only.
void
sched_writable_reset(void)
{
    irqstatus_t flag = irq_save();
    sched_writable_depth = 0;
    mpu_set_region0_ap(RASR_AP_RO);
    irq_restore(flag);
}

void
mpu_protect_init(void)
{
    uint32_t start = (uint32_t)&_sched_protected_start;
    uint32_t end   = (uint32_t)&_sched_protected_end;
    if (end - start != 128u || (start & 127u) != 0u)
        shutdown(".sched_protected size/alignment mismatch");

#if (__CORTEX_M >= 3)
    // MPU must be disabled while its config registers are written; doing so
    // with the MPU enabled is UB.
    MPU->CTRL = 0;
    __DSB();
    __ISB();

    MPU->RNR  = 0;
    MPU->RBAR = start;
    MPU->RASR = RASR_ENABLE
              | RASR_SIZE_128B
              | RASR_AP_RO
              | RASR_TEX0_C_B0_S
              | RASR_XN;

    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
    __DSB();
    __ISB();

    // Routes MPU violations to MemManage so early-init faults (before
    // fault_handler_init's DECL_INIT) hit the proper handler; idempotent with
    // the same bit set later in fault_handler_init.
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
#endif
}
