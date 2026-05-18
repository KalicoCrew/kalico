// MPU protection of the scheduler's foundation state (.sched_protected).
//
// Works on Cortex-M4 (STM32F446) and Cortex-M7 (STM32H723) — both implement
// ARMv7-M PMSAv7 with the same MPU programming model. F446 has 8 regions,
// H723 has 16; we use exactly one region for this layer.
//
// Layout
// ------
// The linker places SchedState (and only SchedState) into the
// `.sched_protected` section: a 128-byte, 128-byte-aligned block inside
// `.data`. `_sched_protected_start` / `_sched_protected_end` symbols mark
// its bounds. 60 bytes of state + 68 bytes padding fit inside.
//
// Region configuration
// --------------------
// One MPU region (region 0) covers the block:
//   * Size = 128 bytes (log2 - 1 = 6 in RASR.SIZE).
//   * Permission: read-only for both privileged and unprivileged code
//     (AP = 0b111). All writes from outside sched.c fault into
//     MemManage_Handler.
//   * TEX = 0, C = 1, B = 0, S = 1 (Normal memory, non-shareable,
//     write-back, no-write-allocate — i.e. the DTCM-like default for the
//     region holding SchedState).
//   * XN = 1: never execute from here.
//
// PRIVDEFENA = 1 keeps the default memory map active for everything else,
// so we don't have to enumerate every other region in the firmware.
//
// Open / close
// ------------
// sched_writable_begin() flips region 0's AP to 0b011 (privileged RW,
// unprivileged RO) and DSBs. sched_writable_end() flips back to 0b111
// (RO/RO) and DSBs. Both are called only from sched.c. The hot path
// (timer_dispatch_many in armcm_timer.c) opens the window once per
// SysTick invocation to amortize across the whole dispatch loop —
// per-call toggles on sched_timer_dispatch would add significant jitter
// on the M7 pipeline.
//
// Fault handling
// --------------
// On any unauthorized write, MemManage_Handler captures the faulting PC
// from the stacked exception frame and the MMFAR (faulting address) into
// rt_diag_persistent, which the next-boot fault_handler_report_task
// emits over the wire. addr2line on the captured PC identifies the
// rogue writer.

#include "autoconf.h"
#include "armcm_boot.h" // DECL_ARMCM_IRQ
#include "board/internal.h" // CMSIS MPU/CoreDebug definitions
#include "board/irq.h" // irq_save / irq_restore (re-entrant depth counter)
#include "command.h" // try_shutdown, shutdown macros
#include "sched.h" // sched_writable_begin/end, mpu_protect_init

// Linker-script symbols bracketing the `.sched_protected` section.
extern uint32_t _sched_protected_start;
extern uint32_t _sched_protected_end;

// MemManage faults are caught by the existing FAULT_TRAMPOLINE in
// fault_handler.c. That handler stores PC, LR, MMFAR, CFSR etc. into
// `fault_rec` (in persistent SRAM) and triggers NVIC_SystemReset. On the
// next boot, fault_handler_report_task emits all of it via the existing
// `prior_fault` / `prior_fault_status` outputs — addr2line on PC + the
// MMFAR value identifies any rogue write to `.sched_protected`.
//
// We don't need a separate handler in this file. Bit definitions for
// MPU_RASR fields (ARMv7-M, identical M4 + M7). AP
// encoding per ARM DDI 0403E Table B3-9.
//   0b011 (3) = priv RW / unpriv RW  → "open" window for sched.c writes
//   0b111 (7) = priv RO / unpriv RO  → default protected state
// SIZE field encoding: region_size = 2^(SIZE+1). For 128 bytes, SIZE = 6
// → RASR.SIZE field bits[5:1] = 6 → shifted into place (<<1).
#define RASR_SIZE_128B   (6u << 1)
#define RASR_AP_RO       (7u << 24)
#define RASR_AP_RW_OPEN  (3u << 24)
#define RASR_XN          (1u << 28)
// TEX = 0, C = 1, B = 0, S = 1 → Normal memory, outer/inner write-through,
// no write-allocate, shareable. DTCM (where SchedState lives on H7/F4) is
// never cached anyway, so these bits are effectively cosmetic; we pick a
// well-defined Normal-memory encoding for consistency.
#define RASR_TEX0_C_B0_S ((0u << 19) | (1u << 17) | (0u << 16) | (1u << 18))
#define RASR_ENABLE      (1u << 0)

// Toggle just the AP field of region 0. Read-modify-write because TEX/C/B/S
// /SIZE/ENABLE must be preserved.
static inline void
mpu_set_region0_ap(uint32_t ap_mask)
{
    MPU->RNR = 0;
    uint32_t rasr = MPU->RASR;
    rasr &= ~(7u << 24);   // clear AP field
    rasr |= ap_mask;
    MPU->RASR = rasr;
    __DSB();
    __ISB();
}

// Re-entrant depth counter for the writable window.
//
// `timer_dispatch_many` (armcm_timer.c) opens the window once per SysTick
// invocation and keeps it open across the whole dispatch loop. The
// dispatched timer callbacks run inside that window; some of them
// (e.g. step_time_event in runtime_tick.c — see the
// arm_producer_timer_if_kicked_inline call sites) reach sched_add_timer,
// which pairs its own begin/end. Without depth tracking the inner end()
// unconditionally clamps the region back to RO — the next write from
// the outer dispatcher then faults into MemManage.
//
// volatile because IRQ context (SysTick → timer_dispatch_many) and task
// context (sched_add_timer from DECL_TASK / DECL_INIT) both update it.
// Each update is bracketed by irq_save/irq_restore so the
// read-modify-write of the counter and the paired MPU programming are
// atomic with respect to any preempting IRQ that might also call
// begin/end.
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
    // Underflow guard: end() with no matching begin() should never
    // happen, but if it does we prefer to leave the region closed
    // rather than wrap the counter to UINT32_MAX (which would leave
    // the window stuck open until the next reboot).
    if (sched_writable_depth > 0) {
        sched_writable_depth--;
        if (sched_writable_depth == 0)
            mpu_set_region0_ap(RASR_AP_RO);
    }
    irq_restore(flag);
}

// Forcibly clamp the region back to read-only and reset the depth
// counter to 0. Called by sched_main right after its setjmp returns
// non-zero — a try_shutdown longjmp can bypass the matching end()
// (e.g. sched_add_timer's "Timer too close" path longjmps with the
// window still open). Without this, the post-shutdown run_tasks loop
// would run with a stale non-zero depth and the protection would
// never re-engage.
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
    // Sanity: linker should have given us a 128-byte block aligned to 128 B.
    // If the section grows or shifts, the assertion failure here flags it
    // before any silent MPU mis-protection.
    if (end - start != 128u || (start & 127u) != 0u)
        shutdown(".sched_protected size/alignment mismatch");

    // Disable MPU during config (writes to MPU regs are otherwise UB).
    MPU->CTRL = 0;
    __DSB();
    __ISB();

    // Region 0: .sched_protected, 128 B, read-only, no-execute.
    MPU->RNR  = 0;
    MPU->RBAR = start;        // VALID bit clear: region number from RNR
    MPU->RASR = RASR_ENABLE
              | RASR_SIZE_128B
              | RASR_AP_RO
              | RASR_TEX0_C_B0_S
              | RASR_XN;

    // PRIVDEFENA=1: default memory map applies to all addresses outside
    // configured regions. ENABLE=1: turn the MPU on.
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
    __DSB();
    __ISB();

    // Promote MPU violations to MemManage exception (default routes to
    // HardFault). fault_handler.c installs the MemManage trampoline; we
    // enable the bit here too so violations during early-init (before
    // fault_handler_init's DECL_INIT runs) still go through the proper
    // handler. Idempotent — fault_handler_init sets the same bit later.
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}
