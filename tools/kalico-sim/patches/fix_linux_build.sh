#!/bin/bash
# fix_linux_build.sh — patch the sota-motion source tree for MACH_LINUX builds.
#
# Two independent fixes:
#
# 1. Linux stubs for MPU / Cortex-M scheduler symbols.
#    sched.c calls sched_writable_begin/end/reset (defined in
#    generic/mpu_protect.c) and references timer_wrap_event (defined in
#    generic/armcm_timer.c) as a function-pointer initialiser. Both files
#    are gated to STM32 builds and are never compiled for MACH_LINUX.
#    On Linux the MPU window is a no-op (there is no MPU), so all three
#    sched_writable_* stubs are empty; the timer_wrap_event stub
#    reschedules exactly as the real Cortex-M implementation does.
#
# 2. Rename kalico_runtime_modulated_tick → kalico_runtime_tick_sample.
#    The Rust API was renamed in the 2026-05-20 stepping redesign
#    (see rust/kalico-c-api/src/runtime_ffi.rs, near line 629) but the
#    host-tick driver (src/linux/runtime_tick_host.c) still calls the
#    old name.  A single sed substitution aligns the call site with the
#    exported symbol.
#
# Usage (called from Dockerfile before make):
#   bash tools/kalico-sim/patches/fix_linux_build.sh
#
# Idempotent: re-running after a partial apply is safe.

set -euo pipefail

REPO_ROOT="${1:-/kalico}"

# ---------------------------------------------------------------------------
# Fix 1 — Linux stubs for mpu_protect / armcm_timer symbols
# ---------------------------------------------------------------------------
STUB_FILE="$REPO_ROOT/src/linux/fault_handler_stub.c"

# Guard: only append if the marker comment is absent.
if grep -q "sched_writable_begin" "$STUB_FILE" 2>/dev/null; then
    echo "fix_linux_build: MPU/timer stubs already present, skipping."
else
    echo "fix_linux_build: appending MPU/timer stubs to $STUB_FILE"
    cat >> "$STUB_FILE" <<'EOF'

// ---------------------------------------------------------------------------
// Linux stubs for Cortex-M-only MPU and timer symbols
// ---------------------------------------------------------------------------
//
// sched_writable_begin / sched_writable_end / sched_writable_reset
// ----------------------------------------------------------------
// Defined in src/generic/mpu_protect.c, which is only compiled for STM32
// targets.  On Linux there is no hardware MPU, so the "writable window"
// over .sched_protected is a no-op.  The section attribute placed on
// SchedState is harmless (GCC/Clang silently accept unknown section names
// on ELF targets) and the memory is always writable.
//
// timer_wrap_event
// ----------------
// Defined in src/generic/armcm_timer.c, which is also STM32-only.  The
// function is used as a function-pointer initialiser in SchedState.wrap_timer
// (sched.c line ~58).  The Linux timer backend (linux/timer.c) never calls
// timer_reset, so the wrap_timer is never scheduled; however the symbol must
// exist for the initialiser to link.  The implementation matches the real
// Cortex-M version exactly: reschedule the wrap timer 0xffffff ticks out.

#include "sched.h" // struct timer, SF_RESCHEDULE

void sched_writable_begin(void) {}
void sched_writable_end(void) {}
void sched_writable_reset(void) {}

uint_fast8_t
timer_wrap_event(struct timer *t)
{
    t->waketime += 0xffffff;
    return SF_RESCHEDULE;
}

// Diagnostics counters — defined in src/generic/fault_handler.c (STM32-only).
// runtime_tick.c:runtime_status_drain calls these to report tick stats.
// On Linux, return 0 (no hardware cycle counter).
uint32_t diag_get_rt_tick_cycles_max(void) { return 0; }
uint32_t diag_get_rt_tick_count(void) { return 0; }
EOF
fi

# ---------------------------------------------------------------------------
# Fix 2 — rename kalico_runtime_modulated_tick call site
# ---------------------------------------------------------------------------
TICK_FILE="$REPO_ROOT/src/linux/runtime_tick_host.c"

# Guard: only patch if the old symbol is still present.
if grep -q "kalico_runtime_modulated_tick" "$TICK_FILE" 2>/dev/null; then
    echo "fix_linux_build: renaming kalico_runtime_modulated_tick in $TICK_FILE"
    sed -i \
        's/kalico_runtime_modulated_tick/kalico_runtime_tick_sample/g' \
        "$TICK_FILE"
else
    echo "fix_linux_build: runtime_tick_host.c already uses kalico_runtime_tick_sample, skipping."
fi

echo "fix_linux_build: done."
