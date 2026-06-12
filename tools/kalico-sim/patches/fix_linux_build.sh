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

STUB_FILE="$REPO_ROOT/src/linux/fault_handler_stub.c"

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

#include "sched.h"

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

TICK_FILE="$REPO_ROOT/src/linux/runtime_tick_host.c"

if grep -q "kalico_runtime_modulated_tick" "$TICK_FILE" 2>/dev/null; then
    echo "fix_linux_build: renaming kalico_runtime_modulated_tick in $TICK_FILE"
    sed -i \
        's/kalico_runtime_modulated_tick/kalico_runtime_tick_sample/g' \
        "$TICK_FILE"
else
    echo "fix_linux_build: runtime_tick_host.c already uses kalico_runtime_tick_sample, skipping."
fi

# On real MCU hardware the timer-in-past check catches runaway timers — the
# hardware ISR fires reliably so a timer >100ms late is genuinely broken.
# On MACH_LINUX with virtual time, the MCU's virtual clock advances at CPU
# speed and can race arbitrarily far ahead of klippy's clock estimate. No
# fixed threshold is large enough — disable the check entirely in the sim.
TIMER_FILE="$REPO_ROOT/src/linux/timer.c"

if grep -q 'try_shutdown("Rescheduled timer in the past")' "$TIMER_FILE" 2>/dev/null; then
    echo "fix_linux_build: disabling timer-in-past shutdown check"
    sed -i 's/try_shutdown("Rescheduled timer in the past")/((void)0) \/\* sim: disabled \*\//' "$TIMER_FILE"
else
    echo "fix_linux_build: timer-in-past check already disabled, skipping."
fi

# Same vtime-clock-race root cause applies to "Timer too close" in sched.c.
SCHED_FILE="$REPO_ROOT/src/sched.c"

if grep -q 'try_shutdown("Timer too close")' "$SCHED_FILE" 2>/dev/null; then
    echo "fix_linux_build: disabling timer-too-close shutdown check"
    sed -i 's/try_shutdown("Timer too close")/((void)0) \/\* sim: disabled \*\//' "$SCHED_FILE"
else
    echo "fix_linux_build: timer-too-close check already disabled, skipping."
fi

# On MACH_LINUX without vtime, the runtime tick monopolizes the cooperative
# scheduler and irq_wait() never reaches ppoll, so console_wake is never set.
# Removing the gate costs one extra EWOULDBLOCK read() per task round.
CONSOLE_FILE="$REPO_ROOT/src/linux/console.c"

if grep -q 'sched_check_wake(&console_wake)' "$CONSOLE_FILE" 2>/dev/null; then
    echo "fix_linux_build: removing console_wake gate from console_task"
    sed -i '/sched_check_wake(&console_wake)/{N;d;}' "$CONSOLE_FILE"
else
    echo "fix_linux_build: console_wake gate already removed, skipping."
fi

DISPATCH_FILE="$REPO_ROOT/src/kalico_dispatch.c"
if grep -q 'handle_push_segment_calls_total' "$DISPATCH_FILE" 2>/dev/null && \
   ! grep -q 'mcu-push-diag' "$DISPATCH_FILE" 2>/dev/null; then
    echo "fix_linux_build: adding push_segment + kalico_dispatch traces"
    sed -i 's/handle_push_segment_calls_total++;/handle_push_segment_calls_total++; fprintf(stderr, "[mcu-push-diag] push total=%u body_len=%u\\n", handle_push_segment_calls_total, body_len); fflush(stderr);/' "$DISPATCH_FILE"
    if grep -q 'kalico_dispatch_frame' "$DISPATCH_FILE" && ! grep -q 'mcu-kalico-diag' "$DISPATCH_FILE"; then
        sed -i '/uint16_t body_len = payload_len - PER_MESSAGE_HEADER_LEN;/a\
    fprintf(stderr, "[mcu-kalico-diag] dispatch kind=0x%04x body_len=%u\\n", kind, body_len); fflush(stderr);' "$DISPATCH_FILE"
    fi
fi

PHASE_SRC="$REPO_ROOT/src/linux/phase_stepping_spi.c"
PHASE_HDR="$REPO_ROOT/src/linux/phase_stepping_spi.h"

if [ ! -f "$PHASE_SRC" ]; then
    echo "fix_linux_build: copying phase_stepping_spi to src/linux/"
    cp "$REPO_ROOT/src/stm32/phase_stepping_spi.c" "$PHASE_SRC"
    cp "$REPO_ROOT/src/stm32/phase_stepping_spi.h" "$PHASE_HDR"
fi

LINUX_MK="$REPO_ROOT/src/linux/Makefile"
if ! grep -q "phase_stepping_spi" "$LINUX_MK" 2>/dev/null; then
    echo "fix_linux_build: adding phase_stepping_spi.c to Linux Makefile"
    sed -i '/src-y += runtime_commands.c/a src-y += linux/phase_stepping_spi.c' "$LINUX_MK"
fi

RT_CMD="$REPO_ROOT/src/runtime_commands.c"
if ! grep -q 'CONFIG_MACH_LINUX' "$RT_CMD" 2>/dev/null; then
    echo "fix_linux_build: enabling phase stepping for MACH_LINUX in runtime_commands.c"
    sed -i 's/#if CONFIG_MACH_STM32$/#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX/' "$RT_CMD"
    sed -i '/#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX/{n;s|#include "stm32/phase_stepping_spi.h"|#include "stm32/phase_stepping_spi.h"\n#elif CONFIG_MACH_LINUX\n#include "linux/phase_stepping_spi.h"|;}' "$RT_CMD"
fi

# Same vtime-clock-race root cause as the C-side timer checks above: the
# virtual clock can race arbitrarily far ahead of klippy's clock estimate,
# so the Rust runtime's piece-start-in-past grace (200us) and tick-gap
# fault (2x sample period) trip on infrastructure jitter, not real bugs.
# Relax both for the sim build only.
MOTION_CORE="$REPO_ROOT/rust/runtime/src/motion_core.rs"
if grep -q 'MAX_START_IN_PAST_SECS: f32 = 200e-6' "$MOTION_CORE" 2>/dev/null; then
    echo "fix_linux_build: relaxing piece-start-in-past grace for sim"
    sed -i 's/MAX_START_IN_PAST_SECS: f32 = 200e-6/MAX_START_IN_PAST_SECS: f32 = 10.0/' "$MOTION_CORE"
else
    echo "fix_linux_build: piece-start-in-past grace already relaxed, skipping."
fi

TICK_RS="$REPO_ROOT/rust/runtime/src/tick.rs"
if grep -q 'TICK_GAP_FAULT_MULT: u64 = 2;' "$TICK_RS" 2>/dev/null; then
    echo "fix_linux_build: relaxing tick-gap fault for sim"
    sed -i 's/TICK_GAP_FAULT_MULT: u64 = 2;/TICK_GAP_FAULT_MULT: u64 = 1000000;/' "$TICK_RS"
else
    echo "fix_linux_build: tick-gap fault already relaxed, skipping."
fi

echo "fix_linux_build: done."
