// src/generic/runtime_bench.c
//
// Bench storage + command logic. Selected by CONFIG_RUNTIME_BENCH.
// SWSR invariant per runtime_bench.h.

#include <stdint.h>
#include "autoconf.h"
#include "board/internal.h"      // NVIC_*, IWDG, OTG_HS_IRQn, USART2_IRQn
#include "board/misc.h"          // timer_read_time, timer_from_us
#include "command.h"             // sendf, DECL_COMMAND
#include "generic/runtime_bench.h"

// H7 CMSIS only defines IWDG1/IWDG2; map the generic name to IWDG1
// (matching src/stm32/watchdog.c's pattern) so the bench-loop kick
// compiles cleanly.
#if CONFIG_MACH_STM32H7
#define IWDG IWDG1
#endif

// On H7 the bench buffer is placed in AXI SRAM so the 1 KB does not eat
// into the 128 KB DTCM. Other targets land in regular bss.
#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss")))
#endif
volatile uint32_t runtime_bench_samples_buf[RUNTIME_BENCH_MAX_SAMPLES];
volatile uint16_t runtime_bench_count = 0;
volatile uint16_t runtime_bench_target = 0;
volatile uint8_t  runtime_bench_isolate = 0;

// Strong override of the weak no-op in src/runtime_tick_weak.c.
void
runtime_bench_capture(uint32_t cycles_delta)
{
    if (runtime_bench_count < runtime_bench_target) {
        runtime_bench_samples_buf[runtime_bench_count] = cycles_delta;
        runtime_bench_count++;
    }
}

// Externs into the runtime FFI surface — names retained as-is in this task;
// renamed (if at all) by Task 6's FFI sweep.
extern volatile uint8_t kalico_liveness_ok;     // src/stm32/watchdog.c
extern void* kalico_rt_handle;                  // src/runtime_tick.c

// Bench error codes — all sites use the canonical sendf format
// `runtime_bench_done count=%hu error=%i` per Klipper's one-format-per-message
// rule (compile_time_request rejects format conflicts).
#define RUNTIME_BENCH_OK             0
#define RUNTIME_BENCH_ERR_NOT_INIT  -7
#define RUNTIME_BENCH_ERR_BELOW_WARMUP -4
#define RUNTIME_BENCH_ERR_LIVENESS  -100
#define RUNTIME_BENCH_ERR_ISR_TIMEOUT -101

#if CONFIG_MACH_STM32H7
void
command_runtime_bench_run(uint32_t *args)
{
    if (!kalico_rt_handle) {
        sendf("runtime_bench_done count=%hu error=%i", 0, RUNTIME_BENCH_ERR_NOT_INIT);
        return;
    }

    // Liveness pre-check (round-4 review): if the runtime had already
    // tripped a liveness fault before we got here, manually kicking IWDG
    // inside the bench loop would mask it. Refuse to bench in that case.
    if (!kalico_liveness_ok) {
        sendf("runtime_bench_done count=%hu error=%i", 0, RUNTIME_BENCH_ERR_LIVENESS);
        return;
    }

    uint8_t isolate = args[0];
    uint16_t samples = args[1];
    if (samples > RUNTIME_BENCH_MAX_SAMPLES) samples = RUNTIME_BENCH_MAX_SAMPLES;

    if (isolate) {
        // Selectively mask: USB OTG_HS (Octopus Pro's H723 has only the OTG_HS
        // controller; Klipper aliases it as OTG_IRQn elsewhere) + USART2 (active
        // console). Leave TIM5 (the kalico ISR) and SysTick alone. The implementer
        // MUST verify which IRQs are active in the current build before relying on
        // the masked list — picking the wrong IRQ silently biases Pass A toward
        // overly-optimistic numbers.
        // Cross-check with `arm-none-eabi-objdump -d klipper.elf | grep -E 'IRQ|Handler'`
        // to confirm the IRQ vector names actually present in the firmware image.
        NVIC_DisableIRQ(OTG_HS_IRQn);
        NVIC_DisableIRQ(USART2_IRQn);
    }

    runtime_bench_count = 0;
    runtime_bench_target = samples;
    runtime_bench_isolate = isolate;

    // Wait for the ISR to fill the buffer with a watchdog-respecting timeout.
    // Worst case: 25 µs/sample × 1024 = 25.6 ms. We allow 100 ms before
    // bailing out, and we kick the IWDG ourselves during the wait so we
    // don't trip Klipper's watchdog from foreground starvation. Note: the
    // liveness-heartbeat counter does freeze for the duration of this wait,
    // but that's bounded and known — it's only used during Surface-C bring-up.
    uint32_t start = timer_read_time();
    uint32_t timeout_ticks = timer_from_us(100000);  // 100 ms
    while (runtime_bench_count < runtime_bench_target) {
        // Manually kick the IWDG (foreground watchdog_reset would otherwise
        // get pre-empted by our spin and starve). Spec §5.7 — `kalico_liveness_ok`
        // is set true here because we KNOW the runtime is healthy; the gate
        // is only meaningful for unattended operation.
        IWDG->KR = 0xAAAA;
        if ((uint32_t)(timer_read_time() - start) > timeout_ticks) {
            // ISR didn't fill the buffer — TIM5 stalled or NVIC mask wrong.
            runtime_bench_target = 0;  // tell ISR to stop bracketing
            sendf("runtime_bench_done count=%hu error=%i",
                  runtime_bench_count, RUNTIME_BENCH_ERR_ISR_TIMEOUT);
            if (isolate) {
                NVIC_EnableIRQ(OTG_HS_IRQn);
                NVIC_EnableIRQ(USART2_IRQn);
            }
            return;
        }
    }

    if (isolate) {
        NVIC_EnableIRQ(OTG_HS_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);
    }

    // Discard the first 8 samples (warm-up: cache fill, branch predictor,
    // FPU lazy-stacking on first vector_eval). Spec §6.4 hardened methodology.
    // Underflow guard: refuse if caller didn't request enough samples.
    const uint16_t WARMUP_SKIP = 8;
    if (samples <= WARMUP_SKIP) {
        sendf("runtime_bench_done count=%hu error=%i", 0,
              RUNTIME_BENCH_ERR_BELOW_WARMUP);
        return;
    }

    // Emit one Klipper-framed `runtime_bench_sample value=N` response per
    // measurement (after warmup). The USB-CDC transmit_buf is 192 B and
    // console_sendf silently drops when full (usb_cdc.c:71-74). A tight
    // sendf loop holds the foreground task and starves usb_bulk_in_task,
    // so after ~21 framed messages every subsequent send is dropped —
    // including the trailing runtime_bench_done. Drain by calling the bulk
    // task directly between sends, kicking IWDG so the watchdog doesn't
    // trip during a 1024-sample emit (~10–20 ms wall time).
#if CONFIG_USBSERIAL
    extern void udelay(uint32_t usecs);
    extern void usb_bulk_in_task(void);
    extern void usb_notify_bulk_in(void);
#endif
    for (uint16_t i = WARMUP_SKIP; i < samples; i++) {
        sendf("runtime_bench_sample value=%u", runtime_bench_samples_buf[i]);
#if CONFIG_USBSERIAL
        // Re-arm the wake (sched_check_wake clears it) so usb_bulk_in_task
        // attempts a drain regardless of prior state. udelay yields enough
        // wall time for the USB IN IRQ to ACK the previous packet, freeing
        // the endpoint FIFO so the next usb_send_bulk_in succeeds.
        usb_notify_bulk_in();
        usb_bulk_in_task();
        udelay(80);
#endif
        IWDG->KR = 0xAAAA;
    }
    sendf("runtime_bench_done count=%hu error=%i",
          (uint16_t)(samples - WARMUP_SKIP), RUNTIME_BENCH_OK);
#if CONFIG_USBSERIAL
    usb_notify_bulk_in();
    usb_bulk_in_task();
#endif
}
DECL_COMMAND(command_runtime_bench_run, "runtime_bench_run isolate=%c samples=%hu");
#endif // CONFIG_MACH_STM32H7
