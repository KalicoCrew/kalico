# Step 5 H723 cycle-budget measurements

Surface C bring-up template for `tools/test_h723_cycle_count.py`. The host
script runs two passes (A: isolate=1 with USB+USART IRQs masked, B: isolate=0
with full IRQ load), drops the first 8 warmup samples MCU-side, and reports
min / p50 / p99 in microseconds.

The p99 budget gate is `--p99-budget-us` (default 15.0 µs), enforced
host-side in `test_h723_cycle_count.py`.

| date | git SHA | clock freq | Pass A min / p50 / p99 | Pass B min / p50 / p99 | budget | result |
|------|---------|------------|------------------------|------------------------|--------|--------|
| 2026-05-02 | f8e771781 + bench-drain patch | 520 MHz | 4.165 / 4.181 / **4.437** µs | 4.165 / 4.165 / **4.188** µs | 15.0 µs | **PASS** |

Pass A and Pass B above were collected in separate invocations (Pass B via
`--skip-isolate` first, then Pass A by itself). Running both passes in one
invocation trips a sticky liveness fault: Pass A's ~25 ms busy-wait stalls
kalico's foreground heartbeat → `kalico_liveness_ok=false` → Pass B then
refuses with `KALICO_BENCH_ERR_LIVENESS (-100)`. Power-cycle clears it.

The bench command also needed a firmware fix: the original tight `sendf`
loop emitting `kalico_bench_sample` responses overflowed the 192-byte
USB-CDC `transmit_buf` after ~21 messages and silently dropped the rest
(including the trailing `kalico_bench_done`). Fix: between sends, call
`usb_bulk_in_task()` and `udelay(80)` to let the USB IRQ drain the FIFO.

### M2 extended soak (Step 7-D Phase 2a)

| date | git SHA | rounds × samples | WORST_ISR_CYCLES | WORST_ISR_US | result |
|------|---------|-----------------|------------------|--------------|--------|
| _TBD — Step 7-D Phase 2a Gate B (M2)_ | | 977 × 1024 ≈ 1.0M | | | |

Notes:
- Methodology spec: §6.4 (post-warmup-skip + selective-IRQ-mask).
- The DWT->CYCCNT counter runs at the H723 CPU clock = `CONFIG_CLOCK_FREQ`
  = 520 MHz on the Octopus Pro H723 (PLL DIVP=1, no HPRE divide for the
  CPU domain). Cycles → µs via `cycles / clock_freq_hz`.
- Pass A's isolation is *selective*: TIM5 (the kalico ISR itself) and
  SysTick remain enabled. The intent is to bound the runtime tick
  latency under "minimum reasonable" host load, not to characterize
  it under zero load.
