# Step 5 H723 cycle-budget measurements

Surface C bring-up template for `tools/test_h723_cycle_count.py`. The host
script runs two passes (A: isolate=1 with USB+USART IRQs masked, B: isolate=0
with full IRQ load), drops the first 8 warmup samples MCU-side, and reports
min / p50 / p99 in microseconds.

The p99 budget gate is `--p99-budget-us` (default 15.0 µs), enforced
host-side in `test_h723_cycle_count.py`.

| date | git SHA | clock freq | Pass A min / p50 / p99 | Pass B min / p50 / p99 | budget | result |
|------|---------|------------|------------------------|------------------------|--------|--------|
| _TBD by Surface C bring-up_ | | | | | | |

Notes:
- Methodology spec: §6.4 (post-warmup-skip + selective-IRQ-mask).
- The DWT->CYCCNT counter runs at the H723 core clock (180 MHz for the
  Octopus Pro target). Cycles → µs via `cycles / clock_freq_hz`.
- Pass A's isolation is *selective*: TIM5 (the kalico ISR itself) and
  SysTick remain enabled. The intent is to bound the runtime tick
  latency under "minimum reasonable" host load, not to characterize
  it under zero load.
