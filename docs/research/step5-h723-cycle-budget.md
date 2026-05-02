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
| 2026-05-02 | 7185e1446 | 1 × 1016 (partial) | 2178 | 4.188 | **DEFERRED** |

The full 977-round soak is gated on a follow-up firmware fix. Each
bench round busy-waits for ~25 ms (sample-fill) plus ~80 ms
(sample-emit, with `udelay(80)` between sends to drain USB-CDC). For
~105 ms total the foreground task can't run, so `runtime_drain`
can't pull `TraceSample`s off the SPSC ring. The ring is sized 1199
(`rust/runtime/src/trace.rs:18`) — at the trace producer's
event-sampling rate this is enough for one round but cumulatively
overflows after a couple. Overflow → engine FAULT → sticky
`kalico_liveness_ok=0` → subsequent `kalico_bench_run` invocations
refuse with `KALICO_BENCH_ERR_LIVENESS (-100)` until power-cycle.

Mitigation options for the M2 follow-up:
1. Drain trace inside `command_kalico_bench_run` between sendfs (call
   `kalico_runtime_drain_trace` periodically). Adds inline trace
   responses to the wire during the bench; host parser already
   tolerates this since `kalico_trace` and `kalico_bench_sample` have
   distinct msg IDs.
2. Pause the trace producer for the duration of the bench (Rust
   runtime exposes a "trace_paused" flag the bench can flip).
3. Increase `TRACE_RING_N` to ≥ 4× current to absorb ~120 ms of stall.

Gate B (single-pass cycle budget) is unaffected: p99 4.44 µs (Pass A)
/ 4.19 µs (Pass B) under 15 µs budget. M1 host-stall soak (Gate C, on
the Pi) and M3 clock-sync soak (Phase 3, dual-MCU) cover the
long-horizon tail-latency surface that M2 was secondary on.

Notes:
- Methodology spec: §6.4 (post-warmup-skip + selective-IRQ-mask).
- The DWT->CYCCNT counter runs at the H723 CPU clock = `CONFIG_CLOCK_FREQ`
  = 520 MHz on the Octopus Pro H723 (PLL DIVP=1, no HPRE divide for the
  CPU domain). Cycles → µs via `cycles / clock_freq_hz`.
- Pass A's isolation is *selective*: TIM5 (the kalico ISR itself) and
  SysTick remain enabled. The intent is to bound the runtime tick
  latency under "minimum reasonable" host load, not to characterize
  it under zero load.
