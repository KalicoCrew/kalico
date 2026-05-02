---
date: 2026-05-02
context: Step 7-D Phase 2a (first hardware contact, H723 on Trident)
status: Gate A + Gate B PASS on real H723. Gate B M2 soak DEFERRED (firmware drain-in-bench follow-up). Gate C (M1 host-stall, on Pi) next.
---

# Where we left off — Step 7-D Phase 2a

## Gate A PASS (2026-05-02)

End-to-end first-light run on real H723 (`usb-Klipper_stm32h723xx_490017000851323235363233`):

```
initial: status=IDLE last_err=0
set_homed ok
loaded curve 'straight_line_x' (5000 us)
stream_open ok (stream_id=0)
pushed seg 1 (t_start=5200000000, ~10.0 s of MCU uptime)
stream_arm ok (arm_lead_cycles=520000)
stream_terminal ok (segment_id=1)
drain: observed_running=True observed_drained=True last_err=0
PASS
```

`tools/test_h723_first_light.py` now drives the full §8.3 stream lifecycle:
`set_homed` → pool-slot probe → `load_curve` (X/Y/E) → `stream_open` → `push_segment` → `stream_arm` → `stream_terminal` → poll until DRAINED.

## Findings worth carrying forward

1. **`kalico_clock_sync_request` returns 0 until the engine ISR has ticked at least once.** `widened_now` is published by the engine ISR (`engine.rs::tick → publish_widened_now`), and TIM5 only fires after the first `push_segment` enables it via the producer protocol. From a fresh boot, the host literally cannot sample a real MCU clock — a `clock_sync_request` issued before the first push gets a 0 response. Implication: the priming segment's `t_start` has to be picked as an absolute value large enough to be in the MCU's future even with several seconds of uptime. The test uses `t_start = 10 s × clock_freq = 5.2e9` ticks; the test then waits ~10 s for the engine to actually fire. A warmup-cycle workaround (push with far-future `t_start`, flush, re-push) hit a USB-CDC stall after the second `load_curve` burst (Errno 5 on subsequent writes) — worth investigating but not on the Gate A critical path.

2. **`flush()` does not rewrite `runtime_status`.** `stream::flush` parks the ISR via `force_idle`, drains the queue, calls `engine.clear_current()`, then resumes the ISR. The next tick has `current=None` + queue empty + `stream_open=false`, hits `engine.rs:324` and returns `Ok(())` without writing `runtime_status`. So `kalico_query_status` after a flush can return a stale RUNNING label even when the engine is genuinely idle. Use `last_err` as the post-flush signal of choice.

3. **`runtime_tick.c` disables TIM5 on entry to DRAINED to save CPU** (`runtime_tick.c:334` — also stops USART2 starvation under Renode). After DRAINED, the ISR is dead and `stream_flush`'s 1 ms `force_idle` ack handshake will fail with `KALICO_ERR_LIVENESS_STALLED` (-132). DRAINED is itself the clean end-state on the happy path; flush is for mid-stream aborts. The test therefore does *not* issue a trailing flush.

4. **Re-runs without power-cycle should now work.** The §8.5 flush path resets per-slot `last_retired_gen = current_gen`, freeing pool slots. The natural-drain path also retires curve handles via SEGMENT_END trace samples. The test's pool-state probe at startup turns the old `load_curve result=-3` failure into a clear "power-cycle required" message — but in practice, after a clean DRAINED run, slots should be reusable.

## Gate B PASS (2026-05-02)

| | min | p50 | **p99** | budget |
|--|--|--|--|--|
| Pass A (isolate=1, USB+USART masked) | 4.165 µs | 4.181 µs | **4.437 µs** | 15 µs |
| Pass B (isolate=0, full IRQ load) | 4.165 µs | 4.165 µs | **4.188 µs** | 15 µs |

3.4× headroom at p99. Pass A and Pass B were collected in separate
invocations (one per power-cycle): Pass A's ~25 ms busy-wait stalls the
foreground heartbeat, tripping `kalico_liveness_ok=false` so a second
bench in the same boot refuses with `KALICO_BENCH_ERR_LIVENESS`.

## Findings worth carrying forward (Gate B)

5. **Klipper's USB-CDC `transmit_buf` is 192 B and `console_sendf` silently drops on full** (`src/generic/usb_cdc.c:71-74`). The bench's tight `sendf` loop emitting `kalico_bench_sample` responses (~9 B framed) overflowed after exactly 21 messages — host saw 21/N and never the trailing `kalico_bench_done`. Fix in `src/runtime_tick.c`: between sends, call `usb_bulk_in_task()` + `udelay(80)` so the USB IRQ can ACK and the FIFO drains. Same drain pattern is going to be needed for any future high-rate output stream (telemetry, traces, …) that originates from a foreground task.

6. **The bench command starves `runtime_drain` for the duration of the busy-wait + sample-emit loop** (~105 ms). The `TraceSample` SPSC ring is 1199 entries (`rust/runtime/src/trace.rs:18`); at the trace producer's event-sampling rate this is enough for a single bench round but cumulatively overflows after a couple. Ring overflow → engine FAULT → `kalico_liveness_ok=0` (sticky). This breaks the M2 977-round soak as currently structured.

7. **Cycle-count tool initial host script default `--clock-freq 180000000` was wrong for H723.** CYCCNT runs at the CPU clock = `CONFIG_CLOCK_FREQ` = 520 MHz on H723 (PLL DIVP=1, no HPRE divide for the CPU domain). 180 MHz is an H743/F4x leftover. Now defaults to 520 MHz.

## How to pick up — Gate B M2 (deferred) / Gate C

**Gate B M2 (deferred).** Pick one of:
- Firmware fix: drain trace inside `command_kalico_bench_run` between sendfs (call `kalico_runtime_drain_trace` periodically). Cleanest.
- Or: pause trace producer during bench (Rust-side flag).
- Or: increase `TRACE_RING_N` to ≥ 4× current.

Then re-run with `--samples 1024 --m2-rounds 977 --m2-stir-protocol`; record `WORST_ISR_CYCLES` / `WORST_ISR_US` in `docs/research/step5-h723-cycle-budget.md`.

**Gate C (M1 host-stall soak, 30 min, on the Pi 5).** Plan ref: `docs/superpowers/plans/2026-05-02-step-7d-hardware-bringup.md`. The Pi 5 already has the H723 wired up. Run `python3 tools/measure_m1_host_stall.py --port /dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00 --hours 0.5 --report /tmp/m1.json` from the Pi and record `p99_us` / `p9999_us` / `max_us` in `docs/research/step6-buffer-budget-measurements.md`. The host-stall soak does not exercise the bench command, so the M2 trace-overflow issue does not block it.

## Reference

- Plan: `docs/superpowers/plans/2026-05-02-step-7d-hardware-bringup.md`
- Spec: `docs/superpowers/specs/2026-05-02-step-7d-hardware-bringup-design.md`
- Pi: `dderg@trident.local`, repo at `~/klipper`, branch `sota-motion`. Treat local Mac as source of truth; if Pi diverges, `git reset --hard origin/sota-motion`.
- H723 USB id: `usb-Klipper_stm32h723xx_490017000851323235363233-if00`
- Side-MCUs to leave alone: `usb-Klipper_stm32f446xx_…` (F446 for Z, Phase 3), `usb-Beacon_…`.
- To run the test: `sudo systemctl stop klipper` (klippy holds the USB port), then `python3 tools/test_h723_first_light.py --port <H723> --clock-freq 520000000 -v`. Restart klipper after.

## Codex agent IDs (in case of follow-up)

- `abae5ec846247e689` — first kalico_host_io fix (seq-wrap stale-ACK)
- `a12bb01420332b421` — end-of-dict / zlib EOF detection (superseded, real bug was elsewhere)
- `a990b2ae6290f942f` — pacing + same-seq retransmit (the actual fix)
- `afeba7fbda7416f04` — first-light ABI rewrite
