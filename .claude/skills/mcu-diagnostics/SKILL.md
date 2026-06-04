---
name: mcu-diagnostics
description: Use when reading MCU diagnostics / crash forensics from the kalico structured logs — why an MCU reset/crashed/stalled/wedged, dumping the live MCU diag on demand (KALICO_DIAG_DUMP), reading the event-ring play-by-play, decoding a fault PC / reset cause / USB-ISR hog, or checking for hiccups after a manual test. Covers the events/*.jsonl store and the runtime.*/diag.* event catalog. For general LogsQL filtering use query-logs.
---

# Reading MCU diagnostics & crash forensics

The MCU emits structured records (no `printf`) over `KALICO_MSG_LOG` → the host
writes them to `~/printer_data/logs/events/<source>.jsonl`. **This is the source
of truth for MCU diagnostics — it supersedes the old `klippy.log` text dump.**

Sources: `mcu` (H7 / main), `bottom` (F4) — the value is each MCU's klippy name.
Also present: `host-py` (mirrors klippy.log content, structured), `host-rust`
(motion-bridge). Rotating: 32 MB × 5 backups (`.1`..`.5`) per source.

**For querying by session/print/level/field, use the `query-logs` skill** (LogsQL
over VictoriaLogs). This skill covers the MCU-specific *features and event
catalog* and direct raw-file reading.

## Three ways MCU diag reaches the store

| Tier | Trigger | What you get |
|------|---------|--------------|
| **Live faults** | engine raises a fault | `runtime.fault_latched` immediately (with `code_name`) |
| **On-demand dump** | `KALICO_DIAG_DUMP` gcode | full live diag snapshot now, **no reset** |
| **Crash replay** | the boot *after* a reset | prior-boot crash forensics (auto, once, at axis-config time) |

The MCU only keeps the **last 32 events** in its ring (overwrites oldest) plus
whole-run counters/maxima. So dump *while evidence is fresh*; a hiccup from hour 1
of a long run is gone by hour 3.

## KALICO_DIAG_DUMP — check live state without a reset

Run it from the console, or via Moonraker:
```bash
ssh trident "curl -s -X POST http://localhost:7125/printer/gcode/script \
  -H 'Content-Type: application/json' -d '{\"script\":\"KALICO_DIAG_DUMP\"}'"
```
It sends `kalico_diag_dump` to every MCU; each emits (all `debug`):
`runtime.diag_dump` (header: `uptime_us`, `ring_seq`) + `runtime.isr_phase` /
`block_source` / `tim5_ia` + `runtime.fg_freeze` (if a stall latched) + the live
event ring as `diag.*`, oldest-first. Then read the tail (recipe below).

Manual-test flow: run your test → `KALICO_DIAG_DUMP` → read `events/<mcu>.jsonl`.

## Reading "why did it crash?"

After a reset the next boot replays the prior-boot forensics automatically (once,
at axis-config time). **Start at `block_source`, NOT `mcu_reset`** — the reset
cause bits are usually masked (see gotcha), so they are *not* the cause. Read the
rest as a story:

- `runtime.block_source` `usb_burst`/`stepout_burst` (DWT cycles) — **who hogged
  the CPU; this is the usual root cause.** A burst > ~2× the sample period starves
  the foreground — period ≈ 52000 cyc (H7), ≈ 9000 cyc (F4), so > ~105k / ~18k is
  bad. ÷ core freq (H7 ~520 MHz, F4 ~180 MHz) → µs.
- `runtime.tim5_ia` `min`/`max` vs the sample period (≈52000 H7 / ≈9000 F4 cyc) —
  was the **engine** itself starved? `min ≈ period` & short ISRs ⇒ engine innocent.
- `runtime.isr_phase` — which engine phase ran (RT_PHASE_*; 9 = ISR_EXIT = engine
  was exiting cleanly).
- `runtime.fg_freeze` `pc`/`stall_ticks` — where the foreground hung (`addr2line`).
- `runtime.hard_fault` + `runtime.fault_status` — only for a real CPU fault
  (`exc_kind`, PC, LR, CFSR, HFSR).
- `diag.*` ring — the lead-up timeline (USB gaps, TIM5/OTG long ISRs, TX drops,
  engine transitions, rust faults).

Decode a `pc` (from `fg_freeze`/`hard_fault`) — the elf must match the flashed build:
`ssh trident "arm-none-eabi-addr2line -e ~/klipper/out/klipper.elf <pc-as-hex>"`
(the `arg0` values are decimal in the log; convert to hex first).

**Gotcha:** `block_source`/`tim5_ia` are **whole-run maxima since boot**, not
instantaneous. In a *crash-replay* they're the doomed run's worst → the real
cause. In a *live `KALICO_DIAG_DUMP`* a big `usb_burst` is often just the
connect-time spike, not "now" — trust the `diag.*` ring for the recent timeline.

**Gotcha:** klippy's connect-reset masks the immediate RCC cause, so the crash
report is gated on a per-run freeze flag, not `runtime.mcu_reset`'s `cause bits`
(which may show SFTRST). `iwdg_resets` is a *cumulative* counter, not this crash.

## Event catalog

`level`: `trace|debug|warn|error`. Numeric payload in `arg0`/`arg1`; faults also
carry `code` + `code_name`. Canonical table: `rust/runtime/src/log_codes.rs`.

| event | args | meaning |
|-------|------|---------|
| `runtime.fault_latched` | code=FaultCode, arg0=detail | engine fault (live) |
| `runtime.mcu_ready` | — | boot marker, drain online |
| `runtime.log_drops` | arg0=dropped | log-ring overflow (fail-loud) |
| `runtime.mcu_reset` | arg0=RCC bits, arg1=cum. IWDG | reset marker — **cause bits often masked; not the cause (use `block_source`)** |
| `runtime.hard_fault` | code=exc, arg0=pc, arg1=lr | CPU fault |
| `runtime.fault_status` | arg0=cfsr, arg1=hfsr | CPU fault regs |
| `runtime.fg_freeze` | arg0=pc, arg1=stall_ticks | foreground hung |
| `runtime.rt_progress` | arg0=packed, arg1=fault_count | engine progress at crash |
| `runtime.last_dispatch` | arg0=func, arg1=addr | last scheduler callback |
| `runtime.isr_phase` | arg0=RT_PHASE_*, arg1=ring_overflow | engine ISR phase (high `ring_overflow` = heavy event churn) |
| `runtime.block_source` | arg0=usb_burst, arg1=stepout_burst (cyc) | CPU hog |
| `runtime.tim5_ia` | arg0=min, arg1=max (cyc) | TIM5 inter-arrival |
| `runtime.diag_dump` | arg0=uptime_us, arg1=ring_seq | live-dump header |
| `diag.tim5_long` / `diag.otg_long` | arg0=dur cyc, arg1=enter | long ISR |
| `diag.usb_in_gap` / `diag.usb_out_gap` | arg0=gap, arg1=prev_t | USB stall |
| `diag.tx_drop_kalico` / `diag.tx_drop_klipper` | arg0=len/max, arg1=tpos | dropped TX |
| `diag.engine_xition` | arg0=(prev<<8\|new), arg1=samples | engine state change |
| `diag.rust_fault` | arg0=err, arg1=detail | rust fault in the ring |
| `motion.piece_start_past` / `motion.ring_full` | … | motion engine |
| `tick.interval_exceeded` / `tick.underrun` | … | tick ISR |
| `endstop.trip` / `endstop.arm_timeout` | … | endstop |

## Reading the raw files

Fetch a snapshot first (don't analyze the live file):
```bash
scp trident:'~/printer_data/logs/events/mcu.jsonl' /tmp/mcu-$(date +%s).jsonl
```
Filter (no jq dependency) — level + event + message, newest activity last:
```bash
tail -400 /tmp/mcu-*.jsonl | python3 -c '
import sys,json
for l in sys.stdin:
 l=l.strip()
 if not l: continue
 try: d=json.loads(l)
 except: continue
 ev=d.get("event","")
 if ev.startswith("runtime.") or ev.startswith("diag."):
  print(d.get("level"), ev, "|", d.get("_msg",""))'
```
By one event: `grep -a '"event":"runtime.block_source"' /tmp/mcu-*.jsonl`.
Each boot's forensics follow a `runtime.mcu_ready` marker; the **last** `mcu_ready`
in the file is the current boot, so the crash story is the `runtime.*`/`diag.*`
block right after the most recent `mcu_ready`. A record looks like:
`{"_time":"…","level":"warn","source":"mcu","event":"runtime.block_source","arg0":137808,"arg1":0,"_msg":"block usb_burst=137808 cyc stepout_burst=0 cyc"}`

For indexed queries (by `session_id`, `print_id`, numeric ranges, code
resolution), use **`query-logs`** instead of grepping.

## When klippy.log is still useful

The structured store replaces klippy.log for MCU diagnostics. klippy.log keeps:
host-side Python tracebacks, and the verbose `prior_diag_summary_*` deep-debug
text (TIM5/RT histograms, USB register snapshots, task heartbeats, `bfar`/`mmfar`)
— fields the structured path doesn't carry, retained as the robust always-works
fallback for when the host structured-logging layer is unavailable.

## Adding new MCU logs

See `CLAUDE.md` → "Observability". Briefly: pick/add an event in
`rust/runtime/src/log_codes.rs` (the wire-stable table), mirror any new C event in
`src/kalico_log.h`, and call `kalico_log_emit(level, subsystem, event, code,
arg0, arg1)` at the emit site. Engine faults go through the `raise_*` helpers in
`rust/runtime/src/fault_helpers.rs` (auto-emit `runtime.fault_latched`).
