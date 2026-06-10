# Sensorless homing bench debug — night of 2026-06-09/10

Trident bench (H723 main @520MHz, F446 bottom @180MHz, corexy, TMC5160 A/B,
sensorless X via `tmc5160_stepper_x1:virtual_endstop`, DIAG1 jumpered to PG9).
All times UTC. Every finding below is bench-verified; commit hashes on
`sensorless-homing`.

## Outcome

Sensorless X homing works end-to-end: stallguard arms via the provider hooks,
DIAG trips, position reconstructs. Three full successes recorded
(`homing: X trigger=300.0000 overshoot=+0.108/+0.110`), reproducible once the
chain of engine bugs below was cleared. One bug remains open (post-home
retract kills the bottom MCU — see "Open").

## Fault-detail decoding gotcha (cost us hours)

`PieceStartInPast` detail packs `axis_idx << 16 | min(deficit_us, 0xFFFF)`
(`fault_helpers.rs:131`). The recurring "~132ms deficits" (0x20517, 0x2063A)
were actually **axis 2, deficits 1.3–1.6ms**, and 0x1FFFF was **axis 1,
saturated ≥65ms**. When reading `fault_detail`, always split the fields.

## Bugs found and fixed (in discovery order)

1. **Drip budget counted pieces, not time** (`f61762ec6`). `DRIP_BUDGET=4`
   assumed ~25ms pieces; the emitter produces ~0.43ms knots at 100mm/s →
   1.7ms of runway vs a 2–15ms retire-ack loop → every piece after the first
   window arrived late → PieceStartInPast. Fix: `DRIP_WINDOW_SECS=0.050`,
   time-denominated, floor-locked across participants.

2. **Cohort head globally gated the scheduler** (`537e4de55`). `schedule()`
   released pieces in global mint-time order; a window-blocked participant
   stalled ALL queues, so zero-motion follower pieces (Y/Z/E) trickled behind
   X and dumped stale at move end (the "axis 2, 1.3ms" faults). Fix:
   cap-gated keys are skipped, not gating; StallFull (ring full) still gates
   all.

3. **MCU fast heartbeat was a DRIP_BUDGET fossil** (`4f2951b37`,
   `src/runtime_tick.c`). Fired only at ring occupancy ≤4 — unreachable with
   ~115 pieces in flight — so the retirement floor advanced only on the 100ms
   periodic, eating the 50ms window. Fix: 10ms rate-limited retirement
   heartbeat with a sticky pending flag (a rate-limited advance must emit
   later, not be forgotten).

4. **`home_abort` never reconciled position** (`bef776396`). A homing move
   that timed out ("endstop did not trigger") left `commanded_pos` at the
   pre-move value while the engine sat 40mm away; the next home's first piece
   demanded 6400 steps in one sample → -310 StepsPerSampleExceeded. Fix:
   abort drains, extracts trajectory-final position (last Bernstein
   coefficient), reopens the stream there, updates `commanded_pos`.

5. **Free-streamed followers flooded the F446** (`726a8f401`). After fix 2,
   ~1000 zero-motion Z pieces blasted at the bottom in one burst:
   `usb_burst=162628 cyc` (903µs, 18× its sample period) → klipper timer
   starvation → "Timer too close" → bottom in klipper-shutdown answering
   `is_shutdown` to everything → host calls time out 15s each → klippy
   reactor frozen 41s → runtime-event channel overflow (728 drops). Fix:
   coalesce consecutive bit-identical constant pieces at emit (exact merge —
   same cubic); followers now send ~1 piece per segment.

6. **Coalescing emitted host times from a zero-based accumulator**
   (`bfbf14331`). The rewrite of fix 5 dropped `bp.u_start` from
   `host_secs`; curves are NOT 0-based (segments continue in trajectory
   time), so every piece of every non-first segment (the homing decel tail)
   shifted into the MCU's past → PieceStartInPast (axis 1, saturated) → MCU
   shutdown → klippy reactor abort (silent SIGABRT: the EXIT_ON_FAULT paths
   flush stderr but not the non-blocking tracing appender, so the
   explanation died with the process). Fix: carry `bp.u_start` through the
   merge and emit `t0 + u_start + sub_offset`; regression test pins a
   non-zero-based curve.

   Side finding: kernel USB timestamps prove the H7 reset 92ms BEFORE the
   SIGABRT — the abort is the designed reactor reaction to a critical-MCU
   transport loss. When klippy dies abruptly, BOTH MCUs then freeze in their
   TX paths and IWDG-reset ~1s later; their reset-cause forensics
   (`fg_freeze`, `iwdg_resets`) describe the *aftermath*, not the cause.
   Don't chase them first; find what killed klippy.

Also fixed along the way (Python side): stallguard arm/disarm via the
trip-move provider contract (`ad273b1ae`), home_current actually applied
around homing (`637c2e3a8`) — `set_current_for_homing` had no callers since
the homing rework — and a finally-block that masked primary homing errors
with secondary restore errors (`6eb0ea0c7`).

## Open bug: post-home retract enables Z with a bad print_time

Twice observed (02:15 and 03:5x UTC runs): the homing itself succeeds, then
the retract move — the first toolhead move of the session — fires the
pending motor-enable callbacks of ALL kinematic steppers
(`motion_toolhead._fire_active_callbacks`: no per-axis filtering), including
stepper_z/z1/z2 on the bottom MCU. The bottom rejects the enable-pin command
with **"Timer too close"** (scheduled in its past) and shuts down; the
subsequent z-driver tmcuart writes then block 15s each
(`TMC stepper_z2 _do_enable_bridge failed: bridge_call: transport timed
out`) and klippy transitions to shutdown. Intermittent — the 01:12 success
survived its retract.

Suspects, unverified: `enable_print_time = get_last_move_time()` is floored
on the MAIN MCU's estimated print time; the mapping of that print_time into
the bottom's clock domain (secondary clocksync, `clock_adj`) may be stale or
skewed right after a homing sequence. Next step: log the computed
print_time, the bottom's `print_time_to_clock` result, and the bottom's
actual clock at receipt; compare against the H7's. Also decide whether
firing enables for axes that don't move is correct at all (upstream enables
lazily per moving stepper).

## Diagnostics playbook that worked

- `host-rust.jsonl`: `[anchor-decision]`, `[seg0-deficit]`, `[transit-diag]`
  (arrival leads per piece per MCU) — the feed-health ground truth.
- `[KALICO-FAULT]` records: decode `fault_detail` per `fault_helpers.rs`.
- MCU crash replay (`mcu.jsonl`/`bottom.jsonl` after `runtime.mcu_ready`):
  `block_source` usb_burst (F4 bad >~18k cyc), `diag.rust_fault` in the ring.
- Kernel USB timestamps (`journalctl -k`) to order MCU resets vs host death.
- `systemctl`/journal for the process death mode (SIGABRT vs transition).
- py-spy on the live klippy to distinguish parked-greenlet vs blocked-in-rust.
- `arm-none-eabi-addr2line -e out/klipper.elf` on `fg_freeze`/`last_dispatch`
  PCs (rebuild the right MCU's ELF first; `make clean` between).
