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

7. **Secondary-MCU clocksync never calibrated** (`28d5dddc3` + guard
   `5ed168da5`) — the "Timer too close" root cause. `MotionToolhead.stats()`
   overrode upstream's and dropped the `check_active()` loop, the only
   caller of `SecondarySync.calibrate_clock`. The bottom MCU's print-time
   mapping froze at its connect-time calibration, whose slewing freq
   (`clock_adj=(-1.584, 179700425.5)` vs true 180000693) loses **1.67ms per
   second, forever**. Nothing notices while idle because no clock-scheduled
   command flows to a quiet secondary MCU (bed off ⇒ no PWM re-issues,
   analog self-schedules MCU-side, tmcuart is unscheduled). The first one
   since boot is the Z motor-enable fired by the post-home retract
   (`_fire_active_callbacks` fires pending enables for ALL steppers, not
   just movers); once uptime exceeded ~2.5 minutes it landed >0.25s in the
   bottom's past → "Timer too close" → shutdown → every later bottom call
   blocked 15s (the MCU answers `is_shutdown`, which matches no waiter) →
   reactor frozen → the rest of the 00:15/02:15 symptom set. Fix restores
   `check_active` (also restoring lost-communication detection), skipping
   disconnected non-critical MCUs whose frozen clocksync makes
   `calibrate_clock` divide by zero.

8. **Endstop trip emitted from timer/IRQ context** (`77d77bee1`).
   `endstop_event` called `kalico_native_emit_endstop_trip` directly from
   the timer, racing the shared `tx_buf` and USB transmit cursor against
   foreground senders. The trip clock is still captured in the timer
   (accuracy unchanged); the transport write moved to a `DECL_TASK`,
   mainline's trsync shape. Plus: runtime-event subscriber channel 64→512
   (`5ed168da5`-adjacent) so a stalled reactor doesn't shed sensor
   responses within four seconds.

## Simulator status (kalico-sim, full mode)

A G28-cycling G-code (home, re-home, post-home move, home again) FAILS in
docker full mode with varying timing faults (-308 deficits of a few ms,
-311 tick gaps), even `--privileged`, with non-deterministic survival
times. The pre-tonight baseline image (`kalico-sim-homing-probe-fixed`)
fails identically — pre-existing, matching the skill's own table
("sota-motion: full mode FAIL — catches timing bug"). The kalico-native
engine's realtime deadlines don't survive docker-VM scheduling jitter;
full-mode sim validation of homing likely needs the libvtime virtual-clock
shim wired into the kalico runtime tick. Separate workstream — the real
H723 homed repeatably tonight where the sim faults.

## Known deviations left open (documented, not blocking)

- `motion_toolhead._fire_active_callbacks` enables ALL kinematic steppers on
  any move (upstream enables lazily per stepping motor). Harmless since the
  clocksync fix, but Z holds current after any XY move. A correct filter
  needs kinematics knowledge (corexy X-only ⇒ both A and B motors).
- `kalico-host-rt` response matching (`AwaitingResponse::find_match`) matches
  by response name only; concurrent same-name calls (three tmcuart polls)
  rely on FIFO ordering and eviction timing.
- The `homing.py` G28 is single-pass (no slow second approach). Canonical
  for sensorless; revisit if switch-based homing accuracy matters later.

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
