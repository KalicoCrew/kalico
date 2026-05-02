---
date: 2026-05-02
context: Step 7-D Phase 2a (first hardware contact, H723 on Trident)
status: Gate A in progress — protocol path proven, stream lifecycle next
---

# Where we left off — Step 7-D Phase 2a

## What works now

1. **H723 firmware build + flash from `~/klipper@sota-motion`.** `bash tools/build_production_firmware.sh` builds (62 KB / 256 KB ROM); `make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00` flashes via DFU. The `Error during download get_status` after `Submitting leave request` is the standard benign STM32-DFU quirk; the flash actually succeeds.
2. **`CONFIG_INITIAL_PINS="PA8"`** is baked into `.config` and verified post-boot — keeps the active-low CPAP fan (`!PA8` in `cpap.cfg`) off during MCU reset.
3. **`tools/kalico_host_io.py` identify works against the real H723.** Three codex-rescue iterations fixed: (a) seq-wrap stale-ACK rewinds at 15→0, (b) reconnect with non-zero MCU expected seq via 16-seq sweep, (c) MCU stall under back-to-back identify floods (added per-response synchronous waiting + same-seq retransmit on stall, mirroring `chelper/serialqueue.c`'s pacing).
4. **`tools/test_h723_first_light.py` ABI rewrite** matches current firmware: `kalico_load_curve version=1 …` (no `weights`); `kalico_push_segment id= x_handle= y_handle= z_handle= e_handle= … e_mode= extrusion_ratio=`.
5. **First-light test reaches push_segment.** Run output:
   ```
   initial: status=IDLE last_err=0
   loaded curve 'straight_line_x' (5000 us)
   post-load: status=IDLE last_err=0
   FAIL: post-push status=IDLE last_err=0 (expected RUNNING or DRAINED)
   ```
   Identify, query_status, load_curve, push_segment all accepted by the MCU.

## What's blocking Gate A PASS

1. **Stream-arm sequence missing in `test_h723_first_light.py`.** After push_segment, the test expects engine status `RUNNING` or `DRAINED` but it stays `IDLE`. The current firmware ABI requires `kalico_stream_open` + `kalico_stream_arm` + `kalico_stream_flush` to start the engine processing queued segments. The test doesn't issue these.
2. **No idempotent teardown between runs.** Re-running back-to-back without power-cycling the MCU yields `kalico_load_curve_response result=-3` (curve pool slots from the prior run still occupied). Either the test needs to release/clear slots on entry, or each invocation needs a power cycle.

## How to pick up

Quick path to Gate A PASS:

1. Add `stream_open(stream_id=0)` → push_segment → `stream_arm(t_start_t0_lo/hi=…, arm_lead_cycles=…)` → `stream_flush()` → poll `query_status` for `RUNNING` to `DRAINED` transition.
   - Reference encoding in `rust/kalico-host-rt/src/producer.rs` and `rust/motion-bridge/src/router_transport.rs`.
   - `arm_lead_cycles` is the number of MCU cycles between arm-time and t_start; pick something like `0.001 * CLOCK_FREQ` (1 ms lead) for a smoke test.
2. On entry, do a benign reset: `kalico_query_pool_state slot=N` for each slot to detect occupied state, then either fail-fast with a clear message or send `kalico_stream_terminal segment_id=…` to flush. Easiest: have the test `kalico_set_homed` + close/open stream to reset, OR just power-cycle and document the requirement.

Then continue down the 7-D plan to Gate B (cycle count) and Gate C (M1 host-stall soak) — both already use the now-working `kalico_host_io`, so they should work once first-light passes.

## Reference

- Plan: `docs/superpowers/plans/2026-05-02-step-7d-hardware-bringup.md`
- Spec: `docs/superpowers/specs/2026-05-02-step-7d-hardware-bringup-design.md`
- Pi: `dderg@trident.local`, repo at `~/klipper`, branch `sota-motion`. Treat local Mac as source of truth; if Pi diverges, `git reset --hard origin/sota-motion`.
- H723 USB id: `usb-Klipper_stm32h723xx_490017000851323235363233-if00`
- Side-MCUs to leave alone: `usb-Klipper_stm32f446xx_…` (F446 for Z, Phase 3), `usb-Beacon_…`.

## Codex agent IDs (in case of follow-up)

- `abae5ec846247e689` — first kalico_host_io fix (seq-wrap stale-ACK)
- `a12bb01420332b421` — end-of-dict / zlib EOF detection (superseded, real bug was elsewhere)
- `a990b2ae6290f942f` — pacing + same-seq retransmit (the actual fix)
- `afeba7fbda7416f04` — first-light ABI rewrite
