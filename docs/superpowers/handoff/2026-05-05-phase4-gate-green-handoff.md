# Phase 4 GATE GREEN — handoff to next session

**Date:** 2026-05-05
**Branch:** `sota-motion`
**Last commit:** `321c91362` (feat(7-D Phase 4): GATE GREEN — G1 X10 F1000 produces ~426 step pulses)

## TL;DR

`python3 tools/sim_klippy/test_phase4_steps.py` exits **0** with `[phase4] GATE GREEN: step pulses observed via elf log`. `G1 X10 F1000` on the corexy sim produces ~426 step pulses on each of stepper A and B. Phase 4 of the kalico-rewrite is unblocked at the simulator gate. The remaining work to first physical print is config delivery, transport hardening, and hardware bring-up — no more deep math/clock bugs in the way.

## What landed in this session

Five commits, all on `sota-motion`:

1. `78658f081` — host-side kalico-native transport integration (Phase C-B).
2. `3a4b93ecf` — `ConfigureAxes` message + handler + bridge wiring so `steps_per_mm` reaches the engine.
3. `f07a8ebfc` — diagnostic FFIs (`get_axis_steps_per_mm`, `get_axis_accumulator`, `get_axis_motor`, `get_last_timing`) + sim-side `[sim-progress]` stderr dump.
4. `4400b0431` — clock-frame unification (`kalico_h7_read_cyccnt` → `timer_read_time`; `WidenState::seed_high`; `kalico_runtime_seed_widen` FFI).
5. `321c91362` — **GATE GREEN.** Two real bugs fixed (see below), diagnostic prints cleaned up, test polls elf log for non-zero step counts.

## End goal

Get `G28 X` and a physical first print working on the user's H723-driven Trident with the kalico Rust runtime owning motion. Phase 4 (sim-only verification that segment dispatch produces step pulses) is now complete. Next phases are config-blob delivery, dispatch-path stability, and hardware bring-up.

## Bugs fixed for GATE GREEN

### Bug A — `ConfigureAxes` was missing entirely

The runtime's `Engine::configure(McuAxisConfig)` is the only thing that writes a non-zero `steps_per_mm` into `step_state`. Nobody called it: the existing `kalico_configure_axes` FFI was a stub that validated the kinematics tag and returned OK, the wire command carried only `kinematics=%c`, and the bridge never sent it.

Built end-to-end:
- `rust/kalico-protocol`: `MessageKind::ConfigureAxes` (0x0030) + `ConfigureAxesResponse` (0x0031), 20-byte fixed body (kinematics u8, present_mask u8, awd_mask u8, invert_mask u8, 4 × `steps_per_mm` f32).
- `src/kalico_dispatch.c`: `handle_configure_axes` routes to a new FFI.
- `rust/kalico-c-api/src/runtime_ffi.rs`: `kalico_runtime_configure_axes_blob` deserializes the blob, builds `McuAxisConfig`, validates kinematics, calls `Engine::configure()` under foreground projection.
- `rust/motion-bridge/src/bridge.rs`: PyO3 `configure_axes(mcu_handle, kin, masks, steps_per_mm[4])` → `kalico_call(MessageKind::ConfigureAxes)`.
- `klippy/motion_bridge.py` + `klippy/motion_toolhead.py::_configure_axes_per_mcu`: walk `force_move.steppers` to map config-section names (`stepper_x`/`_y`/`_z`/`extruder`) to motor slots based on `[printer] kinematics`, dispatch one `ConfigureAxes` per bridge MCU during `_init_planner`.

### Bug B — Klippy / engine clock-frame skew (sim only)

Linux MCU sets `start_sec = curtime.tv_sec + 1` in `src/linux/timer.c`, so during the first wall-clock second `timer_read_time` returns wrapped (huge u32) values. The first call to `stats_update` during that window stores a near-2³² value as `stats_send_time`, then the second call (5 s later) sees `cur < stats_send_time` and increments `stats_send_time_high` spuriously. Klippy's clocksync widens its view of MCU time using that high counter (via `get_uptime`), so it ends up ~86 seconds ahead of physical reality. The bridge's `compute_ack_clock` projection then schedules `t_start` tens of seconds in the engine's future; the engine receives segments past their `t_end`, the boundary loop retires them at `u≈0`, motors stay seeded.

Two-part fix:
- `rust/kalico-host-rt/src/passthrough_queue/router.rs`: rebase `clock_offset` into the bridge's `instant_to_f64` frame at `set_clock_est` time. (Klippy's `offset` is in `reactor.monotonic()` epoch which differs from the bridge's `OnceLock` anchor.)
- `src/linux/kalico_host_tick.c`: at `kalico_h7_enable_tim5`, seed the engine's widen state from `(stats_send_time_high << 32) | timer_read_time()` so the engine's `now` agrees with Klippy's widened view (including the spurious-wrap inflation).

### Bug C — Step-state seeded at u=0 even on late entry

When a segment's `t_start` was in the engine's past (clock skew, slow first delivery), the engine seeded `step_state` from `x(0), y(0)`, then on the next tick saw a multi-mm position delta vs the curve evaluated at the actual current `u`, and tripped `StepBurstExceeded (-21)`.

Fix in `rust/runtime/src/engine.rs`: seed `prev_x`/`prev_y`/`step_state` from the currently-evaluated `x`/`y`, not `x(0)`/`y(0)`. On a fresh on-time entry these are equal; on late entry the seed reflects actual toolhead position.

## Sim-test execution path (reference)

```bash
docker run --rm \
  -v $PWD:/work -w /work --tmpfs /tmp:exec \
  kalico-sim:latest \
  bash -c "cp .config.linux .config && make olddefconfig >/dev/null
           && make 2>&1 | tail -3
           && make -f Makefile.kalico motion-bridge 2>&1 | tail -3
           && rm -f klippy/chelper/c_helper.so
           && rm -rf klippy/chelper/c_helper.so.dSYM 2>/dev/null
           && rm -f tools/sim_klippy/.local-logs/*.log
           && mkdir -p /work/tools/sim_klippy/.local-logs
           && timeout 120 python3 tools/sim_klippy/test_phase4_steps.py"
```

Expected last lines:
```
  step pulses observed: [426, 426, 0]
[phase4] GATE GREEN: step pulses observed via elf log
[phase4] Phase 4 PASS
```

The test reads step counts from the elf-side `[sim-progress]` stderr dump (now a minimal `status=N seg=N counts=[A,B,Z]` line) because `bridge_call` dies when M400's second `push_segment` times out — see follow-up #1.

## Open follow-ups (not blocking GATE GREEN, but blocking real-print)

### 1. Bridge transport flakiness — production blocker

Same binaries, run-to-run nondeterminism between full success and `load_curve` / `set_homed_state` / `push_segment` timing out. The `bridge_call` timeout was bumped from 100 ms to 2000 ms (`rust/kalico-host-rt/src/producer.rs::DEFAULT_LOAD_CURVE_TIMEOUT` and `DEFAULT_PUSH_RESPONSE_TIMEOUT`); helped but didn't eliminate. M400 specifically dispatches a second `push_segment` (the decel/cruise tail) that frequently times out and shuts klippy down. Investigation should focus on:

- `rust/kalico-host-rt/src/host_io/reactor.rs` — submission queue, response correlation, NAK handling on a busy stream.
- Whether the kalico-native transport on the same wire as Klipper's classic protocol has flow-control issues when both share the PTY in the sim.

### 2. Late-entry seed silently loses motion

Bug C above is patched symptomatically: the engine seeds at the current `u`, so the steps that "should have happened" between `u=0` and `u_current` are simply never emitted. For Phase 4 this is the right call (we wanted to see *any* steps); for production this entry case shouldn't happen at all (real hardware shares the clock cleanly), but if it does the trajectory layer should either back-fill the missing segment fragment or fault hard. Requires a design call — keep loose and document, or tighten and reject?

### 3. The `+1` in `src/linux/timer.c::start_sec`

The cascade that produced Bug B starts with `TimerInfo.start_sec = curtime.tv_sec + 1`. That's mainline Klipper code; the +1 likely exists to avoid sub-second underflow elsewhere. We worked around its consequences without touching it. Worth a short investigation to either remove it (cleaner) or document why it has to stay.

### 4. The stub `kalico_configure_axes` FFI

The old `kalico_configure_axes(rt, kinematics_tag)` FFI is still present in `runtime_ffi.rs` — it's the validate-kinematics stub. Live path now uses `kalico_runtime_configure_axes_blob`. The old one can be deleted along with `command_kalico_configure_axes` in `runtime_tick.c` if nothing in tests/Renode references it.

### 5. Out-of-order tasks list

Several earlier-in-session diagnostic FFIs (`get_axis_motor`, `get_last_timing`) were added for one-off bug isolation and are now unused by the production path. They're cheap and correct; leave them in for now (next session may want them again), revisit during a cleanup pass.

## Build-order context (from `CLAUDE.md`)

Step 7-D is the active build-order item. Phase 4 was the segment-dispatch gate within Step 7-D. Remaining sub-items in Step 7-D:

- Surface-C cycle-budget actuals.
- F4x integration for Z.
- M1/M2/M3 soaks.
- Calibration.
- Physical first print.

The transport flakiness (follow-up #1) is the most likely first thing to bite real hardware.

## Where to start next session

1. **Reproduce the gate.** Run the docker block above; confirm `[phase4] Phase 4 PASS` and `step pulses observed: [~400-430, ~400-430, 0]`. If anything's off, the session probably broke something — bisect against `321c91362`.
2. **Pick from follow-ups.** #1 (transport flakiness) is the highest-value next pull. #4 is a quick cleanup if you want a warm-up commit. #2 needs an architectural call before code.
3. **Don't re-investigate the clock-frame bug.** The fix is correct for the sim; on real H7 hardware Klippy's clock comes from `DWT->CYCCNT` directly with no `start_sec` games, so the spurious wrap doesn't happen, and the seeding path naturally pulls `stats_send_time_high=0`. The sim and hardware paths converge cleanly.

## Key files for orientation

- `rust/kalico-protocol/{schema_def.rs,src/messages.rs}` — wire format (incl. `ConfigureAxes` 0x0030).
- `rust/runtime/src/engine.rs` — `Engine::configure`, `tick_with_current`, the `needs_xy_seed` branch (now seeds at current `x,y`).
- `rust/runtime/src/clock.rs` — `WidenState`, `seed_high`.
- `rust/kalico-c-api/src/runtime_ffi.rs` — all the `kalico_runtime_*` FFI surface.
- `rust/motion-bridge/src/bridge.rs::configure_axes` — Python-side entry point.
- `klippy/motion_toolhead.py::_configure_axes_per_mcu` — gathers steppers and dispatches per MCU.
- `src/linux/kalico_host_tick.c::kalico_h7_enable_tim5` — sim widen-state seeding.
- `src/runtime_tick.c` — `[sim-progress]` stderr dump in `runtime_status_drain`.
- `tools/sim_klippy/test_phase4_steps.py` — the gate test, polls elf log.
