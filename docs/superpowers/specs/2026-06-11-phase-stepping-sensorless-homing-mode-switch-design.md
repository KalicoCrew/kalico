# Phase Stepping ↔ Sensorless Homing Mode Switch

**Date:** 2026-06-11
**Branch:** phase-stepping-sensorless-homing
**Status:** Approved design

## Problem

TMC5160 `direct_mode` (phase stepping via XDIRECT) disables StallGuard-based
stall detection. The datasheet states it explicitly in the XDIRECT register
description: StallGuard and coolStep "only can be used when additionally
supplying a STEP signal." The mechanism is a dependency chain on STEP edges:

1. TSTEP measures time between STEP edges. In direct mode no edges arrive, so
   TSTEP freezes at the standstill value.
2. TCOOLTHRS gates the stall output on velocity by comparing against TSTEP.
   With TSTEP frozen, the gate never opens.
3. SG_RESULT only updates on full steps, also driven by STEP edges.
4. The DIAG pin therefore never asserts, regardless of motor load.

Consequence today: sensorless homing on a `phase_stepping: true` stepper
creeps to `max_travel` with DIAG permanently silent. Nothing in the codebase
coordinates the two features. Prusa XL (the prior art for XDIRECT phase
stepping) shipped UI-level mutual exclusion; we do better by switching modes
around the homing move.

## Decision

Switch the affected stepper to Pulse (step/dir) mode before the StallGuard
trip move and restore Phase mode after, orchestrated host-side in
`TMCVirtualPinHelper.arm()` / `disarm()` (klippy/extras/tmc.py), which already
bracket the trip move and perform the analogous stealthchop / TCOOLTHRS /
THIGH save-restore. The runtime exposes primitives; Python owns sequencing,
because the TMC SPI register writes (GCONF.direct_mode, CHOPCONF), ISR-SPI
arbitration (`kalico_phase_stepping_{enable,disable}_spi`), and echeck
suppression already live in the Python TMC5160 driver. A bridge-automatic
switch was rejected: the bridge cannot write TMC registers, so the sequence
would be split across two owners.

## Core correctness requirement: phase handover

Nothing syncs MSCNT with the runtime's phase accumulator today. Entry into
phase mode at motor-enable time tolerates a rotor snap (position is
meaningless pre-homing). The post-homing re-entry does NOT: a snap there
shifts the rotor up to ±2 full steps after the origin was just established,
silently corrupting the homed position.

Key facts the handover builds on:

- The phase-mode ISR drives coils from
  `PHASE_LUT[(last_step_count + phase_offset) & 0x3FF]`
  (rust/runtime/src/dispatch_stepper.rs `dispatch_phase`).
- MSCNT is frozen while in direct mode (no STEP edges), so the value read at
  phase-mode entry (`_xdirect_preload` already reads it) stays valid until
  the mode is exited. The host caches it; no SPI-bus contention with the ISR
  at exit time.
- `set_stepper_offset` (rust/runtime/src/engine.rs) already implements a
  ramped per-stepper phase jog, slewed by the ISR at a capped
  microsteps-per-sample rate.
- `engine.set_step_mode` already refuses while motion is armed, resets the
  step queue, and on Pulse→Phase re-seeds `last_phase_target` to avoid a
  bogus position delta.

### Exit handover (Phase → Pulse), in `arm()` before StallGuard setup

1. Standstill is enforced by the runtime: the mode switch returns an error if
   any axis motion is armed. Any error raises — no retry, no recovery.
2. Compute the shortest signed delta in [-512, 511] from the rotor's current
   phase `(last_step_count + phase_offset) & 0x3FF` to the cached frozen
   MSCNT. Slew it via `set_stepper_offset` (ramped). Poll a new
   "offset ramp settled" query off the reactor until complete. The rotor now
   sits exactly on the driver's hardware LUT position. Pre-homing, this jog
   of ≤2 full steps is harmless.
3. Disable ISR XDIRECT writes (`kalico_phase_stepping_disable_spi`), clear
   `GCONF.direct_mode` (rotor phase now matches `LUT_hw[MSCNT]`, so no snap),
   restore spreadcycle/stealthchop state per existing `arm()` logic, resume
   periodic DRV_STATUS/GSTAT error checks.
4. Runtime `set_step_mode(Pulse)`. Homing proceeds with real STEP edges:
   TSTEP, TCOOLTHRS gating, and DIAG are all live.

### Re-entry handover (Pulse → Phase), in `disarm()` after restore

1. Read MSCNT fresh over foreground SPI (it advanced during homing; ISR SPI
   is off, so the read is safe).
2. Stop echecks, write CHOPCONF (toff>0) then `GCONF.direct_mode=1`, preload
   XDIRECT from MSCNT. This is `_xdirect_preload` refactored to be
   re-runnable (it is currently a one-shot post-enable callback).
3. Set `phase_offset_microsteps` and `phase_offset_target` together, with NO
   ramp, such that `(last_step_count + offset) & 0x3FF == MSCNT`. This is
   pure bookkeeping — zero commanded rotor motion — so the homed origin is
   preserved exactly.
4. Enable ISR SPI (`kalico_phase_stepping_enable_spi`), runtime
   `set_step_mode(Phase)` (re-seeds `last_phase_target`).

## New plumbing

The runtime FFI for `set_step_mode` (runtime_ffi.rs:520) and
`set_stepper_offset` (runtime_ffi.rs:948) exists but is not exposed through
motion-bridge or Python. New surface:

- Bridge methods + motion_bridge.py wrappers for:
  - `set_step_mode(stepper, mode)`
  - `set_stepper_offset(stepper, delta_microsteps, max_per_sample)` —
    ramped jog (exit handover)
  - `set_phase_offset_absolute(stepper, offset)` — sets current and target
    atomically, no ramp (re-entry handover)
  - `get_phase_state(stepper)` → `(last_step_count, phase_offset,
    ramp_settled)` — phase arithmetic inputs and slew-completion poll
- klippy/extras/tmc5160.py: refactor `_xdirect_preload` into a re-runnable
  `enter_phase_mode()`; add `exit_phase_mode()`; cache MSCNT at entry.
- klippy/extras/tmc.py `TMCVirtualPinHelper.arm()/disarm()`: call
  exit/enter around the existing register save-restore when the driver
  reports phase stepping active.

## Current scaling across the switch

Direct mode scales XDIRECT by IHOLD; pulse mode scales by IRUN.
`TMC5160CurrentHelper` is constructed with `direct_mode=True` for
phase-stepped drivers. The switch must guarantee IRUN carries the proper run
current during the homing move and the IHOLD-as-run-current convention is
restored after. Implementation must verify what the current helper programs
today and fold IHOLD/IRUN save-restore into exit/enter. (StallGuard
sensitivity depends on run current, so a wrong IRUN during homing would make
SGT tuning silently invalid.)

## Failure handling — fail loudly

- Any error mid-sequence raises and leaves the driver in Pulse mode (the
  safe mode, where StallGuard, echecks, and standard current control all
  work). No silent recovery, no retries.
- Backstop in `arm()`: if the driver is phase-stepping and the mode switch
  did not complete, raise instead of starting a homing move that would creep
  to `max_travel`.
- The runtime mode switch refusing due to armed motion (-2) is surfaced as a
  hard error, not waited out.

## Scope decisions

- No user-facing `SET_STEP_MODE` command in v1. The homing path is the only
  consumer; a debug command is easy to add later.
- Multi-probe homing (probe, retract, slower re-probe) fires arm/disarm per
  trip move, producing double transitions. v1 accepts this — each transition
  is correct and takes milliseconds at standstill. Holding Pulse across the
  whole homing operation is a later optimization if warranted.
- Multi-stepper axes: each stepper jogs independently to its own MSCNT at
  exit. Pre-homing racking of ≤2 full steps is acceptable; sensorless axes
  on current bench hardware are single-stepper.
- Physical-endstop homing needs no mode switch: motion works fine in phase
  mode and GPIO endstops do not depend on STEP edges. Only the StallGuard
  virtual-endstop path switches.

## Testing

- Rust unit tests: shortest-signed-phase-delta math; absolute phase-offset
  set is motion-free (no delta accumulates into `position_count`);
  mode-switch seeding of `last_phase_target`; ramp-settled query.
- kalico-sim: full sequence — home with virtual endstop on a phase-stepped
  axis, assert mode transitions, assert post-homing commanded position
  matches reconstructed trip position exactly (no phase-snap drift).
- Bench (Trident): sensorless home with phase stepping enabled; verify DIAG
  trips, homed origin repeatability across phase/pulse boundaries, and no
  clunk on either transition.

## References

- docs/research/tmc5160-open-loop-phase-stepping.md
- TMC2130/TMC5160 datasheets, XDIRECT register description (StallGuard
  requires STEP signal; current scaled by IHOLD in direct mode)
- Prusa XL phase stepping ↔ crash detection mutual exclusion
  (help.prusa3d.com/article/phase-stepping-xl_681760,
  Prusa-Firmware-Buddy issue #4305)
