# Servo lazy enable — EtherCAT torque as a stepper enable line

## Problem

The EtherCAT servo drive energizes at claim time and stays energized for the
session. `ec_rt_bringup()` runs the CiA 402 ladder to Operation Enabled before
the endpoint even answers the claim handshake (`kalico-ethercat-rt.rs:70-89`,
`bench/libecrt.c:186-206`), and every DC cycle re-asserts controlword `0x000F`
(`libecrt.c:209-212`). De-energization happens only on endpoint process death.
M84/M18, `SET_STEPPER_ENABLE`, idle_timeout, and `stepper_enable` status are
all blind to the drive.

Steppers are the opposite: de-energized at boot, enable pin scheduled at
`print_time` on the first move that turns that motor
(`motion_toolhead.py:415,423-444` → `stepper_enable.py`), disabled by
M84/idle_timeout/restart, re-armed for the next move.

## Goal

The servo's holding torque follows the stepper enable lifecycle — and wherever
possible runs **the same code**, not parallel code with the same behavior:

- No torque at claim/boot. Claim is purely passive: EtherCAT reaches AL
  OPERATIONAL, PDO exchange and position tracking run, CiA 402 parks short of
  torque.
- Torque on the first move that moves the servo axis (and on homing moves).
- M84/M18, `SET_STEPPER_ENABLE STEPPER=servo_x`, idle_timeout, and
  `gcode:request_restart` affect the servo exactly as they affect steppers,
  including the `stepper_enable:motor_off` → homing-clear flow (which already
  clears all axes today).
- `stepper_enable` `get_status` lists `servo_x` like any stepper.

## Design

### Host: one seam in stepper_enable.py, everything else shared

`PrinterStepperEnable.register_stepper` splits into a thin wrapper plus a
generic entry point:

```python
def register_stepper(self, config, mcu_stepper):
    enable = setup_enable_pin(self.printer, config.get("enable_pin", None))
    self.register_motor(mcu_stepper.get_name(), mcu_stepper, enable)

def register_motor(self, name, motor, enable):
    self.enable_lines[name] = EnableTracking(motor, enable)
```

The servo registers as
`register_motor("servo_x", servo_rail, StepperEnablePin(torque_line, 0))` —
a real `StepperEnablePin` whose "pin" is the drive torque line.
`EnableTracking`, `StepperEnablePin`, refcounting, M84, `SET_STEPPER_ENABLE`,
`get_status`, the 100 ms dwells, and the `stepper_enable:motor_off` event all
run unmodified for the servo.

### Host: the two genuinely new pieces

- **`BridgeTorqueLine`** — exposes `set_digital(print_time, value)`, the same
  contract as `MCU_digital_out`, and forwards to the bridge as a SetTorque
  command. This is the transport adapter: the only place where "GPIO edge"
  becomes "CiA 402 transition". `StepperEnablePin` only ever calls
  `set_digital` (the servo path constructs it directly, bypassing
  `setup_enable_pin`), so no other `MCU_digital_out` surface is needed.
  Registration happens at ServoRail/toolhead construction (config time, same
  as steppers); the torque line resolves its EtherCAT bridge handle lazily at
  first `set_digital` call — the handle exists from `klippy:mcu_identify`,
  long before any move can fire the enable callback.
- **Arming** — `ServoRail` gains the one-shot `add_active_callback` /
  `_active_callbacks` contract of `MCU_stepper` (what `EnableTracking` already
  calls). `_fire_active_callbacks` gets a servo-rail pass that arms on the
  **axis** delta (`(dx,dy,dz)["xyz".index(rail.axis)]`) — not the motor-slot
  delta, which under corexy carries a/b motor values. The same pass covers the
  `drip_move` homing path, which already calls `_fire_active_callbacks`.

### Wire protocol: SetTorque mirrors set_digital

One new command pair, shaped like the MCU's `queue_digital_out`:

`SetTorque { value: bool, execute_at_ns: u64 }` + correlation'd response.
`execute_at_ns` is the host `print_time` mapped into the endpoint clock
domain via the same mapping trajectory pieces already use for start times.

- **`value=1` (enable)**: the endpoint runs the CiA 402 ladder **on receipt**
  — the timestamp is a deadline ("ready by"), not a start time. The ladder is
  a multi-cycle handshake; starting it at `execute_at_ns` would deliver torque
  milliseconds late, under a move that has already begun. Socket FIFO ordering
  guarantees the command lands before the move's pieces. Worst case torque
  arrives queue-depth early — same spirit as the stepper pin rising at
  `get_last_move_time()` rather than at the move's first step.
- **`value=0` (disable)**: the DC loop executes the disable ramp when its
  clock reaches `execute_at_ns` — the exact scheduled-edge semantics of the
  stepper pin, after M84's end-of-queue print_time + dwell. If a re-enable
  arrives while a disable is pending, the pending disable is **cancelled** —
  torque stays on continuously; executing the disable early would de-energize
  while prior motion may still be draining.

### Endpoint: park at Ready to Switch On (0x0006)

- `ec_rt_bringup()` keeps SDO setup (CSP mode 8, DC SYNC0) → SAFE-OP → DC
  stabilize → AL OPERATIONAL, but stops short of torque: it parks asserting
  controlword `0x0006` (CiA 402 Ready to Switch On, the manual's "Servo
  ready" state) with `target_position` tracking `position_actual` every
  cycle. PDO exchange and position tracking run from claim onward.
- The CiA 402 ladder code moves from `ec_rt_bringup()` into a new
  `ec_rt_enable()` (bounded cycles; fault-reset pulsing preserved; returns
  nonzero on timeout/fault).
- `ec_rt_cycle()` becomes state-aware: enabled → `0x000F` + ring-driven
  target; parked → `0x0006` + target=actual.
- `ec_rt_disable()` (the `0x0006` ramp) is unchanged and now lands in the
  same state the endpoint parks in — parked-after-boot and parked-after-M84
  are one state, one assertion.
- `CountMap` re-anchoring is unchanged: it already resets when the ring
  empties, so on re-enable the first sampled piece anchors host position to
  wherever the shaft physically is — no snap, stepper-like position loss
  semantics (homing was cleared by motor_off anyway).
- The Rust main loop gains the parked/enabled state and handles SetTorque.
  The response is sent **after execution**, carrying the result code: for
  enable, after the ladder reaches Operation Enabled (or fails); for disable,
  after the scheduled ramp completes. The host does not block on the response
  (fire-and-forget like `set_digital`); the existing async response path
  surfaces failures through the fault machinery. The **stub binary**
  implements the same command with simulated state so sim/tests cover the
  lifecycle without hardware.

### Drive state mapping (A6-EC manual, Figure 8-3)

| Controlword held | CiA 402 state | Manual name | Motor |
|---|---|---|---|
| `0x0006` | Ready to Switch On | "Servo ready" (S-RDY active) | de-energized |
| `0x0007` | Switched On | "Waiting for the S-ON signal" | de-energized |
| `0x000F` | Operation Enabled | "Servo running" | energized |

Parking at `0x0006` over `0x0007`: identical torque state, ~1-2 DC cycles
slower to enable (irrelevant under ready-by semantics), and it unifies the
parked state with the existing disable ramp target.

**Bench config prerequisite (not written by our code):** object 605Ch ("Stop
mode at S-ON OFF") decides the post-disable shaft feel — `0` = de-energized
(free shaft, stepper-like after M84), negative values = dynamic braking
(windings shorted, shaft damped). Our disable always happens at standstill,
so only the "keeping X status" tail of 605Ch matters. Set 605Ch=0 on the
bench for stepper-like behavior.

## Failure modes — all loud

- **Enable ladder fails** (STO open, drive fault, timeout): endpoint latches
  a distinct fault code → fault heartbeat → existing typed fatal-transport
  path ends the session with an error naming the cause. No retry, no
  fallback.
- **Piece sampled while parked**: host-bug invariant violation → latched
  fault, endpoint exits. Cannot happen in correct operation: enable precedes
  pieces on the same FIFO socket.
- **Disable `execute_at_ns` already past on arrival**: fault, not clamp —
  same rule as the planner's late-segment policy.
- Unchanged backstops: SIGTERM → graceful `ec_rt_disable()`; SIGKILL → the
  drive's SM communication watchdog (~100 ms); WKC loss ×2 → halt.
- Enable/disable transitions emit structured log events (`log_codes.rs`),
  not stderr-only.

## Testing

- **Rust**: wire round-trip tests for SetTorque; stub-based lifecycle tests
  (parked at handshake → enable on command → scheduled disable fires at the
  right clock → re-enable; piece-while-parked faults; ladder-failure
  heartbeat). Extends `stub_loop` / `stub_lifecycle` /
  `endpoint_supervision`.
- **Python/sim**: kalico-sim run — M84 → `get_status` shows
  `servo_x: false` → next G1 re-enables; idle_timeout path.
- **Bench**: claim leaves the shaft free, first move stiffens it, M84 frees
  it (with 605Ch=0). Host-side only: bridge cdylib rebuild on the Pi, no MCU
  flash.

## Out of scope

- Servo homing (Part A boundary, unchanged).
- `FORCE_MOVE` / `STEPPER_BUZZ` for the servo (operate on MCU steppers).
- Multi-slave EtherCAT.
- Any change to when steppers energize.
- Writing 605Ch via SDO (bench drive config, done by hand).
