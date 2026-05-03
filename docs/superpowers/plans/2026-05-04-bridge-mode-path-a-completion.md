# Bridge-mode end-to-end completion — Path A implementation plan

**Decision:** Path A (PassthroughRouter as central pipe; `MotionMcuProxy` as real `MCU` replacement in bridge mode).
**Source RFC:** `docs/superpowers/specs/2026-05-04-bridge-mode-completion-rfc.md`
**Iteration tool:** `tools/sim_klippy/run.py` (Linux-host klipper.elf + dedicated klippy). Each phase has a sim gate before moving to the next phase. Real H723 hardware bring-up is the final phase.

## Architectural target

```
                  klippy (Python)
                        │
            ┌───────────┴───────────┐
            │                       │
        printer.py             other modules
            │                  (kinematics,
   constructs MotionMcuProxy    extras, etc.)
            │                       │
            ▼                       ▼
   ┌─────────────────────────────────────┐
   │         MotionMcuProxy              │
   │  (klippy/motion_mcu.py — Path A)    │
   │                                     │
   │  • Same public surface as MCU       │
   │  • lookup_command  → MotionCommand- │
   │     Wrapper                         │
   │  • register_response → bridge       │
   │  • alloc_command_queue → router cq  │
   └────────────────┬────────────────────┘
                    │  (FFI via motion_bridge_native)
                    ▼
   ┌─────────────────────────────────────┐
   │   PyMotionBridge (Rust pyo3)        │
   │                                     │
   │   • PassthroughRouter (central pipe)│
   │   • Reactor (host_io)               │
   │   • Planner / endstop runtime       │
   │   • MsgProtoParser (shared with     │
   │     klippy via set_msgproto_dict)   │
   └────────────────┬────────────────────┘
                    │  (Rust thread owns FD)
                    ▼
            /tmp/klipper_sim_socket  (or /dev/serial/...)
                    │
                    ▼
              klipper_mcu / firmware
```

**Wire ownership:** Rust side owns the FD and reads/writes it from `kalico_host_rt::host_io::Reactor`'s background thread. Klippy never touches the FD in bridge mode.

**Why Rust-owns-FD over klippy-side pump:** simpler ownership model (one writer, one reader), reuses the kalico-host-rt reactor that's already battle-tested with deterministic tests, matches the eventual EtherCAT split (different transport, same router contract).

## Phase gates — each builds on the previous, tested in the sim before moving on

### Phase 1 — Wire ownership + identify handshake

**Goal:** klippy's `Loaded MCU 'mcu' N commands ...` line appears against `klipper.elf` without falling back to legacy serialqueue.

**Work:**

1. `bridge.attach_serial(mcu_handle, serial_path, baud) -> PyResult<()>`
   - Opens the FD (or reuses an existing FD passed in), spawns the host-rt reactor thread that owns it.
   - Performs the identify protocol: sends `identify offset=0 count=40` requests, accumulates the data dict.
   - Hands the data dict back via a new method `bridge.get_identify_data(mcu_handle) -> bytes`.

2. `klippy/serialhdl.py::SerialReader.connect_pipe`
   - In `_use_bridge` mode, instead of `_start_session`: call `mcu._motion_bridge.attach_serial(...)`, then `bytes = mcu._motion_bridge.get_identify_data(...)`, then `self.msgparser.process_identify(bytes)`.
   - Bridge already has its own copy of the parsed dict (`set_msgproto_dict` already in place). This phase confirms the round-trip works on the same dict bytes.
   - `set_clock_est`, `disconnect`, etc. already gated; just verify they still work post-attach.

3. `MsgProtoParser` sharing: confirm `bridge.set_msgproto_dict(raw_dict)` matches what `klippy` produces from `msgparser.get_raw_data_dictionary()` (this code already exists in `mcu.py`).

**Sim gate:** `tools/sim_klippy/run.py 'STATUS'` prints `Loaded MCU 'mcu' N commands` and idles cleanly, no `AttributeError: 'NoneType' object has no attribute 'gc'`. Klippy.log shows kalico_status_v6 heartbeats forwarded as async outputs (proves the inbound async path works through the bridge).

**Risk + mitigation:** the host-rt reactor was tested with deterministic clock; running it against a real PTY/socket may surface scheduling assumptions. Mitigate by reusing the existing `Reactor` rather than building a new one — its tests already cover real-FD I/O.

### Phase 2 — MotionMcuProxy parity with klippy.MCU

**Goal:** A bridge-mode MCU loads its config without exception. `Configured MCU 'mcu' (N moves)` line appears.

**Work:** Inventory every method `klippy/mcu.py:MCU` exposes that other klippy code calls. The set is finite — grep for `mcu\.<method>` across `klippy/`. Implement each in `MotionMcuProxy`:

- **Identity / config:** `get_printer`, `get_name`, `get_constant`, `get_constant_float`, `get_constants`, `is_fileoutput`, `is_shutdown`, `is_active`, `is_non_critical`, `non_critical_disconnected` (attr).
- **Clock:** `estimated_print_time`, `print_time_to_clock`, `clock_to_print_time`, `seconds_to_clock`, `clock32_to_clock64`, `min_schedule_time`, `set_clock_est`.
- **OIDs:** `create_oid`, `register_config_callback`, `add_config_cmd`, `get_query_slot`.
- **Commands:** `alloc_command_queue` → returns a router cq id; `lookup_command(msgformat, cq=None)` → returns `MotionCommandWrapper`; `lookup_query_command(msgformat, respformat, oid=None, cq=None, is_async=False)` → `MotionQueryCommandWrapper`; `lookup_command_tag(msgformat)` → command tag from MsgProtoParser.
- **Responses:** `register_response(callback, name, oid=None)` → forwards to `bridge.register_response(mcu_handle, name, oid, py_callback)`; the bridge dispatches inbound packets to registered callbacks.
- **Step generation:** `register_stepqueue`, `flush_moves` (delegates to bridge planner).
- **Lifecycle:** `register_event_handler` (delegate to printer), `request_restart`, `_handle_shutdown` plumbing.

`MotionCommandWrapper.send(data, minclock, reqclock)`:
1. Encode args via the shared `MsgProtoParser` into a msgproto byte buffer.
2. Submit to `bridge.send_command(mcu_handle, cq_id, bytes, minclock, reqclock)`.
3. Bridge enqueues onto the router; the reactor's outbound thread writes to FD.

`MotionQueryCommandWrapper.send(data)`:
1. Encode + submit + register a one-shot response callback bound to the expected response name + oid.
2. Block (or yield via reactor.completion) until response decoded.
3. Return params dict.

`printer.py` construction site: dispatch on bridge presence — when `motion_bridge` is in printer objects, instantiate `MotionMcuProxy` instead of `MCU`. Both must satisfy the same printer-object contract.

**Sim gate:** `Configured MCU 'mcu' (N moves)` appears, klippy reaches "ready" state, no exceptions in the config phase.

**Risk + mitigation:** the inventory is the dangerous part — missing one method causes a crash mid-config. Mitigate by writing a `compat_check` test that imports klippy and asserts `set(dir(MotionMcuProxy)) ⊇ <set of MCU methods called from rest of klippy>`. Run as part of the sim harness.

### Phase 3 — Async response routing

**Goal:** kalico_status_v6 heartbeats and any test command's response are routed through the bridge to klippy callbacks correctly.

**Work:**
- `bridge.feed_inbound(bytes)` (called by the reactor's RX thread for every msgproto packet) parses via `MsgProtoParser`, looks up registered handlers by `(mcu_handle, name, oid)`, dispatches via a Python GIL-acquired callback.
- `bridge.register_response(mcu_handle, name, oid, callback)` stores the mapping.
- Special handlers: `#unknown`, `#output`, `stats` — same as legacy.
- `kalico_endstop_tripped` registers as before — same code path.

**Sim gate:** Custom `kalico_sim_diag` command sent via klippy → response observed in klippy.log via registered callback. Multiple subscribers to the same response name work.

**Risk + mitigation:** GIL acquisition cost for every async output × kalico_status_v6's 1 Hz cadence is fine. Higher-frequency outputs (kalico_credit_freed under load) need batched dispatch — defer to optimization later if it shows up as a bottleneck.

### Phase 4 — Stepper / move integration

**Goal:** `G91; G0 X10; G90` produces step pulses on the configured X step pin. Verify with `gpioget` on the sim's gpiochip0/gpio9.

**Work:**
- `motion_toolhead.move(newpos, speed)` already calls `bridge.submit_move`. Confirm planner dispatch closure produces `kalico_runtime_push_segment` commands and sends them via the bridge's command path established in Phase 2.
- Per-stepper pin config: each `[stepper_x]` results in `config_stepper oid=N step_pin=... dir_pin=...` commands. These flow through `MotionMcuProxy.add_config_cmd` → bridge → wire. The runtime (in firmware/host-process) drives the step_pin via `gpio_out_setup/gpio_out_toggle`. Verify the runtime's per-axis-scalar evaluator emits step events tied to OIDs.
- `MCU_stepper.bridge_set_position_from_step_count` already exists — confirm it's called only on trip (not on plain submit_move).
- `flush_moves` propagates to bridge.flush.

**Sim gate:**
1. `tools/sim_klippy/run.py 'G28 X0; G1 X10 F1000'` (after disabling endstop arming so G1 can run unhomed isn't an issue) →
2. Use `gpiomon -c10 gpiochip0 9` in a side terminal → observe ~10 toggles for 10 mm × steps_per_mm.

If step pulses don't appear, instrument the runtime's step output path (already real per Step 7-B) to confirm the segment reaches the evaluator.

**Risk + mitigation:** the existing per-axis-scalar evaluator is tested via Step 7-B unit tests, but the wire-level glue (msgproto encoding of `kalico_runtime_push_segment`) is fresh in this phase. Decompose: first confirm the command bytes leave klippy correctly (tcpdump-equivalent on the unix socket), then confirm the runtime decodes and dispatches.

### Phase 5 — Endstop trip path

**Goal:** `G28 X` in the sim, with `kalico_sim_endstop_set_pin gpio=20 level=1` injected mid-move, completes successfully.

**Work:**
- `bridge.endstop_arm` (already in place) — verify it now actually reaches the firmware via the MotionCommandWrapper path from Phase 2.
- `kalico_endstop_tripped` handler — registered via `BridgeTriggerDispatch.start` → `register_response`. Phase 3's async path makes this work.
- `bridge.take_trip_event` returns the decoded event from the runtime's queue.
- `mcu_endstop._home_wait_bridge` picks up the trip, applies step counts via `MCU_stepper.bridge_set_position_from_step_count`.

**Sim driver:**
```python
# tools/sim_klippy/test_g28_x.py
run('G28 X', wait=False)
sleep(0.1)  # let arm + first home start
send_raw_mcu_command('kalico_sim_endstop_set_pin gpio=20 level=1')
wait_for_completion()
assert printer.toolhead.homed_axes contains 'x'
```

**Sim gate:** `G28 X` returns success, klippy.log shows `kalico_endstop_tripped` followed by homed_axes update, no errors.

**Risk + mitigation:** trip-time stepper count must be plausible (not 0). The runtime publishes step counts via the trip-event blob — confirm the encode/decode round-trip is correct. Already covered by `tools/test_renode_endstop_e2e.py`, just need to validate against the new live bridge path.

### Phase 6 — Peripheral validation (TMC, heater, fan)

**Goal:** A moderately complex printer.cfg loads in bridge mode without falling back to legacy paths.

**Work:** No code changes ideally — Phase 2's MotionMcuProxy parity should cover it. But verify:

- `[tmc2209 stepper_x]` via UART: `lookup_query_command` round-trips a register read.
- `[heater_generic]` via ADC: periodic `query_analog_in` outputs decoded in klippy.
- `[fan]` via PWM: `set_pwm` commands acknowledged.

Add minimal versions of each to the sim printer.cfg. Some peripherals will need real Linux GPIO chips that exist on the Pi (gpiochip0). For SPI/UART devices that don't exist, the host can't simulate them — those peripherals are out of scope for the sim and validated only on real hardware.

**Sim gate:** Sim klippy boots a TMC + heater + fan config without error. Real-hardware validation deferred to Phase 7.

### Phase 7 — Real H723 hardware bring-up

**Goal:** `G28 X` on the user's Trident with the production printer.cfg.

**Work:**
- Build production firmware (already in place from Step 7.5).
- Build motion_bridge_native.so for aarch64 on the Pi (already in place).
- Push branch, restart klippy on the Pi.
- Iterate any new bugs the real hardware exposes that the sim missed (e.g., real STM32H7 GPIO timing, real TMC SPI, real DIAG sensorless thresholds).

**Gate:** physical G28 X completes successfully on the user's printer.

## Risk register

| Risk | Phase | Mitigation |
|---|---|---|
| `MotionMcuProxy` API surface incomplete | 2 | `compat_check` test enumerates every `mcu.<method>` call site |
| msgproto parser dict mismatch (klippy vs bridge) | 1 | shared `set_msgproto_dict` already exists; validate byte-for-byte |
| Runtime step output not wired for Linux | 4 | reuse production code paths; gate-test before next phase |
| trip-time clock conversion (bridge clock vs print_time) | 5 | clock-sync regression hooks (`_bridge_clock_est_cb`) already exist; verify they fire post-Phase-1 |
| Klippy threading vs bridge threading | 1, 3 | Bridge's reactor thread feeds inbound bytes via GIL-acquired callbacks; klippy's reactor thread submits outbound via lock-protected enqueue. Document the lock order. |
| Peripherals (TMC, heater) silently broken | 6 | Each peripheral type tested in sim with a synthetic config |
| H723 hardware exposes a sim-missed bug | 7 | Accept; budget a day of bring-up debugging |

## Estimated effort

- Phase 1: ~half day
- Phase 2: 1–2 days (the inventory work)
- Phase 3: ~half day
- Phase 4: ~half day to 1 day (depends on whether step output path is already correct on Linux)
- Phase 5: ~half day
- Phase 6: ~half day
- Phase 7: 1 day

Total: ~5–7 days of focused work.

## Out of scope

- EtherCAT backend wiring (Step 14) — the central-pipe abstraction supports it but the EtherCAT-specific transport is its own project.
- Phase-stepping (Step 10) — orthogonal; does not depend on this work.
- Telemetry/skip detection (Step 11) — uses the same async-event channel that Phase 3 establishes; piggybacks on the work.

## What we are NOT changing

- The existing `tools/sim_klippy/` harness — Phase A+B from yesterday is the iteration tool.
- The `motion_bridge_native` rename — already shipped, prevents the .so/.py shadow.
- The H723 firmware build — only Linux-host code paths and klippy Python change.
