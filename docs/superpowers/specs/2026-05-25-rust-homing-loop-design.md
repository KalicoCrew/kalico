# Rust Homing Loop for External Probes

**Date:** 2026-05-25  
**Status:** Draft  
**Branch:** sota-motion

## Problem

During Z homing with an external probe (Beacon), the entire trigger-detection
and stop-propagation path runs in Python's reactor. TMC UART polling, GC
pauses, or any other reactor work can delay or block trigger processing. The
result: Z keeps moving after the probe triggers, leading to bed/nozzle
collision. The user had to pull the wall plug.

The root cause is architectural: Python's single-threaded reactor is in the
critical path between "probe detected contact" and "stepper MCU stops Z."

## Goal

Move the timing-critical homing loop out of Python entirely. The trigger path
becomes: Beacon wire → Rust reactor thread → bottom MCU wire. Zero Python in
the critical path.

## Architecture

### Two Rust components

**1. Frame interceptor table (generic reactor infrastructure)**

The reactor gets a registration table for inbound frame callbacks:

```
HashMap<(mcu_handle, msg_name, oid), Vec<InterceptorEntry>>
```

Each entry holds a `Box<dyn Fn(&MessageParams) + Send + Sync>` callback and a
registration ID.

API:
- `register_frame_interceptor(handle, name, oid, callback) -> InterceptorId`
- `unregister_frame_interceptor(id)`

Reactor change: in `handle_inbound_frame`, after parsing an unsolicited
response, check the table. If matched, call the callback(s). The event is NOT
consumed — it still flows to the `runtime_rx` channel.

Constraint: callbacks run in the reactor thread and must be non-blocking.

**2. Homing loop thread**

A scoped thread spawned by `run_probe_homing`. Owns:
- Bottom MCU's `Arc<KalicoHostIo>` (for `extend_homing_deadline`)
- Shared `AtomicBool` (set by interceptor on trigger)
- Move parameters and sensor-fault timeout

After submitting the homing move, sends one immediate `extend_homing_deadline`
before entering the loop. This provides margin against the MCU's initial 50ms
grant window (which the MCU opens itself on the first tick past `arm_clock` —
the host doesn't open it, just needs to extend before it expires).

Loop body (25ms tick):
- Check `AtomicBool` → ProbeTriggered
- Check `is_homing_segment_retired` → SegmentRetired
- Check elapsed > sensor_fault_timeout → SensorFault
- Send `extend_homing_deadline` (fire-and-forget)

### Python API

One blocking call replaces the entire extension loop:

```python
result = bridge.run_probe_homing(
    beacon_handle,        # u32 — Beacon MCU bridge handle
    beacon_trsync_oid,    # u8  — trsync OID on Beacon
    stepper_mcu_handle,   # u32 — bottom MCU bridge handle
    arm_id,               # u32 — homing arm ID
    move_pos,             # [f64; 3] — target position
    speed,                # f64 — homing speed (mm/s)
    sensor_fault_timeout, # f64 — seconds before declaring sensor broken
    stepper_oids,         # Vec<u8> — stepper OIDs on bottom MCU
)
```

Result is an enum (exposed as u8 to Python):
- `ProbeTriggered = 0` — normal success
- `SegmentRetired = 1` — move completed without trigger
- `SensorFault = 2` — full travel elapsed, probe broken/misconfigured
- `DeadlineExpired = 3` — MCU dead-man switch fired

### What stays in Python

- Beacon trsync arm/disarm (`trsync_start`, `trsync_set_timeout`, `beacon_home`)
- Motor enable / TMC homing current callbacks (`_fire_active_callbacks`)
- Software endstop arm on bottom MCU (`endstop_arm`)
- Error handling and stepper position bookkeeping after the call
- `home_wait()` completion chain (event passes through, so
  `_handle_trsync_state` fires normally for Python's bookkeeping)

### What moves to Rust

- Deadline extension (25ms timer)
- Trigger detection (frame interceptor)
- Stop command (`software_trip` fire-and-forget)
- Sensor-fault timeout
- Homing move submission (`submit_homing_move_async`)

## Full Sequence

**Python (before motion):**
1. `homing.py` arms Beacon trsync
2. `_drip_move_software_trip` resolves Z steppers, fires TMC callbacks
3. Arms software endstop on bottom MCU

**Handoff:**
4. Python calls `bridge.run_probe_homing(...)` — blocks

**Rust (timing-critical):**
5. Registers frame interceptor: `(beacon_handle, "trsync_state", oid)` →
   callback fires `software_trip` to bottom MCU + sets `AtomicBool`
6. Submits homing move via `submit_homing_move_async`
7. Sends an immediate `extend_homing_deadline` (margin against the MCU's
   initial 50ms grant window, which opens on the first tick past arm_clock)
8. Homing loop thread runs (25ms tick, extends deadline)
8. On trigger: reactor reads `trsync_state(can_trigger=0)` → interceptor fires
   `software_trip` (sub-ms) → sets `AtomicBool`
9. Loop sees flag, exits
10. Unregisters interceptor, returns result

**Python (after motion):**
11. Handles result (raises on SensorFault/DeadlineExpired)
12. `home_wait()` reads trsync completion (event passed through normally)
13. Stepper position bookkeeping, disarm, cleanup

## Safety Layers

| Layer | What | Protects against | Latency |
|-------|------|------------------|---------|
| MCU dead-man switch | Bottom MCU expires 50ms deadline | Host crash, Rust panic, thread stall | ~50ms |
| Rust interceptor | `software_trip` in reactor thread | Normal operation fast-path | < 1ms |
| Sensor-fault timeout | Loop stops extending deadline | Probe malfunction, wrong threshold | seconds (not collision protection) |

No single failure leaves Z moving indefinitely. The MCU dead-man switch is the
collision guard — it fires autonomously if extensions stop arriving, regardless
of host state.

## Interceptor Design Details

The interceptor callback for homing:
1. Parses `can_trigger` field from the `trsync_state` params
2. If `can_trigger == 0` (trigger fired):
   - Sends `runtime_software_trip arm_id={arm_id}` to bottom MCU via
     `send_fire_and_forget` (non-blocking buffer write)
   - Sets shared `AtomicBool` with `Ordering::Release`
3. If `can_trigger == 1` (periodic heartbeat): no-op

The callback captures:
- `Arc<KalicoHostIo>` for the bottom MCU
- `Arc<AtomicBool>` shared with the homing loop thread
- `arm_id: u32`

Registration lifetime: from step 5 to step 10. Unregistered before returning
to Python.

## Sensor-Fault Timeout

This timeout detects a broken probe, not a collision. It fires after
`move_dist / speed + 5.0` seconds. By the time it fires, Z has traveled the
full axis range. The dead-man switch (Layer 1) is what prevents collision in
all failure modes.

The timeout was previously named `safety_timeout` / `SAFETY TIMEOUT` in the
code. Renamed to `sensor_fault_timeout` to make the distinction clear.

## Compatibility

- `homing.py` is unchanged. It still calls `home_start`, `drip_move`,
  `home_wait` in sequence.
- Beacon's Python code still receives `trsync_state` events (not consumed).
  `_handle_trsync_state` fires normally, completing the trsync trigger
  chain that `home_wait` depends on.
- The `drip_completion` from `homing.py` is still passed into `drip_move`.
  Python doesn't use it for trigger detection anymore (Rust handles that),
  but it flows through to `home_wait` via the normal completion chain.
- TMC periodic polling is implicitly suspended during homing because
  Python's reactor is blocked on `run_probe_homing`. This is a benefit,
  not a compatibility concern.

## Scope

- Beacon Z homing only (external probe on non-bridge MCU, steppers on bridge MCU)
- Does not change X/Y sensorless homing (already bridge-native)
- Does not change single-MCU homing
- Frame interceptor is generic infrastructure — reusable for future
  fast-path reactions (EtherCAT, filament runout, etc.)

## Files to Modify

**Rust (new/modified):**
- `rust/kalico-host-rt/src/host_io/reactor.rs` — interceptor table + dispatch
- `rust/kalico-host-rt/src/host_io/mod.rs` — interceptor registration API
- `rust/motion-bridge/src/bridge.rs` — `run_probe_homing` pymethod
- `rust/motion-bridge/src/probe_homing.rs` — new module: loop thread + interceptor setup

**Python (modified):**
- `klippy/motion_toolhead.py` — `_drip_move_software_trip` simplified to call `run_probe_homing`
