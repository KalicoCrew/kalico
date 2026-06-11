# Spec B — External-Trigger Homing and Probing Moves

Design for P2 of [`beacon-fork-survey.md`](beacon-fork-survey.md): homing and
probing moves whose trigger source is not a GPIO pin on a bridge MCU — first
consumer is the Beacon probe (its closed firmware fires a stock-Klipper
trsync on its own MCU). Supersedes the remaining live pieces of
`external-probe-homing.md` (its Piece B credit-window deadman is **dropped**,
see "What is reused"); the beacon fork itself is Spec D.

## Decisions made during brainstorming

1. **Trigger relay lives in Rust**, on the probe MCU's reactor RX thread —
   bounded sub-ms latency, not subject to reactor/GIL stalls.
2. **No MCU-side credit/deadline commands.** The pump's drip cohort already
   bounds MCU motion authority during homing; it is the deadman.
3. **Protocol alignment with Beacon's latch-and-read shape.** The trip event
   becomes a doorbell + cross-check; the latched trip record on the MCU
   becomes the authoritative, queryable timestamp source. One post-stop flow
   for every trigger source: *notification stops motion → host reads
   authoritative detect time → `motion_state_at` turns it into a position.*

## What is reused (no changes)

- **Drip-cohort deadman** (`pump.rs:205`, `DRIP_WINDOW_SECS = 0.100`): during
  homing the pump feeds pieces only within a 100 ms lead horizon of the MCU's
  acked clock, and the cohort watchdog faults on a stalled retired-floor. If
  the host process dies mid-descent, the piece ring runs dry and the axis
  stops within ≤100 ms of travel authority. This is the same safety model
  GPIO homing relies on today; the window is tunable later if 100 ms ever
  matters.
- **Trip handling pipeline** (`bridge.rs::handle_endstop_trip`): `HomingRun`
  matches trips on `(endstop_id, endstop_mcu)` as opaque tokens; the handler
  flushes the pump, broadcasts `Stop`, reconstructs trip + final positions
  with cross-MCU clock conversion (`homing.rs::reconstruct_axis_position`),
  and resumes streams. A relayed Beacon trigger enters here and the whole
  path runs unchanged. Trips with no active run are already ignored.
- **`motion_state_at`** (Spec A): exact executed-state evaluation at any
  clock, cross-MCU. Consumed by the measured-position override (contact
  detect-time evaluation) and available to providers.
- **Python poll loop** in `homing.py::trip_move`: computed deadline
  (`max_travel / speed + margin`), provider hooks `trip_move_begin/end`.

## New work

### 1. Latched trip record + query (firmware + protocol)

`src/endstop.c` already latches `trip_clock` in the IRQ; the trip event ships
that value and nothing ever reads the latch again. Changes:

- Extend the `endstop_state` response (reply to `endstop_query_state`) with
  `tripped=%c trip_clock=%u`. `trip_clock` is the low 32 bits of the latched
  64-bit tick count; the host expands it with the standard clock32→64
  reference (queries happen within seconds of a trip — far inside the wrap
  window). `tripped` distinguishes "armed, no trip" from "tripped" (today
  only `armed` is visible). The latch persists until the next
  `query_endstop` arm.
- The trip event (`kalico_endstop_tripped`) is unchanged and remains
  load-bearing for *stopping*: it is the doorbell. Its clock payload is kept
  and demoted to a cross-check.
- **Cross-check (fail loudly):** after the stop, `trip_move` queries
  `endstop_state` and asserts the expanded latched clock equals the doorbell
  clock bit-for-bit (same struct field on the MCU — any mismatch is a real
  bug: clock-domain confusion, stale latch, duplicate trip). Mismatch raises
  with both values in the message.
- **Doorbell-lost fallback (fail loudly, diagnosable):** if the poll deadline
  expires without a trip result, `trip_move` queries `endstop_state` before
  raising. `tripped=1` means the trip happened but the event was lost — the
  error says so explicitly (with the latched clock) instead of the generic
  "failed to trigger after full travel". No silent recovery.

The latch query, cross-check, and doorbell-lost fallback apply to **GPIO
endstops** (entries backed by our `endstop.c`). Remote endstops have no
`endstop_state` oid on our firmware; their authoritative post-stop read is
provider-defined (for Beacon: `beacon_contact_query` / post-home `_sample()`
via the measured-position override) and their lost-doorbell diagnosis is the
provider's terminal-report handling. Both source kinds share the same shape:
doorbell stops, latch answers.

Both bridge MCUs must be flashed together — this extends a shared response
format.

### 2. Remote-trigger relay (Rust)

New bridge API pair:

- `arm_remote_trigger(probe_mcu, trsync_oid, endstop_id)` — registers a
  reactor interceptor (`host_io/interceptor.rs`, registration via the
  existing submission API at `host_io/mod.rs:652`) on the probe MCU for
  `("trsync_state", oid=trsync_oid)`.
- `disarm_remote_trigger(...)` — unregisters; called unconditionally from
  `trip_move`'s cleanup path.

Interceptor callback (runs on the probe MCU's RX thread):

- Acts only on **terminal** reports (`can_trigger == 0`); trsync emits
  periodic non-terminal state reports while active, which are ignored.
- Fires at most once per arm (latched flag).
- Any terminal reason — `REASON_ENDSTOP_HIT`, comms timeout, host abort —
  triggers the stop. Stopping motion is always the correct response to a
  terminal trsync state; reason discrimination stays in Python (the fork's
  `home_wait` semantics, which still receive the same `trsync_state` message
  via the unchanged passthrough path).
- Expands the report's 32-bit clock to the probe MCU's 64-bit domain via the
  router's clock state, then calls the same internal entry point as a GPIO
  trip: `handle_endstop_trip(probe_mcu, endstop_id, clock64)`. The handler
  spawns its worker thread immediately; the RX thread is not blocked.
- The report clock is **not a trip timestamp** (verified in `trsync.c`):
  `trsync_task` sends `timer_read_time()` at report time (`trsync.c:190`),
  and the host-commanded `trsync_trigger` path sends `clock=0`
  (`trsync.c:176`). The relay uses the nonzero report clock as the
  provisional trip clock (sub-ms after the trigger — fine for a provisional
  position); on `clock == 0` (host-requested abort) it substitutes the
  router's current clock estimate for the probe MCU.
- If the probe MCU's clock is unsynced (`clock_freq == 0`), raise — never
  guess a trip time.

The relayed clock is only used for the *provisional* trip position. Beacon's
accuracy never depends on relay latency: eddy homing measures true Z with a
post-home `_sample()`, and contact homing reads the hardware-latched
`detect_clock` from `beacon_contact_query` after the stop (beacon.py:2397) —
back-dated to contact onset, immune to braking distance.

### 3. Remote endstop variant (Python)

`RemoteBridgeEndstop` (sibling of `BridgeEndstop` in `bridge_endstop.py`):

- Constructed by a provider with `(probe_mcu, trsync_oid)`; `endstop_id`
  from the shared provider allocator (ids ≥ 3).
- `bridge_mcu_handle()` returns the probe MCU's bridge handle (it is claimed
  on the router like every MCU — serialhdl.py:386-406).
- `arm()` / `disarm()` call `arm_remote_trigger` / `disarm_remote_trigger`
  instead of `query_endstop`.
- `is_triggered()` pre-check is provider-defined (the trsync does not exist
  until the device-side arming dance runs); default returns False.

The device-side arming dance (`beacon_home` / `beacon_contact_home`,
`trsync_start`, timeout heartbeats) remains the provider's Python code,
invoked from the existing `trip_move_begin` / `trip_move_end` hooks. The
heartbeat (`trsync_set_timeout` every ~0.25 s) is latency-insensitive and is
the *probe-side* deadman: if the host dies, the beacon trsync times out and
the device disarms itself, complementing the drip-window stop on the motion
side.

Dead scaffolding deleted with this change: `MCU_trsync.start()`'s
`runtime_stop_on_trigger` sender, `_bridge_arm_id`, and the stale log-code
template (host-side remnants of the demolished `_bridge_drives_steppers`
prototype). `TriggerDispatch` keeps raising — nothing on the bridge path uses
it.

### 4. Measured-position override (provider contract extension)

New optional provider hook, completing the contract from
`virtual-endstops-and-probe.md`:

- `measured_trip_position(trip_result) -> float | None` — called by
  `trip_move` after the stop, the latch cross-check, and `trip_move_end`.
  `trip_result` carries `(trip_pos, final_pos, endstop entry)`. A float
  return replaces the trigger-height-derived axis position; `None` (or hook
  absent) keeps current behavior.

This replaces the dead `homing:home_rails_end` listener path the survey
identified. The beacon fork (Spec D) implements it twice: eddy returns the
post-home `_sample()` distance; contact converts `detect_clock` → clock64 →
`motion_state_at` → exact Z at contact onset, plus the cruise-phase
validation (`accel == 0` at detect time, strictly better than mainline's
per-move-constant `trapq_extract_old` sign check).

### 5. Probing-move primitive (public surface)

`Homing.trip_move` is already the primitive (probe.py drives it outside G28
today). This spec promotes it to the documented provider-facing API:

- Inputs: axis, direction, speed, max_travel, endstop entry (GPIO or
  remote).
- Outputs: `(trip_pos, final_pos)` plus the measured override when the
  provider supplies one.
- Error surface unchanged from `virtual-endstops-and-probe.md` (pre-trigger,
  no-trigger-after-travel, trigger-before-movement), extended with the
  cross-check and doorbell-lost errors above.

The beacon fork's three `HomingMove` call sites (contact probing and
calibration descents — all Z-only) map 1:1 onto this in Spec D.

## Error handling (all hard errors)

- Latch/doorbell clock mismatch.
- Poll deadline expiry with `tripped=1` latched ("trip event lost").
- Remote trigger relayed while probe MCU clock unsynced.
- Terminal trsync report with a non-trigger reason → motion stops, Python
  raises with the reason (comms timeout ≠ no-trigger).
- Existing trip_move errors unchanged.

## Testing

- **Rust (`cargo nextest run`)**: interceptor relay unit tests via the
  existing test harness (`host_io/test_harness.rs::register_interceptor`) —
  terminal-only filtering, fire-once latch, clock32→64 expansion, unsynced
  clock rejection; cross-check comparison logic.
- **Firmware/host protocol**: `endstop_state` response extension exercised in
  sim; cross-check equality on GPIO trips (every existing sim homing test now
  validates it implicitly).
- **kalico-sim**: a minimal synthetic remote-trsync provider extra (sim-only)
  plus an emulated second MCU that fires `trsync_state` on command:
  - end-to-end Z home through the relay (arm → trigger → stop →
    reconstruction → measured override applied);
  - doorbell-lost scenario (emulator suppresses the trip event → deadline →
    error names the latched trip);
  - terminal-with-error-reason scenario (motion stops, reason surfaced).
- Full beacon-fork validation (scanning, contact calibration, temp comp) is
  Spec E, against `tools/kalico-sim/emulators/beacon_mcu.py`.

## Out of scope

- The beacon fork integration layer itself (Spec D).
- Probe ecosystem restoration — ProbePointsHelper, z_tilt/QGL (Spec C).
- Probes wired to GPIO pins on bridge MCUs (already work via the existing
  path); multi-MCU rails homed by one remote trigger beyond what
  `handle_endstop_trip` already does.
