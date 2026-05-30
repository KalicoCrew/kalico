# Trsync as the Cross-MCU Homing Mechanism (Bridge Engine)

## Goal

Make **trsync the primary homing trip-distribution mechanism** in the new Rust
bridge motion engine, working **cross-MCU**, and replace the bespoke homing
"credit deadline" safety net with **rate-matched piece dispatch** through the
ring pool.

Two reworks, designed together because they share the homing path:

- **Part A — Cross-MCU trsync stop.** When an endstop trips on any MCU
  participating in a homing move, *all* participating MCUs freeze their curve
  evaluators. The trip is distributed by a host-side relay (the bridge reactor),
  the structural twin of mainline Klipper's C `trdispatch`.
- **Part B — Metered drip safety net.** Homing motion is dispatched as ~25 ms
  pieces into the ring pool, never more than ~50 ms ahead, on a wall-clock
  cadence. The dispatch cadence *is* the host-death dead-man: stop feeding and
  the ring drains in ≤50 ms. This deletes the `deadline_clock` / `grant_ticks` /
  `runtime_extend_homing_deadline` machinery entirely.

This is build-the-general-mechanism-first: cross-MCU trsync is the primary path
even though the current test hardware homes X with the pin and steppers all on
one MCU (H7). The same-MCU fast-path ("local siren") is a later optimization,
explicitly disabled during bring-up so the relay is what's under test.

## Background: current state

### Hardware topology (Trident test bench)

- `[mcu]` = STM32H723 (H7). `[mcu bottom]` = STM32F446 (F446). Beacon is a
  separate USB board.
- X/Y steppers (`stepper_x`, `stepper_x1`, `stepper_y`, `stepper_y1`) and their
  StallGuard `virtual_endstop`s are all on **H7**. Z steppers (`stepper_z`,
  `z1`, `z2`) are on **F446**. Beacon (Z probe) is its own MCU.
- So: sensorless X/Y homing is currently H7-only (same-MCU). Beacon+Z homing is
  genuinely cross-MCU (Beacon source → F446 sink).

### What exists today

- **Local stop works.** `runtime/src/endstop.rs::tick` samples the GPIO/StallGuard
  pin in the TIM5 modulation tick and returns `TripAction::AbortNow`, which
  freezes the curve evaluator on that MCU. Trip is reported to the host via the
  async `kalico_endstop_tripped` message (`src/runtime_tick.c::runtime_endstop_drain`).
- **No cross-MCU fan-out.** Nothing distributes a trip from the detecting MCU to
  other participating MCUs. `bridge.rs::software_trip` and
  `extend_homing_deadline` each target exactly one MCU.
- **External-probe path is a special case.** `rust/motion-bridge/src/probe_homing.rs`
  registers a Rust frame interceptor on Beacon's `trsync_state` and, on
  `can_trigger==0`, fires `runtime_software_trip arm_id=N` at the stepper MCU.
  Heavy churn, never confirmed working on hardware (logs show the relay never
  fired in the captured runs).
- **Credit-deadline safety net.** The homing move goes down as one long segment;
  the curve evaluator carries a `deadline_clock` (~50 ms grant) and the host
  pings `runtime_extend_homing_deadline arm_id=N` every 25 ms
  (`probe_homing.rs::run_loop`). Two moving parts that exist only for homing.
- **Piece A (`mcu.py`) landed.** `_bridge_drives_steppers` flag, with bridge MCU
  trsyncs treated as *ceremonial no-ops* (`mcu.py:348`, `425`). **Part A reverses
  this**: bridge-MCU trsyncs become load-bearing.

### Reference: mainline trsync contract (what we're re-implementing)

Mainline coordinates a multi-MCU homing stop with two separable halves:

1. **The relay** — a C `trdispatch` *fastreader* runs in the serialqueue
   receive thread (no GIL). On any MCU's `trsync_state` with `can_trigger==0` it
   broadcasts `trsync_trigger` to *all* participating trsyncs at ~µs latency
   (`klippy/chelper/trdispatch.c::handle_trsync_state` on `main`).
2. **The stop** — each MCU's `trsync_do_trigger` fires registered
   `trsync_signal` callbacks; `stepper_stop_on_trigger` registers a stepper's
   stop, which clears the **old C step queue** (`src/stepper.c::stepper_stop`).

The bridge keeps the *relay* idea but in a different thread (the bridge reactor,
which owns every MCU's RX), and needs a *different stop* (the C step queue is
gone — only `kalico_software_trip → AbortNow` freezes the curve evaluator).

## Design overview

```
        ┌────────────────── bridge reactor (Rust, owns every MCU RX) ─────────────────┐
        │   TripDispatch  =  trdispatch analog, no GIL, one per homing move            │
        │                                                                              │
 sources│  listens for trip reports:                emits one sink command:            │
 report ┼─►  • trsync_state  (Beacon / classic)  ──►  trsync_trigger oid=…             │
        │    • kalico_endstop_tripped (bridge GPIO)     to ALL participating trsyncs   │
        │                                               + fire host completion         │
        └──────────────────────────────────────────────────────────────────┬─────────┘
                                                                             │
   bridge MCU firmware trsync (armed for the move):  trsync_do_trigger ──────┘
        └─► NEW signal runtime_stop_on_trigger → kalico_software_trip(arm_id)
                                               → endstop.rs AbortNow → curve eval FREEZE
```

Core split: **detection** and **stop** become separate jobs.

- **Detection** stays where it already is and is well-timed: `endstop.rs::tick`
  for bridge GPIO/StallGuard, Beacon firmware for the probe. Detection's only
  cross-MCU duty is to emit a *trip report* (`kalico_endstop_tripped` for bridge,
  `trsync_state` for Beacon). **Crucially, bridge detection does NOT fire the
  firmware trsync** — it leaves the trsync armed, so the relayed `trsync_trigger`
  can stop the *same* MCU (this is what makes the relay testable on one board).
- **Stop** is uniform trsync: every participating bridge stepper MCU arms a real
  firmware `trsync` for the move, carrying a **new** signal whose callback
  freezes the curve evaluator. The relay's `trsync_trigger` fires it.
- **The relay** is `TripDispatch` in the bridge reactor — generalized from the
  existing `probe_homing.rs` interceptor.

## Part A — Cross-MCU trsync stop

### A1. Firmware: `runtime_stop_on_trigger` signal (bridge MCUs)

New command, the bridge twin of `stepper_stop_on_trigger`:

```
runtime_stop_on_trigger arm_id=%u trsync_oid=%c
```

Lives in `src/runtime_commands.c`, directly beside `command_runtime_software_trip`
(which its callback calls) — not in `src/stepper.c`. Rationale: `stepper.c`'s
`stepper_stop` is about the *old C step queue* the bridge doesn't use; placing a
curve-evaluator-freeze signal there invites the misread that this is a step-queue
stop. The `trsync_add_signal` call is MCU-generic, so the file choice is purely
about keeping the "relay → freeze" surface legible and in one place.

Registers a `trsync_signal` (via `trsync_add_signal`) whose callback calls
`kalico_software_trip(arm_id, clock_lo, clock_hi, &status)` — i.e. it freezes the
curve evaluator for that arm. The signal struct stores `arm_id`.

`src/trsync.c` is otherwise **reused unchanged**: `command_trsync_start` arms it
(`TSF_CAN_TRIGGER`), `command_trsync_trigger` → `trsync_do_trigger` fires the
signal. No `trsync_set_timeout` is sent for bridge MCUs (Part B's drain is the
dead-man), and no periodic report is needed (the relay triggers on the *trip
report*, not on trsync status).

The firmware `trsync_do_trigger` guard (`if (!(flags & TSF_CAN_TRIGGER)) goto
done;`, `src/trsync.c:32`) means the relayed trigger only fires while armed —
exactly the property we rely on (detection left it armed).

### A2. Detection → report, with the local siren disabled for testing

`endstop.rs::tick` keeps detecting the GPIO/StallGuard pin. Today it returns
`TripAction::AbortNow` *and* queues `kalico_endstop_tripped`. For bring-up:

- **Keep** the trip-report emission (`kalico_endstop_tripped`).
- **Disable** the immediate local freeze: at the exact spot where `tick`/`arm`
  returns `AbortNow` on a fresh GPIO detection, suppress it and leave a marker:

  ```rust
  // DISABLED FOR TESTING: local siren — the detecting MCU intentionally does
  // NOT self-freeze here so the cross-MCU relay is what stops the motion
  // (lets us verify the relay on a single board). Re-enable as the same-MCU
  // fast-path optimization once the relay is confirmed. See design
  // 2026-05-31-trsync-cross-mcu-homing.
  // return TripAction::AbortNow;
  ```

  Detection still publishes the snapshot + queues the trip report; it just
  doesn't freeze locally. Motion then stops only via the relay's
  `trsync_trigger` (which works because the firmware trsync is still armed).

This makes same-MCU X homing (H7 source + H7 sink) a faithful stand-in for the
cross-MCU path: the stop must round-trip through the host relay.

### A3. Host: arm a real trsync per participating bridge MCU (`mcu.py`)

Reverse the ceremonial no-op. For a bridge MCU participating in a homing move,
`MCU_trsync` arming sends (text commands through the bridge serial shim, as the
non-bridge branch at `mcu.py:401` already does):

- `trsync_start oid=… report_clock=0 report_ticks=0 expire_reason=…` (arm,
  no periodic report, no expire — Part B owns host-death).
- `runtime_stop_on_trigger arm_id=… trsync_oid=…` for the move's arm.

Beacon's own `MCU_trsync` stays **classic and unchanged** (it's a source; it
keeps its real `trsync_start` + report params). The `_bridge_drives_steppers`
gate flips from "no-op" to "arm with `runtime_stop_on_trigger`."

### A4. Bridge reactor: the `TripDispatch` relay

Generalize `probe_homing.rs` into a per-homing-move `TripDispatch`, the
`trdispatch` analog. It holds the participant set: for each participant, its
`KalicoHostIo` handle, its trsync OID, and whether it's a source (reports) and/or
a sink (has a trsync to trigger).

- **Register interceptors** on each *source* MCU's reactor for its trip report:
  - bridge GPIO source → `kalico_endstop_tripped` (filter by `arm_id`)
  - classic/Beacon source → `trsync_state` (filter by trsync OID, `can_trigger==0`)
- **On the first trip from any source** (one-shot `compare_exchange`, as
  `probe_homing.rs` already does): for every *sink* trsync, send
  `trsync_trigger oid=… reason=ENDSTOP_HIT` via that sink's
  `io.send_fire_and_forget` — including the originating MCU. No GIL, mirrors
  `handle_trsync_state`'s broadcast.
- Set the shared "tripped" flag so the host dispatch loop (Part B) stops feeding.

The Python `BridgeTriggerDispatch._on_trip_message` completion handling
(`motion_bridge.py:577`) is **retained** for host bookkeeping (completion +
trigger-time position). The reactor relay does the *fast stop*; Python does the
*completion*. Two consumers of the same trip report, each doing its half.

### A5. Homing contract (unchanged for `homing.py`)

`homing.py`'s contract is preserved: `home_start()` returns a reactor completion,
`drip_move()` feeds interruptible motion, `home_wait()` returns trigger time.
`MCU_endstop.home_start` arms the per-MCU trsyncs + the `TripDispatch`, then the
`drip_move` GPIO branch (Part B) feeds pieces and waits on the shared completion.
No changes to `klippy/extras/homing.py`.

### A6. Beacon folds into the general path

Beacon stops being a special case: it is simply *a source that speaks
`trsync_state`*, with F446 as a *sink* that speaks `trsync_trigger`. The
dedicated `probe_homing.rs` three-phase API is replaced by `TripDispatch` with
`{Beacon: source}` + `{F446: sink}`. Same relay, same code.

## Part B — Metered drip safety net

### B1. Delete the credit-deadline machinery

Remove entirely:

- **Firmware:** `deadline_clock` / `grant_ticks` in the curve evaluator,
  `command_runtime_extend_homing_deadline`, and the deadline / "software-deadline"
  endstop source kind in `endstop.rs` (`tick_software_deadline`, `grant_ticks`,
  `extend_deadline`, `deadline_active`, `store_deadline_clock_seqlocked`).
- **Host/Rust:** the 25 ms `extend` loop in `probe_homing.rs::run_loop` and
  `bridge.rs::extend_homing_deadline` + `motion_bridge.py::extend_homing_deadline`.

**Kept:** `kalico_software_trip` (the curve-evaluator *freeze*) — that is the
stop primitive A1's signal calls. Only the *deadline* half is deleted.

### B2. Rate-matched piece dispatch

The homing move is sliced into `DRIP_PIECE_MS` (= 25 ms) pieces, dispatched into
the ring pool one at a time on a **wall-clock cadence**, keeping the MCU never
more than `DRIP_MAX_AHEAD_MS` (= 50 ms = 2× piece) ahead. This is mainline's
`DRIP_SEGMENT_TIME = 0.050` realized through the bridge ring pool instead of a
bespoke gate.

The host dispatch loop (replaces `probe_homing.rs::run_loop`; lives behind the
`drip_move` GPIO branch in `motion_toolhead.py`):

```
loop every DRIP_PIECE_MS (wall clock):
    if trip_flag:            # set by the TripDispatch relay (A4)
        break                # stop feeding; freeze already happened
    if move_exhausted:
        break                # natural end-of-travel (no-trigger)
    dispatch_next_piece()    # ~25 ms of curve into the participating MCUs' rings
```

Pieces are ordinary ring-pool pieces (same representation as print motion),
addressed to every participating bridge stepper MCU for the move.

### B3. Host-death = ring drain

No deadline command. If the host dies or simply stops feeding (including right
after a trip), each MCU's ring drains in ≤`DRIP_MAX_AHEAD_MS` and the curve
evaluator stops on its own. This is the only host-death safety net and it is
implicit in the cadence.

## How the two halves compose on a trip

1. Endstop trips → detection emits the trip report (no local freeze; siren
   disabled) → reactor `TripDispatch` relays `trsync_trigger` to all sink
   trsyncs → `runtime_stop_on_trigger` signal → `kalico_software_trip` →
   `AbortNow` → **freeze now** (~relay latency). Buffered ≤50 ms of pieces are
   abandoned, not drained.
2. Host dispatch loop sees `trip_flag` → stops feeding (belt to the freeze's
   suspenders).
3. Counterfactual host death (no trip): no freeze, ring drains in ≤50 ms → stop.

## Testing strategy

Per the no-live-test-until-analysis-exhausted rule, verification ladder:

1. **Unit / Rust tests** for `TripDispatch` fan-out (one source → N sinks get
   `trsync_trigger`), one-shot semantics, and the sliced-dispatch cadence.
2. **Dual-MCU Renode sim** (`tools/sim/dual_mcu_docker.resc`, H723+F446;
   `tools/test_renode_endstop_e2e.py`): inject a GPIO trip on MCU A, assert MCU
   B's curve evaluator freezes via the relayed `trsync_trigger`. This is the
   genuine cross-MCU test the hardware can't provide (X is H7-only).
3. **Single-board hardware bring-up:** sensorless X home on H7 with the local
   siren disabled — H7 must halt *only* via the relay round-trip. Captures the
   report→relay→`trsync_trigger`→freeze loop on one board, safely (X moves in
   free air, away from collision).
4. **Cross-MCU hardware:** Beacon source → F446 Z sink, once (2) and (3) pass.

## Constants

| Name | Value | Meaning |
|------|-------|---------|
| `DRIP_PIECE_MS` | 25 | Length of each dispatched homing piece. |
| `DRIP_MAX_AHEAD_MS` | 50 | Max motion buffered ahead (= 2× piece, matches mainline 50 ms). |
| trsync trigger reason | `REASON_ENDSTOP_HIT` | Reason carried by the relayed `trsync_trigger`. |

## Files touched

| File | Change |
|------|--------|
| `src/trsync.c` / `.h` | (reuse) no change to core; relied on for arm/trigger/signal. |
| `src/runtime_commands.c` | add `runtime_stop_on_trigger arm_id trsync_oid` command + `trsync_signal` callback → `kalico_software_trip` (beside `command_runtime_software_trip`); delete `command_runtime_extend_homing_deadline`; keep `command_runtime_software_trip`. |
| `src/runtime_tick.c` | delete curve-evaluator `deadline_clock` checks; keep `kalico_endstop_tripped` drain. |
| `rust/runtime/src/endstop.rs` | disable local `AbortNow` siren (marker comment); delete software-deadline source + `extend_deadline`. |
| `rust/motion-bridge/src/probe_homing.rs` → `trip_dispatch.rs` | generalize interceptor into `TripDispatch` (sources: `kalico_endstop_tripped` + `trsync_state`; sink: `trsync_trigger`); delete extend loop; add sliced dispatch loop. |
| `rust/motion-bridge/src/bridge.rs` | `TripDispatch` FFI; sliced homing dispatch; delete `extend_homing_deadline`. |
| `klippy/mcu.py` | bridge `MCU_trsync` arms with `runtime_stop_on_trigger` (reverse the ceremonial no-op). |
| `klippy/motion_bridge.py` | `TripDispatch` wrappers; delete `extend_homing_deadline`; keep `_on_trip_message` completion. |
| `klippy/motion_toolhead.py` | `drip_move` GPIO branch: arm trsyncs + `TripDispatch`, 25 ms wall-clock sliced dispatch loop, stop on trip. |
| `klippy/extras/homing.py` | unchanged. |

## Out of scope / future

- **Same-MCU local-siren fast-path.** Re-enable `endstop.rs`'s local `AbortNow`
  so the detecting MCU freezes instantly without the relay round-trip; the relay
  still fans out to the others. This is the "optimize same-MCU later" step,
  gated on the cross-MCU relay being confirmed.
- **Occupancy-based metering.** v1 dispatches on wall clock; a future version
  could close the loop on MCU-reported ring depth for jitter robustness.
- **Trigger-time position from curve eval (Piece C).** Carried separately; the
  retained-curve evaluation for `trig_pos` is unaffected by this rework.

## Open risks

- **Relay latency vs mainline.** The reactor relay adds a host round-trip the C
  `trdispatch` avoided. Acceptable for "make it stop at all"; the local-siren
  fast-path closes the same-MCU gap, and cross-MCU latency (~ms) matches the
  external-probe budget already documented.
- **Slice boundaries vs planner.** Slicing a homing move into 25 ms pieces must
  preserve continuity through the ring pool the same way print pieces do;
  verify no spurious re-anchor at slice boundaries.
- **Beacon arming through the bridge serial shim.** Beacon's classic
  `trsync_start` is sent via the bridge text-send path; confirm delivery (this
  was the suspected break in the prior external-probe attempt).
