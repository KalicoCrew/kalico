# Homing: Stock Trsync Liveness + Metered Curve-Piece Drip

Supersedes Part B of `2026-05-31-trsync-cross-mcu-homing-design.md` and amends
its Part A arming. Part A's implementation (branch `trsync-cross-mcu-A`,
commits `504cb62b0..8aec6ec17`) is merged as-is except where §A-rev below says
otherwise.

## Goal

Working homing with failure behavior at parity with mainline Klipper/Kalico,
using the minimum count of mechanisms — each either stock or already built.
Not "safer than mainline": *as safe as mainline, with less machinery*. Beacon
firmware is stock-based and untouched; we speak its native protocol.

## Design principle

Mainline homing is four orthogonal primitives: detection (endstop), group stop
(trsync signal list), relay + liveness (trdispatch), bounded buffering (drip).
We keep that decomposition exactly and substitute implementations only where
the bridge engine forces it:

| Primitive | Mainline | Ours |
|---|---|---|
| Detection | `endstop.c` poll | `endstop.rs::tick` (GPIO/StallGuard) / Beacon firmware — exists |
| Group stop | `trsync_do_trigger` → `stepper_stop` (clears step queue) | `trsync_do_trigger` → `runtime_stop_on_trigger` → `software_trip` → evaluator freeze — **Part A, built** |
| Relay + liveness | `trdispatch.c` fastreader (serialqueue thread) | `TripDispatch` in the bridge reactor — **full port, this spec** |
| Bounded buffering | drip: 50 ms step chunks, time-paced | drip: 25 ms curve pieces through the pump, time-paced — **this spec** |

The same-MCU fast stop remains the degenerate case of the general path: the
local siren stays disabled for bring-up (Part A) so single-board homing
exercises the full relay round-trip; re-enabling it later is a one-line change,
not a second mechanism.

Earlier drafts of this rework explored bespoke liveness (deadline grants —
deleted in commit `208a186ed`; retirement-coupled piece release — rejected
during design). Both rejected for the same reason: they invent a second
liveness protocol beside the one stock trsync already provides and which Beacon
requires us to run anyway.

## Liveness: the trdispatch port

Every participating trsync — bridge sinks and classic sources (Beacon) alike —
is armed the mainline way:

- `trsync_start oid report_clock report_ticks expire_reason=REASON_COMMS_TIMEOUT`
  with `report_ticks = 0.3 × expire_timeout`, first reports staggered
  `report_offset = i / n_participants`.
- `trsync_set_timeout oid clock` arming the expire timer.
- Bridge sinks additionally get `runtime_stop_on_trigger arm_id trsync_oid`
  (Part A); classic MCUs get `stepper_stop_on_trigger` per stepper as today.

`TripDispatch` (bridge reactor, owns every participant's RX, no GIL) ports
`trdispatch.c::handle_trsync_state` faithfully:

1. **Trigger fan-out** — on any participant's `trsync_state can_trigger=0`
   (or a bridge `kalico_endstop_tripped` report): one-shot broadcast
   `trsync_trigger oid reason=REASON_ENDSTOP_HIT` to *all* participating
   trsyncs, including the originator. (Part A built this half.)
2. **Timeout extension** — on any participant's `can_trigger=1` report:
   update that participant's `last_status_clock` (clock32 → clock64 via the
   router's per-MCU clock estimate, the reactor analog of
   `serialqueue_get_clock_est`); compute the minimum acknowledged time across
   participants; for each participant, new
   `expire = anchor_time + expire_ticks` where `anchor_time` is the slowest
   *other* participant's time (the minimum-holder anchors to the
   second-minimum — no participant can extend itself by its own report); send
   `trsync_set_timeout` only when the new expire advances by
   `min_extend_ticks = 0.8 × report_ticks`.

Consequences, identical to mainline:

- Host dies / reactor dies → extensions stop → every MCU's expire fires →
  `trsync_do_trigger(REASON_COMMS_TIMEOUT)` → freeze, within
  `expire_timeout` of the last extension.
- Any participant goes silent (wedged sink, hung Beacon, dead link) → the
  min-acknowledged time stalls → every *other* participant's timeout stops
  advancing and expires → group freeze within `expire_timeout`.
- One protocol covers all participants; "source liveness" is not a separate
  mechanism — Beacon is just a participant whose reports feed the same
  minimum.

Accepted trade (mainline-identical): comms jitter exceeding the timeout aborts
homing with `REASON_COMMS_TIMEOUT` — loud, retryable, constants configurable.
This is mainline's known field failure mode; we inherit it knowingly rather
than invent an untested alternative.

## Bounded buffering: metered curve pieces

The homing move is a normal planner move; metering is two parameter-level
changes at existing seams, not a homing dispatch path:

- **Slicing**: homing-move pieces are subdivided to ≤ `DRIP_PIECE_MS = 25 ms`
  where pieces are minted (`enqueue.rs::flatten_axis` — today pieces inherit
  natural Bézier knot spans; a constant-velocity homing move would otherwise
  flatten to one piece spanning the whole travel).
- **Horizon**: homing pieces carry a tight pump lead —
  `DRIP_MAX_AHEAD_MS = 50 ms` instead of `MAX_LEAD_SECS = 1.0` — through the
  existing `horizon_of` gate. The class travels with the pieces:
  `EnqueueMsg` gains a lead-seconds field set by the planner per move class,
  so the pump stays policy-free. The pump's clock projection
  (`router.rs::ack_clock_and_freq`: `last_clock + elapsed × freq`) sweeps
  continuously, so release is smooth; the `StallAhead` re-poll tightens from
  50 ms to 10 ms while homing pieces are queued.

This is mainline's `DRIP_SEGMENT_TIME = 0.050` realized through the pump: the
MCU never holds more than ~50 ms of homing motion, so a trip abandons ≤ 50 ms
of issued pieces and the ring cannot be loaded with the whole travel. Pacing is
time-based — the same extrapolated-MCU-clock construct as mainline's
`estimated_print_time` drip gate. Liveness is *not* the drip's job (it is
trsync's); the ≤ 50 ms ring drain on host death remains as a passive property
underneath the timeout, exactly as mainline's drip queue sits under its
timeout.

**Stop/cleanup**: on trip, comms-timeout, or natural end, the not-yet-issued
homing pieces are flushed from the pump's host-side queues (new
`PumpMsg::Flush` variant — the only pump code addition) and the stream
re-anchors via the existing `fresh_stream` path before the next move. Frozen
evaluators ignore late pieces; the re-anchor reconciles ring cursors. A flush
that lands a tick late is harmless.

**Natural end (endstop never triggers)**: the move is finite; the planner
slices nothing past its end. Segment retires → `HomingState::Completed` →
homing fails loudly ("no trigger after full movement"). This is the structural
equivalent of mainline's host-fired `REASON_PAST_END_TIME`; no end-time clock
check is needed because, unlike mainline's trapq, there is nothing that could
feed motion past the planned move.

## Part A revisions (§A-rev)

Relative to the merged `trsync-cross-mcu-A` branch:

1. **Arming gains report + timeout params** — Part A armed bridge sinks with
   `report_clock=0 report_ticks=0` and no `trsync_set_timeout` ("Part B owns
   host-death"). Reversed: sinks arm with real report cadence and expire
   timeout per the liveness section. This also resolves Part A's recorded
   follow-ups: F2 (no host-death net) is closed by the expire timer; F1 (no
   clean disarm of untriggered sinks) is closed by mainline's own disarm —
   `TriggerDispatch.stop()` query-triggers with `REASON_HOST_REQUEST`, which
   clears the trsync. On a bridge sink that trigger fires
   `runtime_stop_on_trigger` → `software_trip` against an arm the host has
   already disarmed by that point in the stop sequence; `software_trip` on an
   inactive/mismatched `arm_id` must be a verified no-op (mainline's analog:
   `stepper_stop` on already-stopped steppers). Disarm ordering — endstop arm
   first, trsync stop second — is part of the contract.
2. **`probe_homing.rs` is deleted**, not generalized-in-place: Beacon becomes
   a `TripDispatch` participant `{source: trsync_state, classic arming via the
   bridge serial shim}`. Its three-phase Python API
   (`prepare/run/cleanup_probe_homing`) and `ProbeHomingResult` go away;
   `motion_toolhead.py`'s probe branch collapses into the same drip path as
   GPIO homing.
3. Beacon's trsync timeout is extended by `TripDispatch` like everyone's —
   no `sensor_fault_timeout` special case; a hung Beacon stalls the minimum
   and times the group out.

## Constants

| Name | Value | Provenance |
|---|---|---|
| `single_mcu_trsync_timeout` | 0.25 s | Kalico danger_options (configurable) |
| `multi_mcu_trsync_timeout` | 0.025 s | Kalico danger_options (configurable) |
| report cadence | 0.3 × timeout, staggered i/n | mainline `MCU_trsync.start` |
| extension hysteresis | 0.8 × report_ticks | mainline `min_extend_ticks` |
| `DRIP_PIECE_MS` | 25 ms | 2026-05-31 spec (½ × mainline's 50 ms chunk) |
| `DRIP_MAX_AHEAD_MS` | 50 ms | mainline `DRIP_SEGMENT_TIME` window |
| homing `StallAhead` re-poll | 10 ms | must release 25 ms pieces without starving |

Single- vs multi-MCU is counted over *participants in the homing move*, as
mainline counts trsyncs.

## Failure matrix

| Failure | Mechanism | Bound |
|---|---|---|
| Endstop trips | report → `TripDispatch` broadcast → freeze | ~ms (relay RTT); position exact via trip snapshot regardless |
| Host/process dies | extensions stop → expire on every MCU | ≤ timeout (25/250 ms); ring drain ≤ 50 ms beneath |
| Sink MCU wedges | its reports stop → group expire | ≤ timeout |
| Source (Beacon) hangs | its reports stop → group expire | ≤ timeout |
| Comms jitter > timeout | group expire, `REASON_COMMS_TIMEOUT` | loud abort, retryable (mainline parity) |
| Endstop never triggers | move retires → `Completed` → loud homing failure | end of planned travel |

Out of matrix, as in mainline: shared-axis multi-MCU homing is rejected at
config time (`mcu.py::add_stepper` check — retained).

## Files touched

| File | Change |
|---|---|
| `trsync-cross-mcu-A` branch | merge (6-file conflict resolution vs deadline rip-out + drift) |
| `rust/motion-bridge/src/trip_dispatch.rs` | + timeout-extension port (participants' clock tracking, min-anchor, hysteresis, `trsync_set_timeout` sends) |
| `rust/motion-bridge/src/probe_homing.rs` | delete; Beacon folds into `TripDispatch` |
| `rust/motion-bridge/src/enqueue.rs` | ≤ 25 ms piece subdivision for homing moves |
| `rust/motion-bridge/src/pump.rs` | per-move-class lead horizon; 10 ms homing re-poll; `PumpMsg::Flush` |
| `rust/motion-bridge/src/bridge.rs` / `homing.rs` | natural-end completion via `refresh_after_wait` / retire polling; flush-on-completion |
| `klippy/mcu.py` | bridge trsync arming gains report/timeout params (revise Part A's no-report arming) |
| `klippy/motion_toolhead.py` | probe branch collapses into the common drip path |
| `klippy/extras/homing.py` | unchanged |
| `src/*` firmware | no change beyond merged Part A (`runtime_stop_on_trigger`); `trsync.c` stock |

## Testing

1. **Unit**: extension algorithm against simulated report streams (min-anchor
   correctness, self-extension impossibility, hysteresis, silence → expiry);
   piece slicing (≤ 25 ms, continuity at slice boundaries); horizon gating +
   flush.
2. **Integration (exists, Part A)**: live-reactor relay test
   (`relay_reactor_integration.rs`) — extend with timeout-extension cases.
3. **Dual-MCU Renode sim**: trip on MCU A freezes MCU B via relay; silence on
   MCU A expires MCU B via timeout.
4. **Hardware ladder**: sensorless X on H7 through the relay (siren disabled,
   free air) → Beacon + F446 Z. Current bench symptom (homing travels through
   the crash) is diagnosed at rung 1, not speculatively.

## Out of scope / future

- Same-MCU local-siren fast stop (one-line re-enable once the relay is proven).
- Trigger-time position from retained-curve evaluation for software trips
  (Piece C, carried separately; GPIO trips already get exact MCU snapshots).
- Shared-axis multi-MCU homing (mainline rejects it; so do we).
