# Beacon Support: Fork Decision and Compatibility Survey

Decision record and function-by-function audit for supporting Beacon (and
later Cartographer/Eddy-class) external probes after the homing rewrite
(PR #34). Audited against beacon_klipper @ upstream HEAD (2026-06-10,
beacon.py 3944 lines) and cartographer-klipper scanner.py where noted.

## Decision

**Fork the beacon_klipper host module (separate repo:
`dderg/beacon_klipper`). Do not build a mainline-emulation layer.**

The device firmware is untouched by this decision — it is closed-source,
speaks its own commands over stock Klipper msgproto, and our `mcu.py` +
bridge passthrough already carry that protocol to any non-bridge MCU. Only
the ~4k-line Python module is at stake. It is GPL-3 (verified, as is
cartographer-klipper), so forking is legally clean.

Rationale:

1. **True emulation is unattainable, not merely expensive.** Mainline's
   probe-facing APIs expose mainline's motion model: `pull_move.accel`
   promises piecewise-constant acceleration; `get_past_mcu_position`
   promises a host-side step history. Our motion is polynomial curves
   evaluated on the MCU — those facts do not exist here. Any emulation
   fabricates them based on assumptions about how callers use the values,
   and fails *silently* on unaudited code paths. Concrete proof found in
   this audit: our vestigial `toolhead.get_trapq()` returns `None`, which
   stock beacon.py interprets as "not ready" and silently discards every
   streamed sample, forever. Stock code would "run" and do nothing.
2. **A fork fails loudly and audits mechanically.** Vendor updates arrive
   as `git merge`; every new API usage surfaces as a conflict or new code
   in the integration seam. Unported paths don't exist and raise at call
   time. This matches the project's fail-loudly constraint.
3. **The seam is narrow.** ~85% of beacon.py is device-protocol code
   (stream decode, models, calibration, temperature compensation, contact
   algorithms, accelerometer) that talks to the beacon device and barely
   touches klippy internals — it merges cleanly from upstream forever. The
   incompatible remainder concentrates in identifiable seams, catalogued
   below. Cartographer's scanner.py is a beacon.py descendant with
   near-identical touchpoints; everything here generalizes.

## Audit ledger

Status of every klippy API beacon.py consumes, against this tree.

### Motion history (streaming/scanning backbone)

| API | Beacon's actual usage | Status in our tree | Resolution |
|---|---|---|---|
| `toolhead.get_trapq()` | Readiness guard only (`if self.trapq is None: drop samples`). Cartographer: guard only. | Returns `None` by design (toolhead.py:318) → silent total sample drop | Fork: replace guard with own connect flag |
| `chelper trapq_extract_old` FFI | Contact homing only: sign of `accel` at detect time — reject triggers during accel/decel, accept cruise-phase only (beacon.py:2407) | Symbol deleted from chelper (trapq.c removed) → AttributeError | `bridge.motion_state_at(T).accel` — exact instantaneous accel from retained curve, strictly better than mainline's per-move constant |
| `stepper.get_past_mcu_position(t)` + `mcu_to_commanded_position` + `kin.calc_position` | `_get_position_at_time(print_time)` → toolhead XYZ for every streamed sample. THE backbone of scanning. Identical helper in cartographer. | Methods deleted from `MCU_stepper` (host generates no steps). `extras/angle.py:76` and `kinematics/extruder.py:84` still call them — already broken | `bridge.motion_state_at(T)` → (pos, vel, accel). One call replaces the per-stepper decompose/recompose round trip |

Time→position anchoring (the design that makes this exact): dispatched
pieces are retained with `start_time` in the **axis MCU's own clock ticks**
(`PieceEntry`, runtime/src/piece_ring.rs:285) — the same numbers the MCU
executes against. Host clock resyncs revise estimates of the *mapping
between domains*; they cannot retroactively change what the MCU did at
clock C, so within-domain past queries are exact and resync-immune.
Cross-MCU events (beacon sample clock → motion MCU clock) cross via the
router's sync state (homing.rs:109-143, plumbing already exists for every
MCU — serialhdl.py:386-406 claims all MCUs on the bridge router); for
events under ~1 s old the regression drift contributes sub-µm error at
scan speeds. One wart to fix while generalizing: `PieceEntry.duration` is
stored in seconds and converted to ticks at query time with the *current*
frequency estimate (piece_ring.rs:314); store the dispatched tick count
instead so within-domain queries touch no sync state at all.

### Homing trigger (trsync cluster)

What the closed firmware dictates (immovable): `beacon_home
trsync_oid=… trigger_reason=… trigger_invert=…` and `beacon_contact_home
trsync_oid=…` require the OID of a **trsync on the beacon MCU itself**;
on trigger the firmware fires that local trsync (stock trsync protocol:
`config_trsync`, `trsync_start`, `trsync_set_timeout`, `trsync_state`
reports). `trsync_set_timeout` is the beacon-side deadman.

| API | Beacon's actual usage | Status in our tree | Resolution |
|---|---|---|---|
| `MCU_trsync` on the beacon MCU | Trigger observation: `trsync_state` reports complete the trigger completion | Python-level `_handle_trsync_state` exists (mcu.py:306) and `trsync_state` arrives via bridge passthrough poller (serialhdl.py:128). Needs verification of response registration | Keep — observation path is Python-level, no C relay needed |
| `trdispatch_alloc/start/stop` C relay | Mainline's ~1-2 ms trigger relay: beacon trsync_state intercepted on serial RX thread → `trsync_trigger` to stepper MCU | `_trdispatch_mcu` pinned `None`; relay disabled | Replace with software-trip into the bridge homing arm (see primitives). Relay options: (a) Python observer → `software_trip` (~2-4 ms, unbounded under reactor load) or (b) Rust router-thread relay (~0.1-1 ms, bounded) — decide in spec B |
| `stepper_stop_on_trigger` on motion MCU | Kill queued steps on trigger | Meaningless — no step queue; bridge arm freeze is the stop mechanism | Bridge arm freeze (exists for GPIO trips) triggered via software trip |
| Secondary `MCU_trsync` per stepper MCU (`BeaconEndstopShared.add_stepper`) | Mainline multi-MCU coordination | `src/trsync.c` still compiles (idle alloc harmless), but serves no purpose | Fork: don't create them. One trsync on the beacon MCU only — deletes the ceremonial-trsync/aggregation hack the old spec needed |
| `home_start(print_time, …)/home_wait(home_end_time)` endstop contract | Eddy: precise trigger time NOT needed (`home_wait` returns `home_end_time`; true Z comes from post-home `_sample()` override). Contact: precise detect time from `beacon_contact_query` + cruise-phase validation | Contract removed by homing rewrite | Fork: implement our virtual-endstop provider contract (`setup_bridge_endstop`, `trip_move_begin/end`, `get_position_endstop`) |
| `from .homing import HomingMove` (beacon.py:31!) | Drives its own contact probing + calibration descent moves (`hmove.homing_move(pos, speed, probe_pos=True)` at 622, 1443, 1539) | Import-time crash — class deleted | Fork: replace with our probing-move primitive (trip_move generalized beyond G28) |

Dead scaffolding found in our tree: `MCU_trsync.start()` (mcu.py:357)
sends `runtime_stop_on_trigger arm_id=… trsync_oid=…` — **no firmware
handler exists**; only the Python sender and a stale log-code template
survive. `_bridge_arm_id` is never assigned, so `start()` raises
unconditionally. This is the host-side remnant of the demolished
`_bridge_drives_steppers` prototype (commits 45f0fe737, 3b659eaef).
Spec B replaces it; until then it fails loudly.

Safety requirement that survives from the old spec: an external-probe
homing move has no MCU-local GPIO to stop it. If the host dies
mid-descent, the dispatched curve runs full travel into the bed. The
motion MCU needs a deadman for software-trip homing arms — the
credit-window deadline design (50 ms grants) from
`external-probe-homing.md` Piece B carries forward.

### Homing events / homed-state

| API | Beacon's actual usage | Status in our tree | Resolution |
|---|---|---|---|
| `homing:home_rails_begin/end` events | Beacon SENDS them itself around its own homing (with minimal `BeaconHomingState`: `get_axes()→[2]`, no-op `set_homed_position`) and LISTENS to adjust position post-G28-Z | Listeners alive (gcode_move.py:27, z_thermal_adjust, z_calibration, endstop_phase). Our G28 emits **nothing** — only trad_rack emits these | Beacon's own synthesis keeps working. The listener path is dead on our G28 → post-home `_sample()` adjustment must move into our provider contract (post-trip hook / measured-position override) — spec B |
| `homing_state.set_homed_position([None,None,dist])` | Override homed Z with measured distance after G28 | No Homing-state class; our homing.py sets position from `trigger_height + overshoot` | Provider contract extension: provider supplies measured trigger height post-trip (spec B) |
| `kin.note_z_not_homed()` / `kin.clear_homing_state("z")` / `set_position(homing_axes=…)` | Mark Z unhomed around contact ops | All present on `BridgeKinematics` (motion_toolhead.py:195-210), both string and index forms handled by beacon's own compat shim | Works as-is |

### Toolhead surface

All present and semantically compatible — `MotionToolhead(ToolHead)`
subclasses the legacy class (motion_toolhead.py:235), so `manual_move`,
`get_position`, `wait_moves`, `get_kinematics`, `set_position(homing_axes)`,
`dwell`, `get_status` (incl. `homed_axes` from BridgeKinematics and
`max_accel`), `move`, `get_extruder`, and
`gcode_move.get_status()["homing_origin"]` all work. Verified semantics:
`flush_step_generation` waits for MCU-side execution (compatible with
beacon's sync usage); `get_last_move_time` returns print-time projection
(compatible).

### Probe object interface

Beacon's `BeaconProbeWrapper` implements a hybrid of the old
(`run_probe`/`get_offsets`/`multi_probe_begin`) and new
(`start_probe_session`/`pull_probed_results`/`get_probe_params`) mainline
interfaces — richer than our rewritten `PrinterProbe`.

**Our own tree is broken here independently of beacon** (probe rewrite
816846c19 stranded its consumers):

- `z_tilt.py:176`, `quad_gantry_level.py:35`, `screws_tilt_adjust.py:50`
  reference `probe.ProbePointsHelper` — deleted → config-load failure.
  The Trident bench uses Z_TILT_ADJUST; this blocks any probe, not just
  beacon.
- `axis_twist_compensation.py` needs `get_lift_speed()` (missing) and
  indexes `run_probe(gcmd)[2]` (ours returns a scalar float).
- `bed_mesh.py:624` needs only `get_offsets()` — works.

Resolution: spec C restores a native points-helper/session surface in our
probe.py; the beacon fork then presents the same interface (it nearly
does already).

PROBE/PROBE_ACCURACY command collisions between `[probe]` and `[beacon]`
are pre-existing mainline behavior (mutually exclusive config; beacon has
`prefixed_probe_commands`) — no action.

### Accelerometer

Fully compatible, no gaps. Beacon duck-types the adxl345 contract
(`start_internal_client()` → client with
`finish_measurements/has_valid_samples/get_samples/write_to_file`), uses
`adxl345.AccelCommandHelper` for gcode registration; our
`extras/adxl345.py`, `bulk_sensor.py`, and `resonance_tester.py` are
unrewritten and the toolhead methods they use are present.

### Temperature compensation / models / calibration

Device-protocol + NVM + pure-Python model math; no klippy APIs beyond
those above. Expected to merge-and-run; validated in spec E.

## Native primitives to build (our repo)

- **P1 — Executed-motion history service** (`motion_state_at`):
  generalize the homing-only `homing_trajectory` retention
  (bridge.rs:464) into a bounded ring of dispatched pieces per
  (MCU, axis) for ALL motion; store dispatched tick counts; expose
  `motion_state_at(time) → (position, velocity, acceleration)` over the
  bridge FFI with cross-MCU clock conversion reusing
  `reconstruct_axis_position`'s path. Serves: beacon/cartographer
  streaming, contact cruise-check, broken `angle.py`/extruder callers,
  and future closed-loop servo reference ("commanded state at T" is the
  signal encoder feedback compares against).
- **P2 — External-trigger homing**: software-trip homing arms (no GPIO
  source), MCU-side credit-window deadman, trigger relay (Python observer
  vs Rust router relay — decision in spec), provider-contract extensions
  (software-armed `BridgeEndstop` variant, post-trip measured-position
  override), and a probing-move primitive replacing beacon's direct
  `HomingMove` usage.
- **P3 — Probe ecosystem restoration**: native ProbePointsHelper/session
  interface + `get_lift_speed` + result-shape fix in our probe.py for
  z_tilt/QGL/screws_tilt_adjust/axis_twist_compensation (needed for any
  probe, beacon or not).

## Sub-project decomposition (one spec each, dependency order)

1. **Spec A — executed-motion history service** (P1). No beacon
   dependency; independently valuable; testable in sim.
2. **Spec B — external-trigger homing + probing moves** (P2). Uses the
   existing trip reconstruction; contact-time validation consumes P1.
3. **Spec C — probe interface restoration** (P3). Independent of A/B;
   parallelizable; unblocks Z_TILT_ADJUST on the bench.
4. **Spec D — the beacon fork seam**: rewrite the integration layer in
   `dderg/beacon_klipper` onto P1+P2+P3 (endstop wrappers → provider
   contract, `_get_position_at_time` → `motion_state_at`, HomingMove →
   probing primitive, trapq guard → connect flag, single-trsync
   simplification).
5. **Spec E — capability validation tail**: scanning/bed-mesh, contact
   calibration flows, accel/resonance, temp comp — sim-first with the
   beacon emulator (`tools/kalico-sim/emulators/beacon_mcu.py` +
   `third_party/beacon_klipper` symlink path already exist in the
   runner), then bench.

## Relationship to `external-probe-homing.md`

That spec predates the homing rewrite and assumed mainline's
`drip_move`/`home_wait` flow plus "zero changes to beacon.py" — both
premises are gone, and its Piece A (`_bridge_drives_steppers`) was
implemented and deliberately demolished (45f0fe737). It is superseded by
this survey. Two of its ideas carry forward into spec B: the
credit-window deadman (Piece B) and host-retained-curve trigger-time
evaluation (Piece C, now generalized as P1).
