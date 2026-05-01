# Step 7-C-bridge — Python ↔ Rust motion bridge

**Layer:** Cross-cutting (binds Layers 1–5 into the existing Klipper Python host process). **Build-order step:** 7-C-bridge.

**Scope:** Wire the existing Klipper Python host (`klippy/`) to the new motion stack (`gcode` / `geometry` / `temporal` / `trajectory` / `runtime` / `kalico-host-rt`) so existing user printer configs route motion through the new planner. Keep the Klipper config language, the entire `extras/` ecosystem (heaters, sensors, fans, macros, beacon, motors-sync, etc.), and the reactor model intact; replace the trapezoidal motion path end-to-end. By the end of this step the repository contains zero trapezoidal motion code, and the Trident test bench is one bring-up step (7-D) away from a real print.

This spec is the *design* for 7-C-bridge. The per-phase implementation plan is the next deliverable (`superpowers:writing-plans`); the spec defines architecture, surface area, deletions, and phasing. Hardware bring-up (cycle-budget actuals, calibration, first physical print) lives in 7-D.

## 1. Goals & non-goals

### 1.1 In scope

1. **PyO3 bridge module** exposing a `MotionBridge` Python handle backed by `kalico-host-rt` and the planner crates. Single in-process Python module; klippy `import`s it like any other extension.
2. **Wire ownership transfer:** the bridge owns the serial fd to **every** Klipper-protocol MCU — motion (Octopus, F446 bottom) *and* non-motion (beacon, NIS). Klippy `serialhdl.py`'s C-side `serialqueue.*` is replaced wholesale; klippy `mcu.py` keeps the Klipper-protocol Python machinery (msgproto, command construction, response dispatch, oid management) but talks to the bridge instead of opening fds directly.
3. **`motion_mcu.py` shim** preserves the public API of `klippy/mcu.py` for *all* MCUs, routing message bytes + scheduling metadata (minclock/reqclock, command-queue ids, response handlers) through PyO3 → Rust so existing extras (TMC drivers, heaters, GPIOs, sensors, beacon) continue to work unchanged.
4. **`motion_toolhead.py` shim** preserves the public API of `klippy/toolhead.py` for extras that call `manual_move`, `dwell`, `wait_moves`, `set_position`, `register_lookahead_callback`, `flush_step_generation`, etc. Backed by the bridge.
5. **Live G1 / G5 / G5.1 / G2 / G3 motion** through the bridge. G1 and G2/G3 are normalized to G5 in-line via the existing `compat` crate's primitive API for live (terminal/macro) submissions.
6. **Pre-print file preprocessing** for `.gcode` print files: `compat`'s library normalizes the file once at print-start to a sibling `.g5.gcode`, then klippy's existing print-from-file path runs the normalized file. Mtime+size cache to avoid reprocessing.
7. **Multi-MCU motion routing.** The bridge supports ≥2 motion MCUs (Trident: Octopus H723 for X/Y/E + F446 "bottom" for Z). Segments fan out to the right MCU based on per-stepper assignment.
8. **Drip-move + endstop coordination** for homing and probing. Klippy `homing.py` orchestration stays; bridge exposes `arm_endstops` + `submit_drip_move` for the streaming side. Both endstop kinds work: motion-MCU sensorless TMC endstops (passthrough) and non-motion-MCU endstops (beacon Z) — `trsync` coordinates across MCUs as today.
9. **Per-stepper independent-axis path (Ring 2)** on MCU runtime + host-rt + bridge. Drives a single stepper outside the kinematic transform. Backs `manual_stepper`, `force_move`, `[cflap]`, `motors_sync`, and future per-stepper offsets (phase stepping).
10. **Config translation surface** that accepts the user's existing `printer.cfg` motion sections, hard-errors knobs that became meaningless, and adds new ones for the planner.
11. **Deletion of all trapezoidal motion code** in `klippy/` (toolhead, lookahead, kinematic step generators, `klippy/chelper/itersolve.*` / `stepcompress.*` / `serialqueue.*`, `manual_stepper.py`, `force_move.py`, `gcode_arcs.py`, etc.). Phase 1 of the build does the deletion up-front.
12. **CI extras-audit script** that imports every `klippy/extras/*.py` against the new shim and fails if it touches a deleted method. Catches drift.

### 1.2 Out of scope (deferred to 7-D and beyond)

- **Hardware bring-up:** Surface-C cycle-budget actuals on real Octopus H723, IWDG real-world pacing, USB-CDC physical-unplug semantics, calibration of shaper frequencies on the test bench, first physical print. (7-D.)
- **Pressure Advance.** PA is Step 9. The bridge accepts `pressure_advance` / `pressure_advance_smooth_time` config knobs without erroring (forward-compatible) but they are no-ops until Step 9 lands.
- **Other smooth-shaper kernels** (`smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si`). Step 8.
- **Corner-blend finalization** beyond Step-7-A's MVP behavior. Step 8.
- **EtherCAT.** Step 14.
- **Skip detection / mechanical-frequency tracking.** Steps 11–12.
- **Phase stepping.** Step 10. Bridge's per-stepper override path is deliberately the right shape for future phase-stepping per-stepper offsets, but no phase-stepping logic lands here.
- **Kinematics other than CoreXY and Cartesian.** Polar / delta / hybrid / dual-carriage parse-error with "kinematic <foo> not yet supported under the new motion path." Adding a kinematic later is a small isolated change.
- **`[manual_stepper]` and `motors_sync` upstream-patch coordination.** We provide the new backend; if upstream motors-sync needs a small shim patch to land on it, that's its own conversation with the plugin author.

### 1.3 Non-goals (deliberate punts)

- **Out-of-process daemon model.** PyO3 in-process is the chosen process model (§2.1). No IPC schema is invented on top of the MCU wire.
- **Strict drop-in compat with every Klipper config knob.** Trapezoidal-only knobs hard-error with a clear "remove this line" message; the user does a one-time small edit. No silent-ignore escape hatch.
- **A separate `kalico.toml` motion config file.** Existing `printer.cfg` is the single source.
- **Re-implementing klippy `homing.py` on the Rust side.** Stays Python; bridge offers the streaming primitives only.
- **A new naming for the existing Rust crates** (`kalico-host-rt`, `kalico-c-api`). Deferred to a one-shot rename pass when the fork gets a real name.

### 1.4 CLAUDE.md / dependency-graph amendment for live macro G1/G2/G3

CLAUDE.md and `docs/kalico-rewrite/dependency-graph.md` currently say "no live G0/G1/G2/G3" and "legacy G0/G1/G2/G3 are not handled by the live parser; the compatibility layer (Step 13) normalizes those offline before printing." That framing was written assuming all G-code arrives via files (slicer output). It does not account for the reality that gcode-macros and the Mainsail terminal emit G1/G2/G3 programmatically, and rewriting every macro to G5 is impractical.

This spec therefore commits to a **deliberate amendment** of CLAUDE.md and dependency-graph.md, landing in Phase 1 alongside the rest of 7-C-bridge. The amended rule:

> **The planner crates** (`rust/gcode`, `rust/geometry`, `rust/temporal`, `rust/trajectory`) only ever process G5 / G5.1. The `rust/gcode` lexer rejects G0 / G1 / G2 / G3 as a hard error.
>
> **The bridge** (`rust/motion-bridge`) sits above the planner crates. It accepts G1 / G2 / G3 from klippy's Python gcode handlers (terminal commands, gcode-macros) as *structured* motion intent (deltas + feedrate + arc params), converts to G5 control points using the existing `compat` crate's primitive API (`compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`), and submits G5 to the planner. The bridge is not a planner crate; the lexer-reduce boundary still sees only G5/G5.1.
>
> **File-based prints** are normalized once at print-start by the same `compat` primitives (§4.7), producing a G5-only `.g5.gcode` file that flows through klippy's existing print path — so even the file path produces only G5 at the lexer boundary.

The amendment is tracked under §9 and is a Phase-1 deliverable. Without it, this spec is in tension with CLAUDE.md as written.

## 2. Architecture

### 2.1 Process model: PyO3 in-process

The bridge is a Python extension module loaded by klippy. Single OS process. Klippy's reactor remains the only event loop on the Python side. Rust threads (host-rt I/O reactor, planner workers, dispatch thread) run alongside, communicating with Python through:

- **Synchronous PyO3 method calls** (Python → Rust): `submit_*`, `passthrough_*`, `arm_endstops`, lifecycle. Each releases the GIL via `py.allow_threads(...)` for any blocking work.
- **One-way event queue** (Rust → Python): Rust pushes events onto a lock-free MPSC queue and writes one byte to a `event_fd` pipe. Klippy registers `event_fd` with its reactor; on wake, the reactor calls `bridge.poll_event()` until the queue is empty and dispatches each event to the registered Python callback.

Rust never calls into Python code directly. This matches `klippy/serialhdl.py`'s existing pattern (background thread → reactor wakeup) but moves the boundary from fd-read to PyO3.

```
┌───────────────────────── Python (reactor thread) ─┐
│                                                    │
│  reactor.register_fd(bridge.event_fd, drain)       │
│       │                                            │
│  drain():                                          │
│    while ev := bridge.poll_event():                │
│       dispatch(ev)                                 │
│                                                    │
│  bridge.submit_move(...)         ◄── synchronous   │
│  bridge.submit_drip_move(...)    ◄── synchronous   │
│  bridge.passthrough_send(...)    ◄── synchronous   │
└──────────┬─────────────────────────────────▲──────┘
           │ PyO3 (sync)                     │ event_fd write
           ▼                                 │
┌─────────────────────────── Rust ───────────┴──────┐
│  MotionBridge (Send + Sync)                        │
│   - submit/passthrough → enqueue → planner workers │
│   - poll_event → MPSC pop                          │
│                                                    │
│   ┌─ host-rt I/O reactor (per motion MCU) ─┐       │
│   │   read/write loop, NAK/RTO,            │       │
│   │   clock-sync, event dispatch           │       │
│   └────────────────────────────────────────┘       │
│   ┌─ trajectory worker pool (on demand) ───┐       │
│   │   TOPP-RA + shape-batch + β-medium     │       │
│   └────────────────────────────────────────┘       │
└────────────────────────────────────────────────────┘
```

### 2.2 Wire ownership and component map

The bridge owns the serial fd to **every** Klipper-protocol MCU. `serialqueue.c` and the C-side of `serialhdl.py` go away entirely; klippy `mcu.py` keeps the Klipper-protocol Python machinery (msgproto framing, command construction, response dispatch, oid management) but routes all wire traffic through the bridge. This is cleaner than the original "motion MCUs only" framing because `serialhdl.py`'s `serialqueue_alloc` / `serialqueue_send` / `serialqueue_alloc_commandqueue` are used uniformly across motion and non-motion MCUs — a partial split would require keeping `serialqueue.c` alive for non-motion only.

```
                   ┌── Octopus H723 fd  (motion: X/Y/E + extruder heater + part-MCU sensors)
                   ├── F446 bottom fd   (motion: Z + bed heater + frame thermistors)
   kalico-host-rt ─┼── Beacon fd        (non-motion: Z probe / Z endstop / accelerometer)
        ▲          ├── NIS fd           (non-motion: nozzle adxl345)
        │          └── (any other Klipper-protocol MCU)
        │
        │ passthrough_send(bytes, minclock, reqclock, queue_id)
        │ passthrough_query(...) → notify_id
        │ passthrough_register_handler(name, oid, callback)
        │ event_fd → reactor drain (responses, faults, segment events)
        ▼
   ┌──────────────────────────┐
   │  klippy mcu.py + msgproto│  Klipper-protocol Python machinery preserved.
   │  (per-MCU instance)      │  Calls bridge.passthrough_* instead of writing to a serial fd.
   └────────▲─────────────────┘
            │ same public API as before (lookup_command, add_config_cmd,
            │ register_response, register_flush_callback, alloc_command_queue,
            │ estimated_print_time, print_time_to_clock, …)
            │
   extras/heaters.py, extras/tmc5160.py, extras/temperature_sensor.py,
   extras/output_pin.py, extras/adxl345.py, beacon plugin, motors_sync, …
   (everything that talked to mcu.py before still talks to mcu.py)
```

Salient points:
- **All MCUs** flow through the bridge's passthrough router. Bridge interleaves passthrough commands with motion commands on each MCU's wire under a single seq stream per MCU.
- **`motion_mcu.py` is a thin proxy class.** klippy `mcu.py`'s constructor branches: instead of allocating a `serialqueue` and opening an fd, it asks the bridge for a `MotionMcuProxy` for the given MCU id. The proxy implements the public surface (`lookup_command`, `add_config_cmd`, `register_response`, `register_flush_callback`, `alloc_command_queue`, `estimated_print_time`, `print_time_to_clock`, `clock_to_print_time`, etc.) by delegating to bridge passthrough APIs. Decision deferred to Phase 1: branch in `mcu.py` vs. factor into separate `motion_mcu.py` class. Either way the public Python surface is identical.
- **Endstops on Klipper-protocol MCUs** (sensorless TMC virtual_endstop on motion MCU; beacon Z endstop on beacon MCU): both go through `mcu_endstop` against the proxy, which issues `endstop_home` / `endstop_query_state` as passthrough commands on the right wire. Step 7-B's runtime-side endstop logic preserves the Klipper endstop protocol, so the wire shape is unchanged. `trsync` coordinates triggers across MCUs as today; `trsync_*` commands are also passthrough.
- **The `non-motion` distinction collapses** at the wire-ownership level. It still matters at the *motion* level: only motion MCUs receive `submit_segment` / `arm_endstop` / drip-move commands; non-motion MCUs only see passthrough traffic.

### 2.3 Crate / file layout

**Rust workspace additions:**

```
rust/
  motion-bridge/                             # NEW — PyO3 module crate
    Cargo.toml                               # crate-type = ["cdylib"], depends on host-rt + trajectory + temporal + geometry + gcode + compat
    src/
      lib.rs                                 # PyModule + #[pyclass] MotionBridge
      api.rs                                 # PyO3 method bindings — submit_*, passthrough_*, arm_endstops, lifecycle
      classify.rs                            # delta_xy/delta_e classifier (COUPLED vs INDEPENDENT)
      events.rs                              # Rust event types + Python event drain queue
      preproc.rs                             # File preprocessor (compat → .g5.gcode + mtime cache)
      kinematics_spec.rs                     # KinematicsSpec + per-family parsing helpers
      independent.rs                         # Ring-2 per-stepper independent-axis path host side
  compat/                                    # CHANGED — primitive API exposed for non-CLI use
    src/
      lib.rs                                 # re-export existing primitives:
                                             #   compat::collinear::to_collinear_g5
                                             #   compat::arc::arc_to_g5 (XY-only; bridge wrapper distributes E/F)
                                             #   compat::degree_elev::elevate_g51_to_g5
                                             # text-I/O bits stay confined to converter.rs / main.rs (CLI binary).
                                             # Bridge wraps arc_to_g5 to handle E and F distribution along the
                                             # multi-piece arc decomposition (current arc_to_g5 leaves both to caller)
                                             # and to support R-format arcs (current converter.rs rejects R-form).
```

**Klippy-side changes:**

```
klippy/
  motion_bridge.py                           # NEW — Python wrapper around the PyO3 module; adapts to klippy reactor
  motion_toolhead.py                         # NEW — replaces toolhead.py; full method-by-method compatibility matrix in §3.6.2
  motion_mcu.py                              # NEW — proxy class implementing klippy mcu.py public API by delegating to bridge passthrough; covers ALL MCUs
  motion_kinematics.py                       # NEW — config parsing per kinematic family → KinematicsSpec
  toolhead.py                                # DELETED
  stepper.py                                 # PATCHED — see §5.2 for the full MCU_stepper public-API list preserved (calc_position_from_coord, get_past_mcu_position, dump_steps, get_stepper_kinematics, set_stepper_kinematics, set_trapq, …); trapezoidal motion internals gutted and routed through bridge
  kinematics/
    extruder.py                              # PATCHED — keeps ExtruderStepper config-object surface (extras/extruder_stepper.py imports it directly); trapezoidal pressure-advance bits routed through bridge (PA params remain forward-compatible no-ops until Step 9)
    idex_modes.py                            # PATCHED — IDEX dual-carriage mode-switch logic stays Python; physical motion routed through bridge (out-of-MVP-scope kinematic; verify import-time only)
    cartesian.py, corexy.py, corexz.py, …    # DELETED — all step-generator kinematics removed; replaced by motion_kinematics.py + MCU-side transforms
  chelper/                                   # CHANGED — itersolve.c, stepcompress.c, serialqueue.c (and serialhdl.py's C-loading of it), kin_*.c, trapq.c, trapq.h removed. The remaining non-motion C bits (CRC helper, msgblock util) are tiny; either retain or rewrite in Python. Decided during Phase 1.
  serialhdl.py                               # PATCHED OR DELETED — the C-side serialqueue allocator goes; what remains (Python-side msgparser + identify handshake) either stays as a thin shim over bridge.passthrough_* or is folded into motion_mcu.py. Decision deferred to Phase 1
  mcu.py                                     # PATCHED — constructor branches: instead of allocating serialqueue + opening fd, allocates a MotionMcuProxy via bridge.claim_mcu(serial_path, baud). Public surface (lookup_command, add_config_cmd, register_response, register_flush_callback, alloc_command_queue, estimated_print_time, print_time_to_clock, clock_to_print_time) preserved unchanged via the proxy. (Fallback if branched class grows ugly: factor MotionMcuProxy into motion_mcu.py with factory in mcu.py.)
  extras/
    gcode_move.py                            # UNCHANGED — set_move_transform / position_with_transform / move_with_transform stays as today; it sits above motion_toolhead and chains transforms exactly as it does above toolhead today. (Path is klippy/extras/gcode_move.py.)
    homing.py                                # PATCHED — drip_move via bridge, arm_endstops via bridge
    manual_stepper.py                        # PATCHED — same filename, same MANUAL_STEPPER command surface, internals reimplemented against bridge Ring-2
    force_move.py                            # PATCHED — same filename, same SET_KINEMATIC_POSITION / FORCE_MOVE command surface, internals reimplemented against bridge Ring-2
    input_shaper.py                          # PATCHED — drops trapezoidal IS C-allocation path (input_shaper_alloc / get_stepper_kinematics / set_stepper_kinematics); becomes a config-parser that emits ShaperSpec for the bridge plus a SET_INPUT_SHAPER runtime command that calls bridge.update_shaper(). Also drops the per-stepper note_step_generation_scan_time call (extruder PA scan-time becomes a no-op until Step 9)
    motion_report.py                         # PATCHED — drops the trapq-based DumpTrapQ endpoint (the trapq doesn't exist anymore). Same `motion_report` lookup name preserved for moonraker compatibility, and exposes the same { trapqs: { "toolhead": Dump..., "<extruder>": Dump... } } dict shape backed by bridge-state queries (position / velocity / segment metadata via the existing dump-stepper protocol). load_cell/tap_analysis.py reads motion_report.trapqs["toolhead"] directly — that consumer keeps working iff the dict shape is preserved
    z_tilt.py, z_tilt_ng.py                  # PATCHED — your config uses z_tilt_ng. Both today call s.set_trapq(toolhead.get_trapq()) on Z steppers to detach/reattach during tilt probing. Replacement: stepper.set_trapq(None) becomes "detach this stepper from kinematic transform on the bridge"; stepper.set_trapq(toolhead.get_trapq()) becomes "reattach to bridge's main motion source". MCU_stepper preserves these method names; bridge backs them. About 30 lines of surgical patching per file
    gcode_arcs.py                            # DELETED — bridge handles arcs natively
    audit_extras.py                          # NEW — CI tool, scans all extras for deleted-method references
```

**Hard-disabled-for-MVP (config-time error if user has them enabled, post-MVP for real support):**

- `extras/mixing_extruder.py` — uses `stepper.get_trapq()` + `set_trapq()` heavily for runtime trapq-swapping. Refactor onto bridge Ring-2 is non-trivial; not a Trident config user-feature. Hard-disabled.
- `extras/trad_rack.py` — Tradrack MMU; allocates its own private `trapq` and runs an entire secondary `tr_toolhead`. Substantial port work. Hard-disabled.
- `extras/pwm_tool.py` — uses `stepcompress` directly for pulse-train output. Niche feature; not in your config. Hard-disabled.
- `extras/load_cell/tap_analysis.py` — reads `motion_report.trapqs["toolhead"]`. Kept compatible iff `motion_report` preserves the dict shape (above). If `tap_analysis` reaches into trapq-internal methods we don't replicate, it gets hard-disabled too — verified during the audit.

The "hard-disabled" mechanism: each module's config-loader raises with a clear message identifying the trapezoidal-internal dependency and pointing at the post-MVP backlog item.

## 3. Bridge API surface

The PyO3 surface is shaped around motion *intent*, not G-code text. All inputs are post-G92 / post-G90/G91 / post-M82/M83 / post-feedrate-modal — exactly the structured form klippy's `gcode.py` already produces in its handlers.

### 3.1 Construction and lifecycle

```python
bridge = motion_bridge.MotionBridge(
    motion_mcus = [
        MotionMcuSpec(id="main",   serial=..., baud=..., axes=["X","Y","E"]),
        MotionMcuSpec(id="bottom", serial=..., baud=..., axes=["Z"]),
    ],
    kinematics  = KinematicsSpec.CoreXY(steppers=[...], axis_limits=...),
    shaper      = ShaperSpec(x=SmoothMzv(189.6), y=SmoothMzv(118.8), z=Passthrough()),
    refit_tolerance_mm    = 0.005,
    beta_max_iters        = 8,
    beta_convergence_ratio= 0.999,
    topp_grid_strategy    = "auto",
    worker_threads        = 3,
    e_limits              = AxisLimits(v_max=100, a_max=1000),
    event_fd              = <write end of os.pipe()>,
)
bridge.connect()       # opens transports, runs identify, arms MCUs
bridge.shutdown()      # joins threads, drops handle
```

### 3.2 Motion submission (synchronous, returns immediately)

```python
bridge.submit_move(delta_xy, delta_z, delta_e, feedrate, src_line) -> MoveHandle
bridge.submit_g1(...)        # explicit form; submit_move is the unified entry point
bridge.submit_g5(p0, p1, p2, p3, feedrate, e_mode, ratio, src_line)
bridge.submit_g5_1(p0, p1, p2, feedrate, ...)
bridge.submit_arc(start, end, ij_or_r, plane, dir_cw, delta_e, feedrate, src_line)
bridge.submit_dwell(duration_s)
bridge.submit_set_position(xyz_e)
```

Internal classification:
- `|delta_xy| > eps` and `|delta_e| > eps` → COUPLED, `ratio = de / |delta_xy|`
- `|delta_xy| > eps` and `|delta_e| ≈ 0` → COUPLED travel, `ratio = 0`
- `|delta_xy| ≈ 0` and `|delta_e| > eps` → INDEPENDENT E NURBS
- `|delta_xy| ≈ 0` and `|delta_z| > eps` → Z-only path

`submit_g1` degree-elevates to a single-piece cubic Bézier with collinear control points (1/3, 2/3 lerp) via `compat::collinear`. `submit_arc` decomposes via Goldapp (`compat::arc`) into multi-piece cubic Bézier and submits each piece.

### 3.3 Drip / homing

```python
arm_token = bridge.arm_endstops([
    EndstopArm(mcu_id="main",   oid=12, trigger_time=t),  # tmc5160 sensorless on motion MCU
    EndstopArm(mcu_id="beacon", oid=3,  trigger_time=t),  # non-motion-MCU endstop (klippy mcu.py side)
])

handle = bridge.submit_drip_move(
    end_xyz_e, feedrate,
    arm_token,
    abort_on_trigger = True,
)
```

Bridge streams short pre-shaped sub-segments. On trigger from any armed endstop, bridge halts emission, MCU latches step-time of trigger, event flows back to klippy `homing.py` for `set_position`. `trsync` coordinates the trigger across heterogeneous MCUs as today.

### 3.4 Per-stepper independent path (Ring 2)

```python
bridge.submit_independent_stepper(
    stepper_id            = "extruder" | "cflap" | "stepper_x" | ...,
    nurbs                 = ScalarNurbs(...) | TrapezoidalProfile(end, v, a),
    detach_from_kinematics= False,   # True for motors-sync per-belt offset
) -> MoveHandle
```

Backs `manual_stepper`, `force_move`, `[cflap]`, motors-sync, and future phase-stepping per-stepper offsets.

### 3.5 Passthrough commands — Rust port of `serialqueue.c` + `serialhdl.py` callback dispatch

The passthrough router is **not a thin command relay** — it is a Rust port of `klippy/chelper/serialqueue.c` plus the relevant `serialhdl.py` callback dispatch machinery, integrated with `kalico-host-rt`'s existing wire framing/seq/retransmit. This is the largest chunk of new Rust code in 7-C-bridge; spec §3.5 commits to it explicitly.

Why it's a real port and not "relevant subset":
- `serialqueue.c` runs **two priority queues per command queue** (`upcoming` and `ready`), promotes from upcoming → ready when `req_clock` becomes imminent, and sorts ready by `min_clock` for emission ordering.
- It owns **receive-window backpressure** — limited in-flight bytes, blocks emission when MCU's input queue is near-full, recomputes window from MCU `is_shutdown` / busy state.
- It tracks **notify IDs** for `send_with_response` (klippy's `send_wait_ack` / `query` patterns).
- It owns the **retransmit timer** (already in `kalico-host-rt::host_io` post-7-C-io but currently keyed off the host-rt's own seq stream, not per-command-queue req_clock).
- It owns **sent_time / receive_time annotations** on response dicts (`#sent_time`, `#receive_time` in klippy's parsed-response convention).
- `serialhdl.py` wraps the queue with **flush callbacks** (fire when queue drains), **response handlers** keyed by `(name, oid)` with `#oid` annotation injection, **fastreader** for high-priority response handling.

#### 3.5.1 API parity matrix

Every API the existing klippy code consumes from `serialqueue.c` + `serialhdl.py` maps to a bridge passthrough method. If the parity matrix has gaps after Phase 1, klippy can't boot.

| `klippy/chelper/serialqueue.c` / `serialhdl.py` | Bridge replacement | Notes |
|---|---|---|
| `serialqueue_alloc(serial_fd, type)` | `bridge.claim_mcu(serial_path, baud, mcu_type)` | Returns opaque MCU handle. Bridge opens the fd. |
| `serialqueue_alloc_commandqueue()` | `bridge.alloc_command_queue(mcu_handle)` | One queue per driver instance (TMC SPI, GPIO, etc.). |
| `serialqueue_send(sq, cq, msg, len, min_clock, req_clock, notify_id)` | `bridge.passthrough_send(mcu, queue, bytes, min_clock=, req_clock=, notify_id=)` | Schedules into upcoming/ready queues. |
| `serialqueue_send_batch(sq, cq, msgs[])` | `bridge.passthrough_send_batch(mcu, queue, [(bytes, min_clock, req_clock, notify_id), …])` | Atomic insert of multiple. |
| `SerialReader.send_with_response(...)` | `bridge.passthrough_query(mcu, queue, bytes, response_name, oid, …) → notify_id` | Blocks (GIL released) until matching response or timeout. |
| `CommandWrapper.send_wait_ack()` | `bridge.passthrough_send_wait_ack(mcu, queue, bytes, timeout)` | Synchronous ack wait. |
| `SerialReader.register_response(callback, name, oid=None)` | `bridge.passthrough_register_handler(mcu, name, oid, callback)` | Callback fires from reactor (via event_fd drain). Bridge injects `#sent_time`, `#receive_time`, `#oid` annotations on the dict before delivery. |
| `SerialReader.register_response_callback(callback, name, oid)` | same as above | Klippy has two registration APIs; bridge unifies. |
| `mcu.add_config_cmd(cmd, is_init=False)` | `bridge.passthrough_add_config_cmd(mcu, bytes, is_init=)` | Init-stage commands; emitted at MCU restart before runtime traffic. |
| `mcu.register_flush_callback(cb)` | `bridge.passthrough_register_flush_callback(mcu, callback)` | Fires when this MCU's command queue drains. |
| `serialqueue_set_receive_window(sq, n)` | bridge sets via `claim_mcu(.., receive_window_bytes=N)` | Currently a constant; tunable later. |
| `serialqueue_get_stats()` | `bridge.passthrough_get_stats(mcu)` | Periodic stats for `klippy/extras/stats` printing. |
| `mcu.estimated_print_time(eventtime)` | `bridge.estimated_mcu_print_time(mcu, eventtime)` | Clock-sync derived; finalized (see §3.8). |
| `mcu.print_time_to_clock(print_time)` | `bridge.print_time_to_clock(mcu, print_time)` | |
| `mcu.clock_to_print_time(clock)` | `bridge.clock_to_print_time(mcu, clock)` | |
| `serialhdl.SerialReader.connect_*` (identify) | `bridge.claim_mcu(...)` runs identify | Already in 7-C-io; passthrough router reuses. |
| `serialhdl.SerialReader.disconnect()` | `bridge.release_mcu(mcu)` | Lifecycle. |
| Debug-output to file (`-o` flag for klippy) | **Out of MVP scope.** | Klippy supports recording wire traffic to a file; bridge can add later. |
| CAN transport (`canbus.py`) | **Out of MVP scope.** | Trident config doesn't use it. Bridge is USB/UART only for MVP; CAN deferred to a future step. |

The CAN-bus and debug-output omissions are explicit non-goals. Anyone with `[mcu_*: canbus_uuid:...]` hits a config-time error.

#### 3.5.2 API shape on the Python side

```python
# Command queues — one per driver instance (TMC SPI, GPIO, etc.) preserves
# the in-MCU FIFO ordering Klipper drivers depend on.
queue_id = bridge.alloc_command_queue(mcu_id)

# Scheduled emission: bytes are not sent immediately — they're held in the
# named queue, and Rust emits them onto the wire ordered by minclock with the
# reqclock annotation on the message itself. Mirrors klippy/mcu.py
# CommandWrapper.send(minclock=, reqclock=, cmd_queue=).
bridge.passthrough_send(
    mcu_id, queue_id,
    raw_msg_bytes,
    minclock = clock_value,        # don't emit until MCU clock ≥ minclock
    reqclock = clock_value,        # annotation passed to MCU
)

# Typed-response query: returns sequence number for correlation; the handler
# registered for (mcu_id, response_name) receives a parsed response dict
# including #sent_time / #receive_time / #oid annotations as klippy expects.
seq = bridge.passthrough_query(mcu_id, queue_id, raw_msg_bytes,
                               response_name, oid, minclock=, reqclock=)

# Synchronous send-and-wait — equivalent to klippy CommandWrapper.send_wait_ack().
# Releases the GIL while waiting; bounded by clock-derived timeout.
ack_data = bridge.passthrough_send_wait_ack(mcu_id, queue_id, raw_msg_bytes)

# Response routing. Handlers fire from the reactor (via event_fd drain).
bridge.passthrough_register_handler(mcu_id, response_name, oid, callback)

# Config-stage commands — emitted exactly once at MCU init/restart, before
# any runtime traffic. Mirrors klippy mcu.add_config_cmd().
bridge.passthrough_add_config_cmd(mcu_id, raw_msg_bytes, is_init=False)

# Flush callbacks — fire when all queued commands have been emitted (i.e., the
# Rust-side queue has drained). Used by extras/output_pin.py and friends to
# coalesce GPIO updates against MCU flush boundaries.
bridge.passthrough_register_flush_callback(mcu_id, callback)
```

`motion_mcu.py` wraps these to look like klippy `mcu.py`'s `lookup_command` / `add_config_cmd` / `register_response` / `register_flush_callback`. The Python shim is intentionally thin: scheduling and queueing live in Rust so passthrough commands can interleave correctly with motion commands under one shared seq stream.

#### 3.5.3 Implementation locality

- `kalico-host-rt::host_io` (existing): wire framing, seq, NAK retransmit, identify, clock-sync. Reused.
- `kalico-host-rt::passthrough_queue` (NEW): per-MCU command queues with upcoming/ready promotion by `req_clock`, ready ordering by `min_clock`, receive-window backpressure, notify-id correlation, sent_time/receive_time annotation. This is the bulk of new Rust code. Roughly mirrors `serialqueue.c` structure; ~1500 LOC including tests.
- `motion-bridge::passthrough_api`: PyO3 bindings.

The work is bounded but not small. Plan accordingly: this is the Phase-1 critical path.

### 3.6 Status / synchronization / lookahead-callback surface

```python
bridge.get_status()                  # printing/idle/shutdown, queue depth, axes positions
bridge.get_machine_time_now()        # MCU-clock-derived, host-side
bridge.estimated_print_time(handle)  # finalized (t_start, t_end) once SegmentFinalized fires; provisional before
bridge.flush() -> Future             # commit all queued motion through TOPP-RA + wire
bridge.wait_for_moves(timeout)       # blocks (GIL released) until segment queue drains

# Lookahead callbacks — fire from the reactor at the **finalized** end-time of
# the move (i.e., once SegmentFinalized for the corresponding handle has
# arrived). Mirrors klippy toolhead.register_lookahead_callback() semantics.
# Used by extras/heaters.py, extras/output_pin.py, extras/fan.py to schedule
# side-effect events at known print-time boundaries.
bridge.register_lookahead_callback(callback)
   # callback signature: (print_time_finalized: float) -> None
   # Bridge fires after lookahead window has produced finalized timing for the
   # most recently submitted move at callback registration time.

# Move-queue activity hint — extras/output_pin.py uses it to extend the MCU
# move-queue commit horizon when GPIO writes are scheduled into the future.
bridge.note_mcu_movequeue_activity(future_print_time)

# Dynamic limit changes — SET_VELOCITY_LIMIT and friends. Forwards to
# Layer 2's limit-change-invalidation logic; in-flight unprocessed segments
# get re-planned against the new limits.
bridge.update_limits(
    max_velocity = ..., max_accel = ...,
    max_z_velocity = ..., max_z_accel = ...,
    e_limits = ...,
    apply_to_in_flight = True,   # invalidate dirty segments and re-plan
)
```

#### 3.6.1 Move transform preservation

Klippy's `gcode_move.set_move_transform()` lets a single extras module (`bed_mesh`, `skew_correction`) install a transform that runs in `gcode_move` *before* `toolhead.move()`. The transform is purely above the bridge — it splits/adjusts moves in `gcode_move`'s Python layer, then emits the final post-transform moves to the toolhead.

Under this design, `motion_toolhead.py` is the new `toolhead.move()` recipient. It exposes the *same* `move()` / `manual_move()` / `set_position()` / `get_position()` / `get_status()` surface that `gcode_move` and `bed_mesh.py` and `skew_correction.py` call, then funnels each call into `bridge.submit_move()`. The transform mechanism in `gcode_move` is unchanged. Bed-mesh-style move splitting happens in `bed_mesh.py` exactly as today; the splits arrive at `motion_toolhead.move()` as separate calls.

#### 3.6.2 `motion_toolhead.py` compatibility matrix

Every public method on klippy's existing `Toolhead` class consumed by code we keep, with replacement / shim / reject status. If a row says "shim" the method exists with the listed semantics; "reject" means the method is intentionally absent and any caller of it has been patched or hard-disabled.

| Toolhead method | Status | Real consumers / Notes |
|---|---|---|
| `move(newpos, speed)` | shim | `gcode_move`, `bed_mesh` (post-split), `skew_correction` (post-transform). Funnels to `bridge.submit_move()`. |
| `manual_move(coord, speed)` | shim | `bed_mesh.probe()`, `manual_probe`, calibration extras. Internally builds `newpos` and calls `move()`. |
| `set_position(newpos, homing_axes=())` | shim | `homing.py` (post-trigger), G92 handler in `gcode_move`. Sets bridge axis-position state. |
| `get_position()` | shim | Many extras read commanded position. Returns bridge's tracked end-of-queue position. |
| `get_last_move_time()` | shim | `tmc.py` (TMC register writes scheduled at last-move-time + dwell), `stepper_enable.py`, `palette2.py`, `trad_rack.py` (hard-disabled). Returns bridge's last-submitted finalized t_end. |
| `get_status(eventtime)` | shim | `gcode.py` status reporting. Returns dict with `print_time`, `estimated_print_time`, `print_stall`, `stalls`, `idle_timeout`, `axis_minimum`, `axis_maximum`, `homed_axes`. |
| `dwell(delay)` | shim | `homing.py`, `tmc.py` after-write dwell, `extras/probe.py`, gcode handlers (`G4`). Calls `bridge.submit_dwell()`. |
| `wait_moves()` | shim | `gcode_macro` `wait_moves()` accessor, `M400` handler. Calls `bridge.wait_for_moves()`. |
| `flush_step_generation()` | shim | `idle_timeout.py`, sensor-read paths that need committed step events before reading. Calls `bridge.flush()`. |
| `register_lookahead_callback(callback)` | shim | `heaters.py`, `output_pin.py`, `fan.py`, `led.py`. Fires at finalized t_end (§3.8). |
| `note_mcu_movequeue_activity(print_time)` | shim | `output_pin.py` (when scheduling future GPIO transitions). Bridge extends commit horizon. |
| `note_step_generation_scan_time(delay, old_delay)` | reject | Only consumer is `input_shaper.py`, and IS is being rewritten as a host-side pre-bake config-parser; it stops calling this. Method is absent on `motion_toolhead`. |
| `register_step_generator(step_generator)` | reject | Klippy's step-generator hook for kinematics-time-pin updates (used by extruder PA today). Replaced by Layer 3 host-side pre-bake; no per-step-generator hook needed. Method absent. |
| `check_busy(eventtime)` | shim | `palette2.py` (hard-disabled if it actually uses palette2). Returns `(print_time, est_print_time, lookahead_empty)` tuple. |
| `stats(eventtime)` | shim | `klippy.py` periodic stats. Returns `(busy_bool, "stat1=v stat2=v ...")` string. |
| `get_kinematics()` | shim | `homing.py` (`get_kinematics().get_steppers()` to enumerate steppers per axis), `gcode_move`, `safe_z_home`, `bed_mesh`. Returns a `MotionKinematics` object exposing `get_steppers()`, `calc_position(stepper_positions)`, `note_z_not_homed()`, `clear_homing_state(axes)`, `home(homing_state)`, `set_position(newpos, homing_axes)`. |
| `get_extruder()` | shim | `gcode_macro`, `extras/extruder_stepper`. Returns active extruder object — preserved as today (config-object surface from `kinematics/extruder.py`). |
| `get_trapq()` | reject | Trapq is gone. Callers (`z_tilt`, `z_tilt_ng`, `mixing_extruder`, `trad_rack`) are individually patched per §5.2 to use bridge's per-stepper detach/attach API instead of trapq-swapping. |
| `register_lookahead_callback` (sentinel) / `lookahead.add_move()` / `lookahead.flush()` | reject | Lookahead is internal to bridge; no Python access. |

The matrix is enforced by the audit script (§5.5): import every kept extras module, walk its bytecode for calls into the toolhead surface, and fail CI if any call hits a "reject" row.

### 3.7 Events drained by reactor

- `SegmentFinalized(handle, t_start_actual, t_end_actual, metadata)` — emitted once TOPP-RA + shape-batch commits the segment; refines provisional times.
- `SegmentStarted(handle, mcu_clock)` / `SegmentCompleted(handle, mcu_clock)` — MCU-side execution events.
- `EndstopTriggered(arm_token, mcu_id, trigger_clock, axis_position_at_trigger)`.
- `Fault(severity, code, detail)` — host-rt fault codes plus MCU shutdown notifications.
- `McuResponse(mcu_id, raw_msg_bytes)` — passthrough query responses + unsolicited messages.
- `TelemetrySample(...)` — placeholder for cross-cutting telemetry, fleshed out later.

### 3.8 Print-time accounting (provisional vs finalized)

Klippy uses `print_time` heavily (heater scheduling, fan timing, M400). Today it's computed greedily as moves are added — but in current klippy, by the time a `register_lookahead_callback` fires, the trapezoidal lookahead has already finalized that move's end-time, so callbacks see a value that won't change.

In the new path, finalization happens in TOPP-RA + shape-batch on the worker pool, possibly milliseconds after `submit_move` returns. The bridge gives every API a **definite** semantics — provisional (cheap synchronous estimate) or finalized (after SegmentFinalized event) — so consumers know what they're seeing:

| API | Time semantics | Notes |
|---|---|---|
| `submit_move()` return value (`MoveHandle.provisional_t_end`) | **Provisional** — `path_length / feedrate`, ignoring corner deceleration | Used only for keeping `gcode.py`-side `print_time` advancing greedily so back-to-back motion submissions don't stall. Never used for scheduling MCU side effects. |
| `bridge.estimated_print_time(handle)` | **Finalized** if the handle's `SegmentFinalized` has arrived; **provisional** otherwise (with a `is_finalized` flag) | Caller decides which to trust. |
| `register_lookahead_callback(cb)` | **Finalized** | Fires from reactor once `SegmentFinalized` arrives for the move that was at the head of the queue when the callback was registered. Same semantics as today. |
| `note_mcu_movequeue_activity(t)` | **Caller-asserted** | Caller passes the time it intends to schedule against; bridge extends the MCU's commit horizon. |
| `passthrough_send(... minclock=...)` | **Caller-supplied MCU clock** | Bridge does not interpret; it schedules emission when MCU clock ≥ minclock. |
| `flush()` Future | resolves once everything submitted has finalized **and** committed to the wire | M400 path. |
| `wait_for_moves(timeout)` | blocks until last submitted handle's `SegmentCompleted` arrives | M400 + idle-timeout path. |

`SegmentFinalized` arrival latency: bounded by the worker pool's plan-batch time. Typical < 5 ms for representative segment counts; well within the seconds-of-margin that heaters/fans tolerate. `register_lookahead_callback` consumers (heaters, output_pin, fan) get finalized times in the order their moves finalized — same ordering klippy provides today.

**Lookahead-extension hazard:** if a `register_lookahead_callback` fires at finalized-t-end and `update_limits()` later invalidates and re-plans the same segment, the callback would have been fired against a time that no longer matches. Mitigation: `update_limits(apply_to_in_flight=True)` only invalidates segments that haven't yet finalized; once a segment has finalized and emitted its lookahead callbacks, it is committed and not re-planned. Limit changes that arrive mid-segment apply from the next not-yet-finalized segment onward. Documented behavior; acceptable.

#### 3.8.1 MCU clock-conversion API on `motion_mcu.py`

Klippy's `mcu.py` exposes three closely-related public methods that extras consume directly (not via toolhead):

| `mcu.py` method | Bridge / shim semantics |
|---|---|
| `mcu.estimated_print_time(eventtime)` | Clock-sync derived: `(mcu_clock(eventtime) - last_clock_sync_origin) / mcu_freq + print_time_origin`. Fully deterministic; not affected by lookahead provisional/finalized split. Returns the same value the existing implementation would for a given `eventtime`. Used by `output_pin.py` to schedule async pin transitions, by `heaters.py` to detect stale temp readings (`if eventtime > self.next_temp_time: stale`), by `motion_report.py` to convert MCU step-clocks back to print time. |
| `mcu.print_time_to_clock(print_time)` | `(print_time - print_time_origin) * mcu_freq + last_clock_sync_origin`. Also fully deterministic. Used by command-queue `min_clock`/`req_clock` scheduling. |
| `mcu.clock_to_print_time(clock)` | Inverse of the above. |

These three are **fully deterministic** and not touched by the provisional/finalized distinction — they're a function of clock-sync state plus an `eventtime` or `clock` argument. The bridge exposes them on `motion_mcu.py` proxy with identical semantics; consumers don't observe a behavior change.

#### 3.8.2 Cached print-time values

Some extras *cache* a print-time-derived value and use it later — `heaters.py` keeps `verify_mainthread_time`, `output_pin.py` keeps a queue of `(print_time, value)` entries. The mitigation is the same as above: any value derived from `mcu.estimated_print_time()` or `mcu.clock_to_print_time()` is deterministic and stays valid (the underlying clock-sync state changes only slowly and continuously; cached values stay accurate within heater/fan tolerances). Values derived from `toolhead.get_last_move_time()` are *finalized* (the bridge only returns it from finalized state), so caching them is safe.

The risk surfaces only for callers that try to cache `bridge.estimated_print_time(handle)` while the handle is still provisional — bridge returns `(value, is_finalized=False)` in that case, and any caller that ignores the flag and caches the provisional value gets bitten. Documented; the audit script flags consumers that destructure without checking the flag.

## 4. Config translation

The bridge reads `printer.cfg` via klippy's existing `configfile.py`. Per-section policy:

### 4.1 Sections passing through unchanged (klippy parses, bridge ignores)

All `extras/` sections — heaters, sensors, fans, beacon, motors_sync, gcode_macro, save_variables, output_pin, neopixel, all toolchanger/probe/temperature things, plus `[mcu name]` for non-motion MCUs (beacon, NIS).

### 4.2 Sections the bridge consumes

| Section | Bridge use |
|---|---|
| `[mcu]`, `[mcu bottom]` (motion MCUs) | Bridge claims serial fd; `motion_mcu.py` shim binds |
| `[printer]` | `kinematics`, `max_velocity`, `max_accel`, `max_z_velocity`, `max_z_accel`. Plus new knobs in §4.5 |
| `[stepper_*]` (X/Y/Z and dual variants) | Pin / microsteps / rotation_distance / endstop / limits → KinematicsSpec |
| `[extruder]` | Stepper config + accept (forward-compatibly) `pressure_advance` / `pressure_advance_smooth_time` (no-op until Step 9) |
| `[input_shaper]` | Maps `shaper_type_*` + `shaper_freq_*` → `ShaperSpec`. Z passthrough by default |
| `[firmware_retraction]` | Stays Python; emits G10/G11 → bridge sees as INDEPENDENT-mode E moves |

### 4.3 Knobs that hard-error with "remove this line"

Trapezoidal-only concepts:

- `[printer] square_corner_velocity` — replaced by curvature-continuity junctions
- `[printer] minimum_cruise_ratio` — trapezoidal cruise concept
- `[printer] max_jerk` — TOPP-RA jerk has its own knob
- `[printer] corner_deviation` — replaced by JD-fallback for sharp G5↔G5 junctions
- `[extruder] instantaneous_corner_velocity` — N/A under E-follows-XY
- `[stepper_*] high_precision_step_compress` — step compression doesn't exist
- `[gcode_arcs]` — bridge handles arcs natively; klippy's arc decomposer would compete

No silent-ignore escape hatch. The error message names the specific line and explains the replacement.

### 4.4 Knobs silently ignored with a startup info log

- `[extruder] smooth_time` — rolls into shaper
- `[extruder] max_extrude_only_accel` and friends — kept as bounds for INDEPENDENT-mode E NURBS
- `[input_shaper] target_smoothing` — bleeding-edge knob; reserved for kernel expansion

### 4.5 New knobs the bridge adds (in `[printer]` unless noted)

- `refit_tolerance_mm` (default 0.005) — Layer-3 C¹ Hermite refit L∞ tolerance
- `beta_max_iters` (default 8), `beta_convergence_ratio` (default 0.999) — β-medium outer iteration bounds
- `topp_grid_strategy` (default `auto`) — exposes `temporal::multi::GridStrategy`
- `worker_threads` (default 3) — TOPP / shape-batch worker count
- `[stepper_*] axis_assignment` (optional) — explicit `(axis, kinematic-role)` tag, e.g. `corexy_b1`. Defaults to inferring from section name; explicit kills magic-string parsing
- `[motion_mcu_routing]` (optional) — explicit override for which axis's segments target which MCU when inference from `[stepper_*]` pins is ambiguous

### 4.6 Validation pass

Before Rust threads spin, the bridge does a config-validity check (kinematics consistent, every stepper has a motion-MCU assignment, shaper freqs in `[1, 200]` Hz, limits positive, etc.) and fails fast with one coherent error message.

### 4.7 File preprocessing

For `.gcode` print files (G1/G2/G3-bearing): when `start_print(filename)` runs (intercepted in `virtual_sdcard.py` / Moonraker job-queue glue), the bridge invokes `compat`'s library to produce `<filename>.g5.gcode` next to the original. mtime+size hash caches the result. Klippy's existing print-from-file path runs against the new file.

Live G1 / G2 / G3 from macros and the terminal still works via `submit_g1` / `submit_arc` (one move at a time, same `compat` primitives).

## 5. Deletion / replacement / audit list

Goal: zero trapezoidal motion code in the repo at the end of 7-C-bridge.

### 5.1 Deleted outright (file removed)

- `klippy/toolhead.py`
- `klippy/kinematics/` step-generator modules: `cartesian.py`, `corexy.py`, `corexz.py`, `cartesian_abc.py`, `delta.py`, `deltesian.py`, `polar.py`, `rotary_delta.py`, `winch.py`, `hybrid_corexy.py`, `hybrid_corexz.py`, `limited_cartesian.py`, `limited_corexy.py`, `limited_corexz.py`, `none.py` — replaced by `motion_kinematics.py` + MCU-side transforms. **Not deleted:** `kinematics/extruder.py` and `kinematics/idex_modes.py` (see §5.2).
- `klippy/extras/gcode_arcs.py`
- `klippy/chelper/itersolve.*`, `stepcompress.*`, `serialqueue.*`, `trapq.c`/`trapq.h`, `kin_*.c` — all motion-side and serialqueue C code. (Bridge's all-MCU passthrough router replaces serialqueue for both motion and non-motion MCUs.)

### 5.2 Patched (file kept, internals reimplemented; public API preserved)

- `klippy/stepper.py` — `PrinterStepper`, `MCU_stepper`, `PrinterRail` config-object surface preserved. **Full preserved API**: `get_name()`, `add_to_endstop()`, `set_trapq(trapq_or_none)` (now a bridge attach/detach call, not a C-trapq pointer), `get_trapq()` (returns an opaque "kinematic-attached" sentinel or None — z_tilt/z_tilt_ng compare it for null/non-null), `setup_itersolve(stepper_kinematics_name, *args)` (becomes a no-op recording the kinematic role), `set_stepper_kinematics(sk)` / `get_stepper_kinematics()` (record-only; no C struct), `calc_position_from_coord(coord)` (forwarded to bridge's kinematic transform), `get_past_mcu_position(print_time)` (queried from bridge step-history), `dump_steps(...)` (queried from bridge dump-history), `add_step(...)` / `queue_step(...)` (no-op or replaced; verify no extras call these directly outside `manual_stepper`/`force_move`). Trapezoidal internals (itersolve binding, queue_step) gutted.
- `klippy/mcu.py` — constructor branches: instead of allocating a serialqueue and opening fd, allocates a `MotionMcuProxy` via `bridge.claim_mcu()`. Public surface preserved unchanged: `lookup_command`, `lookup_query_command`, `add_config_cmd`, `try_lookup_command`, `register_response`, `register_flush_callback`, `register_config_callback`, `alloc_command_queue`, `create_oid`, `register_msgproto`, `estimated_print_time`, `print_time_to_clock`, `clock_to_print_time`, `set_clock_est`, `get_status`, `is_fileoutput`, `is_shutdown`, `get_constants`, `seconds_to_clock`, `clock_to_seconds`, etc.
- `klippy/serialhdl.py` — Python-side msgparser + identify handshake glue retained or folded into `motion_mcu.py`. C-side `serialqueue` allocator (`serialqueue_alloc`, `serialqueue_send`, `serialqueue_alloc_commandqueue`) goes; bridge `claim_mcu` runs identify directly.
- `klippy/kinematics/extruder.py` — kept because `extras/extruder_stepper.py` and others import `ExtruderStepper`. Surface preserved: `PrinterExtruder`, `ExtruderStepper`, `cmd_SET_PRESSURE_ADVANCE`, `cmd_SYNC_EXTRUDER_MOTION`. PA params (`pressure_advance`, `pressure_advance_smooth_time`) are forward-compatible no-ops until Step 9.
- `klippy/kinematics/idex_modes.py` — IDEX dual-carriage post-MVP. Imports clean, refuses runtime mode-switch.
- `klippy/extras/homing.py` — patches: `toolhead.drip_move()` → `bridge.submit_drip_move()`; `mcu_endstop` arming routes through `bridge.arm_endstops()` (motion-MCU endstops) or stays as today (non-motion-MCU endstops, same wire path through proxy). Orchestration preserved (multi-stepper, retract-and-rehome, `homing_override` macro).
- `klippy/extras/manual_stepper.py` — same `MANUAL_STEPPER` command surface; reimplemented against bridge Ring-2 (per-stepper independent NURBS).
- `klippy/extras/force_move.py` — same `SET_KINEMATIC_POSITION` / `FORCE_MOVE` command surface; reimplemented against Ring-2.
- `klippy/extras/input_shaper.py` — drops trapezoidal IS C path (`input_shaper_alloc`, `get_stepper_kinematics`, `set_stepper_kinematics`). Becomes a config-parser that emits `ShaperSpec` for the bridge plus a `SET_INPUT_SHAPER` runtime command calling `bridge.update_shaper()`. Drops the per-stepper `note_step_generation_scan_time` call (extruder PA scan-time becomes a no-op until Step 9).
- `klippy/extras/motion_report.py` — drops trapq-based `DumpTrapQ` endpoint. Same `motion_report` lookup name preserved; exposes the same `trapqs: { "toolhead": ..., "<extruder>": ... }` dict shape backed by bridge state (so `extras/load_cell/tap_analysis.py` reading `motion_report.trapqs["toolhead"]` keeps working iff the dict shape and `get_trapq_position(print_time)` interface are preserved).
- `klippy/extras/z_tilt.py`, `klippy/extras/z_tilt_ng.py` — your config uses `z_tilt_ng`. Both call `s.set_trapq(toolhead.get_trapq())` on Z steppers to detach/reattach during tilt probing. Patched: `MCU_stepper.set_trapq(None)` becomes "detach this stepper from kinematic transform on the bridge"; `set_trapq(non_null_sentinel)` becomes "reattach". `toolhead.get_trapq()` returns the kinematic-attached sentinel value the patched `set_trapq` accepts. ~30 lines of surgical patching per file. (`z_tilt_ng` is a third-party fork of `z_tilt`; both patched the same way.)

### 5.3 Hard-disabled for MVP (config-time error if user enables)

These extras reach into trapezoidal internals that don't have a clean Ring-2 mapping. Each module's config-loader raises with a clear message identifying the trapezoidal-internal dependency and pointing at a post-MVP backlog item.

- `klippy/extras/mixing_extruder.py` — runtime trapq-swapping (`stepper.get_trapq()` → store, `stepper.set_trapq(self._trapqs[i])` per active mix-ratio). Refactor onto Ring-2 dispatch is non-trivial and not in your config.
- `klippy/extras/trad_rack.py` — Tradrack MMU; allocates its own private `trapq` (`trapq_alloc()`) and runs an entire secondary `tr_toolhead`. Substantial port. Not in your config.
- `klippy/extras/pwm_tool.py` — uses `stepcompress` directly for pulse-train output (`stepcompress_alloc`, `stepcompress_queue_mq_msg`). Niche feature; not in your config.
- `klippy/extras/load_cell/tap_analysis.py` — reads `motion_report.trapqs["toolhead"]` and probably the trapq's internal `get_trapq_position(print_time)` interface. Compatible iff `motion_report` preserves both; if it reaches deeper, hard-disabled. Determined during the audit pass.

### 5.4 Stays unchanged, runs against shim

- All TMC drivers (`tmc.py`, `tmc2130.py`, `tmc2208.py`, `tmc2209.py`, `tmc2660.py`, `tmc5160.py`, `tmc_uart.py`), heaters (`heaters.py`), temperature sensors, fans (`fan.py`, `heater_fan.py`, `temperature_fan.py`, `controller_fan.py`), GPIO (`output_pin.py`, `pwm_cycle_time.py`), ADC (`adc_temperature.py`), neopixel/LED, accelerometers (`adxl345.py`, `lis2dw.py`, `mpu9250.py`).
- `tmc.py` consumes `toolhead.get_last_move_time()` for register-write scheduling — works against shim per §3.6.2.
- `stepper_enable.py` consumes `toolhead.get_last_move_time()` — same.
- Beacon plugin (third-party): runs against klippy `mcu.py` for beacon's MCU (now via bridge passthrough, but the public mcu.py surface is preserved). `toolhead.manual_move()` for scanning bed mesh works through the shim.
- motors_sync (third-party): uses `force_move` + accelerometer; force_move's rewrite preserves the API.
- `firmware_retraction`, `bed_mesh`, `screws_tilt_adjust`, `manual_probe`, `safe_z_home`, `idle_timeout`, `extras/probe.py`, `extras/gcode_move.py`.
- `gcode_macro`, `save_variables`, `respond`, `webhooks`, `virtual_sdcard` (with the print-start hook).

### 5.5 Inventoried but post-MVP

- `[delayed_gcode]`, `[temperature_fan]`, `[gcode_button]`
- `extras/skew_correction.py` (XY transform — should work above the bridge but verify)
- `extras/exclude_object.py`, `extras/dual_carriage.py` (defer)

### 5.6 Audit script

`klippy/extras/audit_extras.py` — CI-runnable check that imports every extras module against the new shim and verifies it doesn't reference deleted methods. Catches drift on every PR.

### 5.7 Kinematics support for MVP

CoreXY (Trident) and Cartesian only. Other families parse-error with "kinematic <foo> not yet supported under the new motion path." Adding a kinematic later is a small isolated change (new `KinematicsSpec` variant + MCU-side transform).

## 6. Phasing

Strategy: **burn-the-boats**. The dev branch is non-functional for printing until Phase 3 lands in sim. Printer availability is not a constraint — the user prints on a stable branch in the meantime.

Two subtleties:

1. Gutting `klippy/mcu.py` (which allocates `steppersync` from `serialqueue.c`) and `klippy/stepper.py` (which allocates `stepcompress` in `MCU_stepper.__init__`) makes klippy unable to even *construct* its objects, let alone boot. The "boots and idles" milestone cannot land in a delete-only phase.
2. Bridge owns the wire to **all** Klipper-protocol MCUs, motion and non-motion (decision-α, §1.1/§2.2). That means the passthrough router (§3.5) is the load-bearing piece of Phase 1: until it works for non-motion MCUs (heaters, sensors, beacon), klippy can't even read the bed temperature.

Phase 1 is therefore explicitly a "delete the boats AND build the new ones" milestone. It's the largest phase.

| Phase | Scope | Definition of done |
|---|---|---|
| **1. Scaffold + delete + all-MCU passthrough router** | Add `motion-bridge` PyO3 crate + `kalico-host-rt::passthrough_queue` Rust module; build/install path (`make` invokes `cargo build --release`, drops `.so` where klippy imports it); klippy reactor wires `event_fd`; skeleton files for `motion_toolhead.py`, `motion_mcu.py`, `motion_kinematics.py`; patch `klippy/stepper.py`, `klippy/mcu.py`, `klippy/serialhdl.py`, `kinematics/extruder.py`, `kinematics/idex_modes.py`, `motion_report.py`; **delete trapezoidal + serialqueue C per §5**; bring up the passthrough router (per §3.5 parity matrix) for **all** Klipper-protocol MCUs (Octopus + F446 + Beacon + NIS); land the CLAUDE.md / dependency-graph.md amendment per §1.4. | Klippy starts with the user's config, configures TMCs across all MCUs, reads thermistors on Octopus + bottom + frame, drives heaters (extruder + bed) — all through bridge → MCU and back. Heaters reach setpoint. Beacon configures and reports temperature. NIS adxl345 enumerates. No motion possible. CI passes. |
| **2. First motion: straight-line single-axis** | `submit_g1` end-to-end: live G1→G5 elevation via `compat`, planner pipeline, wire to motion MCU, CoreXY transform on MCU. Single-axis test moves (X, Y, Z separately). | `G1 X10 F600` produces correct step events in `kalico-sim` or Renode. |
| **3. Multi-axis + E (COUPLED + INDEPENDENT) + shaper + dynamic limits** | `submit_move` for any combination. β-medium iteration, smooth-MZV shaper bake, multi-MCU segment routing (Octopus X/Y/E + bottom Z). File preprocessing wired up. `update_limits()` invalidation + re-plan. `register_lookahead_callback` firing from `SegmentFinalized`. | Print a small synthetic file end-to-end in sim (no homing — start positions assumed). Frequency-domain check verifies shaper applied. `SET_VELOCITY_LIMIT` mid-print re-plans correctly. Heater/fan scheduled events fire at the finalized print times. |
| **4. Homing + drip + endstops + transforms** | `arm_endstops` + `submit_drip_move`; klippy `homing.py` patches; sensorless TMC (passthrough) + beacon Z (passthrough on beacon MCU); `trsync` coordination across heterogeneous MCUs. `bed_mesh.py` transform works through `motion_toolhead`. `skew_correction.py` chained transform works. | `G28` succeeds in sim with stub endstop triggers; `BED_MESH_CALIBRATE` works against beacon scanning mode. |
| **5. Ring 2 — independent-stepper path** | Per-stepper override on MCU runtime; `manual_stepper.py`, `force_move.py`, `z_tilt_ng.py` patched; `input_shaper.py` config-parser-only; `[cflap]` works. | `MANUAL_STEPPER` works; `Z_TILT_ADJUST` (your config uses `z_tilt_ng`) works in sim with stubbed beacon-probe; motors_sync upstream loads against shim (verify; patch upstream if needed); `SET_INPUT_SHAPER` reconfigures shaper at runtime. |
| **6. Cleanup + audit + tests** | Audit script CI-running, walks bytecode for §3.6.2 `reject` rows; hard-disable list for §5.3 modules wired into config-loader. Document remaining post-MVP gaps; migration notes for users. | All tests green. Audit clean. Bridge is the only motion + serial-protocol path. |

After Phase 6, 7-C-bridge is closed. **7-D** (hardware bring-up + first physical print) starts: real Octopus / F446 / Beacon connected, real motors, iteration on hardware quirks. The user can opportunistically try hardware from Phase 3 onward; no phase strictly requires hardware.

## 7. Testing strategy

- **Unit tests** in each new Rust module (classifier, preproc, kinematics_spec, independent path). Fast.
- **PyO3 integration tests** under `pytest`: import the bridge module, exercise the API against an in-process Rust mock transport. Validates threading, GIL discipline, event drain, lifecycle.
- **`kalico-sim` host MCU sim** for fast inner-loop motion tests (Phases 3–6). Reuses the runtime's existing host-sim feature.
- **Renode** for periodic integration soak. Reuses Step-7-C-io's harness.
- **Corpus replay** from Step 7-C-io for wire-level regressions.
- **klippy-boot smoke test**: a CI job that loads the user-pattern config (a sanitized version of the Trident config) and verifies no parse errors, no silent regressions.
- **Trident hardware** opportunistic from Phase 4 onward.

## 8. Open questions / risks

- **Passthrough router scope.** §3.5 is a Rust port of `serialqueue.c` plus the relevant `serialhdl.py` callback dispatch. Estimate ~1500 LOC including tests, plus integration with the existing `kalico-host-rt::host_io` reactor. Largest single piece of new Rust code in 7-C-bridge. If it slips, every later phase slips. CAN-bus and debug-output-to-file are explicit non-goals for MVP.
- **All-MCU wire ownership puts beacon's wire under the bridge in Phase 1.** Beacon's plugin uses standard Klipper-protocol commands plus its own bulk-data-stream commands. The standard subset goes through the §3.5 parity matrix unchanged. The bulk-stream subset (high-rate Z samples during scanning bed mesh) needs verification: bridge's flush-callback-driven MCU response delivery has to handle the bulk-streaming pattern at the rate beacon expects. Highest-risk single item in Phase 1.
- **`compat` crate primitive surface.** Primitives exist (`to_collinear_g5`, `arc_to_g5`, `elevate_g51_to_g5`); `arc_to_g5` is XY-only (bridge wrapper distributes E/F) and `converter.rs` rejects R-format arcs (bridge needs to add). Either keep `compat` as one crate with gated text-I/O, or split into `compat-core` + `compat-cli`. Decide during Phase 2.
- **`klippy/mcu.py` branching cleanliness.** Branching in-place may grow uglier than expected; fallback is a separate `MotionMcuProxy` class in `motion_mcu.py` with a factory in `mcu.py`. Decide during Phase 1.
- **`MCU_stepper` API preservation correctness.** §5.2 lists the public surface preserved on the gutted stepper. If the audit pass surfaces a method we missed (e.g., something in `extras/load_cell/tap_analysis.py` reaching deeper into trapq internals than `motion_report.trapqs["toolhead"].get_trapq_position()`), that consumer either gets a small additional patch or moves to §5.3 hard-disabled.
- **`motion_report.trapqs` dict shape compatibility.** External consumers (load_cell tap_analysis, possibly Moonraker-side dashboards) read `motion_report.trapqs["toolhead"]` directly. The replacement `DumpTrapQ`-equivalent has to expose at least `get_trapq_position(print_time) → (pos, velocity)` and the same `extract_trapq` history endpoint webhooks expect. Validate during Phase 3.
- **Beacon scanning bed mesh timing.** Beacon polls Z height at high rate while toolhead moves at constant XY velocity. With both wires owned by the bridge under one event-fd queue, message-arrival ordering needs to preserve beacon's expectations. If the host-side timing assumptions break, expect a Phase 4 spike.
- **motors-sync upstream compatibility.** Plugin author may need to coordinate on a small patch if the rewritten `force_move.py` API is not 100% compatible. Worst case: vendor a fork.
- **`print_time` provisional-vs-finalized at boundaries.** §3.8 specifies clear semantics; runtime risk is mismatched expectations from third-party plugins. Validate against gcode-macro-heavy print-start sequences (the user's PRINT_START is a good stress case).
- **`update_limits()` re-plan latency.** SET_VELOCITY_LIMIT during a print invalidates dirty future segments; re-plan must complete fast enough that the wire never starves. Validate worst-case re-plan time against `temporal::multi`'s batch executor.
- **PyO3 build integration with klippy's existing `make` flow.** Klippy ships Python; PyO3 builds need cargo + the right interpreter. Build-time complexity has to land cleanly in Phase 1.
- **Shutdown discipline.** Klippy's shutdown is async + reactor-driven; bridge's Rust threads must join cleanly without deadlocking the reactor. Pattern from `klippy/serialhdl.py` applies.
- **`input_shaper.py` runtime reconfigure.** `SET_INPUT_SHAPER` round-trips through `bridge.update_shaper()` and invalidates the host-side pre-bake. Re-bake is fast for a single update but happens on the worker pool; sanity-check timing during runtime tuning.
- **Transform-chain audit.** §3.6.1 confirms `bed_mesh` and `skew_correction` transforms preserve under `motion_toolhead`. Other transforms (`exclude_object`, third-party transform plugins) need verification; deferred to post-MVP unless something in the user's config pulls them in.

## 9. CLAUDE.md / dependency-graph.md amendments (Phase 1 deliverable)

The amendment is a Phase 1 deliverable, not deferred. Concrete edits:

- **CLAUDE.md** "G5 / G5.1 only" bullet — change "no legacy G-code anywhere in the planner" to "no legacy G-code in the planner *crates*" and add the bridge-as-second-caller text per §1.4.
- **CLAUDE.md** Step 7 / Step 13 prose — clarify that Step 13's `compat` crate has two callers: the offline binary (file→file) and the live bridge (terminal/macro G1/G2/G3 conversion via the same primitive functions).
- **`docs/kalico-rewrite/dependency-graph.md`** Layer 1 G-code parser bullet — change "Legacy G0 / G1 / G2 / G3 are not handled by the live parser" to "Legacy G0 / G1 / G2 / G3 reaching the live parser (the rust `gcode` crate) is rejected; the bridge above the planner converts them via the `compat` crate's primitive API before invoking the planner. File-based prints are normalized once at print-start."
- **`docs/kalico-rewrite/dependency-graph.md`** Step-13 closing notes — same clarification.

These edits land in the Phase 1 commit alongside the rest of the scaffold. Without them, future readers will read this spec as a contradiction of the top-level docs.

## 10. References

- `docs/superpowers/specs/2026-04-30-step7a-layer3-trajectory-shaping-design.md` — Layer 3 trajectory shaping (Step 7-A).
- `docs/superpowers/specs/2026-04-30-step7b-layer4-mcu-evaluator-design.md` — Layer 4 MCU evaluator (Step 7-B).
- `docs/superpowers/specs/2026-04-30-step-7c-io-design.md` and `2026-05-01-step-7c-io-tail-design.md` — Step 7-C-io host I/O hardening + deterministic test battery.
- `docs/superpowers/specs/2026-04-30-step13-compat-layer-design.md` — `compat` crate (Step 13 offline normalizer).
- `docs/kalico-rewrite/dependency-graph.md` — layered architecture and critical-path observations.
- `CLAUDE.md` — top-level constraints (G5-only planner, E-follows-XY, smooth-shaper pre-bake, β-medium iteration).
