# Step 7-C-bridge — Python ↔ Rust motion bridge

**Layer:** Cross-cutting (binds Layers 1–5 into the existing Klipper Python host process). **Build-order step:** 7-C-bridge.

**Scope:** Wire the existing Klipper Python host (`klippy/`) to the new motion stack (`gcode` / `geometry` / `temporal` / `trajectory` / `runtime` / `kalico-host-rt`) so existing user printer configs route motion through the new planner. Keep the Klipper config language, the entire `extras/` ecosystem (heaters, sensors, fans, macros, beacon, motors-sync, etc.), and the reactor model intact; replace the trapezoidal motion path end-to-end. By the end of this step the repository contains zero trapezoidal motion code, and the Trident test bench is one bring-up step (7-D) away from a real print.

This spec is the *design* for 7-C-bridge. The per-phase implementation plan is the next deliverable (`superpowers:writing-plans`); the spec defines architecture, surface area, deletions, and phasing. Hardware bring-up (cycle-budget actuals, calibration, first physical print) lives in 7-D.

## 1. Goals & non-goals

### 1.1 In scope

1. **PyO3 bridge module** exposing a `MotionBridge` Python handle backed by `kalico-host-rt` and the planner crates. Single in-process Python module; klippy `import`s it like any other extension.
2. **Wire ownership transfer:** the bridge owns the serial fd to every motion MCU. Klippy `mcu.py` keeps owning non-motion MCUs (beacon, NIS).
3. **`motion_mcu.py` shim** preserves the public API of `klippy/mcu.py` for motion MCUs, routing message bytes through PyO3 → Rust passthrough so existing extras (TMC drivers, heaters on motion MCUs, GPIOs, sensors) continue to work unchanged.
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

### 1.4 Reconciliation with the G5-only planner constraint

CLAUDE.md and `docs/kalico-rewrite/dependency-graph.md` are firm: "anything that **reaches the planner** is G5 or G5.1; anything else is rejected at the lexer/reduce boundary as a hard error" and "the compatibility layer (Step 13) is a separate offline tool." The bridge described here is consistent with both:

- The **planner crates** (`geometry`, `temporal`, `trajectory`) are G5/G5.1-only. They never see G0/G1/G2/G3, ever.
- The **bridge** sits *in front of* the planner crates. For live (terminal/macro) submissions it converts G1/G2/G3 to G5 before invoking the planner — using the same primitive functions in the existing `compat` crate that the offline Step-13 binary uses (`compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`). The bridge is not a planner crate.
- For **file-based prints** the file is normalized once at print-start by the same `compat` primitives (§4.7), so the planner sees only G5 even on the live path.

The framing in CLAUDE.md and the dependency graph treats Step 13 only as the offline *binary*; this spec extends the same primitives to a second caller (the bridge, for terminal/macro use), without weakening the G5-only planner constraint. Updates to CLAUDE.md / dependency-graph.md to mention this second caller are tracked under §9.

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

```
                   ┌── motion MCU fd #1 (Octopus H723: X/Y/E)
   kalico-host-rt ─┼── motion MCU fd #2 (F446 bottom: Z)
        ▲          └── (extension point: more motion MCUs)
        │
        │ enqueue_command(bytes)            ┌── beacon fd
        │ on_response(bytes → Python event) ├── NIS fd  (adxl345)
        │                                   │
   ┌────┴──────┐    ┌──────────────────┐    │
   │motion_mcu │    │ klippy mcu.py    ├────┴── owns these fds itself
   │   .py     │    │ (non-motion MCUs)│
   │  (shim)   │    └──────────────────┘
   └─────▲─────┘
         │ same public API as klippy mcu.py
         │
   extras/heaters.py, extras/tmc5160.py, extras/temperature_sensor.py,
   extras/output_pin.py, …
   (everything talking to *motion* MCUs goes through this shim)
```

Salient points:
- For **motion MCUs**, the bridge owns the serial fd. Klippy-side code that wants to send a Klipper-protocol message to a motion MCU constructs the message bytes (using existing klippy/`mcu.py` machinery) and hands them to the bridge as a passthrough. The bridge interleaves passthrough commands with motion commands on the same wire under a single seq.
- For **non-motion MCUs** (beacon, NIS), klippy `mcu.py` is unchanged. Owns its own fd, its own seq, its own dispatch.
- **Endstops on motion MCUs** (e.g., `tmc5160_stepper_x1:virtual_endstop` in your config): `mcu_endstop` issues `endstop_home` / `endstop_query_state` via the shim — passthrough commands. Step 7-B's runtime-side endstop logic *is* the Klipper endstop protocol, so the wire shape is unchanged.
- **Endstops on non-motion MCUs** (beacon Z endstop): `mcu_endstop` talks to beacon directly via klippy `mcu.py`. `trsync` coordinates the trigger across MCUs as today, with `trsync` itself being a passthrough command on motion MCUs.

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
  motion_toolhead.py                         # NEW — replaces toolhead.py
  motion_mcu.py                              # NEW — shim implementing klippy mcu.py public API for motion MCUs
  motion_kinematics.py                       # NEW — config parsing per kinematic family → KinematicsSpec
  motion_report.py                           # NEW — replaces extras/motion_report.py against bridge state instead of trapq
  toolhead.py                                # DELETED
  stepper.py                                 # PATCHED — keeps PrinterStepper / MCU_stepper / PrinterRail config-object API used by extras; trapezoidal motion guts (queue_step, itersolve binding) gutted and routed through bridge
  kinematics/
    extruder.py                              # PATCHED — keeps ExtruderStepper config-object surface (extras/extruder_stepper.py imports it directly); trapezoidal pressure-advance bits routed through bridge (PA params remain forward-compatible no-ops until Step 9)
    idex_modes.py                            # PATCHED — IDEX dual-carriage mode-switch logic stays Python; physical motion routed through bridge (out-of-MVP-scope kinematic; verify import-time only)
    cartesian.py, corexy.py, corexz.py, …    # DELETED — all step-generator kinematics removed; replaced by motion_kinematics.py + MCU-side transforms
  chelper/                                   # CHANGED — itersolve.c, stepcompress.c, serialqueue.c, kin_*.c, trapq.c removed; non-motion bits (CRC helper, msgblock util) retained if still used by mcu.py for non-motion MCUs
  extras/
    homing.py                                # PATCHED — drip_move via bridge, arm_endstops via bridge
    manual_stepper.py                        # PATCHED — same filename, same MANUAL_STEPPER command surface, internals reimplemented against bridge Ring-2
    force_move.py                            # PATCHED — same filename, same SET_KINEMATIC_POSITION / FORCE_MOVE command surface, internals reimplemented against bridge Ring-2
    input_shaper.py                          # PATCHED — drops trapezoidal IS C-allocation path (input_shaper_alloc / get_stepper_kinematics / set_stepper_kinematics); becomes a config-parser that emits ShaperSpec for the bridge. The pre-bake replacement runs in Layer 3 host-side
    motion_report.py                         # PATCHED — drops trapq-based dump-trapq endpoint; replaced by bridge-state queries that surface position / velocity / segment metadata via the existing dump-stepper protocol. Same `motion_report` lookup name preserved for moonraker compatibility
    gcode_arcs.py                            # DELETED — bridge handles arcs natively
    audit_extras.py                          # NEW — CI tool, scans all extras for deleted-method references
  mcu.py                                     # PATCHED — branches: motion-MCU path proxies to bridge, non-motion path unchanged
```

`klippy/mcu.py` is not split into two files; the existing class gets a constructor branch that detects "this MCU was claimed by the bridge" and proxies. Less file shuffling than a hard split. (Fallback if the branched class grows ugly: factor `MotionMcuProxy` into `motion_mcu.py` with a factory in `mcu.py`. Decision deferred to Phase 2.)

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

### 3.5 Passthrough commands (Klipper-protocol semantics preserved)

Klippy `mcu.py` exposes a richer surface than raw byte send: scheduled-emission via `minclock`/`reqclock` on per-driver command queues, typed responses by `(name, oid)`, sent/receive timestamps annotated on response dicts, ack-tracking for synchronous send-and-wait, and config-stage-only commands. The bridge replicates this on the Rust side rather than asking the shim to schedule from Python.

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

The Rust side replicates the relevant subset of `klippy/chelper/serialqueue.c`'s scheduling logic — split between `kalico-host-rt::host_io` (existing wire framing + seq + retransmit) and a new `passthrough_queue` module (per-queue FIFO + minclock-ordered emission + typed-response correlation). This is significant but bounded work: the algorithm is well-understood from the existing C implementation; the new code is a Rust port plus integration with the existing reactor.

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
- `klippy/chelper/itersolve.*`, `stepcompress.*`, `serialqueue.*`, `trapq.c`/`trapq.h`, `kin_*.c` — motion-side C code

### 5.2 Patched (file kept, internals reimplemented; public API preserved)

- `klippy/stepper.py` — `PrinterStepper`, `MCU_stepper`, `PrinterRail` config-object surface preserved (extras depend on it). Trapezoidal internals (queue_step, itersolve binding, calc_position_from_coord) gutted and routed through bridge.
- `klippy/mcu.py` — branched: motion-MCU path proxies to bridge, non-motion unchanged.
- `klippy/kinematics/extruder.py` — kept because `extras/extruder_stepper.py` and others import `ExtruderStepper` directly. Config-object surface preserved (`PrinterExtruder`, `ExtruderStepper`, `cmd_SET_PRESSURE_ADVANCE`, `cmd_SYNC_EXTRUDER_MOTION`); trapezoidal step-generation internals routed through bridge. Pressure-advance config knobs (`pressure_advance`, `pressure_advance_smooth_time`) are forward-compatible no-ops until Step 9. Logical home is closer to `klippy/extruder.py` than `kinematics/extruder.py`; rename is left for the post-MVP cleanup pass.
- `klippy/kinematics/idex_modes.py` — IDEX dual-carriage mode-switch is post-MVP, but the module imports cleanly today and is referenced from `gcode.py` discovery; preserve as patched no-op so klippy boots. Refuses to enter IDEX mode at runtime with "not yet supported."
- `klippy/extras/homing.py` — 50–100 lines of surgical changes routing motion through bridge; orchestration logic preserved.
- `klippy/extras/manual_stepper.py` — same filename, same `MANUAL_STEPPER` command surface; internals reimplemented against bridge Ring-2.
- `klippy/extras/force_move.py` — same filename, same `SET_KINEMATIC_POSITION` / `FORCE_MOVE` command surface; internals reimplemented against bridge Ring-2.
- `klippy/extras/input_shaper.py` — drops the trapezoidal IS C path (`input_shaper_alloc`, `get_stepper_kinematics`, `set_stepper_kinematics`, all of which depend on deleted `chelper/kin_*.c` and stepper-kinematics handles). Becomes a config-parser that reads `[input_shaper]` and emits a `ShaperSpec` for the bridge. The `SET_INPUT_SHAPER` runtime command stays — it now calls `bridge.update_shaper(...)`. Pre-bake replacement runs in Layer 3 host-side.
- `klippy/extras/motion_report.py` — drops trapq dump-trapq endpoint (the trapq doesn't exist anymore). Replaced by bridge-state queries that surface position / velocity / segment metadata via the existing `motion_report` lookup name + `dump_stepper`/`dump_history` interfaces (so moonraker's existing consumers keep working). Bridge exposes the necessary status taps to back this.

### 5.3 Stays unchanged, runs against shim

- All TMC drivers (`tmc.py`, `tmc2130.py`, `tmc2208.py`, `tmc2209.py`, `tmc2660.py`, `tmc5160.py`, `tmc_uart.py`), heaters (`heaters.py`), temperature sensors, fans (`fan.py`, `heater_fan.py`, `temperature_fan.py`, `controller_fan.py`), GPIO (`output_pin.py`, `pwm_tool.py`, `pwm_cycle_time.py`), ADC (`adc_temperature.py`), neopixel/LED, accelerometers (`adxl345.py`, `lis2dw.py`, `mpu9250.py`).
- Beacon plugin (third-party): runs against klippy `mcu.py` for beacon's MCU; `toolhead.manual_move()` for scanning bed mesh works through the shim.
- motors_sync (third-party): uses `force_move` + accelerometer; force_move's rewrite preserves the API.
- `firmware_retraction`, `bed_mesh`, `screws_tilt_adjust`, `manual_probe`, `safe_z_home`, `idle_timeout`, `gcode_move`, `extras/probe.py`.
- `gcode_macro`, `save_variables`, `respond`, `webhooks`, `virtual_sdcard` (with the print-start hook).

### 5.4 Inventoried but post-MVP

- `[delayed_gcode]`, `[temperature_fan]`, `[gcode_button]`
- `extras/skew_correction.py` (XY transform — should work above the bridge but verify)
- `extras/exclude_object.py`, `extras/dual_carriage.py` (defer)

### 5.5 Audit script

`klippy/extras/audit_extras.py` — CI-runnable check that imports every extras module against the new shim and verifies it doesn't reference deleted methods. Catches drift on every PR.

### 5.6 Kinematics support for MVP

CoreXY (Trident) and Cartesian only. Other families parse-error with "kinematic <foo> not yet supported under the new motion path." Adding a kinematic later is a small isolated change (new `KinematicsSpec` variant + MCU-side transform).

## 6. Phasing

Strategy: **burn-the-boats**. The dev branch is non-functional for printing until Phase 4 lands in sim. Printer availability is not a constraint — the user prints on a stable branch in the meantime.

A subtlety the original phasing missed: gutting `klippy/mcu.py` (which allocates `steppersync` from `serialqueue.c`) and `klippy/stepper.py` (which allocates `stepcompress` in `MCU_stepper.__init__`) makes klippy unable to even *construct* its objects, let alone boot. The "boots and idles" milestone therefore cannot land in a delete-only phase; the passthrough router has to come up at the same time. Phase 1 (delete + scaffold + passthrough) is one combined milestone.

| Phase | Scope | Definition of done |
|---|---|---|
| **1. Scaffold + delete + passthrough boot** | Add `motion-bridge` PyO3 crate; PyMethods stubs for the §3 surface; klippy reactor wires `event_fd`; build/install path works (`make` invokes `cargo build --release` and drops the `.so` where klippy imports it); `motion_toolhead.py`, `motion_mcu.py`, `motion_kinematics.py`, `motion_report.py` skeleton files; gut `klippy/stepper.py` and motion-side of `klippy/mcu.py`; patch `kinematics/extruder.py` and `kinematics/idex_modes.py`; **delete trapezoidal code per §5**; **bring up the passthrough router** (per §3.5) on the Rust side: per-MCU command queues, minclock-ordered emission, typed-response correlation, `passthrough_send` / `_query` / `_send_wait_ack` / `_register_handler` / `_register_flush_callback`; bridge takes over motion-MCU serial fds. | Klippy starts with the user's config, configures TMCs, reads thermistors, drives heaters, reads MAX31865 — all through bridge → MCU and back. Heaters reach setpoint. Beacon/NIS still on klippy `mcu.py` and untouched. No motion possible. CI passes. |
| **2. First motion: straight-line single-axis** | `submit_g1` end-to-end: live G1→G5 elevation via `compat`, planner pipeline, wire to MCU, CoreXY transform on MCU. Single-axis test moves (X, Y, Z separately). | `G1 X10 F600` produces correct step events in `kalico-sim` or Renode. |
| **3. Multi-axis + E (COUPLED + INDEPENDENT) + shaper + dynamic limits** | `submit_move` for any combination. β-medium iteration, smooth-MZV shaper bake, multi-MCU segment routing (Octopus X/Y/E + bottom Z). File preprocessing wired up. `update_limits()` invalidation + re-plan. `register_lookahead_callback` firing from `SegmentFinalized`. `motion_report.py` reimplemented. | Print a small synthetic file end-to-end in sim (no homing — start positions assumed). Frequency-domain check verifies shaper applied. `SET_VELOCITY_LIMIT` mid-print re-plans correctly. Heater/fan scheduled events fire at the finalized print times. |
| **4. Homing + drip + endstops** | `arm_endstops` + `submit_drip_move`; klippy `homing.py` patches; sensorless TMC (passthrough) + beacon Z (klippy mcu.py side); `trsync` coordination. | `G28` succeeds in sim with stub endstop triggers; `BED_MESH_CALIBRATE` works against beacon scanning mode. |
| **5. Ring 2 — independent-stepper path** | Per-stepper override on MCU runtime; bridge `submit_independent_stepper` API and `klippy/stepper.py`'s `MCU_stepper` motion path wired through it; `manual_stepper.py` and `force_move.py` reimplemented; `input_shaper.py` config-parser-only; `[cflap]` works. | `MANUAL_STEPPER` G-code works; motors_sync upstream loads against shim (verify; patch upstream if needed); `SET_INPUT_SHAPER` reconfigures shaper at runtime. |
| **6. Cleanup + audit + tests** | Audit script CI-running; document remaining post-MVP gaps; migration notes for users. | All tests green. Audit clean. Bridge is the only motion path. |

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

- **Passthrough router scope.** §3.5 commits to replicating per-queue minclock-ordered emission, typed-response correlation, sent/receive timestamping, and config-stage commands on the Rust side. This is the largest piece of new Rust code in 7-C-bridge — a Rust port of the relevant subset of `klippy/chelper/serialqueue.c`. Plan accordingly during Phase 1; if it slips, every later phase slips.
- **`compat` crate primitive surface.** The crate has the primitives (`to_collinear_g5`, `arc_to_g5`, `elevate_g51_to_g5`) but `arc_to_g5` is XY-only and leaves E/F to the caller, and `converter.rs` rejects R-format arcs. Bridge wrapper handles E/F distribution along the arc decomposition; R-format support gets added either to a new `compat::arc` helper or wrapped in the bridge. Either keep `compat` as one crate with carefully gated text-I/O, or split into `compat-core` + `compat-cli`. Pick during Phase 2 when the bridge first depends on it.
- **`klippy/mcu.py` branching cleanliness.** Branching motion vs. non-motion paths in the same file may grow uglier than expected; if so, fall back to `MotionMcuProxy` in `motion_mcu.py` with a factory in `mcu.py`. Decide during Phase 1.
- **Beacon scanning bed mesh timing.** The most timing-sensitive third-party integration. Beacon polls Z height at high rate while toolhead moves at constant XY velocity; both flow through different paths (toolhead through bridge, beacon through klippy mcu.py). If the host-side timing assumptions of beacon's scanner don't hold under the new path, expect a Phase 4 spike.
- **motors-sync upstream compatibility.** Plugin author may need to coordinate on a small patch if the rewritten `force_move.py` API is not 100% compatible. Worst case: vendor a fork.
- **`print_time` provisional-vs-finalized at boundaries.** §3.8 specifies clear semantics for every API; the runtime risk is mismatched expectations from third-party plugins. Validate against gcode-macro-heavy print-start sequences (the user's PRINT_START is a good stress case) plus any extras that read `print_time` directly via the printer object.
- **`update_limits()` re-plan latency.** SET_VELOCITY_LIMIT during a print invalidates dirty future segments; re-plan must complete fast enough that the wire never starves. Validate worst-case re-plan time against `temporal::multi`'s batch executor; if marginal, consider deferring re-plan to a "next opportunity" rather than immediate.
- **PyO3 build integration with klippy's existing `make` flow.** Klippy ships Python; PyO3 builds need cargo + the right interpreter. Build-time complexity that has to land cleanly in Phase 1, or every later phase stalls.
- **Shutdown discipline.** Klippy's shutdown is async + reactor-driven; bridge's Rust threads must join cleanly without deadlocking the reactor. Pattern from `klippy/serialhdl.py` applies.
- **`input_shaper.py` runtime reconfigure.** `SET_INPUT_SHAPER` runs against the existing IS C-allocation API; under the new path it has to round-trip through `bridge.update_shaper(...)` which invalidates the host-side pre-bake. Re-bake is fast for a single shaper update but happens on the worker pool, so timing during runtime tuning needs sanity-checking.

## 9. CLAUDE.md / dependency-graph.md updates tracked

The bridge's use of `compat` primitives for live G1/G2/G3 conversion (terminal/macro path) is consistent with the G5-only planner constraint (the planner crates never see legacy G-codes; bridge converts before submit). The framing in CLAUDE.md and `docs/kalico-rewrite/dependency-graph.md` describes Step 13 only as the offline binary; both should be amended to mention the second caller (the bridge, sharing the same primitives in the live path) so future readers don't read this spec as a contradiction. Tracked here; landed alongside Phase 1.

## 10. References

- `docs/superpowers/specs/2026-04-30-step7a-layer3-trajectory-shaping-design.md` — Layer 3 trajectory shaping (Step 7-A).
- `docs/superpowers/specs/2026-04-30-step7b-layer4-mcu-evaluator-design.md` — Layer 4 MCU evaluator (Step 7-B).
- `docs/superpowers/specs/2026-04-30-step-7c-io-design.md` and `2026-05-01-step-7c-io-tail-design.md` — Step 7-C-io host I/O hardening + deterministic test battery.
- `docs/superpowers/specs/2026-04-30-step13-compat-layer-design.md` — `compat` crate (Step 13 offline normalizer).
- `docs/kalico-rewrite/dependency-graph.md` — layered architecture and critical-path observations.
- `CLAUDE.md` — top-level constraints (G5-only planner, E-follows-XY, smooth-shaper pre-bake, β-medium iteration).
