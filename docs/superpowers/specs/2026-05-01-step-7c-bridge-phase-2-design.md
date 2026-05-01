# Step 7-C-bridge Phase 2 — First motion: straight-line single-axis

**Parent spec:** `docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md` §6, Phase 2 row.

**Scope:** Wire `motion_toolhead.move()` end-to-end through the Rust planner pipeline to produce correct step events on the MCU. Single-axis test moves (X, Y on Octopus via CoreXY; Z on F446 via Cartesian). Full shaper pipeline active (smooth-ZV/MZV + β-medium). Validated in kalico-sim with Renode as a gate.

**Precondition:** Phase 1 complete — PyO3 bridge boots, passthrough router works for all MCUs, legacy trapezoidal code deleted, shim files in place with move methods raising `NotImplementedError`.

## 1. End-to-end data flow

```
motion_toolhead.move(newpos, speed)
  │  compute delta from self.commanded_pos
  ▼
bridge.submit_move(dx, dy, dz, feedrate)            ← PyO3, GIL released
  │  Phase 2 hard-rejects de != 0 ("extrusion not yet supported")
  │  classify: XY-only → COUPLED(ratio=0), Z-only
  │  compat::collinear::to_collinear_bezier(start, end)
  │    → [P0, P1, P2, P3] control points directly (no G5Line text)
  │  build nurbs::VectorNurbs<f64, 3> with knots [0,0,0,0, 1,1,1,1]
  │  wrap as CubicSegment (no text round-trip, no GeometryPipeline)
  │  enqueue PendingMove to planner channel
  ▼
Planner thread (background, spawned at bridge init)
  │  accumulate moves in streaming window (capacity W, default 32)
  │  temporal TOPP-RA on window → velocity profiles
  │  trajectory::shape_batch(profiles, shaper_config) → ShapedSegments
  │      β-medium iteration, selected smooth-ZV/MZV convolution, time-reparam
  │  for each shaped segment:
  │      per-MCU dispatch based on which axes are non-trivial
  ▼
Per-MCU push (still on planner thread)
  │  Octopus: load X + Y scalar curves → handles hX, hY
  │           push_segment(x=hX, y=hY, z=UNUSED, e=UNUSED, kin=CoreXyAndE)
  │  F446:    load Z scalar curve → handle hZ
  │           push_segment(x=UNUSED, y=UNUSED, z=hZ, e=UNUSED, kin=CartesianXyzAndE)
  │  where UNUSED = 0xFFFEFFFE (CurveHandle::UNUSED_SENTINEL)
  ▼
MCU evaluates curves at modulation rate, applies kinematics, generates steps
```

### 1.1 Design decisions and their rationale

**No `GeometryPipeline::process` in the live path.** `GeometryPipeline` is a text-based one-shot API (`fn process(&mut self, text: &str, ...) -> Segments`). The live bridge receives structured moves from Python, not G-code text. Serializing to G5 text and re-parsing would be wasteful. The bridge constructs `CubicSegment` directly from control points. Note: `GeometryPipeline` is a `rust/geometry` planner crate type — Step 13's offline normalizer (which is pure text→text) does not use it either. `GeometryPipeline` is only used by synthetic test harnesses.

**Bridge uses `compat` crate's structured API variant.** CLAUDE.md names `compat::collinear::to_collinear_g5` as a live bridge caller. The existing function returns `G5Line` (I/J/P/Q text format), which the bridge doesn't need. Phase 2 adds a structured variant `compat::collinear::to_collinear_bezier(start: [f64; 3], end: [f64; 3]) -> [[f64; 3]; 4]` that returns the 4 control points directly (same 1/3-2/3 lerp math, no `G5Line` intermediary). This preserves the CLAUDE.md contract — the bridge uses compat — while avoiding a wasteful text round-trip. Phase 3 adds the analogous `compat::arc::arc_to_bezier()` for G2/G3 conversion.

**Full shaper from the start.** Even single-segment terminal test moves go through the selected smooth-ZV/MZV convolution and β-medium iteration. This proves the shaper in the simplest context (one segment, zero boundary velocity). Phase 3 exercises cross-boundary shaping with multi-segment windows.

**Structured bridge moves bypass `geometry::reduce`.** The reduce boundary (CLAUDE.md: "anything reaching reduce that is not G5/G5.1 is a hard error") applies to text G-code entering through the geometry pipeline. Structured bridge moves never touch reduce — they construct `CubicSegment` (cubic Bézier, equivalent to G5) directly and feed into temporal. No legacy G0/G1/G2/G3 handling is reintroduced into `geometry::reduce`.

### 1.2 Verified type chain

Verified against the codebase (kalico-verifier, 2 passes):

```
bridge VectorNurbs<f64, 3>
  → CubicSegment { xyz: VectorNurbs<f64, 3>, e_mode, feedrate_mm_s, ... }
    → temporal::multi::SegmentInput { curve: &seg.xyz, ... }
      → temporal::multi::plan_batch() → BatchOutput
        → trajectory::ShapeSegmentInput { temporal: SegmentInput, ... }
          → trajectory::shape_batch() → ShapeBatchOutput { segments: Vec<ShapedSegment> }
            → ShapedSegment { axes: [ScalarNurbs<f64>; 3], t_start, t_end, ... }
              → f64→f32 truncation → wire encode → MCU curve pool
```

`CubicSegment::try_new()` requires a `SourceRange` — bridge passes synthetic `{ start_line: 0, end_line: 0 }` (metadata-only, not used in computation).

**NURBS construction details:** The `VectorNurbs<f64, 3>` for a collinear cubic Bézier uses degree 3, knot vector `[0, 0, 0, 0, 1, 1, 1, 1]` (clamped uniform), and 4 control points `[P0, P1, P2, P3]` in the unit parameter domain `[0, 1]`. This matches the knot convention used by `geometry::reduce` for G5 segments.

### 1.3 MCU-side constants

| Constant | Value | Source |
|----------|-------|--------|
| `KinematicTag::CoreXyAndE` | `0u8` | `runtime/src/segment.rs`, `#[repr(u8)]` |
| `KinematicTag::CartesianXyzAndE` | `1u8` | same |
| `EMode::CoupledToXy` | `0u8` | `runtime/src/config.rs`, `#[repr(u8)]` |
| `EMode::Independent` | `1u8` | same |
| `EMode::Travel` | `2u8` | same |
| `CurveHandle::UNUSED_SENTINEL` packed | `0xFFFEFFFE` | `runtime/src/curve_pool.rs` |
| CoreXY forward | `a = x+y, b = x-y` | `runtime/src/kinematics.rs` |

## 2. Planner thread and streaming window

### 2.1 Thread lifecycle

Spawned at bridge init (during klippy startup). Joined on bridge shutdown (klippy exit / firmware restart). Single thread — no thread pool for Phase 2. The thread owns no Python objects and never touches the GIL.

### 2.2 Channel protocol

Bounded channel (`std::sync::mpsc` or `crossbeam`). Messages:

```rust
enum PlannerMsg {
    Move(PendingMove),      // classified, bridge-constructed segment
    Dwell(f64, Arc<Notify>),// flush pending, advance print time by duration_s, then wake
    Flush(Arc<Notify>),     // from wait_moves() — process everything, wait for execution, wake
    UpdateLimits(PlannerLimits),  // from SET_VELOCITY_LIMIT
    UpdateShaper(ShaperConfig),   // from SET_INPUT_SHAPER
    Shutdown,               // join cleanly
}
```

### 2.3 Window accumulation and flush triggers

The planner thread loops:

1. Block on channel recv (no busy-spin).
2. Drain all immediately-available messages into a local buffer.
3. Flush the batch if any of:
   - Buffer reaches window capacity W (default 32)
   - A `Flush` or `Dwell` signal was drained
   - A `Shutdown` was drained
4. Run the pipeline on the batch: temporal TOPP-RA → `shape_batch` → per-MCU push.
5. Wait for execution: block until the last segment's `t_end` has elapsed in real time (not just pushed+ACKed — see §2.4).
6. Wake any `Flush` / `Dwell` waiters. For `Dwell`, additionally advance internal print time by the dwell duration before waking.
7. On `Shutdown`: flush remaining buffer, then exit loop.

For Phase 2 terminal commands: klippy's gcode dispatch calls `wait_moves()` after each interactive G1. This sends a `Flush` signal, so each terminal move processes as a window-of-1. That's correct behavior — the shaper convolves the single segment, β-medium iterates, and the user sees immediate execution.

For Phase 3 file printing: moves arrive faster than the planner drains. The buffer fills toward W naturally, producing multi-segment batches with cross-boundary shaping continuity. The transition is seamless — same mechanism, different arrival rate.

### 2.4 `wait_moves()` semantics

Python thread (GIL released) sends `Flush(notify)` to the channel, then blocks on the `notify`. The planner thread wakes the notify only after **all segments from the flushed batch have been pushed, ACKed, AND the last segment's `t_end` has elapsed in real time**. This means `wait_moves()` guarantees: everything submitted before this call has been shaped, pushed, and **physically executed** (motors have stopped). This matches Klipper's `wait_moves()` / `M400` semantics — macros depend on motion being complete when `M400` returns.

Implementation: after push+ACK, the planner computes `wall_clock_deadline = now + (t_end_print_time - current_print_time)` using the clock-sync estimate, and sleeps until the deadline. For Phase 2 single-segment moves, this is a short sleep (sub-second).

### 2.5 Error propagation

If the pipeline fails (TOPP-RA infeasible, shaper error, MCU push rejected, MCU fault), the planner thread stores the error in a shared `Mutex<Option<PlannerError>>`. The next Python call into the bridge (`submit_move`, `wait_moves`, etc.) checks this mutex and raises a Python exception if an error is present. The planner thread does not silently swallow errors.

**Position consistency on error:** `submit_move` updates `commanded_pos` optimistically (before the planner thread processes the move). If the planner later fails, `commanded_pos` may be ahead of reality. This is acceptable because planner errors are fatal — klippy triggers a printer shutdown/restart, which resets all state. No position reconciliation logic is needed.

## 3. Per-MCU dispatch and curve loading

### 3.1 Axis-to-MCU mapping

Built at bridge init from `[stepper_*]` config sections (pin prefix → MCU id):

```
Octopus H723:  X, Y, E  →  kinematics = CoreXyAndE (0)
               4 steppers (2 per CoreXY belt), MCU handles fan-out
F446 bottom:   Z         →  kinematics = CartesianXyzAndE (1)
               3 Z steppers, MCU handles fan-out
```

Stored as a static `Vec<McuAxisConfig>` on the bridge. Multi-stepper fan-out (2 per belt, 3 for Z) is the MCU's responsibility — the host sends one curve per axis, not per stepper.

### 3.2 Per-segment push sequence

For each `ShapedSegment`, for each MCU that owns at least one non-trivial axis:

1. **Load curves:** For each axis this MCU owns, extract `ScalarNurbs<f64>` from `ShapedSegment.axes[axis_idx]`, truncate to f32, send `kalico_load_curve` wire command, receive `kalico_load_curve_response` → `CurveHandle`, pack as `(generation << 16) | slot_idx`.

2. **Assemble `SegmentPushParams`:** Fill axis handles (packed u32 for owned axes, `0xFFFEFFFE` for unowned), set `kinematics` byte, `e_mode = Travel` (Phase 2: no extrusion), convert `t_start`/`t_end` from print-time seconds to 64-bit MCU-clock ticks via `clock_sync`.

3. **`push_segment(params)`:** Send via existing `kalico-host-rt::producer::push_segment()`, wait for ACK.

### 3.3 Implementation gaps (new code)

Curve loading requires two new functions in `kalico-host-rt` plus a conversion utility:

1. **Scalar wire encoder** — `encode_load_curve_v1` is stale (encodes 3D `[f32; 3]` control points from pre-7-B era). **Fix:** Write `encode_load_curve_scalar(degree: u8, knots: &[f32], cp: &[f32]) -> Vec<u8>` in `kalico-host-rt::wire`. Accepts 1D `f32` scalars matching the per-axis-scalar architecture.

2. **Host-side `load_curve()` transport function** — `push_segment()` exists but there's no paired function that sends `kalico_load_curve` and waits for `kalico_load_curve_response`. **Fix:** Write `load_curve<T: Transport>(io: &T, params: &CurveLoadParams) -> Result<CurveHandle, ProducerError>` in `kalico-host-rt::producer`. This function calls the scalar encoder, sends the command, and parses the response into a `CurveHandle`.

Both functions include f64→f32 truncation of `ShapedSegment`'s `ScalarNurbs<f64>` control points and knots as part of the encoding step (no separate utility needed — the truncation is inline in `encode_load_curve_scalar`'s caller).

3. **`compat::collinear::to_collinear_bezier()`** — new structured API variant alongside the existing `to_collinear_g5()`. Returns `[[f64; 3]; 4]` (4 control points) instead of `G5Line` text. Same 1/3-2/3 lerp math. The bridge calls this for G1→cubic conversion, preserving the CLAUDE.md contract that the bridge uses `compat` crate primitives.

## 4. Config surface

### 4.1 Read from existing printer.cfg sections

**`[printer]`:**
- `max_velocity` — global velocity cap (mm/s)
- `max_accel` — global acceleration cap (mm/s²)
- `max_z_velocity`, `max_z_accel` — Z-specific overrides
- `kinematics` — `corexy` or `cartesian` (others hard-error)

**`[input_shaper]`:**
- `shaper_freq_x`, `shaper_freq_y` — resonance frequencies (Hz)
- `shaper_type_x`, `shaper_type_y` — `smooth_zv` or `smooth_mzv` (others hard-error with "unsupported shaper type for MVP")

**`[stepper_*]`:**
- Pin prefix → MCU id (for axis-to-MCU mapping)
- `rotation_distance`, `microsteps`, `full_steps_per_rotation` → steps/mm (test validation only)

### 4.2 New knobs (optional, with defaults)

In `[printer]`:
- `planner_lookahead_window` — default 32. Streaming window capacity W.
- `beta_max_iterations` — default 10. β-medium iteration cap.

### 4.3 Accepted but no-op

- `pressure_advance`, `pressure_advance_smooth_time` — forward-compatible, no-op until Step 9.

### 4.4 Runtime updates

- `SET_VELOCITY_LIMIT` → `bridge.update_limits()` → `PlannerMsg::UpdateLimits`. Planner picks up new limits on next batch.
- `SET_INPUT_SHAPER` → `bridge.update_shaper()` → `PlannerMsg::UpdateShaper`. Planner picks up new shaper config on next batch.

## 5. motion_toolhead.py changes

### 5.1 Methods un-stubbed for Phase 2

| Method | Phase 2 behavior |
|--------|-----------------|
| `move(newpos, speed)` | Compute delta from `commanded_pos`, hard-reject if `de != 0` ("extrusion not yet supported"), clamp speed to config limits, call `bridge.submit_move(dx, dy, dz, feedrate)`, update `commanded_pos` optimistically |
| `manual_move(coord, speed)` | Same as `move()` with partial coordinates (None → no change) |
| `dwell(delay)` | `bridge.submit_dwell(delay)` — flush boundary + advance print time by delay |
| `wait_moves()` | `bridge.wait_moves()` — flush + block until physical execution complete (§2.4) |
| `get_last_move_time()` | `bridge.get_last_move_time()` — estimated print time of last queued move. Before any flush, returns an estimate based on accumulated move distances and configured velocity/accel limits. After flush, returns the precise `t_end` from the planner. |
| `set_position(newpos, homing_axes)` | `bridge.set_position(newpos)` — resets commanded_pos and bridge internal position state |
| `cmd_SET_VELOCITY_LIMIT` | Already parses args in Phase 1; now also calls `bridge.update_limits()` |
| `cmd_SET_INPUT_SHAPER` | Parses `shaper_freq_x/y`, `shaper_type_x/y`; calls `bridge.update_shaper()` |

### 5.2 Still stubbed (later phases)

| Method | Phase |
|--------|-------|
| `drip_move()` | Phase 4 (homing) |
| `register_lookahead_callback()` | Phase 3 (fires on `SegmentFinalized` events) |
| `flush_step_generation()` | Phase 3 |

## 6. Testing strategy

### 6.1 kalico-sim integration tests (fast inner loop)

Rust integration tests in `rust/motion-bridge/tests/`. These exercise the bridge API directly (Rust → bridge → sim MCU), not klippy's Python G-code dispatch. Full-stack tests through Python are a Phase 3 / Renode-gate concern.

1. **Single-axis X move:** Boot bridge + sim MCU. Submit `G1 X10 F600`. `wait_moves()`. Assert: correct stepper direction, step count ≈ `10mm × steps_per_mm`, monotonic timing, total duration consistent with velocity/accel limits.

2. **Single-axis Y move:** Same pattern with `G1 Y10 F600`.

3. **Single-axis Z move (different MCU):** Boot bridge + 2 sim MCUs (Octopus + F446). Submit `G1 Z5 F300`. Assert: steps only on F446's Z stepper, nothing on Octopus.

4. **Shaper validation:** Submit `G1 X50 F6000` (fast move). Capture per-axis position trajectory from sim at high rate. (a) FFT the acceleration profile — assert frequency content at `shaper_freq_x` is attenuated by expected dB for smooth-ZV/MZV. (b) Assert peak shaped acceleration ≤ `max_accel` (the β-medium guarantee).

5. **Velocity limit compliance:** Submit move with speed above `max_velocity`. Assert: actual peak velocity in step events ≤ `max_velocity`.

6. **SET_VELOCITY_LIMIT mid-session:** Submit move, change limits, submit another move. Assert: second move respects new limits.

### 6.2 Renode gate (run once before Phase 2 done)

Basic wire-protocol smoke test — not the full 7-D soak/capture work. All MCUs are simulated (Renode emulated H723 for Octopus, emulated F446 for bottom).

1. Same test commands through Renode with real H723 firmware binary.
2. Verify wire-level protocol: curve load commands arrive with correct scalar data.
3. Verify segment push commands arrive with correct handles and timing.
4. Verify step output pins toggle (Renode GPIO capture).

### 6.3 Real hardware follow-up (user-driven, not a Phase 2 gate)

User connects Trident hardware and runs test moves manually. Validates real motor motion, step timing under real USB-CDC latency, and any hardware quirks. Feeds back into Phase 3 / 7-D.

## 7. Risks and mitigations

- **temporal/trajectory API surface:** The bridge constructs `CubicSegment` and feeds it through `temporal::multi::plan_batch` → `trajectory::shape_batch`. These APIs were built for synthetic test input in Steps 4/4.5/7-A. If they assume properties that bridge-constructed segments don't have (e.g., specific knot vector normalization, arc-length bounds), expect integration friction. Mitigation: write the kalico-sim integration test early, iterate.

- **Curve pool slot exhaustion:** The MCU curve pool has a fixed number of slots. If the planner thread pushes faster than the MCU retires segments, slots exhaust. Mitigation: respect credit-based backpressure already in `push_segment`; for Phase 2's one-move-at-a-time pace, this is a non-issue. Phase 3 must handle it properly.

- **Clock-sync precision for `t_start`/`t_end`:** Print-time → MCU-clock conversion depends on clock-sync quality. Phase 1's clock-sync is battle-tested (24h soak in 7-C-io). If Phase 2 single-segment timing has visible jitter, it's a clock-sync tuning issue, not a design flaw.

## 8. Definition of done

`G1 X10 F600` produces correct step events in kalico-sim. `G1 Z5 F300` produces correct step events on the simulated F446 MCU. Shaper verified via FFT attenuation + peak-acceleration ≤ `max_accel` assertion. Renode wire-protocol smoke test passes with emulated H723/F446 firmware. All existing Phase 1 tests remain green.
