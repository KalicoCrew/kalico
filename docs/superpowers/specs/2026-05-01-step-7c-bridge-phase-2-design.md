# Step 7-C-bridge Phase 2 — First motion: straight-line single-axis

**Parent spec:** `docs/superpowers/specs/2026-05-01-step-7c-bridge-design.md` §6, Phase 2 row.

**Scope:** Wire `motion_toolhead.move()` end-to-end through the Rust planner pipeline to produce correct step events on the MCU. Single-axis test moves (X, Y on Octopus via CoreXY; Z on F446 via Cartesian). Full shaper pipeline active (smooth-ZV/MZV + β-medium). Validated in kalico-sim with Renode as a gate.

**Precondition:** Phase 1 complete — PyO3 bridge boots, passthrough router works for all MCUs, legacy trapezoidal code deleted, shim files in place with move methods raising `NotImplementedError`.

## 1. End-to-end data flow

```
motion_toolhead.move(newpos, speed)
  │  compute delta from self.commanded_pos
  ▼
bridge.submit_move(dx, dy, dz, de, feedrate)       ← PyO3, GIL released
  │  classify: XY-only → COUPLED(ratio=0), Z-only, etc.
  │  construct collinear cubic Bézier directly:
  │    P0 = start,  P1 = start + (end-start)/3,
  │    P2 = start + 2(end-start)/3,  P3 = end
  │  build nurbs::VectorNurbs<f64, 3> from [P0, P1, P2, P3]
  │  wrap as CubicSegment (no text round-trip, no GeometryPipeline)
  │  enqueue PendingMove to planner channel
  ▼
Planner thread (background, spawned at bridge init)
  │  accumulate moves in streaming window (capacity W, default 32)
  │  temporal TOPP-RA on window → velocity profiles
  │  trajectory::shape_batch(profiles, shaper_config) → ShapedSegments
  │      β-medium iteration, smooth-MZV convolution, time-reparam
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

**No `GeometryPipeline::process` in the live path.** `GeometryPipeline` is a text-based one-shot API (`fn process(&mut self, text: &str, ...) -> Segments`). The live bridge receives structured moves from Python, not G-code text. Serializing to G5 text and re-parsing would be wasteful. The bridge constructs `CubicSegment` directly from control points. `GeometryPipeline` remains in scope for the offline compat normalizer (Step 13).

**No `compat::collinear::to_collinear_g5` for the bridge path.** That function returns a `G5Line` (I/J/P/Q offset format), not raw control points. The 1/3-2/3 collinear Bézier construction is trivial; the bridge computes it directly. The `compat` crate primitives remain available for the live G2/G3→G5 conversion path (Phase 3) and the file preprocessor.

**Full shaper from the start.** Even single-segment terminal test moves go through smooth-MZV convolution and β-medium iteration. This proves the shaper in the simplest context (one segment, zero boundary velocity). Phase 3 exercises cross-boundary shaping with multi-segment windows.

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
    Flush(Arc<Notify>),     // from wait_moves() — process everything, then wake
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
   - A `Flush` signal was drained
   - A `Shutdown` was drained
4. Run the pipeline on the batch: temporal TOPP-RA → `shape_batch` → per-MCU push.
5. After all segments from this batch are ACKed by MCUs, wake any `Flush` waiters.
6. On `Shutdown`: flush remaining buffer, then exit loop.

For Phase 2 terminal commands: klippy's gcode dispatch calls `wait_moves()` after each interactive G1. This sends a `Flush` signal, so each terminal move processes as a window-of-1. That's correct behavior — the shaper convolves the single segment, β-medium iterates, and the user sees immediate execution.

For Phase 3 file printing: moves arrive faster than the planner drains. The buffer fills toward W naturally, producing multi-segment batches with cross-boundary shaping continuity. The transition is seamless — same mechanism, different arrival rate.

### 2.4 `wait_moves()` semantics

Python thread (GIL released) sends `Flush(notify)` to the channel, then blocks on the `notify`. The planner thread wakes the notify only after **all segments from the flushed batch have been pushed to wire AND acknowledged by the MCUs**. This means `wait_moves()` guarantees: everything submitted before this call has been shaped, pushed, and ACKed.

### 2.5 Error propagation

If the pipeline fails (TOPP-RA infeasible, shaper error, MCU push rejected, MCU fault), the planner thread stores the error in a shared `Mutex<Option<PlannerError>>`. The next Python call into the bridge (`submit_move`, `wait_moves`, etc.) checks this mutex and raises a Python exception if an error is present. The planner thread does not silently swallow errors.

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

Two gaps identified by verification:

1. **`encode_load_curve_v1` is stale** — encodes 3D `[f32; 3]` control points. The per-axis-scalar architecture (Step 7-B) needs 1D `f32` scalars. **Fix:** Write `encode_load_curve_scalar(degree: u8, knots: &[f32], cp: &[f32]) -> Vec<u8>` in `kalico-host-rt::wire`.

2. **No host-side `load_curve()` transport function** — `push_segment()` exists but there's no paired function that sends `kalico_load_curve` and waits for `kalico_load_curve_response`. **Fix:** Write `load_curve<T: Transport>(io: &T, params: &CurveLoadParams) -> Result<CurveHandle, ProducerError>` in `kalico-host-rt::producer`.

3. **f64→f32 conversion utility** — `fn scalar_nurbs_to_f32(src: &ScalarNurbs<f64>) -> (Vec<f32>, Vec<f32>)` returning (knots, control_points). Small helper, lives in `motion-bridge` or `kalico-host-rt`.

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
| `move(newpos, speed)` | Compute delta from `commanded_pos`, clamp speed to config limits, call `bridge.submit_move(dx, dy, dz, de, feedrate)`, update `commanded_pos` |
| `manual_move(coord, speed)` | Same as `move()` with partial coordinates (None → no change) |
| `dwell(delay)` | `bridge.submit_dwell(delay)` — flush boundary |
| `wait_moves()` | `bridge.wait_moves()` — flush + block until all ACKed |
| `get_last_move_time()` | `bridge.get_last_move_time()` — estimated print time of last queued move |
| `set_position(newpos, homing_axes)` | `bridge.set_position(newpos)` — resets commanded_pos and bridge internal position state |
| `cmd_SET_VELOCITY_LIMIT` | Already parses args in Phase 1; now also calls `bridge.update_limits()` |

### 5.2 Still stubbed (later phases)

| Method | Phase |
|--------|-------|
| `drip_move()` | Phase 4 (homing) |
| `register_lookahead_callback()` | Phase 3 (fires on `SegmentFinalized` events) |
| `flush_step_generation()` | Phase 3 |

## 6. Testing strategy

### 6.1 kalico-sim integration tests (fast inner loop)

Rust integration tests in `rust/motion-bridge/tests/`:

1. **Single-axis X move:** Boot bridge + sim MCU. Submit `G1 X10 F600`. `wait_moves()`. Assert: correct stepper direction, step count ≈ `10mm × steps_per_mm`, monotonic timing, total duration consistent with velocity/accel limits.

2. **Single-axis Y move:** Same pattern with `G1 Y10 F600`.

3. **Single-axis Z move (different MCU):** Boot bridge + 2 sim MCUs (Octopus + F446). Submit `G1 Z5 F300`. Assert: steps only on F446's Z stepper, nothing on Octopus.

4. **Shaper validation:** Submit `G1 X50 F6000` (fast move). Capture per-axis position trajectory from sim at high rate. FFT the acceleration profile. Assert: frequency content at `shaper_freq_x` is attenuated by expected dB for smooth-MZV.

5. **Velocity limit compliance:** Submit move with speed above `max_velocity`. Assert: actual peak velocity in step events ≤ `max_velocity`.

6. **SET_VELOCITY_LIMIT mid-session:** Submit move, change limits, submit another move. Assert: second move respects new limits.

### 6.2 Renode gate (run once before Phase 2 done)

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

`G1 X10 F600` produces correct step events in kalico-sim. `G1 Z5 F300` produces correct step events on the F446 sim MCU. Shaper is verified via FFT on acceleration profile. Renode gate passes with real H723 firmware. All existing Phase 1 tests remain green.
