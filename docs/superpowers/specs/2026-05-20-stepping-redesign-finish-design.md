# Stepping-redesign finish — design

**Status:** design, brainstormed 2026-05-20

**Builds on:**
- `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md` — the original redesign spec (this doc finishes what it started)
- `docs/superpowers/plans/2026-05-19-stepping-redesign-implementation.md` — firmware-only Tasks 1–18, completed but with three documented deviations

**Goal.** Finish the stepping-redesign cutover: replace the NURBS-shaped curve pool with cubic-Bezier monomial-form storage, complete the legacy-path deletion that firmware Task 16 deferred, migrate the host bridge + klippy to emit the new commands, and produce a runtime that is end-to-end cubic-Bezier-only on both H7 and F4. No bench bring-up steps — this spec covers *only* getting the code architecturally correct. Bench validation is a follow-up plan.

---

## 1. Architecture overview

The redesign moves the runtime from NURBS-shaped storage + Newton step-time iteration to cubic-Bezier monomial-form storage + per-sample TIM5-ISR evaluation. Firmware Tasks 1–18 (Apr/May 2026) landed most of the new structure but stopped short of three load-bearing pieces:

1. `curve_pool` still stores NURBS (control_points + knot_vector + weights + degree) instead of arrays of cubic Bezier pieces.
2. `AxisConfig.piece` is a single `Option<BezierPieceMonomial>` rather than a cursor into a multi-piece curve. Multi-piece segments cannot execute.
3. `kalico_configure_axis` doesn't carry per-stepper bindings (step pin / dir pin / dir invert / tmc_cs). The new path can't fire step pulses without falling back to the legacy `command_config_runtime_stepper` binding table.

This spec corrects all three, deletes the legacy stepping stack entirely (no parallel paths), and migrates the host bridge + klippy in the same cutover. After this lands, the runtime is uniformly cubic-Bezier across host planner, transport, MCU storage, and ISR evaluation.

**Both MCUs migrate together.** F4 firmware ships the same Rust staticlib as H7 (only `mcu-f4` feature differs, gating a couple of `nurbs`-crate forwards). There is no architectural reason to keep the legacy path for the Z axis. One-shot replacement, no fallback.

**Scope explicitly excludes** bench bring-up stages, Phase-mode SPI exercises beyond the existing TMC5160 XDIRECT push, sensorless-homing mode switching, and long-print soak validation. Those become follow-up plans once this lands.

---

## 2. Curve pool: shape, sizing, storage

### 2.1 Slot shape

One slot stores one per-axis cubic-Bezier curve: a fixed-cap array of `BezierPieceMonomial` plus a `piece_count: u16`. Slot-allocation, generation versioning, and retire discipline are inherited unchanged from the existing `curve_pool` (`try_alloc_and_load`, the `(current_gen, last_retired_gen)` `AtomicU16` pair, `lookup` with generation match, `confirm_retired` from foreground).

```rust
#[repr(C)]
pub struct LoadedCubicCurve {
    pub piece_count: u16,
    pub _pad: [u8; 2],
    pub pieces: [BezierPieceMonomial; MAX_PIECES_PER_CURVE],
}
```

`BezierPieceMonomial` already exists from firmware Task 2:

```rust
#[derive(Clone, Copy, Debug)]
pub struct BezierPieceMonomial {
    pub coeffs: [f32; 4],     // c0 + c1·t + c2·t² + c3·t³
    pub vel_coeffs: [f32; 3], // pre-baked derivative
    pub duration: f32,
}
```

With f32 align that's 32 bytes per piece. With `MAX_PIECES_PER_CURVE = 16` → ~516 bytes per slot (516 = 4 header + 16 × 32).

### 2.2 Sizing

Match existing `CURVE_POOL_N` Kconfig values:

| Profile | CURVE_POOL_N | MAX_PIECES_PER_CURVE | Per-slot bytes | Total |
|---|---|---|---|---|
| H7 (LARGE) | 16 | 16 | ~516 | ~8.2 KB |
| F4 (SMALL) | 8 | 16 | ~516 | ~4.1 KB |

Today's NURBS `kalico_buf = 14752` bytes. Net savings:
- H7: ~6.5 KB freed from AXI SRAM (relieves the 99.35% pressure from the firmware build).
- F4: ~10.6 KB freed.

Plus the larger NURBS-only sizing constants (`MAX_CONTROL_POINTS = 1830`, `MAX_KNOT_VECTOR_LEN = 1850`, `MAX_DEGREE = 10`) go away from `runtime/build.rs` and `src/Makefile`'s envvar passthrough.

`MAX_PIECES_PER_CURVE = 16` accommodates G2/G3 quarter-arcs (~8 pieces typical) with headroom for spline fits. Worst-case G2 full-circle (~32 pieces) requires the host planner to split into two segments — defensible because the spec already says segments are arbitrary-size units composed by the host.

### 2.3 Storage location

Same as today's `kalico_buf`: `.axi_bss` on H7, `.bss` on F4. The pool is large enough that DTCM placement would crowd stack+heap on F4. AXI access cost matters only on the foreground load path, not the TIM5 ISR — the ISR copies the active piece into a register-resident `BezierPieceMonomial` once per piece-boundary advancement and works from there.

---

## 3. Wire format / FFI surface

### 3.1 `kalico_configure_axis` — extended

Add a variable-length per-stepper sub-message tail. Per-stepper payload is 4 bytes:

```c
struct StepperBindingWire {
    uint8_t stepper_oid;     // klipper oid of an existing command_config_stepper allocation
    uint8_t dir_invert;      // 0 or 1
    uint8_t tmc_cs_oid;      // 0xFF = none (Pulse-only stepper), else oid of command_config_spi
    uint8_t flags;           // reserved, 0 for now
};
```

`DECL_COMMAND` format:

```
kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u
    extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s
```

The `%*s` tail is exactly `stepper_count * 4` bytes. Firmware C handler validates length, then for each stepper:

1. `oid_lookup(stepper_oid, command_config_stepper)` to resolve to the existing `struct stepper *`.
2. If `tmc_cs_oid != 0xFF`, `oid_lookup(tmc_cs_oid, command_config_spi)` for the SPI handle.
3. Populate the C-side mapping table `runtime_motor_steppers[axis_idx][slot] = { .stepper = s, .invert_dir = dir_invert }` — same shape and array as today's `command_config_runtime_stepper`, indexed by axis instead of motor (they're the same here).
4. Call into the Rust FFI to populate `axis.steppers[slot]` with `tmc_cs` only (plus the zero-initialized atomics for `position_count`, `last_coil_A/B`, `phase_offset_*`, `last_phase_target`).

### 3.2 `runtime_handle_load_curve_cubic` — new, replaces NURBS load_curve

One-shot upload. The kalico-native-transport carries frames up to 64 KB; a 16-piece × 32-byte curve plus a small header fits trivially in one frame. The C dispatch handler reads the full frame and calls the FFI once.

Wire frame body:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | `slot_idx` (u16) |
| 2 | 1 | `axis_idx` (u8) |
| 3 | 1 | `piece_count` (u8) |
| 4 | `piece_count * 20` | array of `(bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits)` u32 quintuplets |

So per-piece wire payload is 20 bytes (Bernstein control points + duration as f32-as-u32-bits). Firmware does the Bernstein → monomial conversion on the FFI side using `monomial::bernstein_to_monomial` (already in place from firmware Task 2). The slot stores the monomial form.

FFI:

```rust
pub unsafe extern "C" fn runtime_handle_load_curve_cubic(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    axis_idx: u8,
    piece_count: u8,
    pieces_blob: *const u8,  // piece_count * 20 bytes
    out_handle_packed: *mut u32,
) -> i32;
```

Returns `KALICO_OK` and writes `(generation << 16) | slot_idx` into `out_handle_packed` on success. Atomic: slot transitions to "valid" only after all pieces written and `current_gen += 1`.

Validation:
- `piece_count > 0 && piece_count <= MAX_PIECES_PER_CURVE`
- All `duration > 0 && is_finite()`
- All Bernstein bits decode to finite f32 values

Failure → `KALICO_ERR_INVALID_CURVE` (or new `CurveLoadInvalid`), no slot mutation, no generation bump.

### 3.3 `runtime_handle_push_segment` — semantics extended

Wire signature unchanged (4 packed curve handles for X/Y/Z/E + `t_start_lo/hi` + segment id). What changes: the engine's segment-arm logic.

On arm, for each axis:

```rust
if handle == EMPTY_HANDLE_SENTINEL {
    axis.curve_handle = None;
    axis.piece = None;
    axis.piece_cursor = 0;
} else {
    let curve = curve_pool.lookup(handle)?;
    axis.curve_handle = Some(handle);
    axis.piece_cursor = 0;
    axis.piece = Some(curve.pieces[0]);
    axis.piece_start_time_cycles = segment.t_start;
}
```

### 3.4 Deleted FFI

| Symbol | Reason |
|---|---|
| `runtime_handle_load_curve` (NURBS variant) | Replaced by `_load_curve_cubic` |
| `kalico_runtime_producer_step` | Legacy Newton-fill loop |
| `kalico_runtime_step_ring_peek_head` | Legacy step-ring consumer |
| `kalico_runtime_step_ring_peek_next` | Legacy step-ring consumer |
| `kalico_runtime_step_ring_advance` | Legacy step-ring consumer |
| `kalico_runtime_modulated_tick` | Legacy TIM5 ISR entry (replaced by `_tick_sample`, already in place) |

`cbindgen.toml` updated accordingly; the generated `kalico_runtime.h` regenerated.

---

## 4. TIM5 ISR + per-axis piece advancement

The TIM5 ISR body (`runtime_tick_sample`, firmware Task 8) already evaluates `axis.piece` once per sample. The change here is purely upstream: where `piece` comes from on segment-arm and on piece-boundary crossing.

### 4.1 `AxisConfig` adds a cursor

```rust
pub struct AxisConfig {
    pub mode: AtomicU8,
    pub steppers: heapless::Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub curve_handle: Option<CurveHandle>,   // NEW
    pub piece_cursor: u16,                    // NEW
    pub piece: Option<BezierPieceMonomial>,   // existing: cached active piece
    pub piece_start_time_cycles: u64,
    pub last_step_count: i32,
    pub microstep_distance: f32,
    pub extrusion_per_xy_mm: f32,
}
```

`piece` stays as a cached copy of `curve.pieces[piece_cursor]`. Refresh happens only on piece-boundary advancement, not every sample.

### 4.2 `StepperRef` shrinks

```rust
pub struct StepperRef {
    pub position_count: AtomicI32,
    pub tmc_cs: Option<u32>,
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}
```

The fields `step_pin: u32`, `dir_pin: u32`, `dir_invert: bool` from firmware Task 6 are removed — vestigial, never read. The C-side `runtime_motor_steppers[][]` table holds the actual `struct stepper *` for GPIO emission; the Rust side only needs `tmc_cs` for Phase-mode SPI dispatch.

### 4.3 Segment arm

In the engine's `push_segment` handler (already exists, just extended): decode the 4 axis curve handles, validate each via `curve_pool.lookup`, populate the per-axis `(curve_handle, piece_cursor, piece, piece_start_time_cycles)` quadruple atomically. If any axis's handle is the empty sentinel (defined as `EMPTY_HANDLE_SENTINEL = 0`, which the existing slot allocator never produces because generations start at 1), that axis stays idle for this segment.

### 4.4 Piece advancement

`advance_piece_if_needed` (firmware Task 9) gets the cursor-walking logic:

```rust
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    axis_idx: usize,
    shared: &SharedState,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u8 = 0;
    loop {
        let Some(piece) = axis.piece else { break };
        let t_local = t_sample_end_global - piece_start_seconds(axis, cycles_per_second);
        if t_local <= piece.duration { break; }

        // Advance: bump piece_start by current piece's duration, walk cursor.
        axis.piece_start_time_cycles = axis.piece_start_time_cycles
            .wrapping_add((piece.duration * cycles_per_second) as u64);
        axis.piece_cursor = axis.piece_cursor.saturating_add(1);
        advanced = true;

        match axis.curve_handle {
            Some(handle) => match curve_pool::lookup_active(handle) {
                Some(curve) if (axis.piece_cursor as usize) < curve.piece_count as usize => {
                    axis.piece = Some(curve.pieces[axis.piece_cursor as usize]);
                }
                _ => {
                    // Curve exhausted — segment is either retiring (next segment-arm
                    // will refill) or the host shorted the curve (fault below).
                    axis.piece = None;
                    axis.curve_handle = None;
                }
            },
            None => { axis.piece = None; break; }
        }

        iters = iters.saturating_add(1);
        if iters > 4 {
            raise_piece_advance_underflow(shared, axis_idx);
            break;
        }
    }
    advanced
}
```

The `iters > 4` cap remains as defensive code (catches `duration == 0` foot-guns).

### 4.5 Segment retire

Phase 5 of `runtime_tick_sample` (firmware Task 9 stub) becomes: if every axis has `curve_handle == None` and the segment-local accumulator (`ds_xy_segment`) is non-zero, publish `retired_through_segment_id += 1`, reset the accumulator, signal `curve_pool::confirm_retired` for each retired slot from the foreground reactor.

---

## 5. Per-stepper bindings — concrete model

### 5.1 What we keep from mainline/legacy

- `command_config_stepper(oid, step_pin, dir_pin, invert_step, step_pulse_ticks)` — allocates `struct stepper` via `oid_alloc`, sets up GPIO. **Unchanged.**
- `runtime_emit_step_pulses(motor_idx, n_steps)` — the C-side GPIO emitter. Reads `runtime_motor_steppers[motor_idx][j].stepper->step_pin / dir_pin`. **Unchanged.**
- The mapping table `runtime_motor_steppers[N_MOTORS][N_PARTNERS]`. **Unchanged in shape**, just populated by a different command.

### 5.2 What changes

`command_config_runtime_stepper` (the per-stepper-per-motor bind command) is deleted. `command_kalico_configure_axis` populates the same `runtime_motor_steppers[][]` table via its sub-message:

```c
void command_kalico_configure_axis(uint32_t *args) {
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count   = args[4];
    const uint8_t *blob     = (const uint8_t *)args[5];
    uint16_t blob_len       = args[6];

    if (blob_len != (uint16_t)stepper_count * 4)
        shutdown("configure_axis blob length mismatch");

    // Populate the C-side mapping table.
    runtime_motor_stepper_count[axis_idx] = 0;
    for (uint8_t i = 0; i < stepper_count; i++) {
        uint8_t stepper_oid = blob[i*4 + 0];
        uint8_t dir_invert  = blob[i*4 + 1];
        struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
        runtime_motor_steppers[axis_idx][i].stepper = s;
        runtime_motor_steppers[axis_idx][i].invert_dir = dir_invert ? 1 : 0;
        runtime_motor_stepper_count[axis_idx]++;
    }

    // Call into Rust to populate the per-stepper atomic state.
    StepperBindingRust bindings[MAX_STEPPERS_PER_AXIS];
    for (uint8_t i = 0; i < stepper_count; i++) {
        uint8_t tmc_cs_oid = blob[i*4 + 2];
        bindings[i].tmc_cs_handle = (tmc_cs_oid == 0xFF)
            ? 0
            : (uint32_t)(uintptr_t)oid_lookup(tmc_cs_oid, command_config_spi);
    }
    int32_t rc = kalico_runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits, extrusion_bits,
        stepper_count, bindings);
    if (rc != 0) shutdown("configure_axis rejected by runtime");
}
DECL_COMMAND(command_kalico_configure_axis,
    "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
    " extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s");
```

`StepperBindingRust` is the FFI payload type (just `tmc_cs_handle: u32` for now, room for future flags via the reserved `flags` wire byte).

### 5.3 Direction-pin handling

Identical to today: `runtime_emit_step_pulses` reads `invert_dir` from the C-side table and applies it via `pin_level = (!want_dir) ^ invert_dir`. The Rust side carries no direction state.

---

## 6. What gets deleted

### 6.1 Firmware Rust (`rust/runtime/src/`)

- `step_ring.rs` — entire file, the 1024-entry per-motor step-time ring buffer used by the Newton iteration.
- `step_time.rs` — entire file, `compute_next_step_time` and `solve_monotone_cubic_root`.
- `step_producer.rs` — entire file.
- `engine.rs::producer_step` — the ~900-line Newton-fill loop method.
- `engine.rs::arm_step_timer*` — ~180 LOC.
- `engine.rs::Engine` fields used only by the legacy path: `step_rings`, `producer_states`, `producer_current`, `motor_curve_cursor`, `motor_current_segment_id`.
- NURBS guts of `curve_pool.rs`: the `control_points`, `knot_vector`, `weights` arrays; the sizing constants `MAX_CONTROL_POINTS`, `MAX_KNOT_VECTOR_LEN`, `MAX_DEGREE`. The slot+generation discipline stays.
- `stepping_state.rs::StepperRef` vestigial fields: `step_pin`, `dir_pin`, `dir_invert`.

### 6.2 Firmware C (`src/`)

- `runtime_tick.c::step_time_event` — per-stepper Klipper-timer body.
- `runtime_tick.c::runtime_producer_event` — producer Klipper-timer body.
- `runtime_tick.c::init_step_time_timers` — installer. Replaced by `init_per_axis_step_timers` (already in place from firmware Task 10, just no caller until this spec).
- `runtime_tick.c` defines: `SF_RESCHEDULE_FLOOR`, `EMPTY_POLL_CYCLES`, `STEP_RING_LOW_WATER`.
- `runtime_tick.c::arm_producer_timer_*` helpers.
- `stepper.c::command_config_runtime_stepper` — replaced by `command_kalico_configure_axis` sub-message.
- All `step_time_event_*` and `runtime_producer_*` diag counters and their packed-fault tags (`0xE3`, `0xE7`, `0xE8`).

### 6.3 Firmware FFI (`rust/kalico-c-api/`)

- `runtime_handle_load_curve` (NURBS).
- `kalico_runtime_producer_step`.
- `kalico_runtime_step_ring_peek_head` / `_peek_next` / `_advance`.
- `kalico_runtime_modulated_tick`.
- `cbindgen.toml` updated, `include/kalico_runtime.h` regenerated.

### 6.4 Host (`rust/motion-bridge/`, `klippy/`)

- `motion-bridge/src/bridge.rs` + `producer.rs`: NURBS upload paths (begin/chunk/finalize over control points + knots + weights + degree).
- `motion-bridge` planner serialization: stop shaping segments as NURBS, start emitting cubic Bezier per-axis pieces.
- `klippy/motion_toolhead.py`: drop the `config_runtime_stepper` emit; emit `kalico_configure_axis` per axis instead.

### 6.5 What stays unchanged

- `runtime_emit_step_pulses` — the C-side GPIO emitter.
- `command_config_stepper` — the mainline per-OID stepper allocation. Pin setup unchanged.
- Endstop sampling (`kalico_endstop_tick_step_time`), watchdog/IWDG, fault propagation, shutdown handling.
- Slot+generation discipline inside `curve_pool` (`try_alloc_and_load`, `lookup`, `confirm_retired`).
- Segment push/retire host-sync via `retired_through_segment_id`, `accepted_segment_id`, `credit_epoch`.
- The gcode parser, the `compat` crate (G1/G2/G3 → cubic conversion), and the entire host-side planner math.

### 6.6 Rough size estimate

~5,000 LOC removed, ~1,500 LOC added. Net win: simpler code, one curve shape, one ISR path, one timer pattern, freed AXI SRAM, lower compile time.

---

## 7. Cross-MCU coordination

**No architectural change.** Each MCU runs its own TIM5 ISR independently and evaluates only its own axes (H7: motors A/B/E for CoreXY X/Y/E; F4: motor Z). The host coordinates by sending per-MCU segments through the existing `accepted_segment_id` / `credit_epoch` telemetry path. The `kalico_status` frame already reports per-MCU engine state.

The new `kalico_configure_axis` is emitted per MCU, per axis-on-that-MCU, with the steppers that physically live on that MCU. Klippy's existing `motion_toolhead.py` already does this partitioning for the legacy `configure_axes_blob`; the new emit path reuses the same partitioning.

---

## 8. Faults

All faults from firmware Task 6 carry over:

- `StepQueueOverflow(axis_idx)`
- `SpiQueueOverflow(bus_idx)`
- `MathNonFinite(axis_idx)`
- `PieceAdvanceUnderflow(axis_idx)` — now triggers when the ISR's piece-advance loop exhausts the curve while the segment isn't retiring (host shorted a curve).
- `SampleRateMisconfigured`
- `PositionCountOverflow(stepper_idx)`
- `JogParametersInvalid`
- `StepRateExceedsMcuCeiling(axis_idx)`

**New fault:** `CurveLoadInvalid` — fires from `runtime_handle_load_curve_cubic` on non-finite Bernstein, `piece_count` out of range, or `duration <= 0`. Foreground reject (returns `KALICO_ERR_INVALID_CURVE` to host), no shutdown.

**Piece-queue overflow.** The original brainstorming considered this. Verdict: not a runtime condition — pieces live in slots, slots are tracked via the credit-epoch protocol. A "queue full" condition would be a slot-allocation failure from `try_alloc_and_load`, returned to the host as an error from the `_load_curve_cubic` FFI. Host backs off via the credit-epoch mechanism that already exists. No new fault code needed.

**`runtime_storage.c` static_assert update.** Drop the NURBS `kalico_buf` size term from the AXI budget computation. Add the new cubic-curve-pool size term. Both H7 and F4 ceilings stay within their existing AXI/total-SRAM budgets with net headroom freed.

---

## 9. Testing

Two tiers, neither bench-dependent:

### 9.1 Unit / integration tests in `rust/runtime/tests/`

Each new module gets isolated coverage:

- `curve_pool` retyped for cubic pieces — `try_alloc_and_load` with `LoadedCubicCurve`, `lookup` with generation match, `confirm_retired` slot reclamation.
- `configure_axis` stepper-binding decoder — round-trip blob encoding/decoding, multiple stepper counts, edge cases (count=0, tmc_cs=0xFF, max stepper_count).
- Piece-advancement cursor walking — single-piece curve, multi-piece curve advancing through all pieces, cursor past `piece_count` triggers `axis.piece = None`, the `iters > 4` guard.
- End-to-end single-MCU ISR — feed a 1-piece linear curve through `dispatch_axis` (Pulse mode), confirm step_queue entries with expected `cycle_abs` + `dir`.

### 9.2 klipper-sim integration

Finish firmware Task 15 (scaffolded but not wired). The simulator's step-pulse generator runs through the new firmware build (`thumbv7em-none-eabihf` for H7, native for the host-side comparison). Apples-to-apples: feed identical G-code to mainline Klipper + our fork, compare step-time traces.

Threshold: per spec, max drift per step < 500 ns at typical accel. This is the primary correctness check before any bench attempt. Stages 1–7 from the original redesign spec are validated here in simulation, not on hardware.

### 9.3 No bench validation in scope

On-bench validation is explicitly out of scope. The bench bring-up — boot-and-enumerate, single-axis jog, multi-axis sustained motion, Phase mode SPI, sensorless homing, long-print soak — is a follow-up plan that takes the klipper-sim-validated firmware to hardware. No `boot` / `enumerate` / `move motor` steps appear in this spec or its implementation plan.

---

## 10. Migration ordering

Flag-day cutover. Build will fail mid-sequence (and that's OK because the current state doesn't work either). Each step is a single commit; commits stay reviewable even though intermediates don't build:

1. **Reshape `curve_pool` slot type** — `LoadedCubicCurve` replaces NURBS struct fields. Delete NURBS sizing constants from `runtime/build.rs` and `src/Makefile` envvars. Build broken: nothing yet calls the new shape.
2. **Delete legacy Rust** — `step_ring.rs`, `step_time.rs`, `step_producer.rs`, `Engine::producer_step`, `Engine::arm_step_timer*`, vestigial `StepperRef` fields. Build still broken: FFI consumers of those symbols won't link.
3. **Update FFI surface** — drop NURBS `runtime_handle_load_curve`, add `_load_curve_cubic`, extend `kalico_runtime_configure_axis` for stepper bindings. Rebuild `cbindgen` header. Build closer to green: only C-side stragglers remain.
4. **Delete legacy C** — `step_time_event`, `runtime_producer_event`, `init_step_time_timers`, `command_config_runtime_stepper`, the `SF_RESCHEDULE_FLOOR` family of defines, diag counters. Firmware builds green.
5. **Bridge migration** — `motion-bridge` emits cubic curves via the new FFI, no NURBS path. `host-rt` updates. Host-side compiles.
6. **Klippy migration** — `motion_toolhead.py` emits the new `configure_axis` per axis, not legacy `config_runtime_stepper`. End-to-end host + firmware build green.
7. **Tests + klipper-sim** — unit tests pass on host, klipper-sim apples-to-apples comparison ≤ 500 ns step-time drift across the Stage-1..7 G-code matrix.

Bench validation is a separate follow-up plan started only after step 7 passes.

---

## Out of scope

- Bench bring-up Stages 1–7 from the original redesign spec.
- TMC5160 register pre-config (chopconf, stallguard) — printer.cfg's existing TMC config handles it.
- klippy-side host planner changes beyond the bridge serialization layer. The planner's internal math already operates on cubics for most of the pipeline.
- F4 firmware-side feature parity with H7 (Phase mode, multi-stepper-per-axis) — F4 in this redesign just runs Z in Pulse mode through the same code, no special-casing.
- Performance characterization (per-MCU step-rate ceiling profiling) — telemetry surfaces it via `sample_isr_peak_cycles`; characterization is bench-side work.
- Closed-loop encoder integration, hardware capture-compare GPIO toggling, DMA-driven GPIO — all explicitly deferred per the original spec's "Open items" section.
