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

So per-piece wire payload is 20 bytes (Bernstein control points + duration as f32-as-u32-bits). The Bernstein control points are in **standard unit-interval form** — i.e., they define P(t) for t ∈ [0, 1]. The firmware converts to seconds-domain monomial form on load so the ISR's `t_local_for_axis` (in seconds) can be evaluated directly by Horner without per-sample rescaling.

**Load-time conversion** (per piece, in `runtime_handle_load_curve_cubic`):

1. Algebraic Bernstein → unit-interval monomial via the existing `monomial::bernstein_to_monomial(bp)` (firmware Task 2): yields `(c0, c1, c2, c3)` such that `P(τ) = c0 + c1·τ + c2·τ² + c3·τ³` for τ ∈ [0, 1].
2. Rescale to seconds domain by the piece's duration `d`: `(c0_s, c1_s, c2_s, c3_s) = (c0, c1/d, c2/d², c3/d³)`. After this, `P(t_sec) = c0_s + c1_s·t_sec + c2_s·t_sec² + c3_s·t_sec³` for `t_sec ∈ [0, d]` produces the same physical mm value.
3. Pre-bake derivative coefficients: `vel_coeffs = (c1_s, 2·c2_s, 3·c3_s)`. The ISR reads these directly for velocity.
4. Store the resulting `BezierPieceMonomial { coeffs: [c0_s, c1_s, c2_s, c3_s], vel_coeffs, duration: d }` into the slot.

Because the rescale is purely algebraic and the existing `bernstein_to_monomial` doesn't take a duration, the cleanest realization is a thin wrapper `monomial::bernstein_to_monomial_with_duration(bp, duration_sec) -> BezierPieceMonomial` that does steps 1–3 and returns the final piece. Firmware Task 2's `bernstein_to_monomial` stays as-is for unit-interval math (used by host-side tests and the offline klipper-sim path).

Firmware Task 8's integration test (`rust/runtime/tests/tick_integration.rs`) currently pre-scales CPs on the host side and relies on the unit-interval-becoming-seconds-domain accident; that test should be updated to use `bernstein_to_monomial_with_duration` and pass standard unit-interval CPs as part of this migration. The misleading scale-factor comment in the test header goes away.

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

Validation (Phase 1, before any slot mutation):

- `piece_count > 0 && piece_count <= MAX_PIECES_PER_CURVE`
- All `duration > 0 && is_finite()`
- All Bernstein bits decode to finite f32 values

**No minimum-duration check at load.** Earlier drafts proposed a lower bound on `piece.duration` relative to the sample period to keep the piece-advancement loop bounded. That's not necessary: the loop's hard cap is `MAX_PIECES_PER_CURVE` (16) iterations, and a sample can structurally never advance past the curve's end. Validating `duration > 0 && is_finite()` is sufficient — a piece with `duration = 1 ns` is legal-but-pathological and the iter cap handles it.

Failure → `KALICO_ERR_INVALID_CURVE` (or new `CurveLoadInvalid`), no slot mutation, no generation bump.

### 3.3 `runtime_handle_push_segment` — semantics extended, wire unchanged

The existing wire/FFI signature is preserved verbatim — all fields stay (see `rust/kalico-c-api/src/runtime_ffi.rs:206` for the live signature):

```c
int32_t runtime_handle_push_segment(
    KalicoRuntime *rt,
    uint32_t id,
    uint32_t x_handle_packed, uint32_t y_handle_packed,
    uint32_t z_handle_packed, uint32_t e_handle_packed,
    uint64_t t_start_cycles, uint64_t t_end_cycles,
    uint8_t kinematics,            // 0=Cartesian, 1=CoreXY
    uint8_t e_mode,                // EMode: 0=CoupledToXy, 1=Independent, 2=Travel
    uint32_t extrusion_ratio_bits, // f32 E/XY ratio for CoupledToXy mode
    uint32_t *out_accepted_segment_id,
    uint32_t *out_credit_epoch);
```

The 42-byte command body and the response back through the host pipeline are unchanged.

**What does change is the engine's per-axis arm logic.** The existing `rust/runtime/src/segment.rs::Segment` struct already carries `id`, the 4 handles, `t_start`/`t_end`, `kinematics`, `e_mode`, and `extrusion_ratio` — every per-segment field the ISR needs. We preserve it as-is. The `Engine::current: Option<Segment>` field also stays (already present in engine.rs's `Engine` struct; that's what `engine.rs:2266` reads `current.id` from on retire).

We add two small bitmasks on the Engine for retire bookkeeping, alongside `current`:

```rust
pub struct Engine {
    // ... existing fields ...
    pub current: Option<Segment>,         // EXISTING; the active segment for the ISR
    pub participating_mask: u8,           // NEW: 4 bits, frozen at arm — which axes participate
    pub pending_mask: u8,                 // NEW: 4 bits, mutated during evaluation — which still pending
}
```

`participating_mask` is computed at arm-time from the segment's handles and `e_mode` (see §4.5). `pending_mask` starts equal to it and clears bits as each axis's curve exhausts. Retire fires when `pending_mask == 0`. The existing legacy `motor_curve_cursor` / `motor_current_segment_id` per-motor bitmasks are deleted along with the rest of the legacy path (§6.1).

Per-axis arm:

```rust
if handle == CurveHandle::UNUSED_SENTINEL {
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

The `participating_mask` is built per-axis using both the handle and e_mode (see §4.5). The segment-level fields drive the E-follower math (§4.6).

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
}
```

`piece` stays as a cached copy of `curve.pieces[piece_cursor]`. Refresh happens only on piece-boundary advancement, not every sample.

**Note on the deleted `extrusion_per_xy_mm: f32` field.** Firmware Task 6 added this speculatively. The per-segment `extrusion_ratio_bits` field on `runtime_handle_push_segment` is the correct location for the E/XY ratio because the ratio varies per move (flow rate, extrusion width, retract semantics). The axis-level field is removed; the ISR reads `Engine::current_segment.extrusion_ratio_per_xy_mm` for the follower math (see §4.6).

### 4.2 `StepperRef` shrinks

```rust
pub struct StepperRef {
    pub position_count: AtomicI32,
    pub tmc_cs_oid: Option<u8>,         // OID of command_config_spi for this stepper's TMC; None = Pulse-only
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}
```

The fields `step_pin: u32`, `dir_pin: u32`, `dir_invert: bool` from firmware Task 6 are removed — vestigial, never read by anything. The C-side `runtime_motor_steppers[][]` table holds the actual `struct stepper *` for GPIO emission; the Rust side only needs to know which TMC the SPI drain task should target.

**Why `tmc_cs_oid: Option<u8>` and not a pointer.** Firmware Task 6's `tmc_cs: Option<u32>` was speculatively typed as a "raw handle" and never wired through. Casting a `struct spidev_s *` to `u32` truncates on the host build (64-bit pointers) and doesn't help the SPI driver anyway — `spidev_transfer` needs the full `struct spidev_s *`, not just its address. The OID is small (u8), portable across host and MCU, and the SPI drain task does `oid_lookup(tmc_cs_oid, command_config_spi)` exactly once per transfer to recover the device pointer.

### 4.2a `SpiWrite` carries the OID

`SpiWrite` (in `spi_queue.rs`, firmware Task 14) currently carries `cs_pin: u32`. Re-typed:

```rust
#[repr(C)]
pub struct SpiWrite {
    pub tmc_cs_oid: u8,
    pub reg: u8,
    pub _pad: [u8; 2],
    pub value: i32,
}
```

Down from 12 to 8 bytes. The `SpiQueue` slot count stays the same (16 entries per axis); each entry is now smaller. The foreground SPI drain task in `runtime_tick.c::spi_drain_event` pops a `SpiWrite`, calls `oid_lookup(tmc_cs_oid, command_config_spi)`, and dispatches via `spidev_transfer`. The current stub implementation that "drops the entry without dispatching" becomes a real `spidev_transfer` call by this spec's completion (Stage 5 of the original bench-bringup spec, now scope-deferred).

### 4.3 Segment arm

In the engine's `push_segment` handler (already exists, just extended): decode the 4 axis curve handles, validate each via `curve_pool.lookup`, populate the per-axis `(curve_handle, piece_cursor, piece, piece_start_time_cycles)` quadruple atomically. If any axis's handle is `CurveHandle::UNUSED_SENTINEL` (the existing sentinel used by the legacy path, preserved as-is — the slot allocator never produces this value), that axis stays idle for this segment.

**Engine-level current-segment tracking.** The Engine keeps a single `current_segment_id: u32` field — the id of the segment whose pieces the ISR is currently evaluating across all four axes. It's the same as the existing `Engine::current` mechanism today (which also retains a `Segment` struct with id + handles). On segment arm, the new segment's id replaces the previous one. This id is what gets published on retire — **not** a counter, **not** derived from per-axis state after curves have been cleared. Per-axis `curve_handle` is for evaluation; segment-level id is for retire bookkeeping. They serve different purposes and have different lifetimes.

### 4.4 Piece advancement

`advance_piece_if_needed` (firmware Task 9) gets the cursor-walking logic. Two corrections from earlier drafts:

1. **Iter cap = `MAX_PIECES_PER_CURVE`**, not 4. A sample can legitimately cross many short pieces (corner-rounding splines, host-side step-burst smoothing). The cap of 4 would false-fault on valid curves. The true upper bound on iterations per call is `MAX_PIECES_PER_CURVE` (16) — we can never advance more times than the curve has pieces. Combined with the load-time minimum-duration validation (see §3.2 — pieces with `duration <= 0` are rejected at load), runaway loops are structurally impossible.
2. **Early exhaustion is a fault when another participating axis is still pending.** If X's curve runs out while Y still has pieces, that's a host duration-mismatch bug. Fault is raised here, before retirement bookkeeping runs.

```rust
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    axis_idx: usize,
    engine_current: Option<&ActiveSegment>,
    shared: &SharedState,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u16 = 0;
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
                    // Curve exhausted. Check whether any OTHER participating
                    // axis still has work to do — if so, this is early exhaustion
                    // (host duration-mismatch bug); fault.
                    axis.piece = None;
                    axis.curve_handle = None;
                    if let Some(seg) = engine_current {
                        let other_pending = seg.participating_mask
                            & !(1u8 << axis_idx)
                            & engine_get_pending_mask(seg);
                        if other_pending != 0 {
                            raise_piece_advance_underflow(shared, axis_idx);
                        }
                    }
                    break;
                }
            },
            None => { axis.piece = None; break; }
        }

        iters = iters.saturating_add(1);
        if iters >= MAX_PIECES_PER_CURVE as u16 {
            raise_piece_advance_underflow(shared, axis_idx);
            break;
        }
    }
    advanced
}
```

`engine_get_pending_mask` is the Engine's view of which axes are still mid-segment (see §4.5). The fault fires only when X exhausts AND Y is still pending — symmetric and not biased by per-axis order.

### 4.5 Segment retire — participating_mask + pending_mask

Phase 5 of `runtime_tick_sample` (firmware Task 9 stub) becomes:

**Retire condition.** The current segment is retiring when every participating axis has exhausted its curve. The condition is purely structural — no dependence on `ds_xy_segment` (which is zero for pure-Z moves, retract/prime, intrinsic-E-only segments, etc., and would never gate retire on those).

**Building `participating_mask` (4 bits, A/B/Z/E) at segment-arm:**

| Axis | Bit included if … |
|---|---|
| A (X primary) | `x_handle != UNUSED_SENTINEL` |
| B (Y primary, or CoreXY motor B) | `y_handle != UNUSED_SENTINEL` |
| Z | `z_handle != UNUSED_SENTINEL` |
| E | `e_handle != UNUSED_SENTINEL` **AND** `e_mode == Independent` |

The E axis's participation gates on `e_mode` because:
- **CoupledToXy:** E motion is derived from XY arc length (follower math, §4.6). The intrinsic E curve, if present, may legitimately have a shorter duration than XY — the follower keeps producing E motion past the intrinsic curve's exhaustion. E does NOT gate retire; XY does.
- **Independent:** E has its own duration, independent of XY. E DOES gate retire alongside X/Y/Z.
- **Travel:** No E motion at all. Host should send `e_handle = UNUSED_SENTINEL`. E doesn't participate.

**`pending_mask` (also 4 bits):** initialized equal to `participating_mask` at segment-arm. Each bit clears when its axis's curve exhausts (advancement-loop sets `axis.curve_handle = None`). Retire fires when `pending_mask == 0`.

Both masks live on `Engine::current_segment` (the `ActiveSegment` struct above) alongside `current_segment_id`. `engine_get_pending_mask(seg)` exposes the current value for the per-axis advancement loop's early-exhaustion check (§4.4).

**Early exhaustion → fault.** If any axis whose bit is in `participating_mask` exhausts while OTHER bits in `pending_mask` are still set, `raise_piece_advance_underflow(axis_idx)` fires from the advancement loop. The host shorted the curve relative to its siblings — a duration mismatch that must surface immediately, not silently hold-and-wait. Specifically because participating excludes CoupledToXy-mode E, the E intrinsic curve may legitimately exhaust early in that mode without faulting.

**What gets published on retire.**

1. `shared.retired_through_segment_id.store(self.current_segment_id, Ordering::Release)` — the actual segment id, monotonic. Identical semantics to the existing legacy path at `engine.rs:2266` so the host's `slot_pool::release_through(retired_through)` logic continues to work unchanged.
2. Emit a `SEGMENT_END` trace sample carrying `current_segment_id`. The foreground `reclaim::drain_and_reclaim` already consumes these traces and looks up the 4 handles in the `RetirementTable` (registered by the foreground at `push_segment` time, before the segment entered the SPSC queue — see `reclaim.rs:60-67`). The ISR does **not** carry handles for retire — the foreground table maps id → handles. This is the existing pattern; we preserve it.
3. Reset segment-local accumulators: `ds_xy_segment = 0.0`, plus any future per-segment caches.
4. Stream-state hook (existing): call `crate::stream::check_terminal_on_retire(shared, current_segment_id)`.

After publish, the Engine clears `current_segment_id`, `participating_mask`, `pending_mask` to await the next segment-arm.

**Handle lifecycle.** Per-axis `axis.curve_handle` is set to `None` when the curve exhausts (during piece advancement). This is correct — those handles are no longer needed for ISR evaluation, and the foreground `RetirementTable` (a separate data structure indexed by segment id, not by axis) is the source of truth for which slots get reclaimed. The ISR does not need to remember handles past curve exhaustion; clearing them frees `AxisConfig` state for the next segment's arm.

### 4.6 E-axis follower math — three e_mode cases

The E axis is evaluated in Phase 3 of `runtime_tick_sample`. Its position output depends on the current segment's `e_mode`, NOT solely on whether `axis_e.piece` is `Some`. This is the crucial fix that lets normal extruding XY moves produce E motion even without an intrinsic E curve.

Let `current = Engine::current_segment.as_ref()` (the `ActiveSegment` from §3.3). For each sample:

```rust
fn evaluate_e_axis(
    axis_e: &mut AxisConfig,
    current: Option<&ActiveSegment>,
    ds_xy_this_sample: f32,        // computed from Phase 2 of runtime_tick_sample
    pa_k: f32,                     // pressure-advance coefficient, sign-of-accel-aware
    v_xy_this: f32,                // from Phase 2
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> f32 {
    let Some(seg) = current else { return axis_e.last_p_e; };

    // Intrinsic part: evaluated from axis_e.piece if any, else zero.
    let intrinsic = if let Some(piece) = axis_e.piece {
        let t_local = t_sample_end_global - piece_start_seconds(axis_e, cycles_per_second);
        let (p, _v) = monomial_horner_eval(piece, t_local);
        p
    } else {
        // No intrinsic E curve for this segment (or curve already exhausted).
        // Total E motion comes from the follower term for CoupledToXy; zero for
        // Independent (E truly idle) or Travel.
        0.0
    };

    match seg.e_mode {
        EMode::CoupledToXy => {
            // E = intrinsic + extrusion_ratio * ds_xy_segment + PA correction
            let follower = seg.extrusion_ratio_per_xy_mm * ds_xy_segment_accumulator;
            let pa_correction = pa_k * seg.extrusion_ratio_per_xy_mm * v_xy_this;
            intrinsic + follower + pa_correction
        }
        EMode::Independent => intrinsic,
        EMode::Travel => 0.0,
    }
}
```

**Implication for arm logic.** When `e_mode == CoupledToXy` and `e_handle == UNUSED_SENTINEL`, the segment-arm still leaves `axis_e.piece = None` (no intrinsic), but the Phase-3 evaluator runs the follower math anyway because of `seg.e_mode`. This is the case for a "normal extruding XY move" — host doesn't need to allocate a curve slot just to send a zero intrinsic E polynomial.

**Implication for participating_mask.** Per §4.5, the E bit goes into `participating_mask` only when `e_mode == Independent` (and handle non-sentinel). CoupledToXy follower segments retire on XY exhaustion regardless of when (or whether) the E intrinsic curve runs out. The early-exhaustion fault doesn't fire on E's intrinsic exhaustion in CoupledToXy mode.

**Independent mode with no intrinsic.** If `e_mode == Independent` and `e_handle == UNUSED_SENTINEL`, E truly doesn't move — this is the "Z-only hop" or "purge before retract" pattern. The E bit is NOT in participating_mask (handle is sentinel), so E doesn't gate retire; XY/Z does.

This preserves the existing per-segment `e_mode` + `extrusion_ratio` model — the firmware redesign doesn't break or replace it. The axis-level `extrusion_per_xy_mm` field that firmware Task 6 added is stale; the per-segment field on the existing `runtime_handle_push_segment` FFI is the authoritative source.

---

## 5. Per-stepper bindings — concrete model

### 5.1 What we keep from mainline/legacy

- `command_config_stepper(oid, step_pin, dir_pin, invert_step, step_pulse_ticks)` — allocates `struct stepper` via `oid_alloc`, sets up GPIO. **Unchanged.**
- `runtime_emit_step_pulses(motor_idx, n_steps)` — the C-side GPIO emitter. Reads `runtime_motor_steppers[motor_idx][j].stepper->step_pin / dir_pin`. **Unchanged.**
- The mapping table `runtime_motor_steppers[N_MOTORS][N_PARTNERS]`. **Unchanged in shape**, just populated by a different command.

### 5.2 What changes

`command_config_runtime_stepper` (the per-stepper-per-motor bind command) is deleted. `command_kalico_configure_axis` populates the same `runtime_motor_steppers[][]` table via its sub-message.

**Two-phase commit.** All inputs must be validated and all OID lookups resolved into a fully-built stepper array before any byte of `runtime_motor_steppers[axis_idx]` or any Rust-side `AxisConfig` is mutated. This matches the legacy `command_config_runtime_stepper` pattern at `src/stepper.c:216-220` (range/count checks before write) and prevents the C-side bindings table or the Rust `AxisConfig` from being left in a partially-updated state on failure.

```c
void command_kalico_configure_axis(uint32_t *args) {
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count   = args[4];
    // Klipper's %*s blob format consumes TWO args slots: length FIRST,
    // then an encoded pointer that MUST go through command_decode_ptr.
    // See src/i2ccmds.c (canonical pattern) and src/runtime_tick.c:1411.
    uint16_t blob_len       = args[5];
    const uint8_t *blob     = command_decode_ptr(args[6]);

    // ── Phase 1: validate everything; no mutations.
    if (axis_idx >= RUNTIME_MOTOR_COUNT)
        shutdown("configure_axis axis_idx out of range");
    if (mode > 1)
        shutdown("configure_axis mode invalid");
    if (stepper_count > RUNTIME_MAX_STEPPERS_PER_MOTOR)
        shutdown("configure_axis too many steppers per axis");
    if (blob_len != (uint16_t)stepper_count * 4)
        shutdown("configure_axis blob length mismatch");

    // Resolve every OID, build a local staging array. Reject reserved flag bits.
    // `tmc_cs_oid` is the OID of the existing command_config_spi allocation
    // (i.e., a klipper-protocol OID, NOT a struct spidev_s * pointer). The
    // foreground SPI drain task (`runtime_tick.c`'s spi_drain_event)
    // dispatches through `oid_lookup(tmc_cs_oid, command_config_spi)` +
    // `spidev_transfer(...)`. Storing the OID (a u8, padded into a u32 for
    // ABI clarity) avoids the host/MCU pointer-size mismatch and the
    // confusion of "is this a CS pin or a spidev_s *?".
    struct {
        struct stepper *stepper;
        uint8_t invert_dir;
        uint8_t tmc_cs_oid;          // 0xFF = none
    } staged[RUNTIME_MAX_STEPPERS_PER_MOTOR] = {0};

    for (uint8_t i = 0; i < stepper_count; i++) {
        uint8_t stepper_oid = blob[i*4 + 0];
        uint8_t dir_invert  = blob[i*4 + 1];
        uint8_t tmc_cs_oid  = blob[i*4 + 2];
        uint8_t flags       = blob[i*4 + 3];

        if (flags != 0)
            shutdown("configure_axis reserved stepper flags must be zero");
        if (dir_invert > 1)
            shutdown("configure_axis dir_invert must be 0 or 1");

        struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
        // oid_lookup shuts down on bad oid; reaching here means s is valid.

        if (tmc_cs_oid != 0xFF) {
            // Validate the spi OID is allocated — oid_lookup shuts down on
            // bad oid. Return value not stored (the OID itself is what we
            // carry forward; the lookup is purely a validation step).
            (void)oid_lookup(tmc_cs_oid, command_config_spi);
        }

        staged[i].stepper = s;
        staged[i].invert_dir = dir_invert;
        staged[i].tmc_cs_oid = tmc_cs_oid;
    }

    // ── Phase 2: Rust-side validation (microstep_distance finite/positive,
    // engine not mid-motion for mode change, etc.). Pass tmc_cs as the OID.
    StepperBindingRust bindings[RUNTIME_MAX_STEPPERS_PER_MOTOR];
    for (uint8_t i = 0; i < stepper_count; i++) {
        bindings[i].tmc_cs_oid = staged[i].tmc_cs_oid;
    }
    int32_t rc = kalico_runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits, extrusion_bits,
        stepper_count, bindings);
    if (rc != 0) shutdown("configure_axis rejected by runtime");

    // ── Phase 3: commit. Both sides validated; safe to mutate.
    runtime_motor_stepper_count[axis_idx] = stepper_count;
    for (uint8_t i = 0; i < stepper_count; i++) {
        runtime_motor_steppers[axis_idx][i].stepper = staged[i].stepper;
        runtime_motor_steppers[axis_idx][i].invert_dir = staged[i].invert_dir;
    }
    // Reset the per-axis direction-cache so the first emit forces a dir_pin
    // write regardless of prior state.
    runtime_motor_last_dir[axis_idx] = -1;
}
DECL_COMMAND(command_kalico_configure_axis,
    "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
    " extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s");
```

`StepperBindingRust` is the FFI payload type (just `tmc_cs_handle: u32` for now, room for future flags via the reserved `flags` wire byte).

**Failure handling.** Any check failure between Phase 1 and Phase 3 leaves `runtime_motor_steppers[axis_idx]` and the Rust `AxisConfig` unchanged from their previous state. Klipper's `shutdown(...)` semantics will halt the firmware on a validation failure; on a Rust-side `rc != 0` rejection the same shutdown fires. Either way, no partial state.

**Ordering of Rust-side mutations.** The Rust `configure_axis` engine method itself must also be two-phase internally: validate `microstep_distance > 0 && is_finite()`, `mode ∈ {Pulse, Phase}`, no segment in flight for this axis, and (for Phase mode) every binding has `tmc_cs_handle != 0`. Only after all validations clear, mutate `axis.steppers`, `axis.microstep_distance`, `axis.mode`, etc. — atomic at the engine level. If the host re-configures an axis mid-motion the request is rejected (`ERR_MOTION_IN_PROGRESS`); the C-side commit phase is skipped because Rust returned a non-zero rc.

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
