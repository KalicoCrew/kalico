# Step 7-B: Layer 4 MCU evaluator and step output — design spec

Build-order step: **7-B** (Layer 4 for first-print MVP).

## Scope

Per-axis scalar NURBS evaluation at 40 kHz, E-follows-XY integration,
multi-step burst output, and generic axis configuration. This is the MCU
runtime that consumes Layer 3's shaped trajectory and produces physical
stepper motion.

### In scope

- Curve pool refactor: scalar slots replacing 3D vector slots, sized for
  degree-9 post-shape NURBS with up to 80 control points.
- Segment struct: 4 per-axis curve handles (X, Y, Z, E) plus E-mode and
  extrusion ratio metadata.
- Engine evaluator rewrite: per-axis scalar de Boor evaluation,
  CoupledToXy E integration via v_xy finite differences, Independent E
  NURBS evaluation, Travel E hold.
- Accumulator-based multi-step burst generation (up to ~4 steps/tick at
  peak speed), with AWD dual-GPIO support.
- Generic MCU axis configuration: each MCU is told which axes it owns at
  init time, evaluates only those axes.
- Wire protocol: per-axis `kalico_load_curve` (scalar), extended
  `kalico_push_segment` with 4 handles + E-mode fields.
- Trace struct: 4-motor sample (A, B, Z, E).
- Safety gate: `homed` flag in SharedState, engine refuses to run until
  set by host.

### Out of scope

- Homing/endstop implementation (7-D).
- Phase stepping current synthesis (Step 10).
- F4x Z-axis firmware (7-D).
- Corner-blend finalization (Step 8).
- Tanh nonlinear PA (Step 9).
- Host-side routing logic for axis→MCU assignment (7-C).

## Design decisions

### Per-axis scalar curve-pool slots (Approach A)

Each segment consumes 3–4 curve-pool slots: one per axis. Chosen over
blob-pool (variable-size, fragmentation risk) and interleaved 3D vector
slots (wastes memory when axes differ in CP count).

Rationale:
- Simplest extension of the proven Step-5/6 curve-pool architecture.
- `ScalarNurbsRef::try_from_wire` already exists for zero-copy parse.
- Maps naturally to multi-MCU split: each MCU only stores its own axes.
- Per-slot overhead is modest (684 bytes at max capacity).

### Generic axis ownership

The MCU firmware does not hardcode which axes it owns. A config struct
received from the host at init time specifies the owned axes, kinematics
type, and per-axis stepper parameters. This supports any split: all axes
on one MCU, XY+E on H723 with Z on F4x, E on a separate toolhead MCU,
or future EtherCAT servos for XY with stepper E.

### E on a separate MCU

When E and XY share an MCU, the host sends `CoupledToXy` metadata and
the MCU computes E from v_xy at sample rate (saves a curve slot and a de
Boor eval per tick). When E is on a different MCU, the host pre-bakes
E(t) as a scalar NURBS by numerically integrating
`extrusion_ratio * integral(|v_xy|) dt` from the shaped XY trajectory,
then sends it to the E MCU as `Independent`. The E MCU evaluates E(t)
directly without needing XY data.

The MCU firmware handles both modes; the host picks based on config.
This maps cleanly to the EtherCAT future: servos own XY, stepper MCU
gets pre-baked E(t).

Note: Layer 3 (`trajectory::shape_batch`) currently produces
`e_independent` only for `EMode::Independent` segments. The host-side
conversion of `CoupledToXy` segments to pre-baked E(t) NURBS for a
separate E MCU is a **7-C** responsibility (host routing logic). 7-B
provides the MCU-side `Independent` eval path that 7-C will target.

### CoreXY constraint

X and Y must be on the same MCU. The kinematic transform
`(x, y) -> (a, b)` needs both inputs at the same tick.

### Homing deferred to 7-D

Homing is hardware-coupled and hard to test without real endstops. 7-B
provides only a `homed: AtomicBool` safety gate in SharedState. The host
sets it after completing the homing sequence. 7-B tests set it directly.

### Hybrid stepping only (phase stepping deferred to Step 10)

All axes use accumulator-based position-to-step conversion. Phase
stepping (40 kHz current synthesis for 5160 drivers) arrives in Step 10.

## Curve pool

### Constants

| Constant | Step 5/6 | Step 7-B | Rationale |
|----------|----------|----------|-----------|
| `MAX_DEGREE` | 3 | 10 | Degree-9 post-convolution + 1 margin |
| `MAX_CONTROL_POINTS` | 8 | 80 | 64 observed at production frequencies + 25% margin |
| `MAX_KNOT_VECTOR_LEN` | 12 | 91 | `MAX_CONTROL_POINTS + MAX_DEGREE + 1` |
| `MAX_DIM` | 3 | 1 | Scalar (per-axis), not vector |
| `CURVE_POOL_N` | 16 | 64 | 8 segments * 4 axes (worst case with Independent E) * 2 pipeline headroom |

Observed post-shape sizes (50 mm straight line, smooth_zv @ 180 Hz X,
smooth_mzv @ 120 Hz Y, N=25 grid, 0.5 mm fit tolerance):
- X: degree 9, 64 CPs, 74 knots
- Y: degree 9, 64 CPs, 74 knots
- Z (passthrough): degree 4, 13 CPs, 18 knots

### LoadedScalarCurve

```rust
pub struct LoadedScalarCurve {
    pub control_points: [f32; MAX_CONTROL_POINTS],  // 80 * 4 = 320 B
    pub knots: [f32; MAX_KNOT_VECTOR_LEN],          // 91 * 4 = 364 B
    pub n_cp: u8,
    pub n_knots: u8,
    pub degree: u8,
}
// Per-slot: ~688 bytes. Total pool: 64 * 688 = ~43 KB.
```

Weights array dropped — all post-shape NURBS are polynomial (no rational
NURBS in the live pipeline). The `try_alloc_and_load` API validates
`degree <= MAX_DEGREE` and `n_cp <= MAX_CONTROL_POINTS` on load;
oversized curves are rejected with `CurvePoolError::DegreeTooHigh` /
`CurvePoolError::InvalidLengths`.

Existing `CurveHandle`, generation-based ABA guard, and retire API
unchanged.

## Segment struct

```rust
#[repr(C)]
pub struct Segment {
    pub id: u32,
    pub x_handle: CurveHandle,
    pub y_handle: CurveHandle,
    pub z_handle: CurveHandle,
    pub e_handle: CurveHandle,       // sentinel when e_mode != Independent
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: KinematicTag,
    pub e_mode: EMode,
    pub extrusion_ratio: f32,        // mm E per mm XY arc-length
    pub flags: u8,
    pub _pad: [u8; 2],
}
// ~56 bytes (up from 32). Queue holds Q_N=8 segments.

#[repr(u8)]
pub enum EMode {
    CoupledToXy = 0,
    Independent = 1,
    Travel = 2,
}
```

Handles for axes the MCU does not own carry
`CurveHandle::UNUSED_SENTINEL` (a new sentinel distinct from
`HOLD_SEGMENT_SENTINEL`). The evaluator skips these.

### Multi-handle retirement

The trace sample carries only one handle (X, for diagnostics). The
foreground maintains a `segment_id → [CurveHandle; 4]` lookup table,
populated when the producer pushes each segment. On SEGMENT_END (keyed
by `segment_id` from the trace), the foreground iterates all 4 handles
from the table and calls `confirm_retired` on each non-sentinel handle.
Table size is bounded by queue depth + pipeline headroom (~16 entries).

This avoids expanding the trace sample or emitting multiple trace events
per segment retirement.

## Engine evaluator

### Per-tick pipeline (40 kHz ISR)

1. **Force-idle check** — unchanged from Step 6.
2. **Clock widen** — unchanged.
3. **Segment activation / boundary loop** — same structure, new segment
   carries 4 handles.
4. **Resolve owned axis handles** — iterate `mcu_config.axes`; for each
   owned axis, `pool.resolve(handle)`. Skip sentinel handles. Any
   resolution failure latches a fault.
5. **Compute parameter** — `u = (t_segment as f32) / (duration as f32)`,
   clamped to [0, 1].
6. **Eval per-axis scalar NURBS** — `nurbs::eval::eval(view, u)` via
   `ScalarNurbsRef` for each owned axis. Three 1D de Boor evaluations
   (degree 9, ~80 CPs) on M7 at f32.
7. **E-mode dispatch:**
   - `CoupledToXy`: `v_xy = sqrt((x - prev_x)^2 + (y - prev_y)^2) / dt`;
     `e_accumulator += extrusion_ratio * v_xy * dt`. Engine stores
     `prev_x`, `prev_y` (persist across segment boundaries).
   - `Independent`: resolve `e_handle`, eval E NURBS at `u`. On
     segment completion, sync `e_accumulator` to the E NURBS endpoint
     so the next CoupledToXy segment resumes from the correct position.
   - `Travel`: E position unchanged; `e_accumulator` persists.
   - **Stream start**: `e_accumulator` initialized to 0.0. On the
     first tick of a stream (or after flush/force-idle), `prev_x` and
     `prev_y` are seeded by evaluating the first segment's X(t) and
     Y(t) NURBS at u=0, so the first finite-difference delta is zero
     and no spurious E extrusion occurs. The engine tracks a
     `needs_xy_seed: bool` flag, set on stream arm and cleared after
     the first eval.
   - **After force-idle / flush**: `e_accumulator` reset to 0.0,
     `needs_xy_seed` set to true. The host re-seeds on the next
     stream arm.
8. **NaN/Inf check** — all axis positions.
9. **Kinematic transform** — dispatch on `kinematics` tag:
   - `CoreXyAndE`: `(x, y, e) -> (a, b, e)` where `a = x+y`, `b = x-y`.
   - `CartesianXyzAndE`: identity.
10. **Step generation** — per owned axis (see below).
11. **Trace emit** — 4-motor sample `(motor_a, motor_b, motor_z, motor_e)`.
12. **Tick counter / status update** — unchanged.

### Engine struct additions

```rust
prev_x: f32,
prev_y: f32,
e_accumulator: f32,
step_state: [StepMotorState; 4],   // per motor (post-kinematic-transform)
mcu_config: McuAxisConfig,         // set at init, immutable during printing
```

### Cycle budget estimate (M7 @ 520 MHz, 25 us tick)

| Operation | Estimated cycles | Notes |
|-----------|-----------------|-------|
| Clock widen + segment logic | ~100 | Unchanged from Step 6 |
| 3x scalar de Boor (degree 9) | ~3000 | ~1000 per axis (degree-9 triangle) |
| E integration (sqrt + mul) | ~50 | VSQRT.F32 ~14 cycles + arithmetic |
| Kinematic transform | ~20 | 2 adds |
| 3x step generation | ~200 | ~4 steps * 3 axes * ~15 cycles each |
| Trace emit | ~50 | Ring buffer enqueue |
| **Total** | **~3400** | **~6.5 us of 25 us budget (26%)** |

Comfortable headroom for Step 10 phase-stepping addition.

## Step generation

### Accumulator-based multi-step burst

Per motor, persistent state:

```rust
pub struct StepMotorState {
    step_accumulator: f64,   // f64 for drift prevention (H723 has hardware DP-FPU)
    steps_per_mm: f32,
    is_awd: bool,
    // GPIO pin references resolved at init
}
```

Per-tick logic:

```
new_position_steps = motor_position_mm * steps_per_mm
delta = new_position_steps - step_accumulator
n_steps = delta.trunc() as i32
step_accumulator += n_steps as f64
// Set direction pin based on sign(n_steps)
// Emit |n_steps| step pulses
```

At peak speed (1000 mm/s, 160 microsteps/mm, 40 kHz):
`160000 / 40000 = 4 steps per tick`. Each pulse: set BSRR, ~100 ns
delay (NOPs), clear BSRR. For AWD axes, both step pins toggle via a
single BSRR write (same GPIO port).

TMC5160 timing constraints: minimum step pulse high time 100 ns,
minimum step pulse low time 100 ns, direction setup time 20 ns. At 4
steps per tick, total pulse time is ~800 ns — well within the 25 µs
window. For AWD, both step pins must be on the same GPIO port to allow
single-BSRR-write toggling; if pins span ports, use two sequential
writes (adds ~10 ns, negligible).

## MCU axis configuration

```rust
pub struct McuAxisConfig {
    /// Per-motor config, indexed in motor space (post-kinematic-transform):
    /// CoreXyAndE: [A=0, B=1, Z=2, E=3]; CartesianXyzAndE: [X=0, Y=1, Z=2, E=3].
    pub motors: [Option<MotorConfig>; 4],
    pub kinematics: KinematicTag,
}

pub struct MotorConfig {
    pub steps_per_mm: f32,
    pub is_awd: bool,
    pub invert_dir: bool,
}
```

Sent from host to MCU once at init via a `kalico_configure_axes`
command. Immutable during printing. The evaluator loop iterates only over
`Some` entries — an MCU that owns only Z evaluates one de Boor per tick.

Note: step generation operates on motor-space positions (after the
kinematic transform), so `steps_per_mm`, GPIO pins, AWD, and direction
inversion are all per-motor properties. For standard CoreXY with equal
belts/pulleys, `steps_per_mm` is numerically identical for A and B.

## Wire protocol changes

### Curve loading

The Klipper wire schema string (`cps=%*s knots=%*s weights=%*s`) is
blob-based and dimension-agnostic, so the wire format itself does not
change. However, the C command handler (`kalico_load_curve` in
`runtime_tick.c`) currently hardcodes 3D control points (`cps_len % 12`,
scratch buffers `[8 * 3]`) and the Rust FFI (`runtime_ffi.rs`)
constructs slices with `n_cp * MAX_DIM`. Both must be rewritten to pass
raw blobs through to `ScalarNurbsRef::try_from_wire` (which already
handles scalar NURBS with variable degree/CP count). Scratch buffers
must be resized or eliminated in favor of direct blob parsing.

The host calls `kalico_load_curve` once per axis. It sends curves only
for axes this MCU owns.

### Segment push

Extended command signature:

```
kalico_push_segment id={u32}
    x_handle={u32} y_handle={u32} z_handle={u32} e_handle={u32}
    t_start_hi={u32} t_start_lo={u32} t_end_hi={u32} t_end_lo={u32}
    kinematics={u8} e_mode={u8} extrusion_ratio={u32_bits} flags={u8}
```

`extrusion_ratio` transmitted as `f32::to_bits()` / `from_bits()` (the
Klipper wire protocol doesn't natively carry floats).

### New commands

- `kalico_configure_axes axes={blob}` — MCU axis config at init.
- `kalico_set_homed` — host sets the `homed` gate after completing homing
  (7-D scope; command registered in 7-B so the interface exists).

## Trace struct

```rust
#[repr(C)]
pub struct TraceSample {
    pub tick: u64,
    pub motor_a: f32,
    pub motor_b: f32,
    pub motor_z: f32,
    pub motor_e: f32,
    pub segment_id: u32,
    pub curve_handle: CurveHandle,  // primary handle (X) for diagnostics
    pub flags: u8,
    pub _pad: [u8; 7],             // align to 8 bytes (u64 alignment)
}
// Size: 40 bytes (up from 32). repr(C) with u64 field requires 8-byte
// alignment, so the struct rounds to 40 not 36.
```

Total sample grows by 8 bytes; at 40 kHz with ring size 1201, ring
buffer grows by ~9.6 KB. C-side drain buffer and wire send length in
`runtime_tick.c` must be updated to match.

## Testing strategy

All tests run on the host against synthetic NURBS (no hardware). The
existing test infrastructure (engine_tick, stream_lifecycle, etc.)
is extended.

1. **Curve pool**: load/resolve scalar curves at various degrees (1, 4,
   9, 10). Reject oversized curves. Generation-based retirement with
   multi-handle segments.
2. **Evaluator accuracy**: load known scalar NURBS, tick through a
   segment, verify positions match `nurbs::eval::scalar_eval` reference
   at each tick.
3. **E-mode dispatch**: CoupledToXy — verify E accumulates proportional
   to XY arc length. Independent — verify E tracks its own NURBS. Travel
   — E stays constant.
4. **Step generation**: verify step counts match expected position deltas.
   Test reversal (negative steps). Test multi-step burst (> 1 step per
   tick).
5. **AWD**: verify dual-step emission (test via mock GPIO or step-count
   tracking).
6. **Generic axis config**: MCU configured with subset of axes; verify
   unused axes are not evaluated.
7. **Safety gate**: engine refuses to run when `homed = false`.
8. **Wire integration**: round-trip scalar NURBS through
   `try_from_wire`, verify eval matches host-side reference.

## Approach

Bottom-up (Approach 1): curve pool refactor first, then evaluator
rewrite, then E-mode dispatch, then step generation. Each layer is
testable in isolation before wiring to the next.

## Migration notes

Existing compile-time size assertions that must be updated:
- `Segment`: 32-byte assert in `segment.rs` → update to new size (~56 bytes).
- `TraceSample`: 32-byte assert in `trace.rs` → update to new size (+4 bytes for `motor_z`).
- `SharedState`: add `homed: AtomicBool` field; update any size asserts.
- C-side `runtime_tick.c`: scratch buffers, `DECL_COMMAND` signatures,
  and blob parsing must be rewritten for scalar curves with higher
  degree/CP limits.
- Rust FFI `runtime_ffi.rs`: `MAX_DIM`-based slice construction replaced
  with direct blob passthrough to `ScalarNurbsRef::try_from_wire`.
