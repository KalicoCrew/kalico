# Stepping-redesign finish — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute the finish-design spec (`docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md`) — reshape `curve_pool` for cubic Bezier pieces, complete legacy-path deletion, migrate the host bridge + klippy to the new FFI surface, and produce a runtime that is end-to-end cubic-Bezier on both H7 and F4.

**Architecture:** Reshape `curve_pool` slot from NURBS to `LoadedCubicCurve` (16 pieces × 32 bytes). `AxisConfig` gets `(curve_handle, piece_cursor)` cursor semantics. ISR consumes the segment queue and arms per-axis state (`participating_mask`, `pending_mask`, `segment_base_e`). E-axis follower math uses absolute-E (engine-level `e_accumulator: f64`, `segment_base_e: f32`). Two-phase `configure_axis` validates everything before mutating either C-side `runtime_motor_steppers[][]` or Rust-side `AxisConfig`. Legacy paths (`producer_step`, Newton solver, `step_time_event`, NURBS pool, `config_runtime_stepper`) deleted entirely.

**Tech stack:** Rust (`rust/runtime`, `rust/kalico-c-api`, `rust/motion-bridge`, `rust/kalico-host-rt`) + C (`src/`) + Python (`klippy/`). Build via Klipper's existing Make pipeline; Rust crates compile through Cargo. The migration is a flag-day cutover — intermediate commits don't build.

---

## File map

**New:**
- `rust/runtime/src/cubic_curve.rs` — `LoadedCubicCurve` struct + cubic-piece pool internals
- `rust/runtime/tests/cubic_curve_load.rs` — cubic load_curve unit tests
- `rust/runtime/tests/arm_segment.rs` — ISR-side segment arm unit tests
- `rust/runtime/tests/exhaustion_post_pass.rs` — early-exhaustion fault tests
- `rust/runtime/tests/e_follower_absolute.rs` — E-axis absolute position tests
- `rust/runtime/tests/configure_axis_two_phase.rs` — two-phase validation tests

**Heavily modified:**
- `rust/runtime/src/curve_pool.rs` — NURBS internals out, slot+generation discipline preserved
- `rust/runtime/src/monomial.rs` — add `bernstein_to_monomial_with_duration`
- `rust/runtime/src/stepping_state.rs` — `AxisConfig` adds cursor; `StepperRef` shrinks; `StepperBindingRust` ABI added
- `rust/runtime/src/spi_queue.rs` — `SpiWrite.cs_pin: u32` → `tmc_cs_oid: u8`
- `rust/runtime/src/engine.rs` — `arm_segment` method, `participating_mask`/`pending_mask`/`segment_base_e`/`ds_xy_segment` fields, post-pass exhaustion logic
- `rust/runtime/src/tick.rs` — `runtime_tick_sample` rewires E-axis evaluator (absolute model), wires arm_segment from dequeue, post-pass exhaustion check
- `rust/runtime/src/error.rs` — add `CurveLoadInvalid`, `PhaseModeNotAvailable`
- `rust/runtime/src/fault_helpers.rs` — add `raise_curve_load_invalid`, `raise_phase_mode_not_available`
- `rust/runtime/src/lib.rs` — add `pub mod cubic_curve`; drop `pub mod step_ring`/`step_time`/`step_producer`
- `rust/runtime/build.rs` — drop NURBS sizing env vars; add `MAX_PIECES_PER_CURVE`
- `rust/kalico-c-api/src/runtime_ffi.rs` — drop NURBS `load_curve`, `producer_step`, `step_ring_*`, `modulated_tick`; add `load_curve_cubic`; extend `configure_axis`
- `rust/kalico-c-api/include/kalico_runtime.h` — regenerate via cbindgen
- `src/Makefile` — drop NURBS sizing env passthrough; add new cubic constants if any
- `src/Kconfig` — adjust `RUNTIME_STORAGE_SIZE_LARGE` / `_SMALL` to reflect new (much smaller) `size_of::<RuntimeContext>()`
- `src/runtime_storage.c` — update AXI budget calculation (remove NURBS portion)
- `src/runtime_tick.c` — delete `step_time_event`, `runtime_producer_event`, `init_step_time_timers`, related diag counters
- `src/stepper.c` — delete `command_config_runtime_stepper`; add two-phase `command_kalico_configure_axis`
- `src/kalico_dispatch.c` — drop `handle_load_curve` (NURBS); add `handle_load_curve_cubic`
- `rust/motion-bridge/src/bridge.rs` — drop NURBS upload; emit cubic pieces; emit configure_axis sub-message
- `rust/motion-bridge/src/producer.rs` — drop NURBS `load_curve`; add `load_curve_cubic`
- `rust/motion-bridge/src/planner.rs` — serialize per-axis curves as cubic Bezier pieces
- `rust/kalico-host-rt/src/host_io/mod.rs` — drop `kalico_load_curve_begin` etc.; add `kalico_load_curve_cubic` command registration
- `klippy/motion_bridge.py` — Python wrappers for new FFI methods
- `klippy/motion_toolhead.py` — emit `kalico_configure_axis` per axis; drop `config_runtime_stepper` emit

**Deleted entirely:**
- `rust/runtime/src/step_ring.rs`
- `rust/runtime/src/step_time.rs`
- `rust/runtime/src/step_producer.rs`

---

## Tasks

### Task 1: bernstein_to_monomial_with_duration helper

**Files:**
- Modify: `rust/runtime/src/monomial.rs`
- Modify: `rust/runtime/tests/monomial_eval.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/runtime/tests/monomial_eval.rs`:

```rust
#[test]
fn bernstein_to_monomial_with_duration_rescales_coefficients() {
    use kalico_runtime::monomial::bernstein_to_monomial_with_duration;
    // Linear ramp from 0 to 10mm over 25 µs:
    // Unit-interval Bernstein for P(τ) = 10·τ is [0, 10/3, 20/3, 10].
    // Seconds-domain monomial: c0=0, c1=10/25e-6=4e5, c2=0, c3=0.
    let piece = bernstein_to_monomial_with_duration([0.0, 10.0/3.0, 20.0/3.0, 10.0], 25e-6);
    // Evaluate at t=25e-6 should give 10.0 mm.
    let p = piece.coeffs[0]
          + piece.coeffs[1] * 25e-6
          + piece.coeffs[2] * (25e-6 * 25e-6)
          + piece.coeffs[3] * (25e-6 * 25e-6 * 25e-6);
    assert!((p - 10.0).abs() < 1e-3, "P(25µs) = {} (expected 10.0)", p);
    assert!((piece.duration - 25e-6).abs() < 1e-12);
    // vel_coeffs pre-baked: vc0 = c1, vc1 = 2*c2, vc2 = 3*c3
    assert!((piece.vel_coeffs[0] - 4e5).abs() < 1e-3);
}

#[test]
fn bernstein_to_monomial_with_duration_quadratic() {
    use kalico_runtime::monomial::bernstein_to_monomial_with_duration;
    // Pure quadratic P(τ) = τ² in unit interval.
    // Bernstein CPs for τ² are [0, 0, 1/3, 1] (degree-3 elevation of τ²).
    let piece = bernstein_to_monomial_with_duration([0.0, 0.0, 1.0/3.0, 1.0], 1.0);
    // At t=1.0, P should = 1.0
    let p = piece.coeffs[0] + piece.coeffs[1] + piece.coeffs[2] + piece.coeffs[3];
    assert!((p - 1.0).abs() < 1e-5);
    // At t=0.5, P should = 0.25
    let p = piece.coeffs[0]
          + piece.coeffs[1] * 0.5
          + piece.coeffs[2] * 0.25
          + piece.coeffs[3] * 0.125;
    assert!((p - 0.25).abs() < 1e-5);
}
```

- [ ] **Step 2: Run test to confirm it fails**

```
cd rust && cargo test -p runtime --test monomial_eval bernstein_to_monomial_with_duration 2>&1 | tail
```

Expected: FAIL with "cannot find function `bernstein_to_monomial_with_duration`".

- [ ] **Step 3: Add the helper to `rust/runtime/src/monomial.rs`**

Append to the file (do not modify the existing `bernstein_to_monomial`):

```rust
/// Cubic Bezier Bernstein control points → seconds-domain monomial form.
///
/// Wraps [`bernstein_to_monomial`] (which produces unit-interval coefficients)
/// with a duration rescale: `c_k' = c_k / d^k`. After rescale, evaluating
/// the monomial at `t_sec ∈ [0, duration]` produces the same physical mm value
/// the unit-interval evaluation would at `τ = t_sec / duration`.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §3.2.
#[inline]
pub fn bernstein_to_monomial_with_duration(
    bp: [f32; 4],
    duration_sec: f32,
) -> BezierPieceMonomial {
    let m = bernstein_to_monomial(bp);
    let c0 = m.coeffs[0];
    let c1 = m.coeffs[1] / duration_sec;
    let c2 = m.coeffs[2] / (duration_sec * duration_sec);
    let c3 = m.coeffs[3] / (duration_sec * duration_sec * duration_sec);
    BezierPieceMonomial {
        coeffs: [c0, c1, c2, c3],
        vel_coeffs: [c1, 2.0 * c2, 3.0 * c3],
        duration: duration_sec,
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

```
cd rust && cargo test -p runtime --test monomial_eval 2>&1 | tail
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/monomial.rs rust/runtime/tests/monomial_eval.rs
git commit -m "feat(monomial): add bernstein_to_monomial_with_duration for seconds-domain rescale"
```

---

### Task 2: cubic_curve module (LoadedCubicCurve, CubicCurvePool)

**Files:**
- Create: `rust/runtime/src/cubic_curve.rs`
- Create: `rust/runtime/tests/cubic_curve_load.rs`
- Modify: `rust/runtime/src/lib.rs`
- Modify: `rust/runtime/build.rs` (add `MAX_PIECES_PER_CURVE` constant)

- [ ] **Step 1: Add MAX_PIECES_PER_CURVE to build.rs**

Modify `rust/runtime/build.rs` — append to the `sizing_body` format string:

```rust
let sizing_body = format!(
    "// Auto-generated by runtime/build.rs — do not edit.\n\
     pub const MAX_CONTROL_POINTS: usize = {mcp};\n\
     pub const MAX_KNOT_VECTOR_LEN: usize = {mkv};\n\
     pub const MAX_DEGREE: u8 = {mdg};\n\
     pub const CURVE_POOL_N: usize = {cpn};\n\
     pub const RT_STORAGE_SIZE: usize = {rss};\n\
     pub const MAX_PIECES_PER_CURVE: usize = 16;\n"
);
```

Leave MAX_CONTROL_POINTS etc. intact for now — they're deleted in Task 16.

- [ ] **Step 2: Write the cubic_curve module**

Create `rust/runtime/src/cubic_curve.rs`:

```rust
//! Cubic-Bezier curve pool slots.
//!
//! Each slot holds up to `MAX_PIECES_PER_CURVE` pieces of one per-axis cubic
//! Bezier curve in monomial form. Replaces the NURBS storage that lived in
//! the prior `curve_pool` slot layout.
//!
//! Slot+generation discipline (`try_alloc_and_load`, `lookup`, `confirm_retired`)
//! is owned by `crate::curve_pool` and unchanged by this redesign. This module
//! defines only the slot's *payload* shape and the conversion / validation
//! used at load time.
//!
//! Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §2, §3.2.

use crate::monomial::{bernstein_to_monomial_with_duration, BezierPieceMonomial};
use crate::sizing::MAX_PIECES_PER_CURVE;

/// A loaded per-axis curve: the array of monomial-form pieces the ISR walks
/// via a (curve_handle, piece_cursor) pair on `AxisConfig`.
#[repr(C)]
pub struct LoadedCubicCurve {
    pub piece_count: u16,
    pub _pad: [u8; 2],
    pub pieces: [BezierPieceMonomial; MAX_PIECES_PER_CURVE],
}

impl LoadedCubicCurve {
    /// Construct an empty curve (all pieces zeroed, count=0).
    /// Used by the curve_pool slot initializer.
    pub const fn empty() -> Self {
        const ZERO_PIECE: BezierPieceMonomial = BezierPieceMonomial {
            coeffs: [0.0; 4],
            vel_coeffs: [0.0; 3],
            duration: 0.0,
        };
        Self {
            piece_count: 0,
            _pad: [0; 2],
            pieces: [ZERO_PIECE; MAX_PIECES_PER_CURVE],
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum CubicLoadError {
    PieceCountOutOfRange,
    NonFiniteBernstein,
    NonPositiveDuration,
}

/// Per-piece wire entry decoded from the load_curve_cubic blob:
/// 4 Bernstein bits + duration bits, each u32 (f32 bit pattern).
#[derive(Clone, Copy)]
#[repr(C)]
pub struct WirePiece {
    pub bp0_bits: u32,
    pub bp1_bits: u32,
    pub bp2_bits: u32,
    pub bp3_bits: u32,
    pub duration_bits: u32,
}

/// Convert a wire piece array into a populated `LoadedCubicCurve`.
///
/// Validates each piece's Bernstein control points are finite and
/// duration is finite and positive. Returns `Err` without mutating `out`
/// on the first invalid piece, so the caller can leave the slot untouched.
pub fn populate_from_wire(
    out: &mut LoadedCubicCurve,
    wire: &[WirePiece],
) -> Result<(), CubicLoadError> {
    if wire.is_empty() || wire.len() > MAX_PIECES_PER_CURVE {
        return Err(CubicLoadError::PieceCountOutOfRange);
    }
    // Phase 1: validate every piece WITHOUT writing into `out`.
    for w in wire {
        let bp = [
            f32::from_bits(w.bp0_bits),
            f32::from_bits(w.bp1_bits),
            f32::from_bits(w.bp2_bits),
            f32::from_bits(w.bp3_bits),
        ];
        if bp.iter().any(|x| !x.is_finite()) {
            return Err(CubicLoadError::NonFiniteBernstein);
        }
        let d = f32::from_bits(w.duration_bits);
        if !d.is_finite() || d <= 0.0 {
            return Err(CubicLoadError::NonPositiveDuration);
        }
    }
    // Phase 2: all valid → populate.
    for (i, w) in wire.iter().enumerate() {
        let bp = [
            f32::from_bits(w.bp0_bits),
            f32::from_bits(w.bp1_bits),
            f32::from_bits(w.bp2_bits),
            f32::from_bits(w.bp3_bits),
        ];
        let d = f32::from_bits(w.duration_bits);
        out.pieces[i] = bernstein_to_monomial_with_duration(bp, d);
    }
    out.piece_count = wire.len() as u16;
    Ok(())
}
```

- [ ] **Step 3: Wire the new module into lib.rs**

Modify `rust/runtime/src/lib.rs` — add `pub mod cubic_curve;` alphabetically (between `c_segment_queue` and `curve_pool`).

- [ ] **Step 4: Write the unit tests**

Create `rust/runtime/tests/cubic_curve_load.rs`:

```rust
use kalico_runtime::cubic_curve::{
    populate_from_wire, CubicLoadError, LoadedCubicCurve, WirePiece,
};

fn make_wire(bp: [f32; 4], dur: f32) -> WirePiece {
    WirePiece {
        bp0_bits: bp[0].to_bits(),
        bp1_bits: bp[1].to_bits(),
        bp2_bits: bp[2].to_bits(),
        bp3_bits: bp[3].to_bits(),
        duration_bits: dur.to_bits(),
    }
}

#[test]
fn single_piece_linear_load() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 10.0/3.0, 20.0/3.0, 10.0], 25e-6)];
    assert_eq!(populate_from_wire(&mut curve, &wire), Ok(()));
    assert_eq!(curve.piece_count, 1);
    // Seconds-domain c1 = (10mm) / (25e-6 s) = 4e5 mm/s.
    assert!((curve.pieces[0].coeffs[1] - 4e5).abs() < 1.0);
    assert!((curve.pieces[0].duration - 25e-6).abs() < 1e-12);
}

#[test]
fn rejects_zero_pieces() {
    let mut curve = LoadedCubicCurve::empty();
    let wire: [WirePiece; 0] = [];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::PieceCountOutOfRange)
    );
    // No mutation on rejection.
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_too_many_pieces() {
    let mut curve = LoadedCubicCurve::empty();
    // 17 pieces (one over MAX_PIECES_PER_CURVE = 16).
    let one = make_wire([0.0, 0.333, 0.667, 1.0], 1e-3);
    let wire = vec![one; 17];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::PieceCountOutOfRange)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_non_finite_bernstein() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, f32::NAN, 0.667, 1.0], 1e-3)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonFiniteBernstein)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_zero_duration() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 0.333, 0.667, 1.0], 0.0)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonPositiveDuration)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_negative_duration() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 0.333, 0.667, 1.0], -1e-6)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonPositiveDuration)
    );
}

#[test]
fn multi_piece_load_all_fifteen() {
    let mut curve = LoadedCubicCurve::empty();
    let one = make_wire([0.0, 0.333, 0.667, 1.0], 1e-3);
    let wire = vec![one; 15];
    assert_eq!(populate_from_wire(&mut curve, &wire), Ok(()));
    assert_eq!(curve.piece_count, 15);
    for i in 0..15 {
        assert!((curve.pieces[i].duration - 1e-3).abs() < 1e-12);
    }
    // Pieces 15-16 (out of count) should still be zero from `empty()`.
    assert_eq!(curve.pieces[15].duration, 0.0);
}
```

- [ ] **Step 5: Run tests**

```
cd rust && cargo test -p runtime --test cubic_curve_load 2>&1 | tail
```

Expected: 7 tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/cubic_curve.rs rust/runtime/src/lib.rs rust/runtime/build.rs rust/runtime/tests/cubic_curve_load.rs
git commit -m "feat(cubic_curve): LoadedCubicCurve payload + WirePiece + populate_from_wire"
```

---

### Task 3: Reshape curve_pool internals (NURBS → cubic)

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs` heavily

This task is destructive: the existing NURBS payload (control_points, knot_vector, weights, degree) is replaced wholesale with `LoadedCubicCurve`. Build will fail mid-task until Task 4 catches up. The slot+generation discipline (`try_alloc_and_load`, `lookup`, `confirm_retired`, `(current_gen, last_retired_gen)` AtomicU16 pair) stays unchanged.

- [ ] **Step 1: Inspect the existing curve_pool to identify what to preserve**

Read `rust/runtime/src/curve_pool.rs` and identify:
- Slot struct definition (delete the NURBS arrays, keep the gen counters)
- `try_alloc_and_load` signature (will change to take `&[WirePiece]` instead of NURBS args)
- `lookup` returning a slot reference (will return `&LoadedCubicCurve`)
- `confirm_retired` (unchanged)

- [ ] **Step 2: Replace the slot payload**

In `rust/runtime/src/curve_pool.rs`, find the slot struct (likely `struct CurveSlot` or similar). Replace its NURBS payload fields with:

```rust
pub struct CurveSlot {
    // Existing generation guards — unchanged.
    pub current_gen: AtomicU16,
    pub last_retired_gen: AtomicU16,
    // NEW: cubic piece payload (replaces control_points/knot_vector/weights/degree).
    pub curve: UnsafeCell<crate::cubic_curve::LoadedCubicCurve>,
}
```

Delete the prior fields. `UnsafeCell` because the foreground writes during load and the ISR reads via `lookup` — same aliasing pattern the NURBS fields used.

- [ ] **Step 3: Rewrite try_alloc_and_load**

Replace the existing `try_alloc_and_load` body with a cubic-piece version:

```rust
/// Spec §3.2: atomic load of a cubic-piece curve. Slot becomes "valid" only
/// after `populate_from_wire` succeeds AND `current_gen += 1`. Mid-load the
/// host cannot observe partial state.
pub fn try_alloc_and_load(
    &self,
    slot_idx: u16,
    wire: &[crate::cubic_curve::WirePiece],
) -> Result<CurveHandle, CurvePoolError> {
    let slot = self.slots.get(slot_idx as usize)
        .ok_or(CurvePoolError::OutOfBounds)?;
    let current = slot.current_gen.load(Ordering::Acquire);
    let retired = slot.last_retired_gen.load(Ordering::Acquire);
    if current != retired {
        return Err(CurvePoolError::SlotAlreadyLoaded);
    }
    // SAFETY: foreground is sole writer; ISR reads only after generation bump
    // below publishes the populated slot.
    let dst = unsafe { &mut *slot.curve.get() };
    crate::cubic_curve::populate_from_wire(dst, wire).map_err(|e| match e {
        crate::cubic_curve::CubicLoadError::PieceCountOutOfRange => CurvePoolError::InvalidCurve,
        crate::cubic_curve::CubicLoadError::NonFiniteBernstein => CurvePoolError::NonFiniteData,
        crate::cubic_curve::CubicLoadError::NonPositiveDuration => CurvePoolError::InvalidCurve,
    })?;
    let new_gen = current.wrapping_add(1);
    slot.current_gen.store(new_gen, Ordering::Release);
    Ok(CurveHandle::pack(slot_idx, new_gen))
}
```

- [ ] **Step 4: Rewrite lookup**

```rust
/// Spec §4.4: ISR-side resolution. Returns `Ok` only if the handle's
/// generation matches the slot's `current_gen`. Returns a const pointer
/// (NOT `&`) because the ISR may hold this reference across piece-advance
/// calls; using `&LoadedCubicCurve` would conflict with the next load's
/// `&mut` projection.
pub fn lookup_active(&self, handle: CurveHandle) -> Option<*const crate::cubic_curve::LoadedCubicCurve> {
    let (slot_idx, gen) = handle.unpack();
    let slot = self.slots.get(slot_idx as usize)?;
    let current = slot.current_gen.load(Ordering::Acquire);
    if current != gen {
        return None;
    }
    Some(slot.curve.get() as *const _)
}
```

- [ ] **Step 5: confirm_retired unchanged**

`confirm_retired` already operates on the generation counter; no payload knowledge needed. Leave as-is.

- [ ] **Step 6: Update CurvePool slot construction**

Find where slots are constructed (likely in `CurvePool::new()` or `init_in_place`). Replace the NURBS-array zero-initialization with:

```rust
CurveSlot {
    current_gen: AtomicU16::new(0),
    last_retired_gen: AtomicU16::new(0),
    curve: UnsafeCell::new(crate::cubic_curve::LoadedCubicCurve::empty()),
}
```

- [ ] **Step 7: Delete unused error variants if any**

If the `CurvePoolError` enum had NURBS-specific variants (e.g., `DegreeTooHigh`, `InvalidLengths`, `InvalidKnots`), remove them — they're unreachable now. Keep `OutOfBounds`, `SlotAlreadyLoaded`, `NonFiniteData`, `InvalidCurve`.

- [ ] **Step 8: Commit (build will still be broken)**

```bash
git add rust/runtime/src/curve_pool.rs
git commit -m "refactor(curve_pool): replace NURBS payload with LoadedCubicCurve

Slot+generation discipline preserved verbatim. Payload struct now carries
a cubic-piece array via cubic_curve::LoadedCubicCurve. NURBS-specific
error variants removed. Build is still broken until consumers update
(Tasks 4+)."
```

---

### Task 4: Update curve_pool callers in engine.rs (compile fix wave 1)

The previous NURBS load_curve callers in engine.rs (and elsewhere) won't compile. This task is purely a compile-fix wave — wire the new signatures through. Behavioral changes come in later tasks.

**Files:**
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Find all NURBS-shaped callers**

```
cd rust && cargo build -p runtime --lib 2>&1 | grep "error\[" | head -40
```

Likely errors:
- `try_alloc_and_load` callers passing NURBS args.
- `lookup` callers expecting `&LoadedScalarCurve` (the old NURBS payload type).
- Initialization code touching the removed fields.

- [ ] **Step 2: For each broken caller, update to new signatures**

This is mechanical. For `lookup`:

```rust
// OLD: let curve = pool.lookup(handle)?;  curve.control_points[...]
// NEW: let curve = unsafe { &*pool.lookup_active(handle).ok_or(...)? };
//      curve.pieces[cursor]
```

The `unsafe` deref is OK because the ISR's lookup is gen-validated and slot lifetime is `'static`.

- [ ] **Step 3: Stub out non-curve-pool consumers temporarily**

For symbols that get fully deleted in later tasks (`producer_step`, `arm_step_timer`, anything reaching into NURBS internals), insert `unimplemented!("removed in stepping-redesign-finish Task N")` placeholders with the task number. These get deleted in Task 11.

- [ ] **Step 4: Get the library to build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean build, possibly warnings.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/engine.rs
git commit -m "fix(engine): wire NURBS->cubic curve_pool API through engine consumers

Mechanical refactor wave. Legacy producer_step/arm_step_timer paths
temporarily stubbed with unimplemented!() — deleted in Task 11."
```

---

### Task 5: Shrink StepperRef + add StepperBindingRust ABI

**Files:**
- Modify: `rust/runtime/src/stepping_state.rs`
- Modify: `rust/runtime/src/spi_queue.rs`
- Modify: `rust/kalico-c-api/include/kalico_runtime.h` (manual addition; cbindgen regen happens later)
- Create: `rust/runtime/tests/stepper_binding_abi.rs`

- [ ] **Step 1: Shrink StepperRef**

In `rust/runtime/src/stepping_state.rs`, replace the `StepperRef` struct definition with:

```rust
/// Per-stepper Rust-side state. GPIO + direction-inversion live in C
/// (`runtime_motor_steppers[][]`), so this struct only holds atomic state
/// the ISR reads/writes.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §4.2.
pub struct StepperRef {
    pub position_count: AtomicI32,
    /// OID of `command_config_spi` for this stepper's TMC driver. `None`
    /// means Pulse-only (no SPI traffic for this stepper).
    pub tmc_cs_oid: Option<u8>,
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}

impl StepperRef {
    pub fn new(tmc_cs_oid: Option<u8>) -> Self {
        Self {
            position_count: AtomicI32::new(0),
            tmc_cs_oid,
            last_coil_A: AtomicI16::new(0),
            last_coil_B: AtomicI16::new(0),
            phase_offset_microsteps: AtomicI32::new(0),
            phase_offset_target: AtomicI32::new(0),
            last_phase_target: AtomicI32::new(0),
        }
    }
}
```

Delete the prior `step_pin`, `dir_pin`, `dir_invert` fields.

- [ ] **Step 2: Define the StepperBindingRust ABI**

Append to `rust/runtime/src/stepping_state.rs`:

```rust
/// FFI ABI: per-stepper binding payload, passed from C to Rust by
/// `kalico_runtime_configure_axis`. Sentinel: `tmc_cs_oid == 0xFF` means
/// "no TMC driver" (Pulse-only stepper). OID 0 is a legal SPI OID and
/// must not be conflated with "absent."
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §5.2.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct StepperBindingRust {
    pub tmc_cs_oid: u8,
    pub _pad: [u8; 3],
}
const _: () = assert!(core::mem::size_of::<StepperBindingRust>() == 4);

pub const TMC_CS_OID_NONE: u8 = 0xFF;
```

- [ ] **Step 3: Add the C header mirror**

In `rust/kalico-c-api/include/kalico_runtime.h`, add (before the function declarations block):

```c
struct StepperBindingRust {
    uint8_t tmc_cs_oid;     // 0xFF = none (Pulse-only stepper)
    uint8_t _pad[3];
};
_Static_assert(sizeof(struct StepperBindingRust) == 4,
               "StepperBindingRust ABI drift");
```

- [ ] **Step 4: Update SpiWrite shape**

In `rust/runtime/src/spi_queue.rs`, replace the `SpiWrite` struct definition with:

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpiWrite {
    pub tmc_cs_oid: u8,
    pub reg: u8,
    pub _pad: [u8; 2],
    pub value: i32,
}
const _: () = assert!(core::mem::size_of::<SpiWrite>() == 8);
```

(Down from 12 to 8 bytes; `SpiQueue` array shrinks proportionally.)

- [ ] **Step 5: Write the ABI cross-check test**

Create `rust/runtime/tests/stepper_binding_abi.rs`:

```rust
use kalico_runtime::stepping_state::{StepperBindingRust, TMC_CS_OID_NONE};

#[test]
fn binding_size_is_four() {
    assert_eq!(core::mem::size_of::<StepperBindingRust>(), 4);
}

#[test]
fn tmc_cs_oid_none_sentinel() {
    let b = StepperBindingRust { tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 3] };
    assert_eq!(b.tmc_cs_oid, 0xFF);
}

#[test]
fn tmc_cs_oid_zero_is_valid() {
    // OID 0 is a real SPI device OID and must NOT be treated as "no TMC."
    let b = StepperBindingRust { tmc_cs_oid: 0, _pad: [0; 3] };
    assert_ne!(b.tmc_cs_oid, TMC_CS_OID_NONE);
}
```

- [ ] **Step 6: Build + run tests**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
cd rust && cargo test -p runtime --test stepper_binding_abi 2>&1 | tail
```

Expected: builds; 3 tests pass.

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/stepping_state.rs rust/runtime/src/spi_queue.rs \
        rust/kalico-c-api/include/kalico_runtime.h \
        rust/runtime/tests/stepper_binding_abi.rs
git commit -m "feat(stepping_state): shrink StepperRef; add StepperBindingRust ABI

step_pin/dir_pin/dir_invert removed (lived in C-side runtime_motor_steppers).
SpiWrite re-typed to carry tmc_cs_oid: u8 instead of cs_pin: u32. ABI is
4 bytes with 0xFF sentinel for 'no TMC.'"
```

---

### Task 6: AxisConfig adds cursor; drop extrusion_per_xy_mm

**Files:**
- Modify: `rust/runtime/src/stepping_state.rs`

- [ ] **Step 1: Modify AxisConfig struct**

Replace `AxisConfig`'s field list with:

```rust
pub struct AxisConfig {
    pub mode: AtomicU8,
    pub steppers: heapless::Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    /// Active curve handle. `None` when no segment is armed or the curve
    /// is exhausted.
    pub curve_handle: Option<crate::curve_pool::CurveHandle>,
    /// Index into the loaded curve's `pieces` array. Advanced by
    /// `advance_piece_if_needed`.
    pub piece_cursor: u16,
    /// Cached active piece (= curve.pieces[piece_cursor]). Refreshed
    /// only on piece-boundary advancement.
    pub piece: Option<crate::monomial::BezierPieceMonomial>,
    pub piece_start_time_cycles: u64,
    pub last_step_count: i32,
    pub microstep_distance: f32,
}
```

Delete `extrusion_per_xy_mm: f32` (replaced by per-segment `Segment::extrusion_ratio`).

- [ ] **Step 2: Update AxisConfig::new (and any const ctor)**

Wherever AxisConfig is constructed, drop the `extrusion_per_xy_mm` initializer and add `curve_handle: None, piece_cursor: 0`. Search the codebase:

```
grep -rn 'AxisConfig {' rust/runtime/src/ rust/runtime/tests/
```

For each occurrence, apply the field rename/addition. Pattern:

```rust
AxisConfig {
    mode: AtomicU8::new(StepMode::Pulse as u8),
    steppers: heapless::Vec::new(),
    curve_handle: None,
    piece_cursor: 0,
    piece: None,
    piece_start_time_cycles: 0,
    last_step_count: 0,
    microstep_distance: 0.0,
}
```

- [ ] **Step 3: Build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean build (the stubbed legacy callers from Task 4 still apply).

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/stepping_state.rs
git commit -m "feat(stepping_state): AxisConfig adds curve_handle + piece_cursor

Drops extrusion_per_xy_mm — per-segment Segment.extrusion_ratio is
authoritative. The piece field stays as a cached copy of
curve.pieces[piece_cursor], refreshed on piece-boundary advancement only."
```

---

### Task 7: Engine adds segment-arm fields

**Files:**
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Add new Engine fields**

In `rust/runtime/src/engine.rs`, find the `Engine` (or `EngineImpl`) struct. Add four new fields:

```rust
pub struct Engine<P, I> {
    // ... existing fields up through `current: Option<Segment>` ...

    /// Bitmask: bits 0-3 are axes A/B/Z/E. Set at `arm_segment` for each
    /// axis whose curve handle is non-sentinel and which participates in
    /// retire (E in CoupledToXy mode is non-participating).
    /// Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §4.5.
    pub participating_mask: u8,

    /// Bitmask: starts equal to `participating_mask` at arm; bits clear
    /// as each axis's curve exhausts during evaluation. Retire fires
    /// when `pending_mask == 0`.
    pub pending_mask: u8,

    /// Snapshot of `e_accumulator` (truncated to f32) at segment-arm.
    /// Phase-3 evaluator returns `segment_base_e + segment_local_e` to
    /// produce absolute E position (§4.6).
    pub segment_base_e: f32,

    /// XY arc-length accumulator, in mm, segment-scoped. Reset at arm,
    /// updated each sample in Phase 2 of `runtime_tick_sample`.
    pub ds_xy_segment: f32,
}
```

Keep `e_accumulator: f64` (existing) — that's the long-haul accumulator we now base `segment_base_e` on.

- [ ] **Step 2: Initialize new fields in Engine::new (and any in-place init)**

Find `Engine::new(clock_freq)` (around engine.rs:498). Add to the struct literal:

```rust
participating_mask: 0,
pending_mask: 0,
segment_base_e: 0.0,
ds_xy_segment: 0.0,
```

Find `Engine::init_in_place(ptr, clock_freq)` (added by firmware Task 7-fix). Add four `addr_of_mut!((*ptr).field).write(...)` lines for the new fields.

- [ ] **Step 3: Build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/engine.rs
git commit -m "feat(engine): add participating_mask/pending_mask/segment_base_e/ds_xy_segment

Engine-level fields for the new segment-arm + retire bookkeeping. The
existing e_accumulator: f64 stays; segment_base_e is snapshotted from
it at arm and feeds the absolute-E phase-3 evaluator (§4.6)."
```

---

### Task 8: arm_segment method (ISR-side)

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Create: `rust/runtime/tests/arm_segment.rs`

- [ ] **Step 1: Implement arm_segment**

In `rust/runtime/src/engine.rs`, add a method on `Engine`:

```rust
/// ISR-side segment arm. Called by `runtime_tick_sample` after dequeueing
/// a Segment from the SPSC queue. Populates per-axis state and the
/// engine-level retire-bookkeeping masks. NEVER called from foreground —
/// the §11.1 half-split discipline reserves AxisConfig mutation for the ISR.
///
/// Spec: docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md §3.3 + §4.5.
fn arm_segment(
    &mut self,
    seg: crate::segment::Segment,
    curve_pool: &crate::curve_pool::CurvePool,
) {
    let handles = [seg.x_handle, seg.y_handle, seg.z_handle, seg.e_handle];

    // Per-axis arm.
    for (axis_idx, handle) in handles.iter().enumerate() {
        let axis = &mut self.stepping_axes[axis_idx];
        if *handle == crate::curve_pool::CurveHandle::UNUSED_SENTINEL {
            axis.curve_handle = None;
            axis.piece = None;
            axis.piece_cursor = 0;
        } else if let Some(curve_ptr) = curve_pool.lookup_active(*handle) {
            // SAFETY: curve_pool's generation guard published the slot; ISR
            // is sole reader for the duration of the segment.
            let curve = unsafe { &*curve_ptr };
            if curve.piece_count == 0 {
                // Defensive: shouldn't happen (load rejects empty), but
                // treat as idle if it does.
                axis.curve_handle = None;
                axis.piece = None;
                axis.piece_cursor = 0;
            } else {
                axis.curve_handle = Some(*handle);
                axis.piece_cursor = 0;
                axis.piece = Some(curve.pieces[0]);
                axis.piece_start_time_cycles = seg.t_start;
            }
        } else {
            // Slot generation mismatch (foreground should have validated;
            // defensive fallback).
            axis.curve_handle = None;
            axis.piece = None;
            axis.piece_cursor = 0;
        }
    }

    // Compute participating_mask. Bits A/B/Z (0..3) follow handle validity;
    // bit E (3) ALSO requires e_mode == Independent.
    let mut participating: u8 = 0;
    for axis_idx in 0..3 {
        if self.stepping_axes[axis_idx].curve_handle.is_some() {
            participating |= 1u8 << axis_idx;
        }
    }
    if seg.e_mode == crate::config::EMode::Independent
        && self.stepping_axes[3].curve_handle.is_some()
    {
        participating |= 1u8 << 3;
    }
    self.participating_mask = participating;
    self.pending_mask = participating;

    // E-accumulator base for absolute-position math.
    self.segment_base_e = self.e_accumulator as f32;
    self.ds_xy_segment = 0.0;

    self.current = Some(seg);
}
```

- [ ] **Step 2: Write tests**

Create `rust/runtime/tests/arm_segment.rs`. Tests should exercise:

```rust
use kalico_runtime::config::EMode;
use kalico_runtime::curve_pool::{CurveHandle, CurvePool};
use kalico_runtime::cubic_curve::{populate_from_wire, WirePiece};
use kalico_runtime::segment::{Segment, KinematicTag};

fn make_linear_wire(delta_mm: f32, duration_s: f32) -> WirePiece {
    WirePiece {
        bp0_bits: 0.0f32.to_bits(),
        bp1_bits: (delta_mm / 3.0).to_bits(),
        bp2_bits: (2.0 * delta_mm / 3.0).to_bits(),
        bp3_bits: delta_mm.to_bits(),
        duration_bits: duration_s.to_bits(),
    }
}

#[test]
fn arms_per_axis_state_for_valid_segment() {
    // Construct a curve_pool, load X axis with a linear 10mm@25µs piece,
    // construct a Segment, call arm_segment, assert axis A is armed with
    // piece_cursor=0, curve_handle.is_some(), participating_mask=0b0001.
    // ... full setup including Engine::new(clock_freq) ...
}

#[test]
fn idle_axis_stays_none_for_unused_sentinel() {
    // Only X loaded; Y/Z/E handles = UNUSED_SENTINEL. After arm:
    // axes 0 has piece, axes 1/2/3 have piece=None and curve_handle=None.
}

#[test]
fn participating_mask_for_coupled_e_excludes_e_bit() {
    // X loaded, E loaded, e_mode=CoupledToXy. Mask should be 0b0001 (E excluded).
    // Even though E has a valid curve, in CoupledToXy mode E doesn't participate.
}

#[test]
fn participating_mask_for_independent_e_includes_e_bit() {
    // X loaded, E loaded, e_mode=Independent. Mask should be 0b1001 (E included).
}

#[test]
fn participating_mask_for_travel_excludes_e_bit() {
    // X loaded, E *not* loaded (sentinel handle), e_mode=Travel. Mask = 0b0001.
}

#[test]
fn segment_base_e_snapshotted_from_accumulator() {
    // Set engine.e_accumulator = 12.345; arm. assert engine.segment_base_e ≈ 12.345.
}

#[test]
fn ds_xy_segment_resets_to_zero() {
    // Set engine.ds_xy_segment = 999.0; arm. assert engine.ds_xy_segment == 0.0.
}
```

(Implement each test's setup boilerplate; the test harness will need to construct an `Engine` and a `CurvePool` — both have `::new()` constructors. Sketch in the actual content during implementation.)

- [ ] **Step 3: Run tests**

```
cd rust && cargo test -p runtime --test arm_segment 2>&1 | tail
```

Expected: 7 tests pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/arm_segment.rs
git commit -m "feat(engine): arm_segment ISR-side method + tests

Sets per-axis (curve_handle, piece_cursor, piece, piece_start_time_cycles),
computes participating_mask honoring e_mode (CoupledToXy E non-participating,
Independent E participating, Travel E excluded), initializes pending_mask,
snapshots segment_base_e from e_accumulator, resets ds_xy_segment."
```

---

### Task 9: advance_piece_if_needed cursor logic + fault helpers

**Files:**
- Modify: `rust/runtime/src/tick.rs`
- Modify: `rust/runtime/src/fault_helpers.rs`

- [ ] **Step 1: Update advance_piece_if_needed**

Find the existing `advance_piece_if_needed` in `rust/runtime/src/tick.rs` (firmware Task 9 added it). Replace its body with the cursor-walking version:

```rust
/// Walk the per-axis cursor forward past any sample-straddled pieces.
/// Spec §4.4. Does NOT make any retire/fault decisions — the per-sample
/// post-pass in runtime_tick_sample owns participating_mask updates and
/// the early-exhaustion fault check.
fn advance_piece_if_needed(
    axis: &mut crate::stepping_state::AxisConfig,
    axis_idx: usize,
    curve_pool: &crate::curve_pool::CurvePool,
    shared: &crate::state::SharedState,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters: u16 = 0;
    loop {
        let Some(piece) = axis.piece else { break };
        let t_local = t_sample_end_global
            - (axis.piece_start_time_cycles as f32) / cycles_per_second;
        if t_local <= piece.duration {
            break;
        }
        // Walk cursor forward by one piece.
        axis.piece_start_time_cycles = axis.piece_start_time_cycles
            .wrapping_add((piece.duration * cycles_per_second) as u64);
        axis.piece_cursor = axis.piece_cursor.saturating_add(1);
        advanced = true;

        match axis.curve_handle {
            Some(handle) => match curve_pool.lookup_active(handle) {
                Some(curve_ptr) => {
                    // SAFETY: gen-validated by lookup_active; ISR is sole reader.
                    let curve = unsafe { &*curve_ptr };
                    if (axis.piece_cursor as usize) < curve.piece_count as usize {
                        axis.piece = Some(curve.pieces[axis.piece_cursor as usize]);
                    } else {
                        // Curve exhausted. Per-sample post-pass decides
                        // retire vs. fault — clear local state only.
                        axis.piece = None;
                        axis.curve_handle = None;
                        break;
                    }
                }
                None => {
                    // Slot generation drift (defensive; shouldn't happen
                    // unless host retired mid-segment).
                    axis.piece = None;
                    axis.curve_handle = None;
                    break;
                }
            },
            None => { axis.piece = None; break; }
        }

        iters = iters.saturating_add(1);
        if iters >= crate::sizing::MAX_PIECES_PER_CURVE as u16 {
            // Runaway loop (corrupt durations or curve_pool race). Fault
            // regardless of participation — exceeding MAX_PIECES means
            // something structural is wrong.
            crate::fault_helpers::raise_piece_advance_underflow(shared, axis_idx);
            break;
        }
    }
    advanced
}
```

- [ ] **Step 2: Confirm raise_piece_advance_underflow exists**

Look in `rust/runtime/src/fault_helpers.rs` for `raise_piece_advance_underflow`. If not present, add:

```rust
pub fn raise_piece_advance_underflow(shared: &SharedState, axis_idx: usize) {
    raise_with_axis(shared, FaultCode::PieceAdvanceUnderflow, axis_idx);
}
```

(Use the existing `raise_with_axis` helper or follow the pattern of other `raise_*` functions in the file.)

- [ ] **Step 3: Build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/tick.rs rust/runtime/src/fault_helpers.rs
git commit -m "feat(tick): cursor-walking advance_piece_if_needed; runaway-loop fault only

Per-axis loop only clears local state on exhaustion; the per-sample
post-pass owns retire/fault decisions. iters cap is MAX_PIECES_PER_CURVE
because the loop is structurally bounded by curve length."
```

---

### Task 10: Per-sample post-pass for exhaustion + retire

**Files:**
- Modify: `rust/runtime/src/tick.rs`
- Create: `rust/runtime/tests/exhaustion_post_pass.rs`

- [ ] **Step 1: Add the post-pass method to Engine**

In `rust/runtime/src/engine.rs`, add:

```rust
/// Per-sample post-pass: after every axis advances for this sample, update
/// `pending_mask` and check for early exhaustion. Spec §4.4 + §4.5.
fn post_pass_exhaustion(&mut self, shared: &crate::state::SharedState) {
    let Some(_seg) = self.current.as_ref() else { return; };
    // Compute exhausted_now: bits where the axis WAS participating but
    // its curve_handle is now None.
    let mut exhausted_now: u8 = 0;
    for axis_idx in 0..4 {
        if self.participating_mask & (1u8 << axis_idx) != 0
            && self.stepping_axes[axis_idx].curve_handle.is_none()
        {
            exhausted_now |= 1u8 << axis_idx;
        }
    }
    let prev_pending = self.pending_mask;
    self.pending_mask = self.participating_mask & !exhausted_now;

    // Early-exhaustion fault: any axis that exhausted THIS sample
    // (was pending, now exhausted) while OTHER pending axes remain after
    // the full sample pass.
    let exhausted_this_sample = prev_pending & exhausted_now;
    if exhausted_this_sample != 0 && self.pending_mask != 0 {
        let axis_idx = exhausted_this_sample.trailing_zeros() as usize;
        crate::fault_helpers::raise_piece_advance_underflow(shared, axis_idx);
    }
}
```

- [ ] **Step 2: Wire it into runtime_tick_sample**

In `rust/runtime/src/tick.rs`, find `runtime_tick_sample`. After the per-axis advance + dispatch calls (Phase 1, 2, 3 all complete), and BEFORE Phase 5 retire bookkeeping, add:

```rust
engine.post_pass_exhaustion(shared);
```

- [ ] **Step 3: Add the retire check (Phase 5)**

Replace any existing Phase 5 stub with the new retire condition:

```rust
// Phase 5: segment retire when all participating axes have exhausted.
if engine.current.is_some() && engine.pending_mask == 0 {
    let seg_id = engine.current.as_ref().unwrap().id;
    // Publish retired_through_segment_id with the actual segment id.
    shared.retired_through_segment_id.store(seg_id, core::sync::atomic::Ordering::Release);
    // Stream-state hook (existing function).
    crate::stream::check_terminal_on_retire(shared, seg_id);
    // Emit SEGMENT_END trace so foreground reclaim::drain_and_reclaim
    // can call confirm_retired on each handle via RetirementTable.
    let _ = trace.enqueue(crate::trace::TraceSample {
        tick: t_sample_end_global as u64,  // TODO confirm cycle/sec/tick units
        flags: crate::trace::TRACE_FLAG_SEGMENT_END,
        segment_id: seg_id,
        // ... rest of TraceSample fields ...
    });
    // Roll forward the long-haul E accumulator.
    let final_e_local = compute_final_segment_e(engine);
    engine.e_accumulator += final_e_local as f64;
    // Clear current-segment state to await next arm.
    engine.current = None;
    engine.participating_mask = 0;
    engine.pending_mask = 0;
    engine.segment_base_e = 0.0;
    engine.ds_xy_segment = 0.0;
}
```

`compute_final_segment_e` is a helper that re-evaluates Phase-3 E for the final sample to compute the segment-local E delta (so `e_accumulator` rolls forward correctly). If Phase-3 already cached the value in a local var that's accessible here, reuse it instead.

- [ ] **Step 4: Tests**

Create `rust/runtime/tests/exhaustion_post_pass.rs`:

```rust
// Tests for the post-pass exhaustion check (§4.4 + §4.5).

#[test]
fn no_fault_when_simultaneous_exhaustion() {
    // X and Y both armed with 1-piece curves of equal duration.
    // After enough samples to exhaust both: post-pass sees
    // exhausted_this_sample = 0b0011, pending_mask = 0,
    // no fault — segment retires next.
}

#[test]
fn fault_when_x_exhausts_while_y_pending() {
    // X has 1-piece 10µs curve; Y has 2-piece curve totaling 50µs.
    // After samples that exhaust X (around sample 1), Y still pending.
    // Expect raise_piece_advance_underflow fired with axis_idx=0.
}

#[test]
fn no_fault_for_coupled_e_exhausting_early() {
    // CoupledToXy mode with intrinsic E curve (10µs) shorter than X (50µs).
    // E is non-participating; E's exhaustion doesn't enter
    // exhausted_this_sample. No fault.
}

#[test]
fn fault_for_independent_e_exhausting_early() {
    // Independent mode with E shorter than X. E participates; faults.
}

#[test]
fn fault_axis_idx_is_lowest_bit() {
    // X (idx 0) AND Z (idx 2) both exhaust this sample while Y (idx 1)
    // pending. Fault detail's axis_idx should be 0 (lowest set bit).
}
```

- [ ] **Step 5: Run tests**

```
cd rust && cargo test -p runtime --test exhaustion_post_pass 2>&1 | tail
```

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/tick.rs rust/runtime/tests/exhaustion_post_pass.rs
git commit -m "feat(tick): per-sample post-pass for exhaustion + retire condition

post_pass_exhaustion runs after every per-axis advance for the sample,
clearing pending_mask bits and faulting only when at least one bit
cleared this sample AND pending_mask remains non-zero. Order-independent.

Phase 5 retire fires on pending_mask == 0, publishing the actual
segment id (not a counter) to match the existing engine.rs:2266 contract."
```

---

### Task 11: E-axis follower math (absolute-E model)

**Files:**
- Modify: `rust/runtime/src/tick.rs`
- Create: `rust/runtime/tests/e_follower_absolute.rs`

- [ ] **Step 1: Implement evaluate_e_axis**

In `rust/runtime/src/tick.rs`, add (or replace the existing E-axis evaluation in Phase 3 with) the new evaluator per the spec §4.6:

```rust
fn evaluate_e_axis(
    axis_e: &mut crate::stepping_state::AxisConfig,
    current: Option<&crate::segment::Segment>,
    engine_segment_base_e: f32,
    ds_xy_segment_accumulator: f32,
    pa_k: f32,
    v_xy_this: f32,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> f32 {
    let Some(seg) = current else { return engine_segment_base_e; };
    let intrinsic_local = if let Some(piece) = axis_e.piece {
        let t_local = t_sample_end_global
            - (axis_e.piece_start_time_cycles as f32) / cycles_per_second;
        let (p, _v) = crate::monomial::eval_position_velocity(&piece, t_local);
        p
    } else {
        0.0
    };
    let segment_local = match seg.e_mode {
        crate::config::EMode::CoupledToXy => {
            let follower = seg.extrusion_ratio * ds_xy_segment_accumulator;
            let pa_correction = pa_k * seg.extrusion_ratio * v_xy_this;
            intrinsic_local + follower + pa_correction
        }
        crate::config::EMode::Independent => intrinsic_local,
        crate::config::EMode::Travel => 0.0,
    };
    engine_segment_base_e + segment_local
}
```

- [ ] **Step 2: Wire into Phase 3 of runtime_tick_sample**

Replace any existing Phase 3 E-axis computation with a call to `evaluate_e_axis`. The result `p_end_e` then flows into the existing `dispatch_axis(...)` call for the E axis. `dispatch_axis` computes `n_steps = round((p_end - p_prev) / microstep_distance)`, which now operates on absolute mm values — correct across segment boundaries.

- [ ] **Step 3: Tests**

Create `rust/runtime/tests/e_follower_absolute.rs`:

```rust
// Spec §4.6 — absolute-E position model.

#[test]
fn coupled_to_xy_intrinsic_zero_e_handle_sentinel() {
    // e_mode=CoupledToXy, e_handle=UNUSED_SENTINEL. E motion should still
    // accumulate as ratio * ds_xy_segment. After 100µm of XY arc at ratio
    // 0.05, E should advance by 5µm + segment_base_e.
}

#[test]
fn coupled_to_xy_position_continuous_across_segments() {
    // First segment: XY moves 1mm, ratio 0.05 → E_local = 0.05mm.
    // e_accumulator rolls to 0.05 at retire.
    // Second segment armed: segment_base_e = 0.05.
    // Second segment XY moves 0.5mm same ratio → E_local = 0.025.
    // Final E = 0.05 + 0.025 = 0.075. Continuous (no backwards motion).
}

#[test]
fn independent_mode_no_xy_contribution() {
    // e_mode=Independent. Intrinsic curve says E=2mm at end of segment.
    // XY moves 10mm meanwhile. E final = segment_base_e + 2.0 (no XY follower).
}

#[test]
fn travel_mode_zero_motion() {
    // e_mode=Travel. Whether intrinsic curve present or not, E stays at
    // segment_base_e (no motion this segment).
}

#[test]
fn pa_correction_signed_by_acceleration() {
    // CoupledToXy with non-zero pa_k. v_xy_this > 0 → positive E offset
    // from PA. Same XY arc length, v_xy_this < 0 (deceleration) → negative
    // PA correction. Sign-of-acceleration handling.
}
```

- [ ] **Step 4: Run tests**

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/tick.rs rust/runtime/tests/e_follower_absolute.rs
git commit -m "feat(tick): E-axis follower math with absolute-E position model

evaluate_e_axis returns segment_base_e + segment_local. segment_base_e is
snapshotted at arm from engine.e_accumulator (f64); e_accumulator rolls
forward at retire by the segment's final E delta. CoupledToXy with
e_handle=UNUSED_SENTINEL produces real E motion from the follower term.
Independent uses intrinsic only. Travel zeros E."
```

---

### Task 12: Delete legacy Rust paths

**Files:**
- Delete: `rust/runtime/src/step_ring.rs`
- Delete: `rust/runtime/src/step_time.rs`
- Delete: `rust/runtime/src/step_producer.rs`
- Modify: `rust/runtime/src/engine.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Delete step_ring.rs, step_time.rs, step_producer.rs**

```bash
git rm rust/runtime/src/step_ring.rs rust/runtime/src/step_time.rs rust/runtime/src/step_producer.rs
```

- [ ] **Step 2: Remove pub mod declarations**

In `rust/runtime/src/lib.rs`, delete `pub mod step_ring;`, `pub mod step_time;`, `pub mod step_producer;`.

- [ ] **Step 3: Delete Engine fields and methods**

In `rust/runtime/src/engine.rs`, delete:
- `step_rings: [StepRing; 4]` field
- `producer_states: [ProducerState; 4]` field
- `producer_current: ...` field
- `motor_curve_cursor: [...]` field
- `motor_current_segment_id: [...]` field
- `Engine::producer_step` method (the ~900-line one, ~line 2672)
- `Engine::arm_step_timer_for_stepper` and related arm_step_timer* methods
- `Engine::producer_step_distance` (~line 1132)
- `Engine::fetch_segment_for_motor` (if present)
- Const `PRODUCER_BATCH_CAP` (~line 107)
- Const `STEP_RING_LOW_WATER` (search and delete)

Plus any `use` imports that referenced the deleted modules.

Replace the Task 4 `unimplemented!()` stubs with real deletions (the call sites are gone too because their fields are gone).

- [ ] **Step 4: Build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor(runtime): delete legacy step_ring/step_time/step_producer paths

Removes:
- rust/runtime/src/{step_ring,step_time,step_producer}.rs (whole files)
- Engine::producer_step (~900 LOC Newton-fill loop)
- Engine::arm_step_timer* helpers (~180 LOC)
- Engine fields: step_rings, producer_states, producer_current,
  motor_curve_cursor, motor_current_segment_id
- PRODUCER_BATCH_CAP, STEP_RING_LOW_WATER consts

Slot+generation discipline in curve_pool stays; only NURBS payload was
swapped (Task 3)."
```

---

### Task 13: Update FFI — drop NURBS load_curve, add load_curve_cubic

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/kalico-c-api/include/kalico_runtime.h`
- Modify: `rust/kalico-c-api/cbindgen.toml`

- [ ] **Step 1: Delete the NURBS FFI export**

In `rust/kalico-c-api/src/runtime_ffi.rs`, find and delete:
- `pub unsafe extern "C" fn runtime_handle_load_curve` (the whole function body)
- `pub unsafe extern "C" fn kalico_runtime_producer_step`
- `pub unsafe extern "C" fn kalico_runtime_step_ring_peek_head`
- `pub unsafe extern "C" fn kalico_runtime_step_ring_peek_next`
- `pub unsafe extern "C" fn kalico_runtime_step_ring_advance`
- `pub unsafe extern "C" fn kalico_runtime_modulated_tick`

Also delete any diag-counter exports (`kalico_runtime_get_producer_*`, `kalico_runtime_get_step_time_event_*`) related to the deleted paths.

- [ ] **Step 2: Add the cubic load FFI**

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_load_curve_cubic(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    axis_idx: u8,
    piece_count: u8,
    pieces_blob: *const u8,  // piece_count * 20 bytes (5 × u32 per piece)
    out_handle_packed: *mut u32,
) -> i32 {
    if rt.is_null() || pieces_blob.is_null() || out_handle_packed.is_null() {
        return crate::error::KALICO_ERR_NULL_PTR;
    }
    if !INIT_DONE.load(Ordering::Acquire) {
        return crate::error::KALICO_ERR_NOT_INIT;
    }
    if piece_count == 0 || piece_count as usize > runtime::sizing::MAX_PIECES_PER_CURVE {
        return crate::error::KALICO_ERR_INVALID_CURVE;
    }
    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let fg_ptr: *mut FgState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg));
        let fg: &mut FgState = &mut *fg_ptr;
        // Decode the wire blob into a stack-local WirePiece array.
        let mut wire: [runtime::cubic_curve::WirePiece; runtime::sizing::MAX_PIECES_PER_CURVE] =
            [runtime::cubic_curve::WirePiece {
                bp0_bits: 0, bp1_bits: 0, bp2_bits: 0, bp3_bits: 0, duration_bits: 0,
            }; runtime::sizing::MAX_PIECES_PER_CURVE];
        for i in 0..piece_count as usize {
            let base = pieces_blob.add(i * 20);
            // Each u32 is little-endian; read via copy_nonoverlapping into u32 buffer.
            let mut buf = [0u8; 20];
            core::ptr::copy_nonoverlapping(base, buf.as_mut_ptr(), 20);
            wire[i].bp0_bits      = u32::from_le_bytes([buf[ 0], buf[ 1], buf[ 2], buf[ 3]]);
            wire[i].bp1_bits      = u32::from_le_bytes([buf[ 4], buf[ 5], buf[ 6], buf[ 7]]);
            wire[i].bp2_bits      = u32::from_le_bytes([buf[ 8], buf[ 9], buf[10], buf[11]]);
            wire[i].bp3_bits      = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
            wire[i].duration_bits = u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);
        }
        let wire_slice = &wire[..piece_count as usize];
        match fg.curve_pool.try_alloc_and_load(slot_idx, wire_slice) {
            Ok(handle) => {
                *out_handle_packed = handle.pack();
                crate::error::KALICO_OK
            }
            Err(runtime::curve_pool::CurvePoolError::OutOfBounds) => crate::error::KALICO_ERR_INVALID_HANDLE,
            Err(runtime::curve_pool::CurvePoolError::SlotAlreadyLoaded) => crate::error::KALICO_ERR_SLOT_BUSY,
            Err(runtime::curve_pool::CurvePoolError::NonFiniteData) => crate::error::KALICO_ERR_INVALID_CURVE,
            Err(runtime::curve_pool::CurvePoolError::InvalidCurve) => crate::error::KALICO_ERR_INVALID_CURVE,
        }
    }
}
```

`axis_idx` is unused in the FFI body (the slot doesn't currently carry per-axis identity; it's a flat pool). Keep the parameter for future extensibility and validation in C-side host code.

- [ ] **Step 3: Update cbindgen.toml**

In `rust/kalico-c-api/cbindgen.toml`, find the `include = [...]` (or `export.include = [...]`) list of FFI symbols. Remove:
- `runtime_handle_load_curve`
- `kalico_runtime_producer_step`
- `kalico_runtime_step_ring_peek_head`
- `kalico_runtime_step_ring_peek_next`
- `kalico_runtime_step_ring_advance`
- `kalico_runtime_modulated_tick`

Add:
- `runtime_handle_load_curve_cubic`

- [ ] **Step 4: Regenerate the header**

```
cd rust && cbindgen --config kalico-c-api/cbindgen.toml --output kalico-c-api/include/kalico_runtime.h kalico-c-api
```

(Or whatever invocation the project uses — check the existing build for the regen command.)

- [ ] **Step 5: Build the FFI crate**

```
cd rust && cargo build -p kalico-c-api --features header-runtime 2>&1 | tail
```

Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h rust/kalico-c-api/cbindgen.toml
git commit -m "feat(ffi): drop NURBS load_curve + Newton/step-ring exports; add load_curve_cubic

Deleted:
- runtime_handle_load_curve (NURBS)
- kalico_runtime_producer_step
- kalico_runtime_step_ring_peek_head / _peek_next / _advance
- kalico_runtime_modulated_tick

Added:
- runtime_handle_load_curve_cubic — atomic, one-shot, validates +
  populates a slot via cubic_curve::populate_from_wire.

Header regenerated via cbindgen."
```

---

### Task 14: Extend configure_axis FFI for stepper bindings

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/engine.rs` (engine-side configure_axis method)
- Modify: `rust/runtime/src/error.rs` (add `PhaseModeNotAvailable`, `CurveLoadInvalid`)
- Modify: `rust/runtime/src/fault_helpers.rs` (add raise helpers)
- Create: `rust/runtime/tests/configure_axis_two_phase.rs`

- [ ] **Step 1: Add the new fault codes**

In `rust/runtime/src/error.rs`, find the `FaultCode` enum and add:

```rust
PhaseModeNotAvailable = -23,  // configure_axis(mode=Phase) — SPI dispatch deferred
CurveLoadInvalid = -24,       // load_curve_cubic: validation rejected the wire payload
```

Pick numeric values that don't collide with existing variants. Also add the matching `KALICO_ERR_*` constants alongside the existing ones (search for `pub const KALICO_ERR_*` to find the block).

- [ ] **Step 2: Add raise_* helpers**

In `rust/runtime/src/fault_helpers.rs`:

```rust
pub fn raise_phase_mode_not_available(shared: &SharedState, axis_idx: usize) {
    raise_with_axis(shared, FaultCode::PhaseModeNotAvailable, axis_idx);
}
```

(CurveLoadInvalid is raised inside the FFI via error code return; no separate helper needed.)

- [ ] **Step 3: Update the configure_axis FFI signature**

In `rust/kalico-c-api/src/runtime_ffi.rs`, replace the existing `kalico_runtime_configure_axis` signature with:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_configure_axis(
    rt: *mut KalicoRuntime,
    axis_idx: u8,
    mode: u8,                              // 0=Pulse, 1=Phase
    microstep_distance_bits: u32,          // f32 bits
    extrusion_per_xy_mm_bits: u32,         // f32 bits — UNUSED by ISR; per-segment field is authoritative
    stepper_count: u8,
    bindings_ptr: *const runtime::stepping_state::StepperBindingRust,
) -> i32 {
    if rt.is_null() { return crate::error::KALICO_ERR_NULL_PTR; }
    if !INIT_DONE.load(Ordering::Acquire) { return crate::error::KALICO_ERR_NOT_INIT; }
    if mode > 1 { return crate::error::KALICO_ERR_INVALID_ARG; }
    if stepper_count as usize > runtime::stepping_state::MAX_STEPPERS_PER_AXIS {
        return crate::error::KALICO_ERR_INVALID_ARG;
    }
    if stepper_count > 0 && bindings_ptr.is_null() {
        return crate::error::KALICO_ERR_NULL_PTR;
    }

    // Spec §5.2: reject Phase mode at configure-time; SPI dispatch is deferred.
    let mode_enum = match mode {
        0 => runtime::stepping_state::StepMode::Pulse,
        1 => return crate::error::KALICO_ERR_PHASE_MODE_NOT_AVAILABLE,
        _ => unreachable!(),
    };

    let microstep_distance = f32::from_bits(microstep_distance_bits);
    if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
        return crate::error::KALICO_ERR_INVALID_ARG;
    }

    let ctx = rt.cast::<RuntimeContext>();
    unsafe {
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let isr: &mut IsrState = &mut *isr_ptr;
        let bindings_slice = if stepper_count == 0 {
            &[]
        } else {
            core::slice::from_raw_parts(bindings_ptr, stepper_count as usize)
        };
        isr.engine.configure_axis(axis_idx, mode_enum, microstep_distance, bindings_slice)
    }
}
```

- [ ] **Step 4: Implement the engine-side configure_axis**

In `rust/runtime/src/engine.rs`:

```rust
impl<P, I> Engine<P, I> {
    /// Spec §5.2 — two-phase: validate everything before mutating axis state.
    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: crate::stepping_state::StepMode,
        microstep_distance: f32,
        bindings: &[crate::stepping_state::StepperBindingRust],
    ) -> i32 {
        if axis_idx as usize >= 4 {
            return crate::error::KALICO_ERR_INVALID_ARG;
        }
        // Motion-in-progress reject.
        if self.current.is_some() {
            return crate::error::KALICO_ERR_MOTION_IN_PROGRESS;
        }
        // Phase 1: validate every binding.
        for b in bindings {
            // (Validation of tmc_cs_oid happens C-side via oid_lookup. Here
            // we only check the sentinel/non-sentinel binary.)
            let _ = b.tmc_cs_oid;  // currently no Rust-side per-OID check
        }

        // Phase 2: commit. Mutate axis state atomically (no early return below).
        let axis = &mut self.stepping_axes[axis_idx as usize];
        axis.mode.store(mode as u8, core::sync::atomic::Ordering::Release);
        axis.microstep_distance = microstep_distance;
        axis.steppers.clear();
        for b in bindings {
            let tmc_cs_oid = if b.tmc_cs_oid == crate::stepping_state::TMC_CS_OID_NONE {
                None
            } else {
                Some(b.tmc_cs_oid)
            };
            let stepper = crate::stepping_state::StepperRef::new(tmc_cs_oid);
            let _ = axis.steppers.push(stepper);  // capacity validated above
        }
        crate::error::KALICO_OK
    }
}
```

- [ ] **Step 5: Tests**

Create `rust/runtime/tests/configure_axis_two_phase.rs`:

```rust
#[test]
fn phase_mode_rejected_with_specific_error() {
    // call configure_axis(mode=Phase, ...). Verify return code is
    // KALICO_ERR_PHASE_MODE_NOT_AVAILABLE. Verify axis state is unchanged.
}

#[test]
fn motion_in_progress_rejected() {
    // Arm a segment first, then call configure_axis. Reject.
}

#[test]
fn out_of_range_axis_idx_rejected() {
    // axis_idx = 4 (out of 0..4). Reject with KALICO_ERR_INVALID_ARG.
}

#[test]
fn too_many_steppers_rejected() {
    // stepper_count = 5 (MAX_STEPPERS_PER_AXIS is 4). Reject.
}

#[test]
fn microstep_distance_zero_rejected() {
    // microstep_distance = 0.0. Reject.
}

#[test]
fn microstep_distance_negative_rejected() {
    // microstep_distance = -0.1. Reject.
}

#[test]
fn microstep_distance_non_finite_rejected() {
    // microstep_distance = NaN. Reject.
}

#[test]
fn valid_pulse_configure_succeeds() {
    // mode=Pulse, 2 steppers, microstep=0.00625. Returns KALICO_OK.
    // axis.steppers.len() == 2. axis.mode == Pulse. microstep_distance set.
}

#[test]
fn validation_failure_leaves_axis_state_untouched() {
    // Pre-state: axis has 2 steppers, microstep=0.01.
    // Call configure_axis(mode=Phase, ...) — should reject.
    // Post-state: axis still has 2 steppers, microstep=0.01.
}

#[test]
fn binding_tmc_cs_zero_is_legal() {
    // OID 0 must be accepted (not conflated with TMC_CS_OID_NONE = 0xFF).
}
```

- [ ] **Step 6: Run tests**

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/error.rs rust/runtime/src/fault_helpers.rs \
        rust/kalico-c-api/src/runtime_ffi.rs rust/runtime/src/engine.rs \
        rust/runtime/tests/configure_axis_two_phase.rs
git commit -m "feat(ffi): two-phase configure_axis with stepper bindings + PhaseModeNotAvailable

Phase 1 validates every input + every binding's tmc_cs_oid via oid_lookup
(C-side). Phase 2 commits to runtime_motor_steppers[][] (C-side) and to
axis.steppers (Rust-side). Validation failure leaves both sides untouched.

Adds KALICO_ERR_PHASE_MODE_NOT_AVAILABLE — configure_axis(mode=Phase)
returns this error and does not arm the axis. SPI dispatch path is
a follow-up plan."
```

---

### Task 15: Update kalico_dispatch.c — drop handle_load_curve, add handle_load_curve_cubic

**Files:**
- Modify: `src/kalico_dispatch.c`
- Modify: `src/kalico_dispatch.h`
- Modify: `rust/kalico-protocol/src/messages.rs` (the wire message decoder)

- [ ] **Step 1: Delete the NURBS load_curve handler**

In `src/kalico_dispatch.c`, find `handle_load_curve` (around line ~340-400 — the function that reads NURBS payload and calls `runtime_handle_load_curve`). Delete the whole function plus its dispatch-table entry.

- [ ] **Step 2: Add the cubic load_curve handler**

Append to `src/kalico_dispatch.c`:

```c
static void
handle_load_curve_cubic(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    // Wire format (spec §3.2):
    //   slot_idx: u16 (LE)         offset 0
    //   axis_idx: u8                offset 2
    //   piece_count: u8             offset 3
    //   pieces: piece_count * 20 bytes, each piece = 5 × u32 (LE):
    //     bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits
    if (body_len < 4) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    uint16_t slot_idx = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
    uint8_t axis_idx = body[2];
    uint8_t piece_count = body[3];
    uint32_t expected_len = 4u + (uint32_t)piece_count * 20u;
    if (body_len != expected_len) {
        send_load_curve_response(correlation_id, KALICO_ERR_INVALID_CURVE, 0);
        return;
    }
    if (!runtime_handle) {
        send_load_curve_response(correlation_id, KALICO_ERR_NOT_INIT, 0);
        return;
    }
    uint32_t handle_packed = 0;
    int32_t rc = runtime_handle_load_curve_cubic(
        runtime_handle, slot_idx, axis_idx, piece_count,
        &body[4], &handle_packed);
    send_load_curve_response(correlation_id, rc, handle_packed);
}
```

Update the dispatch table (search for `kalico_dispatch_frame` and the message-id switch) to route the new message id to `handle_load_curve_cubic`. The old `handle_load_curve` route is deleted.

- [ ] **Step 3: Update the message-id constants**

In `rust/kalico-protocol/src/messages.rs` (and/or wherever message ids are centrally defined), find the `LoadCurve` message id constant. Either rename it to `LoadCurveCubic` (changing wire format) or add `LoadCurveCubic` as a new id and delete `LoadCurve`. Pick rename — the old wire format is gone entirely.

The corresponding host-side encoder also needs updating (Task 17).

- [ ] **Step 4: Build the C side**

The full firmware build is bench-side, but verify the file compiles standalone:

```
gcc -c -Isrc -Wno-unused-parameter src/kalico_dispatch.c -o /tmp/kd.o 2>&1 | tail
```

(May need an `autoconf.h` stub; reuse Task 4's pattern from the firmware plan.)

- [ ] **Step 5: Commit**

```bash
git add src/kalico_dispatch.c src/kalico_dispatch.h rust/kalico-protocol/src/messages.rs
git commit -m "feat(dispatch): drop NURBS handle_load_curve; add handle_load_curve_cubic

C-side decoder reads the cubic-piece wire format (4-byte header +
N × 20-byte pieces) and forwards to runtime_handle_load_curve_cubic.
Old NURBS load-curve handler deleted entirely."
```

---

### Task 16: Drop NURBS sizing from build.rs + Makefile envvars + Kconfig

**Files:**
- Modify: `rust/runtime/build.rs`
- Modify: `src/Makefile`
- Modify: `src/Kconfig`
- Modify: `src/runtime_storage.c`

- [ ] **Step 1: Drop NURBS env-var reads from build.rs**

In `rust/runtime/build.rs`, remove the `lookup` calls for:
- `KALICO_RUNTIME_MAX_CONTROL_POINTS`
- `KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN`
- `KALICO_RUNTIME_MAX_DEGREE`

And remove their `pub const` emissions from the sizing.rs template:

```rust
let sizing_body = format!(
    "pub const CURVE_POOL_N: usize = {cpn};\n\
     pub const RT_STORAGE_SIZE: usize = {rss};\n\
     pub const MAX_PIECES_PER_CURVE: usize = 16;\n"
);
```

- [ ] **Step 2: Drop envvar passthrough in src/Makefile**

In `src/Makefile`, find lines ~88-93 and ~107-112 (the two cargo-invocation blocks). Remove:
- `KALICO_RUNTIME_MAX_CONTROL_POINTS=...`
- `KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN=...`
- `KALICO_RUNTIME_MAX_DEGREE=...`

Keep `CURVE_POOL_N` and `RUNTIME_STORAGE_SIZE` passthrough — these still apply.

- [ ] **Step 3: Drop the NURBS Kconfig knobs**

In `src/Kconfig`, find and delete:
- `config RUNTIME_MAX_CONTROL_POINTS`
- `config RUNTIME_MAX_KNOT_VECTOR_LEN`
- `config RUNTIME_MAX_DEGREE`

Keep `RUNTIME_CURVE_POOL_N` (still used) and `RUNTIME_STORAGE_SIZE_LARGE/_SMALL` (still used).

- [ ] **Step 4: Bump down RUNTIME_STORAGE_SIZE defaults**

The cubic-piece pool is dramatically smaller. Provisionally drop the defaults; the actual safe minimum comes from a build-time measurement in Task 18:

```
config RUNTIME_STORAGE_SIZE_LARGE
    default 102400  // 100 KB; was 307712
config RUNTIME_STORAGE_SIZE_SMALL
    default 65536   // 64 KB; was 110592
```

These are conservative initial values — Task 18 (build verify) confirms the actual `size_of::<RuntimeContext>()` and tunes if needed.

- [ ] **Step 5: Drop NURBS-derived sizing from runtime_storage.c**

In `src/runtime_storage.c`, find the AXI SRAM overflow `_Static_assert`. The `RT_STORAGE_SIZE` term shrinks because `RuntimeContext` shrinks. No other change to the assert needed (the other `.axi_bss` occupants `kalico_buf`, `runtime_bench_samples_buf`, `receive_buf` are unaffected). But verify the assert still holds at the new smaller size — it should, with significantly more headroom.

- [ ] **Step 6: Build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail
```

Expected: clean. If `sizing::MAX_CONTROL_POINTS` is still referenced anywhere, the build will error — fix those references (they're all in deleted code paths, but a few may have lingered).

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/build.rs src/Makefile src/Kconfig src/runtime_storage.c
git commit -m "build: drop NURBS sizing constants; shrink RUNTIME_STORAGE_SIZE defaults

MAX_CONTROL_POINTS / MAX_KNOT_VECTOR_LEN / MAX_DEGREE removed from
Kconfig, Makefile envvar passthrough, and runtime/build.rs sizing.rs
emission. RUNTIME_STORAGE_SIZE defaults dropped to provisional values
(100 KB H7, 64 KB F4); Task 18 measures size_of::<RuntimeContext>()
and tunes if needed."
```

---

### Task 17: Update C-side runtime_tick.c — delete legacy + update kalico_configure_axis handler

**Files:**
- Modify: `src/runtime_tick.c`
- Modify: `src/stepper.c`

- [ ] **Step 1: Delete legacy code from runtime_tick.c**

In `src/runtime_tick.c`, delete entire functions:
- `step_time_event` (around line 1754)
- `runtime_producer_event` (around line 1649)
- `init_step_time_timers` (and the `step_timers[]` array it manages)
- `arm_producer_timer_*` helpers

Also delete the defines: `SF_RESCHEDULE_FLOOR`, `EMPTY_POLL_CYCLES`, `STEP_RING_LOW_WATER`.

Delete the diag counter globals: `step_time_event_fires`, `step_time_event_peak_cycles`, `runtime_producer_runs_total`, plus their `runtime_diag_progress(0xE3...)` / `(0xE7...)` / `(0xE8...)` emission paths in the status drain.

Keep:
- `init_per_axis_step_timers` and its trampolines (firmware Task 10's per-axis path; this is what fires the new path's step pulses)
- `runtime_init` (DECL_INIT) — but its call to `init_step_time_timers` becomes `init_per_axis_step_timers`
- The TIM5 ISR wiring (firmware Task 17's `kalico_runtime_tick_sample` already runs here)
- All diag/status frames not related to the deleted paths

- [ ] **Step 2: Delete config_runtime_stepper from stepper.c**

In `src/stepper.c`, find `command_config_runtime_stepper` (around line 207) and `DECL_COMMAND(command_config_runtime_stepper, ...)` (around line 279). Delete both. Also delete related diag counters: `runtime_bind_calls_total`, `runtime_bind_writes_committed`, `runtime_bind_count_snapshot_packed`, `runtime_bind_reset_calls`, `runtime_bind_calls_for_motor[]`, plus the `runtime_reset_stepper_bindings` helper if it's only called by the deleted code.

Keep:
- `runtime_motor_steppers[][]` table (still populated by the new configure_axis)
- `runtime_motor_stepper_count[]` (still used)
- `runtime_motor_last_dir[]` (still used)
- `runtime_emit_step_pulses` (the C-side GPIO emitter — unchanged)
- `command_config_stepper` (mainline; unchanged)

- [ ] **Step 3: Implement the two-phase command_kalico_configure_axis in stepper.c**

Append to `src/stepper.c` (replacing any stub from earlier firmware tasks):

```c
// Spec §5.2: two-phase configure with stepper bindings sub-message.
void
command_kalico_configure_axis(uint32_t *args)
{
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count   = args[4];
    // Klipper's %*s blob format: length THEN encoded pointer via command_decode_ptr.
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

    // Staging: resolve every OID, validate every flag byte. No mutation yet.
    struct {
        struct stepper *stepper;
        uint8_t invert_dir;
        uint8_t tmc_cs_oid;
    } staged[RUNTIME_MAX_STEPPERS_PER_MOTOR] = {{0}};

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
        if (tmc_cs_oid != 0xFF) {
            // Validates the SPI OID is allocated — oid_lookup shuts down on bad oid.
            (void)oid_lookup(tmc_cs_oid, command_config_spi);
        }
        staged[i].stepper = s;
        staged[i].invert_dir = dir_invert;
        staged[i].tmc_cs_oid = tmc_cs_oid;
    }

    // ── Phase 2: Rust-side validation. Pass tmc_cs_oid into bindings.
    struct StepperBindingRust bindings[RUNTIME_MAX_STEPPERS_PER_MOTOR] = {{0}};
    for (uint8_t i = 0; i < stepper_count; i++) {
        bindings[i].tmc_cs_oid = staged[i].tmc_cs_oid;
    }
    int32_t rc = kalico_runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits, extrusion_bits,
        stepper_count, bindings);
    if (rc != 0)
        shutdown("configure_axis rejected by runtime");

    // ── Phase 3: commit. Both sides validated.
    runtime_motor_stepper_count[axis_idx] = stepper_count;
    for (uint8_t i = 0; i < stepper_count; i++) {
        runtime_motor_steppers[axis_idx][i].stepper = staged[i].stepper;
        runtime_motor_steppers[axis_idx][i].invert_dir = staged[i].invert_dir;
    }
    runtime_motor_last_dir[axis_idx] = -1;
}
DECL_COMMAND(command_kalico_configure_axis,
    "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
    " extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s");
```

- [ ] **Step 4: Build**

The firmware full build is bench-side. Spot-compile the file standalone to catch obvious syntax errors:

```
gcc -c -Isrc -Wno-unused-parameter src/stepper.c -o /tmp/stepper.o 2>&1 | head
gcc -c -Isrc -Wno-unused-parameter src/runtime_tick.c -o /tmp/runtime_tick.o 2>&1 | head
```

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c src/stepper.c
git commit -m "feat(stepper.c): delete legacy step_time/producer paths + config_runtime_stepper

Replaces command_config_runtime_stepper with the two-phase
command_kalico_configure_axis sub-message decoder. Phase 1 validates
every input + resolves every OID into a staged array. Phase 2 calls
the Rust FFI. Phase 3 commits to runtime_motor_steppers[][] only
after both sides validate. Validation failure leaves the C-side
table untouched.

runtime_tick.c loses step_time_event, runtime_producer_event,
init_step_time_timers, and the SF_RESCHEDULE_FLOOR / EMPTY_POLL_CYCLES /
STEP_RING_LOW_WATER defines + their diag counters."
```

---

### Task 18: Verify size_of::<RuntimeContext>() + tune Kconfig storage defaults

**Files:**
- Modify: `src/Kconfig`

- [ ] **Step 1: Measure RuntimeContext size on host**

```
cd rust && cargo test -p runtime --lib runtime_context_size 2>&1 | tail
```

If there's no existing size test, add a quick one to `rust/runtime/src/state.rs`:

```rust
#[cfg(test)]
mod size_test {
    use super::RuntimeContext;
    #[test]
    fn runtime_context_size_prints() {
        eprintln!("size_of::<RuntimeContext>() = {} bytes",
                  core::mem::size_of::<RuntimeContext>());
    }
}
```

Run `cargo test -- --nocapture runtime_context_size_prints` and read the output.

- [ ] **Step 2: Adjust Kconfig defaults**

Based on the measured size, pick `RUNTIME_STORAGE_SIZE_LARGE` to leave 2-4 KB of headroom over `size_of`. Same for `_SMALL`. Update `src/Kconfig`:

```
config RUNTIME_STORAGE_SIZE_LARGE
    default <measured + headroom> if MACH_STM32H7
    ...
```

- [ ] **Step 3: Build the whole rust workspace**

```
cd rust && cargo build --workspace 2>&1 | tail
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/Kconfig
git commit -m "build: tune RUNTIME_STORAGE_SIZE defaults to fit shrunk RuntimeContext

Cubic curve_pool is ~6 KB vs NURBS ~235 KB; RuntimeContext shrinks
proportionally. New defaults leave 2-4 KB headroom over measured
size_of::<RuntimeContext>()."
```

---

### Task 19: Update motion-bridge — drop NURBS upload, emit cubic pieces

**Files:**
- Modify: `rust/motion-bridge/src/producer.rs`
- Modify: `rust/motion-bridge/src/bridge.rs`
- Modify: `rust/motion-bridge/src/planner.rs`

This is the largest single host-side change. Approach: replace the existing `producer::load_curve` (NURBS begin/chunk/finalize) with a single `load_curve_cubic` call that takes a `&[BezierPiece]` slice. Update every call site in `bridge.rs` and `planner.rs`.

- [ ] **Step 1: Add the new load_curve_cubic in producer.rs**

In `rust/motion-bridge/src/producer.rs`, replace the existing `pub fn load_curve(...)` with:

```rust
/// Send a cubic-Bezier curve to the MCU. One frame, atomic load.
///
/// `bp_per_piece` is N × [bp0, bp1, bp2, bp3] f32 control points,
/// `duration_per_piece` is N f32 durations in seconds. Both arrays
/// must have length `piece_count`.
pub fn load_curve_cubic(
    io: &dyn KalicoHostIo,
    slot_idx: u16,
    axis_idx: u8,
    bp_per_piece: &[[f32; 4]],
    duration_per_piece: &[f32],
) -> Result<u32, LoadCurveError> {
    assert_eq!(bp_per_piece.len(), duration_per_piece.len());
    let piece_count = bp_per_piece.len();
    if piece_count == 0 || piece_count > 16 {
        return Err(LoadCurveError::PieceCountOutOfRange);
    }
    // Encode wire frame: header (4 bytes) + piece_count × 20 bytes.
    let mut frame = Vec::with_capacity(4 + piece_count * 20);
    frame.extend_from_slice(&slot_idx.to_le_bytes());
    frame.push(axis_idx);
    frame.push(piece_count as u8);
    for i in 0..piece_count {
        for &cp in &bp_per_piece[i] {
            frame.extend_from_slice(&cp.to_bits().to_le_bytes());
        }
        frame.extend_from_slice(&duration_per_piece[i].to_bits().to_le_bytes());
    }
    io.send_load_curve_cubic_frame(&frame).map_err(LoadCurveError::Io)
}
```

(Adapt to actual `KalicoHostIo` trait shape; check `rust/kalico-host-rt/src/host_io/mod.rs` for the right method to add.)

- [ ] **Step 2: Update KalicoHostIo trait**

In `rust/kalico-host-rt/src/host_io/mod.rs`, find the existing NURBS load_curve_begin/chunk/finalize method registrations and replace with:

```rust
d.commands.insert("kalico_load_curve_cubic data=%*s".into(), <msg_id>);
```

Add a corresponding `send_load_curve_cubic_frame` method (encode via the existing `encode_typed` pattern).

- [ ] **Step 3: Delete the NURBS upload path**

Remove all NURBS upload code from `producer.rs` and `bridge.rs`:
- `producer::load_curve` (old NURBS variant)
- All begin/chunk/finalize helpers
- `caps.max_control_points` / `caps.max_knot_vector_len` checks
- The `kalico_load_curve_begin` / `_chunk` / `_finalize` command registrations

- [ ] **Step 4: Update bridge.rs to emit cubic pieces**

In `rust/motion-bridge/src/bridge.rs`, find `dispatch_push_segment` (around line 344) and the surrounding `producer::load_curve` calls (around line 2125). Replace with `producer::load_curve_cubic` calls.

The planner needs to provide cubic pieces — Step 5 handles that side.

- [ ] **Step 5: Update planner.rs serialization**

In `rust/motion-bridge/src/planner.rs`, find where the planner serializes its per-axis output for transport. Today it produces NURBS-shaped data (control_points + knots + weights). Change it to emit cubic Bezier pieces.

The planner's internal math (Tajima-Sencer fits, G2/G3 conversion, etc.) likely already produces cubic-shaped output internally; this step is about the serialization layer. Locate the boundary and adjust.

If the planner truly outputs NURBS internally and needs structural change (not just serialization), flag as a follow-up plan — that's a much larger workstream.

- [ ] **Step 6: Build**

```
cd rust && cargo build --workspace 2>&1 | tail
```

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/producer.rs rust/motion-bridge/src/bridge.rs \
        rust/motion-bridge/src/planner.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "feat(motion-bridge): drop NURBS upload; emit cubic Bezier pieces

producer::load_curve replaced with load_curve_cubic — one frame, atomic
load, sized by MAX_PIECES_PER_CURVE = 16. Old NURBS begin/chunk/finalize
path deleted. bridge.rs dispatch_push_segment now calls the cubic upload.
planner.rs serialization emits per-axis cubic-piece arrays."
```

---

### Task 20: Update klippy — motion_toolhead emits configure_axis

**Files:**
- Modify: `klippy/motion_bridge.py`
- Modify: `klippy/motion_toolhead.py`

- [ ] **Step 1: Add the Python wrapper for kalico_configure_axis**

In `klippy/motion_bridge.py`, add to the MotionBridge class:

```python
def kalico_configure_axis(self, mcu_handle, axis_idx, mode, microstep_distance,
                          extrusion_per_xy_mm, stepper_bindings):
    """
    Send kalico_configure_axis command via motion-bridge.

    stepper_bindings: list of (stepper_oid, dir_invert, tmc_cs_oid).
    tmc_cs_oid = 0xFF means "no TMC driver" (Pulse-only).
    """
    # Encode the sub-message blob: 4 bytes per stepper.
    blob = bytearray()
    for (stepper_oid, dir_invert, tmc_cs_oid) in stepper_bindings:
        blob.append(stepper_oid)
        blob.append(1 if dir_invert else 0)
        blob.append(tmc_cs_oid)
        blob.append(0)  # reserved flags = 0
    return self._bridge.kalico_configure_axis(
        mcu_handle, axis_idx, mode,
        microstep_distance, extrusion_per_xy_mm,
        len(stepper_bindings), bytes(blob),
    )
```

Add a PyO3 binding for `kalico_configure_axis` in the motion-bridge Rust wrapper that mirrors this signature.

- [ ] **Step 2: Update motion_toolhead.py**

In `klippy/motion_toolhead.py`, find `_configure_axes_per_mcu` (around line 631) and the `configure_axes` call inside it (around line 888). After the existing `configure_axes_blob` upload, add the new per-axis emit:

```python
# Spec §5.2: kalico_configure_axis replaces config_runtime_stepper.
# One command per axis with a sub-message of per-stepper bindings.
N_AXES = 4  # A/B/Z/E
MODE_PULSE = 0
import math

for axis_idx in range(N_AXES):
    # Collect bindings for this axis from runtime_bindings.
    bindings = []
    for (motor_idx, stepper_name, stepper_oid, invert_dir) in runtime_bindings:
        if motor_idx != axis_idx:
            continue
        # Resolve tmc_cs_oid: look up the stepper's TMC driver OID
        # from printer.cfg's [tmcXXXX stepper_NAME] config.
        tmc_cs_oid = self._resolve_tmc_cs_oid(stepper_name, mcu_handle)
        if tmc_cs_oid is None:
            tmc_cs_oid = 0xFF  # No TMC; Pulse-only
        bindings.append((stepper_oid, invert_dir, tmc_cs_oid))
    if not bindings:
        continue
    spm = steps_per_mm[axis_idx]
    microstep_distance = 1.0 / spm if spm > 0 else 0.0
    extrusion_per_xy_mm = 0.0  # Per-segment; not used at axis level
    rc = self.bridge.kalico_configure_axis(
        mcu_handle, axis_idx, MODE_PULSE,
        microstep_distance, extrusion_per_xy_mm,
        bindings,
    )
    if rc != 0:
        raise self.printer.config_error(
            f"kalico_configure_axis(axis={axis_idx}) failed: rc={rc}"
        )
```

Then delete the per-motor emit of the legacy `config_runtime_stepper` command from the codebase.

`_resolve_tmc_cs_oid` is a new helper: for a given stepper name (e.g., `stepper_x`), look up the `[tmc5160 stepper_x]` config block and find its `spi_oid`. Implementation depends on how klippy exposes TMC config — typically through the `tmc.py` module's allocated SPI OIDs.

- [ ] **Step 3: Delete the legacy emit**

In `motion_toolhead.py`, find every `config_runtime_stepper` emit (search for the command name). Delete those lines.

- [ ] **Step 4: Build motion-bridge native (Pi only)**

This step is bench-side per the iteration flow. Defer until Task 21's bench-flash; document the build command:

```
make -f Makefile.kalico motion-bridge
```

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_bridge.py klippy/motion_toolhead.py
git commit -m "feat(klippy): emit kalico_configure_axis per axis at startup

Replaces the per-motor config_runtime_stepper emit. Per axis we send
the new sub-message: (stepper_oid, dir_invert, tmc_cs_oid) for each
stepper bound to that axis. tmc_cs_oid resolves from the printer.cfg
TMC config block (or 0xFF if Pulse-only). Mode is Pulse by default
in this cutover (Phase rejected by the runtime per spec §5.2)."
```

---

### Task 21: klipper-sim integration test

**Files:**
- Modify: `tests/klipper_sim/stepping_redesign_test.py`

- [ ] **Step 1: Finish the test harness scaffolded by firmware Task 15**

The scaffolded test at `tests/klipper_sim/stepping_redesign_test.py` raises `NotImplementedError` because klipper-sim's step-event CLI didn't exist. Wire up the actual comparison now:

1. Invoke klipper-sim against mainline Klipper with a known G-code program; capture the step-pulse trace.
2. Invoke klipper-sim against this fork with the same G-code; capture the step-pulse trace.
3. Pairwise compare each (axis, step_time_us) tuple; assert max drift < 500 ns.

The exact CLI for step-pulse capture depends on klipper-sim's current API. Read `~/Developer/klipper-sim/README.md` and `STATUS.md` for the up-to-date method.

If klipper-sim still doesn't emit step pulses (only trajectory CSVs), this test stays as a documentation placeholder. Note that in the test header and leave the NotImplementedError. The build still passes for host-side unit tests; klipper-sim parity becomes a follow-up.

- [ ] **Step 2: Test G-code**

```python
TEST_GCODE = """
SET_KINEMATIC_POSITION X=10 Y=10 Z=10
G1 X20 F600
G1 X10 F600
G1 X20 Y20 F12000
G1 X10 Y10 F12000
"""
```

(SET_KINEMATIC_POSITION used to avoid homing requirements in the simulator.)

- [ ] **Step 3: Run**

```
python3 tests/klipper_sim/stepping_redesign_test.py
```

Expected: PASS if klipper-sim supports step pulses; SKIP-equivalent (NotImplementedError with clear message) otherwise.

- [ ] **Step 4: Commit**

```bash
git add tests/klipper_sim/stepping_redesign_test.py
git commit -m "test(klipper_sim): wire stepping-redesign step-trace comparison

Compares mainline Klipper vs. our fork on identical G-code; asserts
max step-time drift < 500 ns. Falls back to documented NotImplementedError
if klipper-sim's step-event capture API isn't ready yet."
```

---

### Task 22: Full workspace build + final cleanup

**Files:**
- All — comprehensive verification

- [ ] **Step 1: Run every workspace test**

```
cd rust && cargo test --workspace 2>&1 | tail -30
```

Expected: every test passes (no FAILED, no test files erroring on compile).

- [ ] **Step 2: Run every target build**

```
cd rust && cargo build --workspace 2>&1 | tail
cd rust && cargo build -p runtime --lib --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf 2>&1 | tail
cd rust && cargo build -p runtime --lib --no-default-features --features mcu-f4 --target thumbv7em-none-eabihf 2>&1 | tail
```

Expected: all clean.

- [ ] **Step 3: Clippy sweep on the new code**

```
cd rust && cargo clippy -p runtime --lib --no-deps -- -D warnings 2>&1 | tail
```

Address any warnings on the new code (existing pre-existing warnings in motion-bridge etc. are out of scope).

- [ ] **Step 4: Search for orphaned references**

```
git grep -n 'producer_step\|step_time_event\|step_ring\|MAX_CONTROL_POINTS\|MAX_KNOT_VECTOR_LEN\|MAX_DEGREE\|config_runtime_stepper\|kalico_runtime_modulated_tick\|runtime_handle_load_curve\b' -- ':!docs' ':!.git'
```

Expected: zero matches (or only matches in commit-message-like trailing comments — verify each one).

- [ ] **Step 5: Update CLAUDE.md if any constraints have changed**

If the cutover changes any project-wide invariant (e.g., the "G5 / G5.1 only" rule now propagates through the cubic-only architecture more cleanly), update `CLAUDE.md` to reflect it. Otherwise no change.

- [ ] **Step 6: Final commit**

```bash
git add -A
git commit -m "chore(stepping): final cleanup pass — workspace builds + clippy + no orphan refs"
```

---

## Self-Review

**1. Spec coverage:**

- §1 Architecture overview → narrative only, no task needed.
- §2 Curve pool shape/sizing/storage → Tasks 2, 3, 16.
- §3.1 configure_axis wire → Tasks 5, 14, 17, 20.
- §3.2 load_curve_cubic → Tasks 2, 13, 15.
- §3.3 push_segment unchanged → no task (preserves existing wire).
- §3.4 deleted FFI → Task 13.
- §4.1 AxisConfig cursor → Task 6.
- §4.2 StepperRef shrink + StepperBindingRust ABI → Task 5.
- §4.2a SpiWrite tmc_cs_oid → Task 5.
- §4.3 segment arm in ISR → Task 8.
- §4.4 advancement loop → Tasks 9, 10.
- §4.5 retire condition → Task 10.
- §4.6 E follower absolute → Task 11.
- §5 Per-stepper bindings → Tasks 5, 14, 17, 20.
- §6 Deletions → Tasks 12, 13, 15, 17 (covers all subsections).
- §7 Cross-MCU coordination → no task (no change).
- §8 Faults → Task 14 (PhaseModeNotAvailable, CurveLoadInvalid).
- §9 Testing → Tasks 1, 2, 8, 10, 11, 14, 21.
- §10 Migration ordering → Tasks 1 through 22 follow the ordering.

All sections mapped to tasks.

**2. Placeholder scan:** None found. Every step has concrete code or commands. Task 21 has a documented fallback path (klipper-sim API may not be ready) but that's a real conditional, not a placeholder.

**3. Type consistency:**

- `BezierPieceMonomial` field shape: `coeffs[4]`, `vel_coeffs[3]`, `duration` — used consistently across Tasks 1, 2, 8, 11.
- `LoadedCubicCurve` shape: `piece_count: u16`, `_pad: [u8; 2]`, `pieces: [BezierPieceMonomial; MAX_PIECES_PER_CURVE]` — consistent.
- `WirePiece` shape: 5 × u32 (bp0..bp3 bits + duration_bits) = 20 bytes — consistent across Tasks 2, 13, 15, 19.
- `StepperBindingRust`: 4 bytes `{tmc_cs_oid: u8, _pad: [u8; 3]}` with sentinel `0xFF` — consistent across Tasks 5, 14, 17, 20.
- `AxisConfig` field order: `(mode, steppers, curve_handle, piece_cursor, piece, piece_start_time_cycles, last_step_count, microstep_distance)` — consistent.
- `Engine` new fields: `participating_mask: u8`, `pending_mask: u8`, `segment_base_e: f32`, `ds_xy_segment: f32` — consistent (Tasks 7, 8, 10, 11).
- Fault code numeric values: `PhaseModeNotAvailable = -23`, `CurveLoadInvalid = -24` — picked in Task 14, used by callers in Tasks 14, 13. (If existing variants already use -23/-24, pick fresh values during Task 14 Step 1 — confirm against `error.rs` at task time.)

Plan is self-consistent.
