# Layer 4 — MCU Framework with Stub NURBS Evaluator

**Date:** 2026-04-28
**Status:** Spec — design under brainstorm review; implementation plan to follow on green-light
**Layer:** 4 (MCU runtime — partial)
**Driver:** Build-order Step 5 — "MCU framework with stub NURBS evaluator and basic kinematics — partial Layer 4, with the runtime-evaluation slots designed in even if unused."

---

## 1. Context

Layer 4 is the MCU runtime. CLAUDE.md describes it in five bullets:

1. **Real-time MCU framework** — sample-rate clock at 40 kHz, segment buffer holding 2–3 adjacent segments for shaper-boundary handling.
2. **Per-axis evaluator** — composes (in order) base/pre-shaped NURBS evaluation, kinematic transform (CoreXY/Cartesian), runtime PA tanh evaluation if applicable, runtime shaper application if applicable (only for E with nonlinear PA).
3. **Phase-stepping current synthesis** — TMC5160 XDIRECT writes for 5160-class drivers. ⇒ Step 10.
4. **Hybrid stepping** — step/dir for non-phase-capable drivers. ⇒ Step 7 MVP.
5. **Skip detection** — MSCNT / encoder reads at ~100 Hz. ⇒ Step 11.

Step 5 ships the **framework + per-axis evaluator** with all output stages stubbed to a trace ring. Steps 7/9/10/11 fill in the slots. The framework's commitment is **architectural**: the runtime-eval composition (NURBS → kinematics → PA → IS → output) lives in the type system from day one so later steps additively replace `Noop` slots without restructuring the ISR.

The runtime targets the **STM32H723** on the BTT Octopus Pro (primary phase-stepping target per CLAUDE.md). The F4x on the Octopus (Z-only via TMC2209) is not the Step 5 bring-up target; the Rust crate compiles for both via `mcu-h7`/`mcu-f4` features but only H723 is integration-tested at Step 5. Multi-MCU coordination is Step 6's concern.

What this spec does not cover (deferred to later build-order steps):

- Live host↔MCU comms protocol — Step 6.
- Step/dir GPIO output — Step 7 MVP.
- Real Layer 1/2/3 input — Step 7 MVP wires real planner output; Step 5 uses test-harness-synthesized segments.
- Smooth-shaper convolution — Step 8.
- Tanh PA runtime evaluation — Step 9.
- Phase-stepping current synthesis — Step 10.
- Skip detection acquisition — Step 11.
- F4x integration testing — Step 6+ (multi-MCU bring-up).

### 1.1 Driving constraints (inherited)

- **Rust end-to-end** for new code; single source compiled f64 host / **f32 MCU**. Rust links as staticlib into Klipper's existing C MCU build, which stays C.
- **NURBS-native** internal primitive. Step 5 evaluates 3D NURBS in (X, Y, E) at f32 on the MCU.
- **40 kHz modulation rate** for true phase stepping. Validated as feasible on H723 by the Prusa Buddy precedent (`docs/research/open-loop-phase-stepping-prior-art.md`, `docs/research/tmc5160-open-loop-phase-stepping.md`). Step 5's ISR is far lighter than Prusa's (no SPI, no LUT, no current synthesis), so cycle headroom is comfortable; running the full 40 kHz tick from day one validates the framework against the eventual Step 10 budget.
- **MCU receives the shape with PA and IS already baked in** (XY pre-shaped) **except** when nonlinear PA is on the extruder (then runtime IS+PA on E). Step 5's `Engine` is generic over PA/IS slots; both default to `Noop` with `Engine<P, I>` instantiated as `Engine<NoopPa, NoopIs>` at Step 5 build time.
- **Explicit position/step decoupling.** Step 5's evaluator emits motor *positions*; the output stage (step/dir at Step 7, XDIRECT at Step 10) is a separate concern.
- **Per-axis offsets applied outside the planner** (bed mesh, thermal, probing). Out of Step 5 scope.

### 1.2 Non-goals

- **Real comms protocol.** No `kalico_runtime_handle_packet` FFI; Step 6 designs the wire format.
- **Multi-MCU clock sync.** Single H723 at Step 5; Step 6.
- **Phase-stepping output** (TMC5160 XDIRECT). Output stage at Step 5 is the trace ring only.
- **Step/dir output.** Step 7 MVP fills this slot.
- **Production-grade test coverage.** Step 5 ships the framework with first-light validation; full MVP test surface lands at Step 7.
- **Automatic recovery from FAULT.** Latch + report; manual reset for Step 5. No `kalico_runtime_reset()` FFI.
- **Allocator on MCU.** All MCU-side data lives in `static` storage; no `alloc`, no `Box`, no `Vec`.

---

## 2. Architecture

### 2.1 Build-system shape

A single Rust staticlib (`libkalico.a`) links into Klipper's existing C MCU build. Klipper's foreground super-loop continues to run `DECL_TASK` work (USB-CDC RX, command dispatch, telemetry); the kalico runtime adds a **40 kHz hardware-timer ISR (TIM3) that preempts the foreground every 25 µs**. ISR must be tightly bounded — at 3 µs, foreground retains 88% of cycles for Klipper tasks; at 15 µs, 40%. Step 5's paper estimate is ~5.5–7.3 µs / tick; Surface C measures the actual.

The ISR fires, calls into Rust to evaluate the current NURBS segment, mixes to motor coordinates (CoreXY for AB, identity for E), writes three samples (motor_a, motor_b, motor_e) to a trace ring, returns. Step 5 has no real output stage — the trace ring **is** the output. A debug command pulls trace from host over Klipper's existing USB-CDC framing.

### 2.2 Workspace layout

The current workspace is `nurbs/, nurbs-c-api/, gcode/, geometry/, temporal/`. Step 5 adds one new pure-Rust crate (`runtime/`) and renames the FFI staticlib (`nurbs-c-api/` → `kalico-c-api/`):

```
rust/
  nurbs/        (existing — Layer 0 substrate)
  geometry/     (existing — Segment types from Layer 1)
  gcode/        (existing)
  temporal/     (existing — Layer 2)
  runtime/      (NEW — Step 5 logic: SegmentQueue, Engine, TraceRing, kinematics)
  kalico-c-api/ (RENAMED from nurbs-c-api — umbrella staticlib + FFI surface)
```

Reasons recorded from brainstorming + Codex review:

- **Single staticlib pattern is required.** The Rust linkage docs explicitly note that multiple `staticlib` crates linked into one foreign binary "are likely to conflict" (panic_handler duplication, eh_personality duplication, runtime symbol overlap). One umbrella staticlib that depends on multiple Rust `rlib`s is the documented path.
- **Rename now is cheaper than later.** `nurbs-c-api` was only meaningful while the FFI surface was nurbs-only; with `kalico_runtime_*` symbols added, the name lies. Rename-to-`kalico-c-api` touches one Cargo.toml, the cbindgen config, the gen-headers binary, and the Klipper Makefile reference. Cheap with one downstream consumer; expensive once Steps 6/7/8/9/10 each link against it.
- **Two cbindgen-generated headers** from the same crate — `kalico_nurbs.h` (existing, regenerated) and `kalico_runtime.h` (new). cbindgen has no prefix-filter mode, so the gen-headers binary runs cbindgen twice with different `cfg` flags (`header-nurbs` / `header-runtime`) gating which FFI module is exposed.
- **One `#[panic_handler]`** in `kalico-c-api/src/lib.rs`, gated on `not(feature = "host")`, loops forever (watchdog handles reset). `panic = "abort"` is already in the workspace `[profile.release]`.

### 2.3 Build invocation

```sh
# MCU (H7) build:
cargo build -p kalico-c-api \
  --no-default-features --features mcu-h7 \
  --target thumbv7em-none-eabihf \
  --release

# Host build (for unit tests + FFI integration tests):
cargo build -p kalico-c-api  # uses default features (host)
cargo test -p runtime --features host
```

`--no-default-features` avoids workspace feature unification across host and MCU profiles (the resolver-v2 unification rule still applies if you `--workspace`-build; the `-p` + `--no-default-features` invocation isolates it).

### 2.4 Klipper-side files

Two C files added under `src/`, matching Klipper convention (per-MCU code under `src/<arch>/`, portable code under `src/`):

- **`src/stm32/kalico_h7_timer.c`** — H723-specific TIM3 init (auto-reload at `SystemCoreClock / 40000`), NVIC priority registration (below SysTick / Klipper's core-motion timer; above USB only if ISR is proven sub-µs after Surface C measurement), the IRQ handler that:
  1. Acks `TIM3->SR.UIF` immediately on entry (entry-time ack — avoids tail-chain re-entry / starvation per Codex review).
  2. Reads `DWT->CYCCNT` into `now_clock` (u64 widened from 32-bit cycle counter).
  3. Calls `kalico_runtime_tick(rt, now_clock)`.
  4. Returns. (No late ack.)

- **`src/runtime_tick.c`** — portable: `DECL_INIT(runtime_init)` calls `kalico_runtime_init`; `DECL_TASK(runtime_drain)` periodically calls `kalico_runtime_drain_trace` and ships samples over USB-CDC; `DECL_COMMAND(...)` exposes the test-harness commands (load_curve, push_segment, query_status, drain_trace). Foreground only — never preempts.

Klipper Makefile gets:
- `src/stm32/kalico_h7_timer.c` and `src/runtime_tick.c` added to `src-y` under appropriate Kconfig gates.
- Link line append: `libkalico.a` placed **after** C objects that reference `kalico_runtime_*` symbols, **before** `-lgcc -lc_nano`. (Archive extraction is demand-driven and order-sensitive in the Klipper toolchain's GCC link.)
- New Kconfig `CONFIG_KALICO_RUNTIME` (default off; enabled for H7 builds in Step 5; F4x build available via Cargo features but not exercised at Step 5).

Linker pitfalls to watch (per Codex review):
- Klipper builds with `-flto=auto -fwhole-program -fno-use-linker-plugin --gc-sections`. Use **native Rust staticlib output** (not Rust linker-plugin LTO), `panic = "abort"` (set), hard-float ABI matched (`thumbv7em-none-eabihf` matches Klipper's hard-float on H7).
- No `compiler-builtins` / `libgcc` symbol overlap. Rust pulls a minimal compiler-builtins set; Klipper's `-lgcc` covers the rest.
- No GC'd symbols. Anything reachable only via reflection (e.g., panic info string) needs `#[used]`; Step 5's FFI surface is all `extern "C" fn` so this is moot, but document the rule for future symbols.

---

## 3. Components

### 3.1 `rust/runtime/` (new no_std crate)

```
runtime/
  Cargo.toml       # depends on nurbs (no_std-capable), heapless 0.8
  src/
    lib.rs         # #![no_std] root, #[deny(...)] lint policy
    queue.rs       # SegmentQueue: facade over heapless::spsc::Queue<Segment, 8>
    segment.rs     # Segment + KinematicTag types
    curve_pool.rs  # static slab of NURBS curves; CurveHandle indexes it
    engine.rs      # Engine<P: PaSlot, I: IsSlot>; tick(), boundary loop
    kinematics.rs  # CoreXY transform; identity pass-through; const tags
    slot.rs        # PaSlot/IsSlot traits operating on TickState; ZST Noop impls
    trace.rs       # TraceRing<1024> with drop-newest + overflow flag
    state.rs       # TickState (xyz_e, motors, dt) — passed through slot pipeline
    clock.rs       # widening u32→u64 for the cycle counter; cycle math helpers
    error.rs       # internal RuntimeError enum (FFI maps to i32)
```

Key types and rationale:

- **`SegmentQueue`** wraps `heapless::spsc::Queue<Segment, 8>` (effective capacity 7 per heapless 0.8's `N-1` rule). Producer/Consumer halves owned by the foreground (test harness at Step 5; comms task at Step 6+) and the ISR (`Engine::tick`) respectively. heapless's atomics ship correct for ARMv7-M; Step 5 doesn't validate them under concurrency, but they ship with proven correctness from the library.
- **`Segment { id: u32, curve: CurveHandle, t_start: u64, t_end: u64, kinematics: KinematicTag }`** — small POD; no inline curve data. `enqueue` / `dequeue` `memcpy`s the segment, so keeping it small minimizes ISR-boundary cost.
- **`CurveHandle`** is a small index (u16) into a static slab `static CURVE_POOL: CurvePool<{ CURVE_POOL_N }>`. The slab owns the NURBS data (control points, knots, weights). `CURVE_POOL_N = 16` at Step 5 (see §7 open question 1; revisited at Step 7 MVP). Producer-side rule: a curve must be fully loaded into a slab slot **before** any Segment referencing it is pushed onto the queue. ISR-side rule: handles trusted; no validation in the hot path.
- **`Engine<P: PaSlot, I: IsSlot>`** is the per-axis evaluator. Generic over the two slot types. Concrete instantiation chosen at compile time via Cargo features:
  ```rust
  #[cfg(feature = "pa-tanh")]      type Pa = TanhPa;
  #[cfg(not(feature = "pa-tanh"))] type Pa = NoopPa;
  #[cfg(feature = "input-shaper")] type Is = SmoothShaper;
  #[cfg(not(feature = "input-shaper"))] type Is = NoopIs;
  pub type RuntimeEngine = Engine<Pa, Is>;
  ```
  Step 5 builds with neither feature → `Engine<NoopPa, NoopIs>`. Step 8 enables `input-shaper`; Step 9 enables `pa-tanh`. C-side sees one opaque handle (`*mut KalicoRuntime`); no runtime config branching in the ISR.
- **`PaSlot` / `IsSlot` traits** operate on a compact `TickState` and have ZST `Noop` impls:
  ```rust
  pub struct TickState { pub dt: f32, pub xyz_e: [f32; 3], pub motors: [f32; 3] }
  pub trait PaSlot { #[inline(always)] fn apply(&mut self, _: &mut TickState) {} }
  pub struct NoopPa;
  impl PaSlot for NoopPa {}
  ```
  Optimizer fully removes Noop branches; no runtime overhead vs. open-coding `if has_pa { ... }`.
- **`TraceRing<1024>`** is an SPSC ring (heapless::spsc::Queue underlying), ISR producer / foreground consumer. Sample = `{ tick: u64, motor_a: f32, motor_b: f32, motor_e: f32, segment_id: u32, flags: u8 }`. ~32 bytes / sample × 1024 = 32 KB. Drop-newest policy with overflow flag carried into the **next** successfully enqueued sample (heapless::spsc doesn't permit modifying an already-enqueued item; see §4.3 trace-overflow protocol).
- **Memory placement**: `TraceRing` storage in DTCM (CPU-only access at Step 5; no DMA touches it). Future DMA-driven trace shipping (Step 6+) would relocate to AXI SRAM. `CurvePool` storage in DTCM for fast ISR access. `SegmentQueue` storage in DTCM. Stack allocation per ISR call (de Boor workspace) implicitly DTCM via Klipper's existing stack placement.

### 3.2 `rust/kalico-c-api/` (renamed from `nurbs-c-api`)

```
kalico-c-api/
  Cargo.toml          # name = "kalico-c-api"; crate-type = ["staticlib", "rlib"]
  cbindgen.toml       # existing — kalico_nurbs.h
  cbindgen-runtime.toml  # NEW — kalico_runtime.h
  include/
    kalico_nurbs.h    # existing, regenerated post-rename
    kalico_runtime.h  # NEW
  src/
    lib.rs            # crate root, panic_handler, init-once cell, FFI re-exports
    nurbs_ffi.rs      # cfg(feature = "header-nurbs") — existing kalico_nurbs_*
    runtime_ffi.rs    # cfg(feature = "header-runtime") — NEW kalico_runtime_*
    bin/
      gen_headers.rs  # runs cbindgen twice with different cfg flags
```

FFI surface (all `extern "C"`, all return `i32` or opaque pointers; no panics, no Rust types crossing):

```rust
// Init-once. Returns null on second call, valid handle on first.
pub unsafe extern "C" fn kalico_runtime_init(...) -> *mut KalicoRuntime;

// Producer-side. Foreground calls these.
pub unsafe extern "C" fn kalico_runtime_load_curve(
    rt: *mut KalicoRuntime,
    slot_idx: u16,
    control_points: *const f32, n_cp: u16,
    knots: *const f32, n_knots: u16,
    weights: *const f32, n_weights: u16,
    degree: u8,
) -> i32;

pub unsafe extern "C" fn kalico_runtime_push_segment(
    rt: *mut KalicoRuntime,
    id: u32, curve_handle: u16,
    t_start: u64, t_end: u64,
    kinematics: u8,
) -> i32;

// ISR entrypoint. C-side ISR shim guarantees rt is non-null.
pub unsafe extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, now_clock: u64);

// Foreground drain.
pub unsafe extern "C" fn kalico_runtime_drain_trace(
    rt: *mut KalicoRuntime,
    out_buf: *mut TraceSample, out_cap: u32,
) -> u32;

// Status / diagnostics.
pub unsafe extern "C" fn kalico_runtime_status(rt: *mut KalicoRuntime) -> u8;
pub unsafe extern "C" fn kalico_runtime_last_error(rt: *mut KalicoRuntime) -> i32;
```

`KalicoRuntime` is opaque to C; the concrete type is `RuntimeEngine` chosen by Cargo features. Storage:

```rust
struct RuntimeCell(core::cell::UnsafeCell<core::mem::MaybeUninit<RuntimeEngine>>);
unsafe impl Sync for RuntimeCell {}

static RT: RuntimeCell = RuntimeCell(core::cell::UnsafeCell::new(MaybeUninit::uninit()));
static RT_STATE: AtomicU8 = AtomicU8::new(STATE_UNINIT);
const STATE_UNINIT: u8 = 0;
const STATE_INITING: u8 = 1;
const STATE_READY: u8 = 2;
```

Init-once protocol: `compare_exchange(UNINIT, INITING, AcqRel, Acquire)` on entry; initialize the `MaybeUninit` payload; `store(READY, Release)` on exit. Readers `load(Acquire)` and observe READY before touching the cell. Catches partial-init reads under the (paranoid) two-thread C-side scenario.

### 3.3 Workspace edits

```toml
# rust/Cargo.toml
[workspace]
members = [
  "nurbs", "kalico-c-api", "gcode", "geometry", "temporal", "runtime"
]
exclude = ["gcode/fuzz"]
resolver = "2"

[workspace.dependencies]
heapless = { version = "0.8", default-features = false }   # NEW
# (existing: thiserror, clarabel, ...)
```

Workspace edition: migrate from 2021 to **2024** as a prep commit before Step 5 work begins (mechanical via `cargo fix --edition`). Required for `#[unsafe(no_mangle)]` and `unsafe extern { ... }` blocks the FFI surface uses.

---

## 4. Data flow

### 4.1 Time-unit contract

`now`, `t_start`, `t_end` are all in **MCU clock cycles** (H723 core clock, 550 MHz, u64 monotonic). u64 wraps in ~1063 years — irrelevant. The `DWT->CYCCNT` register is 32-bit and wraps every ~7.8 s at 550 MHz; widening to u64 happens in the **Rust** runtime (`runtime/src/clock.rs::widen_cyccnt`), called once per tick from the ISR with the previous u64 value carried in `static MONOTONIC_NOW: AtomicU64`. The C-side shim reads the raw 32-bit register; widening lives in Rust to keep the wrap-handling invariant testable on the host. One unit, one type, no ambiguity. Field doc strings name the unit.

### 4.2 Hot path: 40 kHz ISR tick

```
TIM3 update IRQ fires →
  C wrapper:  ack SR.UIF; read DWT->CYCCNT into now_clock (u64); call kalico_runtime_tick(rt, now_clock); return.
  Rust:       Engine::tick(now)
```

`Engine::tick(now: u64) -> Result<(), RuntimeError>`:

1. **Queue + idle check.** If consumer is empty AND `current` is None → `status.store(IDLE, Release)`; **self-disable TIM3** by clearing `CR1.CEN` (Rust calls a tiny `extern "C" fn kalico_h7_disable_tim3()` helper); return `Ok(())`. Foreground re-enables on next push.

2. **Segment activation.** If `current` is None and queue has a segment → `consumer.dequeue()` into `current`; `current.t_start = now` (or inherited from boundary carry; see step 3).

3. **Sub-tick boundary loop (bounded by queue depth).**
   ```
   t_segment = now - current.t_start
   while t_segment >= duration(current):
       Δt = t_segment - duration(current)
       drop current
       if queue empty:
           goto step 1's idle path; return Ok
       current = consumer.dequeue()
       current.t_start = now - Δt          # invariant: t_segment = now - t_start = Δt
       t_segment = Δt
   if loop iterated queue-depth times without resolving:
       latch FAULT; return
   ```
   Producer rejects segments shorter than `MIN_SEGMENT_CYCLES` (default `2 * ONE_TICK_CYCLES` = 27,500 cycles ≈ 50 µs of motion). Defense in depth ensures the loop never iterates the full queue depth in normal operation.

4. **Curve evaluation.** `let curve = curve_pool.resolve(current.curve_handle);` `let u = clamp(t_segment / duration(current), 0.0, 1.0);` `let xyz_e = nurbs::vector_eval(curve, u);` → `[f32; 3]`.
   **Spec invariant**: the runtime ISR ASSUMES time-parameterized input segments. Any arc-length-to-u inverse solve happens at Layer 3, never in the ISR. At Step 5 the test harness synthesizes already-time-parameterized NURBS; at Step 7 MVP, Layer 3's `time_reparameterize` produces the same shape.

5. **NaN/Inf check.** `if !xyz_e.iter().all(f32::is_finite) { latch FAULT; return; }` — IEEE-754 quiet NaN/Inf can arise from `0/0` divisions, NURBS overflow at high control-point magnitudes, zero-derivative reparameterization edge cases. Cortex-M7 FPv5 produces these quietly without trapping unless FPSCR is configured to trap (it's not). Per-axis cost measured in Surface C; paper estimate ~10 cycles total.

6. **Slot pipeline.**
   ```rust
   let mut state = TickState { dt: 1.0/40_000.0, xyz_e, motors: [0.0; 3] };
   pa_slot.apply(&mut state);  // NoopPa at Step 5: optimized out
   is_slot.apply(&mut state);  // NoopIs at Step 5: optimized out
   ```

7. **Kinematic transform.** `state.motors = [state.xyz_e[0] + state.xyz_e[1], state.xyz_e[0] - state.xyz_e[1], state.xyz_e[2]]` (CoreXY for AB; identity for E).

8. **Trace emit.** Sample `flags` is `OR`'d with the value of the `trace_overflow_pending` flag (which is cleared on successful enqueue). `flags |= SEGMENT_END` is set on the sample whose `tick` value will be the *last* one emitted from the current segment — detected by checking `t_segment + ONE_TICK_CYCLES >= duration(current)` (i.e., the next tick will trigger the boundary loop in step 3). Push `TraceSample { tick: now, motor_a, motor_b, motor_e, segment_id: current.id, flags }` via `producer.enqueue()`. On full → set `trace_overflow_pending = true`, sample dropped. See §4.3 for the protocol.

`ONE_TICK_CYCLES = 550_000_000 / 40_000 = 13_750` cycles; defined as a `const` in `runtime/src/clock.rs`.

9. **Tick counter** (liveness heartbeat per §6.3): `tick_counter.fetch_add(1, Relaxed)`.

10. **Status update.** `status.store(RUNNING, Release)`. Return `Ok(())`.

### 4.3 Trace-overflow protocol

heapless::spsc forbids modifying an already-enqueued item (the consumer may concurrently dequeue). Instead:

```rust
struct TraceState {
    overflow_pending: AtomicBool,  // ISR sets on full; ISR clears on next successful enqueue
}
```

ISR producer side:
```rust
let flags = base_flags | if overflow_pending.load(Relaxed) { OVERFLOW } else { 0 };
match producer.enqueue(TraceSample { tick, motor_a, motor_b, motor_e, segment_id, flags }) {
    Ok(()) => overflow_pending.store(false, Relaxed),  // carry consumed
    Err(_) => overflow_pending.store(true, Relaxed),   // sample dropped, mark
}
```

Foreground consumer side (drain):
```rust
let drained = consumer.drain_into(out_buf, out_cap);
if drained == 0 && overflow_pending.load(Relaxed) {
    // Synthetic OVERFLOW marker for host so the gap is visible even if drain returned empty.
    emit_synthetic_overflow_marker_to_usb_cdc();
}
```

`Relaxed` ordering is correct: the ISR–foreground synchronization for trace contents is via the SPSC queue itself (heapless's atomics handle ordering). The bool's role is purely advisory for "did I drop a sample since the last successful enqueue."

### 4.4 Cold path A: producer side (foreground)

```
test_harness or DECL_COMMAND handler:
  → kalico_runtime_load_curve(rt, slot_idx, ...)        → 0/-1
  → kalico_runtime_push_segment(rt, id, curve_handle, t_start, t_end, kinematics) → 0/-1
     - rejects if t_end - t_start < MIN_SEGMENT_CYCLES
     - rejects if curve_handle is unloaded
     - on first successful push when status was IDLE: re-enables TIM3
       (foreground sets CR1.CEN; clears UIF first to avoid spurious immediate fire)
```

Curve-load-before-segment-push invariant is producer-enforced.

### 4.5 Cold path B: trace drain (foreground)

```
DECL_TASK(runtime_drain) — Klipper super-loop:
  → kalico_runtime_drain_trace(rt, out_buf, out_cap) → count
  → klipper ships samples over USB-CDC via existing command framing
```

Burst behavior: drain pulls whatever's in the ring (up to `out_cap`); if foreground is slow, samples accumulate (up to 1024 = ~25 ms at 40 kHz) before drop-newest kicks in. Host validation strategy: pull continuously; expect zero OVERFLOW flags during a normal trace.

### 4.6 Cold path C: status query

```
DECL_COMMAND(query_kalico_status):
  → kalico_runtime_status(rt) → u8 (IDLE / RUNNING / DRAINED / FAULT)
  → Acquire-load of RT_STATE; foreground infers trace visibility from RUNNING / FAULT
```

### 4.7 Concurrency invariants

| Object                         | Producer            | Consumer        | Mechanism                                      |
|--------------------------------|---------------------|-----------------|------------------------------------------------|
| `SegmentQueue`                 | foreground          | ISR             | heapless::spsc (atomics ship correct on M7)    |
| `TraceRing`                    | ISR                 | foreground      | heapless::spsc                                 |
| `status: AtomicU8`             | ISR                 | foreground      | Release / Acquire                              |
| `last_error: AtomicI32`        | ISR                 | foreground      | Release / Acquire                              |
| `tick_counter: AtomicU64`      | ISR                 | foreground      | Relaxed (advisory)                             |
| `trace_overflow_pending: AtomicBool` | ISR (RMW)     | foreground (R)  | Relaxed; carried into next sample              |
| `CurvePool` slot               | foreground (W)      | ISR (R)         | producer rule: full-load before referencing    |
| `TIM3 enable bit`              | ISR (clear) + FG (set) | —            | mutex via status: FG sets only when IDLE/DRAINED |

### 4.8 Cycle budget (paper estimate; measured in Surface C)

H723 @ 550 MHz with `-O3 + LTO`:
- Queue check + bookkeeping: ~100–400 cycles
- `nurbs::vector_eval` degree-3 3D rational: estimated ~3000 cycles (degree-3 de Boor × 3 axes + weight handling). **Conservative**.
- CoreXY transform + slot Noop ZSTs: ~10–30 cycles
- Trace push + status atomic + tick counter: ~30–60 cycles
- NaN check (3 axes): ~10 cycles

Total: ~3000–4000 cycles ≈ **5.5–7.3 µs / tick**. Foreground retains 70%+ for Klipper tasks. **Caveat**: paper estimate. Surface C measures actual; if real cost exceeds 15 µs, the framework still functions but bites into Klipper's task budget — investigate before Step 7 MVP.

---

## 5. Error handling

### 5.1 Error taxonomy

- **Producer-side errors** (foreground, FFI): rejected at the FFI boundary, return `i32` per the table below.
- **ISR-side errors**: latched FAULT in `RT_STATE`, code stored in `last_error: AtomicI32`, ISR self-disables TIM3.
- **System-level errors** (HardFault, BusFault, MemManage, UsageFault): handled by Klipper's existing C-side handlers in `armcm_boot.c`. Rust runtime relies on them; no Rust-side hardware-fault handler.

### 5.2 FFI return codes

```c
#define KALICO_OK                     0
#define KALICO_ERR_QUEUE_FULL        -1
#define KALICO_ERR_INVALID_CURVE     -2   // bad degree, knot vector, NaN/Inf in CPs
#define KALICO_ERR_INVALID_HANDLE    -3   // out of bounds or unloaded
#define KALICO_ERR_INVALID_DURATION  -4   // t_end <= t_start, or below MIN_SEGMENT_CYCLES
#define KALICO_ERR_INVALID_KINEMATICS -5
#define KALICO_ERR_NULL_PTR          -6
#define KALICO_ERR_NOT_INIT          -7
#define KALICO_ERR_FAULT_LATCHED     -8
```

Internal `RuntimeError` enum maps to `i32` via explicit `From<RuntimeError> for i32`. Never crosses FFI as a Rust type.

### 5.3 Panic exclusion

Single `#[panic_handler]` in `kalico-c-api/src/lib.rs`, `cfg(not(feature = "host"))`:

```rust
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop { core::hint::spin_loop(); }
}
```

`panic = "abort"` set in workspace `[profile.release]`. Watchdog reset is the recovery mechanism — Klipper's existing C-side watchdog config covers it.

**Lint policy on the runtime crate** (more aggressive than `unwrap_used` alone):
```rust
#![deny(
    clippy::panic, clippy::unwrap_used, clippy::expect_used,
    clippy::indexing_slicing,        // catches arr[i] panic on out-of-bounds
    clippy::panic_in_result_fn,
    clippy::todo, clippy::unimplemented, clippy::unreachable,
    clippy::integer_division,        // catches int div-by-zero panic
    unsafe_op_in_unsafe_fn,          // Rust-2024-aligned
)]
```

ISR-callable code paths use `.get()` / `.checked_*()` / `.try_into()` on indexing, division, and conversions. Every fallible path returns `Result<_, RuntimeError>`; the ISR maps errors to FAULT latch + return.

**LLVM-IR audit at Step 5 completion**: `cargo rustc -p kalico-c-api --release --target thumbv7em-none-eabihf --features mcu-h7 -- --emit=llvm-ir`, grep the resulting IR for `core::panicking` / `panic_bounds_check` / `_ZN4core9panicking` symbols. Zero hits proves the panic-exclusion invariant for that build configuration. (CI gate.)

### 5.4 NaN / Inf guard

After every `vector_eval`, the ISR checks `xyz_e.iter().all(f32::is_finite)`. Non-finite → latch FAULT. Necessary even with producer-side validation: NaN/Inf can arise from valid finite inputs through `0.0/0.0`, NURBS arithmetic overflow, and zero-derivative reparameterization. Cost measured in Surface C; paper estimate ~10 cycles.

### 5.5 FAULT semantics

On FAULT latch (sub-tick boundary loop exhaustion, NaN/Inf from eval, invalid CurveHandle resolved at ISR time, internal invariant violation):
1. `last_error.store(code, Release)`.
2. `status.store(FAULT, Release)`.
3. Push **one** trace sample with `flags = FAULT_MARKER` and motor positions = last-known-good (NOT zero — zero looks like valid commanded motion, masking the fault on host plots).
4. Self-disable TIM3 (same path as idle-disable).
5. Foreground polls FAULT, queries `last_error`, reports to host.
6. **No automatic recovery at Step 5.** Manual harness restart.

### 5.6 FFI null-ptr guards

Every FFI taking `*mut KalicoRuntime` checks `if rt.is_null() { return KALICO_ERR_NULL_PTR; }` at entry. Same for `*mut TraceSample` in drain.

**Exception**: `kalico_runtime_tick` skips the null check. Called from the C-side ISR shim with a stable handle established once at init; checking on every 40 kHz call wastes cycles. Documented contract: C shim guarantees non-null after init returns OK.

### 5.7 Watchdog ↔ liveness

Klipper's C-side watchdog kicks every foreground iteration. **Watchdog alone is insufficient** — Rust ISR can no-op-loop while foreground keeps kicking (the ISR returns each tick, so foreground sees a healthy ticking machine). Add a **liveness heartbeat**:

- ISR increments `tick_counter: AtomicU64` on every successful tick (post-eval, pre-return).
- Foreground task records `last_tick_seen_value` and the wall-clock timestamp at each `runtime_drain` invocation.
- If `now - last_tick_seen_at > 100 ms` AND counter unchanged → foreground latches a *liveness fault*: stop kicking the Klipper watchdog. Watchdog resets within Klipper's existing watchdog interval.
- Liveness fault also triggered by `status == FAULT`.

Catches "ISR returns to C but makes no progress" — a class of bug pure watchdog can't see.

### 5.8 Arithmetic overflow

Workspace `[profile.release]` has `overflow-checks = false` — release builds wrap on overflow per RFC 560.

- u64 cycle math: irrelevant in practice (1063-year wrap), but `now - t_start` near `u64::MAX` could wrap to a huge positive value and suppress a boundary transition. Use `checked_sub` for time math; `wrapping_sub` only where wrap is intentional and asserted.
- f32 overflow → Inf, caught by §5.4 NaN/Inf guard.
- Queue head/tail / segment IDs: u32; expected to be modular-arithmetic in the queue impl (heapless handles this); `wrapping_add` on segment_id increment if/when needed.

Tests (Surface A) place `now` near `u64::MAX` and exercise duration subtraction across the wrap.

---

## 6. Testing

### 6.1 Surface A: pure-Rust host unit tests

Built with `cargo test -p runtime --features host` (workspace `host` feature pulls in std).

Targets:
- **Engine state machine** (`engine.rs`): mock TIM3 by manually advancing `now`; assert correct boundary detection, sub-tick carry, idle/fault transitions, status value at each step.
- **SegmentQueue** (`queue.rs`): producer/consumer round-trip, full/empty edge cases, capacity = N-1 for `Queue<_, 8>`.
- **CurvePool** (`curve_pool.rs`): handle resolution, slot-not-loaded rejection, out-of-bounds rejection.
- **TraceRing** (`trace.rs`): drop-newest policy, overflow flag carry to next sample, drain semantics.
- **Slot pipeline** (`slot.rs`): NoopPa/NoopIs ZST-ness via `mem::size_of`; LLVM-IR spot-check that Noops vanish (`cargo rustc --emit=llvm-ir`).
- **Kinematics** (`kinematics.rs`): CoreXY round-trip ([X,Y,E] → [A,B,E] → [X,Y,E] within ε).
- **Sub-tick boundary loop** (in `engine.rs`): synthetic chain of short segments, assert position continuity across boundaries; 25-µm sawtooth absent.
- **Wrap arithmetic**: place `now` near `u64::MAX`, exercise duration subtraction, sub-tick boundary near wrap.

Coverage target: ≥80% line coverage on `runtime/src/*.rs` excluding FFI bindings.

### 6.2 Surface A+: loom + miri

- **Loom tests** (`runtime/tests/loom.rs`, `cfg(loom)` gated): host-only models of the SPSC trace-overflow-pending atomic carry, status publication ordering, init state-machine. Loom can't model `cortex-m` interrupts but exhaustively exercises Acquire/Release interleavings on the bool/u8/u32 atomics.
- **Miri runs** on Surface A unit tests touching `UnsafeCell` init or FFI layout shims. Catches UB the regular test runs miss.

### 6.3 Surface B: FFI integration tests

Two parts:

- **Rust-side FFI tests** (`kalico-c-api/tests/*.rs`): drive the FFI surface via `unsafe extern` calls.
  - Init-once enforcement: second init returns null.
  - `kalico_runtime_push_segment` rejection paths (invalid handle, full queue, short duration, NaN in curve).
  - Drain pulls expected count after N synthetic ticks.
  - FAULT path latches and `last_error` returns expected code.

- **C smoke build** in CI: a single `.c` translation unit `#include "kalico_runtime.h"`, calls each exported function once with valid args, links against `libkalico.a`. Compiled with `arm-none-eabi-gcc` (target H7) and host `gcc/clang`. Catches cbindgen header drift, `repr` mismatches, struct-size disagreements that Rust-side tests cannot see. Includes `_Static_assert(sizeof(...) == ..., "...")` for every ABI-relevant type.

### 6.4 Surface C: H723 bring-up validation

Build:
```sh
cargo build -p kalico-c-api --no-default-features --features mcu-h7 \
  --target thumbv7em-none-eabihf --release
```

Linked into Klipper, flashed to Octopus Pro.

- **First-light**: ISR fires; LED toggles on `IDLE → RUNNING` flip. Verifies timer + IRQ + Rust-call works at all.
- **Cycle-count measurement** (hardened methodology):
  - DWT `CYCCNT` brackets `Engine::tick` only.
  - Measure empty-bracket overhead first; subtract.
  - Disable unrelated interrupts (USB, USART) during the microbench window.
  - Run warm-cache and cold-cache passes; report **min / p50 / p99**, not a point estimate.
  - For long-window timeline capture, use ITM/SWO or SEGGER SystemView (lower observer effect than USB-CDC).
- **Trace dump**: load 4 hand-built test segments from `runtime/tests/fixtures/step5_segments.json` (line, arc, smooth corner, halt sentinel); shared with Surfaces A and B. Drain over USB-CDC; host Python script plots against expected `nurbs::vector_eval` output.
  - Validates trajectory continuity across segment boundaries.
  - OVERFLOW flag fires correctly under deliberate slow-drain.
  - FAULT marker visible when invalid input pushed (e.g., a curve with NaN in a control point).
  - SEGMENT_END markers at expected ticks.
- **Soak test**: 30-minute synthetic replay; monitor FAULT count, OVERFLOW count, heartbeat counter stalls.

### 6.5 Scriptable manual hardware gate

`make test-h723` (or shell script) that:
1. Flashes the Octopus Pro via `dfu-util` or `stm32flash`.
2. Resets the board.
3. Captures USB-CDC output (segments loaded, ticks consumed, trace samples shipped, cycle-count results) into `target/h723-test-$(git rev-parse HEAD).log`.
4. Greps for a machine-readable `PASS` / `FAIL` marker.
5. Stores trace + cycle artifacts under the build SHA for cross-run comparison.

Reproducibility anchored to git SHA + board revision; "ran once and assumed working" gap closed.

### 6.6 CI matrix

```yaml
matrix:
  - target: x86_64-unknown-linux-gnu  # host tests, miri, loom (gated)
  - target: thumbv7em-none-eabihf     # mcu-h7 build
  - target: thumbv7em-none-eabihf     # mcu-f4 build (must compile)

checks:
  - cargo build (all targets)
  - cargo test --features host
  - cargo miri test (host, on UnsafeCell-touching tests)
  - cargo test --features loom (loom-gated tests)
  - cargo clippy --all-targets -- -D warnings  # with the expanded lint policy
  - cargo fmt -- --check
  - cargo run -p kalico-c-api --bin gen-headers   # verify regenerate is no-op
  - C smoke build: arm-none-eabi-gcc against generated header, link against staticlib
  - cargo deny check                              # license/security audit on heapless + transitive
  - LLVM-IR panic-symbol grep on Engine::tick    # proves panic-free invariant
```

Hardware-loop tests stay manual via `make test-h723`. CI hardware automation deferred past Step 5.

### 6.7 Shared test fixtures

`runtime/tests/fixtures/step5_segments.json` contains 4 NURBS test segments + expected `vector_eval` traces at sampled `u` values. Used by Surface A unit tests, Surface B C-smoke build (where applicable), and Surface C host Python comparison. Single source of truth — divergences across surfaces have a fixture as the diff target.

### 6.8 What testing intentionally does NOT cover at Step 5

- No real concurrent-producer hardware test (Step 6 introduces live producer; loom + hardware-stress lands then).
- No phase-stepping current-output validation (Step 10).
- No step/dir GPIO output validation (Step 7 MVP).
- No PA / shaper correctness (Steps 8/9).
- No multi-MCU clock-sync correctness (Step 6).
- No real-print soak validation (Step 7 MVP).

---

## 7. Open questions / explicit non-decisions

The following are knowingly deferred — flagged here so they don't get lost:

1. **Curve-pool size N.** Step 5 ships with `N = 16` (~16 distinct NURBS curves resident at once). Step 7 MVP will revisit when real Layer-1 output volumes are known.
2. **Trace ring depth.** 1024 samples = ~25 ms at 40 kHz. Fine for Step 5 manual testing; Step 7 MVP may need to raise (continuous-print drain) or lower (RAM pressure once other subsystems land).
3. **`MIN_SEGMENT_CYCLES` exact value.** Currently `2 * ONE_TICK_CYCLES` = 27,500 cycles (~50 µs of motion at 40 kHz). Surface A boundary-loop tests refine; producer-rejection threshold may need to align with whatever Layer 1 actually produces.
4. **DTCM memory budget.** TraceRing + CurvePool + SegmentQueue + ISR stack must fit comfortably in H723's 128 KB DTCM. Surface C measures actual; if pressure surfaces, AXI SRAM relocation (with cache-coherency considerations) is the fallback.
5. **F4x integration scope at Step 5.** F4x compiles via `mcu-f4` Cargo feature but is not run-tested at Step 5. Step 6 multi-MCU bring-up exercises it; if surprises surface there, this spec gets an amendment entry.
6. **Whether to ship a `kalico_runtime_reset()` FFI at Step 5.** Currently deferred (manual restart on FAULT). If Surface C testing reveals a frequent-enough fault class to warrant in-place recovery, add at that point.

---

## 8. References

### Research artifacts (this repo)

- `docs/research/firmware-survey.md` — broad 2026 firmware landscape; Klipper architecture; H7 step rates; HAL/LinuxCNC abstractions.
- `docs/research/tmc5160-open-loop-phase-stepping.md` — TMC5160 register-level mechanism; SPI throughput math; 20–40 kHz modulation rationale (this session).
- `docs/research/open-loop-phase-stepping-prior-art.md` — Prusa Buddy / RepRapFirmware MB6HC prior art; Pattern A (40 kHz ISR + segment buffer) as the production-grade approach (this session).

### Prior specs in this build

- `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md` — Layer 0 NURBS substrate; the `nurbs::vector_eval` API Step 5's ISR consumes.
- `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md` — workspace layout precedent; the `nurbs ↔ nurbs-c-api` FFI split this spec extends.
- `docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md` — Layer 2 batch-planning architecture; produces the segment buffers Step 7 MVP eventually feeds Step 5's runtime.

### External references (per Codex review)

- Rust Reference, [linkage chapter](https://doc.rust-lang.org/reference/linkage.html#mixed-rust-and-foreign-codebases) — multiple `staticlib`s into one foreign link "are likely to conflict"; umbrella crate is the documented path.
- [`heapless` crate documentation](https://docs.rs/heapless/0.8/heapless/spsc/index.html) — SPSC queue capacity is `N-1`; M7 atomic ordering ships correct.
- STM32H723 Reference Manual ST RM0468 — TIM3, NVIC priority, FPU FPSCR exception trapping.
- Cortex-M7 [DWT cycle counter usage](https://doc.rust-lang.org/embedded-book/concurrency/index.html) and [`core::sync::atomic`](https://doc.rust-lang.org/core/sync/atomic/) — `target_has_atomic = "ptr"` available on `thumbv7em-none-eabihf`.

---

## 9. Brainstorm-decision provenance

Decisions locked through Q1–Q6 in this session's brainstorm:

| Q  | Decision                                                                                  | Rationale source                                       |
|----|--------------------------------------------------------------------------------------------|--------------------------------------------------------|
| Q1 | Step 5 = real hardware + minimal harness (option B, not pure simulation)                   | User direction                                         |
| Q2 | H723 first; F4x deferred to Step 6 multi-MCU                                               | User direction                                         |
| Q3 | Output stage = trace ring only (option α); step/dir + phase-stepping in later steps        | User direction + research-informed (Pattern-A coexists with Pattern-C)|
| Q4 | Workspace = umbrella `kalico-c-api` + new pure-Rust `runtime`; rename `nurbs-c-api` now    | Codex Rust linkage docs review                         |
| Q5 | Axes = AB + E (CoreXY for AB; identity for E); 3D NURBS in (X, Y, E)                       | User direction                                         |
| Q6 | Buffer = `heapless::spsc::Queue<Segment, 8>` (effective 7) with sub-tick boundary handling | 6-agent parallel adversarial review (3 verifier + 3 codex) converged on refined-(b) |

Section-level adversarial reviews (Codex):
- §2 Architecture: TIM3 timer choice, NVIC priority, file-granularity split, link-line specifics, `--no-default-features` invocation, `panic = "abort"` interaction, GCC-LTO ↔ Rust `lto = "fat"` compatibility.
- §3 Components: feature-gated `Engine<P, I>` instantiation; `CurveHandle` slab pool over inline curve data (avoids `memcpy` cost on enqueue); `Queue<_, N>` capacity `= N-1` clarification; init-once via `UnsafeCell + AtomicU8` state machine (`UNINIT/INITING/READY`); ZST `Noop` slot impls with `#[inline(always)]`; cbindgen cfg-gating for two headers; `i32` return codes at FFI boundary.
- §4 Data flow: time-unit contract (cycles, u64 monotonic); ack-then-call-Rust ordering; sub-tick boundary loop bounded by queue depth; trace overflow protocol via separate `AtomicBool` carry; segment-finished trace marker; ISR self-disable on idle/fault; per-cycle-budget paper estimate with measurement caveat.
- §5 Error handling: expanded clippy lint policy beyond `unwrap_used`; LLVM-IR panic-symbol grep gate; NaN/Inf check necessity (M7 FPv5 produces quiet NaN); init-once state machine; watchdog + liveness heartbeat (watchdog alone insufficient); `checked_sub` for time math under `overflow-checks = false`.
- §5 Testing: loom-gated host tests for atomic-ordering coverage; miri runs on UnsafeCell init; C smoke build for cbindgen drift; scriptable `make test-h723` gate for Surface C reproducibility; shared `step5_segments.json` fixtures across all three surfaces; DWT methodology (warm/cold, min/p50/p99); `cargo deny` for supply-chain audit.
