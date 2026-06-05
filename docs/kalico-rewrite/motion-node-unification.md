# Motion Node Unification

> North-star design. AI-assisted; flag drift, don't drift silently. Companion to
> [`mcu-c-rust-boundary.md`](mcu-c-rust-boundary.md), whose invariants this design
> must not violate.

## Problem

The per-axis piece-ring **walk + evaluate** loop exists twice and the copies have
already diverged:

- **`runtime::engine`** — `get_position_and_velocity` + `arm_next` + `eval_horner`
  (the hardened four-branch walker carrying the ⚠️ DO-NOT-MODIFY banner that fixes
  the `PieceStartInPast` hardware regression). Returns `(pos, vel)`, then
  `dispatch_axis` turns that into step pulses.
- **`kalico-ethercat-rt::curves::AxisRing::sample`** — a *re-implementation* of the
  same walk + Horner eval, but thinner and already drifted: throws velocity away,
  has no `PieceStartInPast` fault, slightly different boundary semantics. Then it
  writes the position to a servo PDO.

Two loops, one hardened and one casual, and the casual one drives a real servo.
That is the drift hazard. **Unify on the single hardened core; let each target
differ only in *dispatch*.**

## Pillars (locked decisions)

1. **Same-or-better MCU performance, proven by disassembly diff.** Static
   monomorphized generics — **never `dyn`** on the ISR path. Clean, simple code;
   no defensive `if`-soup. Unexpected states **fail loud** (per `CLAUDE.md`), they
   do not silently branch.
2. **The hardened walker moves verbatim.** Its four-branch structure, ordering, the
   2-tick adoption fault, and the no-gap-branch/saturating-elapsed behavior are
   preserved exactly. The *only* edit is that the fault is raised through a
   `FaultSink` trait method instead of the hardwired `raise_piece_start_in_past`.
3. **Output contract `(pos, vel)` now.** Velocity is already computed and currently
   discarded — unifying hands the servo path velocity for free (feedforward).
   Acceleration / jerk / per-axis dynamics (inertia, CoreXY torque ratio) get a
   designed extension seam but are **not built** now.
4. **"A motion node is an MCU."** STM32 firmware, another board, or a real-time
   Linux EtherCAT process all share one role: receive pieces → loop at their own
   rate → walk+eval the shared core → dispatch → emit retirement heartbeat → report
   faults via `FaultSink`. Endstops→trsync and temperature are a **future** stage
   (seam designed, nothing built). `menuconfig` selects which dispatch modules a
   given firmware build compiles in, for future mixed-MCU systems.

## Architecture

**The shared core is the drift-prone WALKER, not a single generic engine.** Each
node *composes* shared primitives into its own loop; the MCU `Engine` stays
non-generic. We considered making `Engine<D: Dispatch>` generic so the EtherCAT node
would reuse the whole engine — but that ripples generics through the runtime crate
and the 82-symbol FFI for the benefit of a separate-crate consumer, fighting the
"keep it clean / don't hurt the MCU" constraint. The only thing that *must* be
single-sourced is the hardened walker; the rest (rings, retirement) is already a
shared primitive (`RingDescriptor`).

- **`runtime::motion_core`** (new, `pub`) — the hardened walker
  (`get_position_and_velocity` + `arm_next` + `eval_horner`) and `ArmedPiece`,
  generic over `FaultSink`, taking **split borrows** (`&mut Option<ArmedPiece>`,
  `&mut RingDescriptor`, `&[PieceEntry]`) so *any* node can call it. THIS is the
  single source of the piece loop. `#[inline]` so it still inlines into the MCU
  `Engine::tick` byte-for-byte (verified by the disasm gate).
- **`FaultSink` trait** (`pub`, **unsealed** — it is the node-extensibility seam, so
  the separate EtherCAT crate can implement its own). `SharedFaultSink` (MCU, wraps
  `&SharedState`) and the EtherCAT node's own sink implement it. **Contract**
  (documented, not enforced): impls must be allocation-free and non-blocking — they
  run on the real-time path — and must preserve the detail-before-code store
  ordering of the fault path.
- **`Dispatch` trait + `MotionSample`/`SampleTiming`** (`pub`) — the NAMED shape of
  "the different part." `StepperDispatch` (wraps today's `dispatch_axis`) and
  `ServoDispatch` (PDO write of pos[+vel]) implement it. `MotionSample { p_end,
  v_end, p_sample_start }` with reserved (commented) `accel`/`jerk` extension
  fields; per-axis dynamics (inertia, CoreXY ratio) live on the axis config, **not**
  the per-sample struct. Monomorphized, never `dyn`.
- **MCU `Engine` stays non-generic.** `Engine::tick` calls
  `motion_core::get_position_and_velocity(&mut axis.armed, &mut axis.ring, …)` then
  dispatches. All 82 `extern "C"` symbols and the `RuntimeContext`/`rt_storage`
  layout are **untouched** — no generics ripple through the FFI.
- **EtherCAT endpoint = its own node** composing the same primitives: per-axis
  `RingDescriptor` + `motion_core::get_position_and_velocity` + a `ServoDispatch` +
  its own `FaultSink`, driven from its DC loop. It links `runtime` (for the core)
  but **not** `kalico-c-api`; the C-scheduler-timer path is an MCU concept.
- **(Future) `PeripheralContext` + `NullPeripherals`** — a zero-sized-default seam
  for endstop/trsync/temp; MCU keeps endstops C-ISR-driven for now. Designed, not
  built.

"A motion node is an MCU" is realized as a **role** — compose `motion_core` +
`RingDescriptor` + `Dispatch` + `FaultSink` + retirement (`retired_count`) — not as
a single shared generic type. This keeps the MCU engine simple and the C/Rust
boundary honest.

## What we deliberately do NOT unify (anti-over-abstraction)

- **`Engine` stays a concrete, non-generic struct.** We share the *walker*
  (`motion_core`), not the whole engine. No `Engine<D>`, no generics through the FFI.
- **`SharedState` stays concrete.** It holds ~50 stepper diagnostic atomics read by
  the C diagnostic rotation *by struct offset*; a trait/generic would hide the
  layout. Splitting it into `NodeState` + `StepperDiagState` is a **deferred,
  standalone, behavior-preserving** commit — not part of the initial trait work.
- **`RuntimeContext` and the C FFI layer stay concrete `#[repr(C)]`.** No
  `Box<dyn …>` / `&dyn` crosses `extern "C"` (boundary rule B3 — the 2026-05-18 SPSC
  miscompile). The generic type parameter surfaces only at the `EngineImpl` alias.
- **The timer call-path is not abstracted.** C scheduler timer (MCU) vs. RT-Linux
  cyclic loop (EtherCAT) are different event models; unifying them buys nothing.
- **`seed_position`, `set_axis_mode`, `step_queue` reset, `TickCaches`/`last_motors`/
  `StepMotorState`** are stepper-specific — kept concrete / `#[cfg]`-gated, never in
  a shared trait.

## Kconfig dispatch-module selection

Mirror the existing `mcu-h7` / `kalico-sim` passthrough exactly:

- `src/Kconfig`: a "Motion dispatch modules" menu with
  `CONFIG_MOTION_MODULE_STEPPER` (default y on STM32) and
  `CONFIG_MOTION_MODULE_ETHERCAT` (default n), guarded `depends on` the runtime
  targets.
- `src/Makefile`: comma-append `motion-module-stepper` / `motion-module-ethercat`
  to `KALICO_RUST_FEATURES` (same pattern as `kalico-sim`, lines 74-76).
- `rust/kalico-c-api/Cargo.toml`: passthrough features →
  `runtime/motion-module-*`. `rust/runtime/Cargo.toml`: leaf features gating the
  `#[cfg(feature = "motion-module-*")]` dispatch impls.
- `build.rs` (or feature combination): assert **exactly one** dispatch module is
  active — fail loud at build time otherwise.

## Performance gate (mandatory, before every walker/dispatch-touching commit)

Verified runnable on macOS (`thumbv7em-none-eabi` installed, `/usr/bin/objdump`
Apple LLVM 21 is Thumb-capable). Baseline captured to `/tmp/mnu-disasm/before_*`.

```
# build (H7); STORAGE/RING/RATE env are mandatory on bare-metal targets
cd rust && KALICO_RUNTIME_STORAGE_SIZE=122880 KALICO_RUNTIME_PIECE_RING_SIZE=63488 \
  KALICO_RUNTIME_SAMPLE_RATE_HZ=40000 cargo build -p kalico-c-api \
  --no-default-features --features mcu-h7,header-nurbs,header-runtime \
  --target thumbv7em-none-eabi --release
# resolve mangled names dynamically (the body-hash suffix changes every edit),
# objdump -j .text.<sym>, sed-strip the address column, diff before/after for
# Engine::tick + dispatch_axis + isr_sample_tick.
```

Require **identical-or-better** codegen on `Engine::tick` (which contains the
inlined walker) and `dispatch_axis`. `runtime` unit tests stay green; run MIRI for
the `arm_next` reborrow. F4 spot-check at the end; G0 needs `rustup target add
thumbv6m-none-eabi`. On-bench cycle-counter confirmation
(`isr_eval_cycles_max` etc.) when hardware is available — not blocked by the servo
drive being off (the stepper MCU is USB-powered).

## Implementation sequence (bisectable; disasm-gated) — STATUS

1. **`FaultSink` trait** — walker takes `&impl FaultSink`; `SharedFaultSink` impls it.
   ✅ DONE — verified **byte-identical** MCU codegen.
2. **Relocate the walker → `pub motion_core`** (split borrows); `FaultSink` cross-crate
   `pub`. ✅ DONE — verified **byte-identical**.
3. **Fix `dispatch_axis` `_ => {}`** → `raise_unknown_step_mode` (`FaultCode::UnknownStepMode
   = -312`); also removed the dead `PHASE_LUT` `else` branch (direct index, compile-time
   bound assert, no panic path). ✅ DONE — `Engine::tick`/`isr` byte-identical; `dispatch_axis`
   +20 instrs (intended fail-loud).
4. **Kconfig dispatch modules** — `CONFIG_MOTION_MODULE_STEPPER` → `motion-module-stepper`
   feature gating `dispatch_stepper`; a bare-metal build with no dispatch module now
   **fails loud** (`compile_error!`). ✅ DONE — byte-identical with the module on; servo
   node builds without it.
5. **EtherCAT endpoint as a node** — `curves.rs` `AxisRing::sample` routes through
   `motion_core` (duplicate loop deleted); endpoint fault **latches + propagates** to the
   host (no silent position-hold) and is allocation-free; servo node uses `runtime` with
   `default-features=false`. ✅ DONE (crate). ⏳ Host bridge/klippy wiring (`#38`) NOT yet
   ported — endpoint is validated standalone (stub) but not yet end-to-end from klippy.
6. **`Dispatch` trait formalization + `PeripheralContext` seam** — DEFERRED as design (no
   load-bearing consumer yet under the minimal-core approach; avoid speculative scaffolding).
7. **Hardening** (off-bench, user away): adversarial review (15 findings, all verified) +
   bench-risk tests — end-to-end moving trajectory through `Engine::tick`, `motion_core`
   property tests (fault boundary `>` not `>=`), endpoint origin/no-jump, C0/C1 piece-boundary
   continuity, PieceStartInPast boundary, **sustained streaming past one ring depth** (the
   "stopped after first move" class). ✅ DONE — no bugs found in core logic.

### Deferred / next
- **`#38` host bridge + klippy wiring** — additive port onto the shared `motion-bridge`;
  the one remaining piece for end-to-end "usable." Touches the bridge shared with the
  stepper path, so it must be kept strictly additive (EtherCAT-only branches) and reviewed
  for stepper-path safety; full validation is hardware-gated.
- Real-drive handshake (operator-supervised, drive currently off).
- On-bench cycle-counter confirmation of the byte-identical claim (`isr_eval_cycles_max`).

## Keepers to port (from the `ethercat` branch)

- **Protocol** (`PushPieces` 0x0060, `PushPiecesResponse` 0x0061, `StatusHeartbeat`
  0x0083, `retired_counts`) is **sota-native** — byte-identical on both branches,
  zero porting.
- **Endpoint driver-comms:** the SOEM/ecrt DC loop + PDO mapping (`wkc != 3` fault
  halt), `server.rs` non-blocking discipline, `wire`/`scale`/`clock`/`ffi`,
  `bench/libecrt.h`, stub binary, `ec-test-client`, `stub_loop` test.
- **Host bridge:** `unix_native_conn.rs` (UDS NativeCall + heartbeat callback),
  `pump.rs` `WireSink`→`McuTransport` enum, `bridge.rs` branches (`claim_ethercat_node`,
  1 GHz clock registration, `attach_heartbeat_callback`, `ethercat_mcu_ids`),
  `ethercat_transport.rs`.
- **Klippy config:** `[ethercat_node]`, `[servo_<axis>]` / `ServoRail`,
  `MotionToolhead._register_axis` branch, `configure_axes` binding.
- **Delete:** `curves.rs` `AxisRing` (the casual loop); any dead
  `dispatch_target_tests.rs` from the removed node-trait era.
