We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

- Fail loudly. When adding checks for unexpected things to the code, instead of trying
  to recover, unless it was discussed and agreed on explicitly, the default solution is
  to fail loudly with a clear error code. This helps us catch bugs quicker. Example: movement segment arrives to the planner late, causing the start time to be in the past. Do not advance or pad the
  start time, raise an error instead. this way we notice the issue and have a chance to address it

- Comments are a failure of expression. Instead of writing one, make the code say it:
  rename, extract, assert, or compute the value. A comment earns its line only if it
  stops a future editor from breaking something the code can't defend (e.g. a
  load-bearing sleep). Never restate names/signatures, doc-comment fields, narrate
  steps, or justify decisions in code — justifications go in commit messages.
  TODO-style markers are fine. Self-check on every diff: "what mistake does this
  comment prevent?" — none → delete.

- Unit tests live in a separate file from the tested code.

# High level feature scope
- Rust for new code by default; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now. **C is acceptable for low-level building blocks where Rust's borrow / aliasing abstractions misbehave or obscure debugging** — e.g., the MCU-side segment SPSC queue is a C struct in `.axi_bss` because LLVM miscompiled the borrow-projected `heapless::spsc::Consumer` pattern (2026-05-18 bench: `qlen_sd=6 qlen_ps=1` from the same `Consumer` instance across two call sites, with the Producer's enqueues visible from one site but not the other). The rule is "Rust for the engine, C where the engine's primitives need to be trivially debuggable."
- NURBS-native internal primitive through the planner. **Uniform cubic Bézier (degree-3 polynomial) representation** across Layer 1 / 2 / 3 / 4 — no rational NURBS anywhere live, no mixed-degree dispatch, no source-gcode-type special-cases.
- **G5 / G5.1 only — no legacy G-code in the planner reduce stage.** G5 → cubic Bézier direct; G5.1 → cubic via exact degree-elevation. The planner reduce stage (`rust/geometry`'s reducer + `rust/temporal` + `rust/trajectory`) has zero internal handling for G0 / G1 / G2 / G3 — no reduction code paths, no `Linear` / `RationalQuadratic` / `FittedSegment` / `ArcSegment` types, no feature-flagged "legacy mode." Anything reaching reduce is G5 or G5.1; anything else is rejected at the reduce boundary as a hard error.

  The `rust/gcode` lexer remains capable of tokenizing legacy G-code, because the `compat` crate (Step 13's normalizer) and the bridge's live-G1-conversion path both depend on it. Tokenization is not the rejection boundary.

  The `compat` crate has two callers: the offline Step-13 binary (file → file) and the live bridge (terminal/macro G1/G2/G3 conversion via `compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`). Both share the lexer.

# Observability / structured logging

Log via the structured pipeline (`kalico_log_emit` → `events/*.jsonl`), not
`printf`/`output()` — it replaces `klippy.log` for MCU/structured diagnostics;
the wire-stable event table is `rust/runtime/src/log_codes.rs`. To read or add
logs — `KALICO_DIAG_DUMP`, crash forensics, filtering — use the `mcu-diagnostics`
and `query-logs` skills.

# Reference docs

- **Target hardware** [`docs/kalico-rewrite/hardware.md`](docs/kalico-rewrite/hardware.md)
- **MCU C/Rust boundary — architectural invariant:** [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](docs/kalico-rewrite/mcu-c-rust-boundary.md). Read this before adding shared state between C and Rust on the MCU, or before reaching for `#[link_section]` on a Rust static. Rules: C owns boot, safety-critical paths, and all shared-memory placement; Rust owns the motion engine; the seam is `extern "C"` + `#[repr(C)]` only.
