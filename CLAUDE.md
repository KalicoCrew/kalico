We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

- Fail loudly. When adding checks for unexpected things to the code, instead of trying
  to recover, unless it was discussed and agreed on explicitly, the default solution is
  to fail loudly with a clear error code. This helps us catch bugs quicker.

# High level feature scope
- Rust for new code by default; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now. **C is acceptable for low-level building blocks where Rust's borrow / aliasing abstractions misbehave or obscure debugging** — e.g., the MCU-side segment SPSC queue is a C struct in `.axi_bss` because LLVM miscompiled the borrow-projected `heapless::spsc::Consumer` pattern (2026-05-18 bench: `qlen_sd=6 qlen_ps=1` from the same `Consumer` instance across two call sites, with the Producer's enqueues visible from one site but not the other). The rule is "Rust for the engine, C where the engine's primitives need to be trivially debuggable."
- NURBS-native internal primitive through the planner. **Uniform cubic Bézier (degree-3 polynomial) representation** across Layer 1 / 2 / 3 / 4 — no rational NURBS anywhere live, no mixed-degree dispatch, no source-gcode-type special-cases.
- **G5 / G5.1 only — no legacy G-code in the planner reduce stage.** G5 → cubic Bézier direct; G5.1 → cubic via exact degree-elevation. The planner reduce stage (`rust/geometry`'s reducer + `rust/temporal` + `rust/trajectory`) has zero internal handling for G0 / G1 / G2 / G3 — no reduction code paths, no `Linear` / `RationalQuadratic` / `FittedSegment` / `ArcSegment` types, no feature-flagged "legacy mode." Anything reaching reduce is G5 or G5.1; anything else is rejected at the reduce boundary as a hard error.

  The `rust/gcode` lexer remains capable of tokenizing legacy G-code, because the `compat` crate (Step 13's normalizer) and the bridge's live-G1-conversion path both depend on it. Tokenization is not the rejection boundary.

  The `compat` crate has two callers: the offline Step-13 binary (file → file) and the live bridge (terminal/macro G1/G2/G3 conversion via `compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`). Both share the lexer.

# Observability / structured logging

We log via a **structured pipeline, not `printf`/`output()`**. The MCU emits
typed records through `kalico_log_emit` → `KALICO_MSG_LOG` → the host writes
NDJSON to `~/printer_data/logs/events/<source>.jsonl` (`mcu`, `bottom`, `host-py`,
`host-rust`; rotating 32 MB × 5). **This store replaces `klippy.log` for
structured / MCU diagnostics.** `klippy.log` is kept only as the always-works
fallback (host Python tracebacks + the verbose `prior_diag_summary_*` deep-debug
text whose fields the structured path doesn't carry).

Tools we built (this branch): the C-owned `kalico_log` ring + `kalico_log_emit`
(the sole ABI seam, `src/kalico_log.{c,h}`); the `McuLog` wire message; the
`events/*.jsonl` store + host re-emit (`rust/motion-bridge/src/mcu_log.rs`);
**`KALICO_DIAG_DUMP`** gcode (on-demand live diag snapshot, no reset); auto
crash-forensics replay on the boot after a reset; and the `diag.*` event-ring
play-by-play. Spec: `docs/superpowers/specs/2026-06-01-mcu-log-endpoint-design.md`.

**Adding a log line:**
- The event/subsystem tables in `rust/runtime/src/log_codes.rs` are the
  **wire-stable source of truth** (event code, name, `{arg0}`/`{arg1}` template).
  Add the `(subsystem, event)` arm there first.
- **C emit:** `#include "kalico_log.h"`, mirror any new event `#define`, then
  `kalico_log_emit(level, subsystem, event, code, arg0, arg1)`. C functions the
  Rust staticlib calls need `__attribute__((used, externally_visible))` to survive
  LTO.
- **Rust emit:** use the gated `extern "C" kalico_log_emit` (see
  `rust/runtime/src/fault_helpers.rs`). Engine faults go through the `raise_*`
  helpers there (auto-emit `runtime.fault_latched` with `code = FaultCode`); to add
  a fault, add the `FaultCode` + `code_name` and a `raise_*` helper.
- This is the mechanism behind the **fail-loud** rule above: surface an
  unexpected condition as a structured fault/log, don't silently recover.

**Reading logs — use the skills:** `mcu-diagnostics` (MCU diag, crash forensics,
`KALICO_DIAG_DUMP`, the event catalog, raw `events/*.jsonl`) and `query-logs`
(LogsQL queries over VictoriaLogs by session/print/level/field).

# Reference docs

- **Target hardware** [`docs/kalico-rewrite/hardware.md`](docs/kalico-rewrite/hardware.md)
- **MCU C/Rust boundary — architectural invariant:** [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](docs/kalico-rewrite/mcu-c-rust-boundary.md). Read this before adding shared state between C and Rust on the MCU, or before reaching for `#[link_section]` on a Rust static. Rules: C owns boot, safety-critical paths, and all shared-memory placement; Rust owns the motion engine; the seam is `extern "C"` + `#[repr(C)]` only.
