# MCU Structured-Log Endpoint — Design (Observability Spec #2)

- **Date:** 2026-06-01
- **Branch / worktree:** `observability` (`.worktrees/observability`)
- **Status:** Design — recon complete, decisions proposed (A–E), **pending user review** before plan/implementation.
- **Depends on:** the host pipeline (Stages 1–3, already shipped on this branch) — schema, `events/*.jsonl`, Vector→VictoriaLogs, the `query-logs` skill. This spec adds `source = mcu-h7` / `mcu-f4` to that pipeline.
- **Boundary doc (binding):** `docs/kalico-rewrite/mcu-c-rust-boundary.md`.

---

## 1. Problem

The MCU is the one place the observability pipeline can't see. Today its diagnostics are scattered across three uncorrelated channels: Klipper `output()` strings (free-form, decoded as `RuntimeEvent::UnknownOutput`), `StatusHeartbeat` (10 Hz, `engine_state`/`fault_code` dropped at decode), `FaultEvent` (`0x0082`, fire-once, raw numeric code with **no host-side name resolution** — "what is 65228?"), and boot-time persistent-diag (`.persistent_diag` / `.bkp_bss`, replayed via `output()` strings). None of these land in the structured store with a session id, a real timestamp, a subsystem, or a resolved code name. The Rust motion engine — where most anomalies are *detected* (`tick.rs`, `engine.rs`) — has **zero** logging today; it can only surface state through FFI accessors that C polls.

**Goal:** a structured MCU log the host decodes into the **same schema** as host-py/host-rust (`source = mcu-h7`/`mcu-f4`), stamped with the current `session_id`/`print_id` and a clock-sync-converted `_time`, with `code`/`code_name`/`event`/`subsystem` resolved host-side — flowing into the existing Vector→VictoriaLogs pipeline so MCU faults show up in the same UI next to host logs, queryable per session.

## 2. Goals / Non-goals

**Goals:**
1. A C-owned MCU log transport that **both C and the Rust engine** can emit into, respecting the C/Rust boundary (§4).
2. Host decode → the Stage-1 schema, with `code`→`code_name` resolution (kills "what is 65228?").
3. Real timestamps via the existing tick→walltime clock-sync.
4. Bandwidth-safe: structured (coded) frames, level-gated at emit, no per-tick wire flooding.
5. Testable entirely in the sim playground (real MCU firmware), no Trident.

**Non-goals (deferred):**
- Replacing `FaultEvent` (kept as the hard-fault signal) or the boot persistent-diag replay (kept; could be re-emitted structured later).
- High-rate per-tick tracing over the wire (physically infeasible on USB-CDC; see §6).
- A host-set runtime level *protocol* beyond a simple min-level (a richer per-subsystem runtime map is a later enhancement).
- MCU-side string formatting (the host owns templates; the wire carries codes + numeric args).

## 3. Decisions (proposed; **pending review**)

| # | Decision | Choice | Rationale |
|---|---|---|---|
| **Spec correction** | "reuse `RuntimeEvent::Trace`" | **Cannot** — `Trace` is vestigial | Recon: `Trace` is a legacy Klipper-protocol path with **no C emitter and no kalico-native message**; decoded then dropped (`events.rs:272`, `bridge.rs:1811`). There is no real channel to reuse. We add a proper message and map it through a new event. |
| **A** | Transport | **New kalico-native message `KALICO_MSG_LOG (0x0084)`** on `KALICO_CHANNEL_EVENTS` | Sits beside `FaultEvent`/`StatusHeartbeat`; first-class, decodable, schema-hash-versioned. |
| **B** | Wire format | **Structured codes + numeric args** (no strings on the wire); host resolves names/templates | Tiny frames (~16–24 B); bandwidth-safe; matches the schema's `event`/`code`/`code_name`/payload. |
| **C** | Host output | **Separate `events/mcu-h7.jsonl` / `mcu-f4.jsonl`** | Matches §5 of the host spec (`source` → one file); keeps MCU logs out of the `host-rust` tracing subscriber (which hardcodes `source`). |
| **D** | v1 call sites | **The existing fault/anomaly paths** (`FaultCode` producers in `tick.rs`/`engine.rs`) + a few key motion lifecycle events; **warn/error gated in motion** | Delivers the "why did it fault" value; bandwidth-safe by default; debug/trace available when the level is raised. |
| **E** | Level control | **Runtime-settable min level** (default `warn` in motion); gated **at emit** | Cheapest place to drop (spec §9 drop-at-emit); a debug session can raise it without a reflash. |

**Deploy-lockstep implication (accepted):** a new message kind bumps `KALICO_SCHEMA_HASH` (recomputed by `kalico-protocol/build.rs`), so the MCU firmware and host bridge must be deployed together — the `IdentifyResponse` schema-hash check (`kalico_native.rs:241`) rejects a mismatch at startup. This is correct fail-loud behavior; the playground rebuilds both together, so testing is unaffected.

## 4. Architecture

```
  Rust engine (tick.rs/engine.rs)  ──┐
   anomaly detected (FaultCode)      │  extern "C" kalico_log_emit(level, subsystem, event, code, a0, a1)
  C foreground/dispatch ────────────┤        (grabs timer_read_time(); short irq_disable; push)
                                     ▼
                    ┌──────────── C-OWNED log ring (src/kalico_log.c) ────────────┐
                    │  #[repr(C)] SPSC/MPSC ring in a C linker section (.bss/DTCM) │
                    │  producers: Rust engine + C ; consumer: C foreground task    │
                    └──────────────────────────────────────────────────────────────┘
                                     │  drain (foreground DECL_TASK) — NEVER in ISR
                                     ▼
                 kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, KALICO_MSG_LOG, …)
                                     │  wire: mcu_tick(u32) level(u8) subsystem(u8)
                                     ▼        event(u16) code(u16) seq(u16) args[u32;N]
  ───────────────────────────────── host ───────────────────────────────────────────
  kalico_native.rs decode 0x0084 → RuntimeEvent::McuLog{…}
                                     │
  EventDispatcher (events.rs:272 region) → injected re-emit hook:
     • widen 32-bit mcu_tick → 64-bit (PushPiecesResponse heuristic)
     • ClockSyncEstimator::wall_time_at_mcu(ticks) → _time            (NEW method)
     • resolve subsystem/event/code → names + _msg template
     • stamp session_id/print_id from the global SessionContext
     • write one NDJSON line → events/<source>.jsonl  (source=mcu-h7/-f4)
                                     ▼
                       Vector → VictoriaLogs (unchanged)
```

### 4.1 The C-owned log ring (boundary §B2/§B3)
A new `src/kalico_log.c` declares `struct kalico_log_entry` (`#[repr(C)]` mirror in Rust) and a fixed-size ring in a C linker section (regular `.bss`; DTCM on H7). Enqueue is a thin `extern "C" kalico_log_emit(...)` that captures `timer_read_time()` and pushes under a short `irq_disable`/`irq_enable` critical section (safe for the ISR-engine + C-foreground multi-producer case; matches Klipper's sched idiom). **Rust never holds a borrow into the ring** — it calls the C function (no Rust-typed structure crosses the ABI; §B3, the 2026-05-18 SPSC lesson). The consumer is a C foreground `DECL_TASK` that drains and transmits — **TX never runs in the ISR**, which also sidesteps the 64-bit-tick-widening-is-foreground-only constraint (§6, tension 4: the frame carries the raw 32-bit tick; the host widens).

### 4.2 The wire message (boundary §B4)
`KALICO_MSG_LOG (0x0084)` added to `MessageKind` in `rust/kalico-protocol/src/messages.rs` (+ `Encode`/`Decode`), regenerating `src/kalico_protocol_schema.h` via `build.rs`. Payload, all fixed-width LE: `mcu_tick: u32`, `level: u8`, `subsystem: u8`, `event: u16`, `code: u16`, `seq: u16`, `args: [u32; 2]` (extensible). `seq` is a per-MCU monotonic counter for host-side drop detection (the ring can overflow under burst → a dropped-count is itself logged).

### 4.3 Host decode + re-emit
- `kalico_native.rs` decodes `0x0084` → new `RuntimeEvent::McuLog { mcu_tick, level, subsystem, event, code, seq, args }`.
- `ClockSyncEstimator` gains `wall_time_at_mcu(mcu_ticks: u64) -> Instant` (the inverse of `mcu_time_at_host`; not present today).
- The re-emit hook is **injected** into `EventDispatcher` at construction (the dispatcher lives in `kalico-host-rt`, which must not depend on `motion-bridge`): a boxed `Fn(McuLogEvent, &ClockSyncEstimator)` the bridge supplies. It resolves names, stamps session/print from the global `SessionContext` (`logging::context::load_context()`), and appends one schema line to `events/<source>.jsonl` via a `RotatingJsonlWriter` (reused from Stage 2 `logging::writer`). `source` = the per-MCU label (`mcu-h7`/`mcu-f4`).

### 4.4 Code/name resolution (the "what is 65228?" fix)
- `rust/runtime/src/error.rs`: add `FaultCode::code_name(&self) -> &'static str` (and a `from_i32`), so a numeric `code` resolves to its symbol host-side.
- A small shared `subsystem` code↔name table and `event` code↔name+template table (the MCU emit sites and the host resolver reference the same constants). `_msg` is composed host-side from the event template + args (no MCU strings).

## 5. Schema mapping
Per record written by the re-emit hook: `_time` (clock-sync wall time of `mcu_tick`), `_msg` (host-composed from event template + args), `level` (from `level` byte), `source` (`mcu-h7`/`mcu-f4`), `subsystem` (resolved), `session_id`/`print_id` (host global context), `target` (`mcu::<subsystem>`), `event` (resolved key), `code`/`code_name` (when non-zero), and payload (`arg0`/`arg1` + `seq`). Identical shape to host records ⇒ the `query-logs` recipes and the playground UI work unchanged.

## 6. Constraints / tensions (from recon)
1. **C-owns-buffer, Rust-logs** — the only Rust→ring path is the thin `extern "C"` call; latency on the ISR path is one bounded-time enqueue under irq-disable. Acceptable; measured in the sim.
2. **Bandwidth** — USB-CDC shares the link with pieces/heartbeats/commands. warn/error at anomaly rate is negligible; debug/trace at continuous rate is infeasible → level-gate at emit (decision E) is mandatory, not optional.
3. **Dedup vs `FaultEvent`/`output()`** — the new frame is *additive* (structured, tick-stamped, session-correlated); `FaultEvent` stays the hard-fault signal, `output()` stays the boot-diag carrier. In-motion gating to warn/error avoids re-emitting the same info three ways.
4. **32-bit ISR tick** — frames carry the raw 32-bit `timer_read_time()`; the host widens with the same heuristic it already uses for `PushPiecesResponse.arrival_clock`. Accuracy is bounded by the clock-sync window, which is what we want.
5. **Schema-hash lockstep** — MCU + host deploy together (accepted; §3).

## 7. Failure modes (fail-loudly)
- **Ring overflow** (burst exceeds drain rate): drop newest, increment a dropped counter, and emit a single `observability/log_drops` frame when the next slot frees — the loss is itself reported, never silent (mirrors the host queue-overflow policy).
- **TX full** (`kalico_console_write_raw` returns −1): the existing `diag_record_tx_drop_kalico` counter applies; the drain task retries next tick. No blocking in the drain task.
- **Unknown code/event on host**: resolve to `code_name="unknown"` + keep the raw numeric `code`/`event` — never drop the record.
- **Clock-sync not yet converged**: if `wall_time_at_mcu` has no valid window, stamp host-receipt time + a `time_estimated:true` field rather than dropping.

## 8. Testing (in the playground — no Trident)
- **Unit (host):** `wall_time_at_mcu` inverse correctness; `0x0084` decode; `code_name` resolution; the re-emit hook produces a schema-conformant line.
- **Unit (protocol):** round-trip `Encode`/`Decode` of `KALICO_MSG_LOG`; schema-hash regeneration.
- **Sim integration (the playground):** the sim ELF emits `KALICO_MSG_LOG` (e.g. inject a `TickIntervalExceeded` in the engine), the host decodes + writes `events/mcu-h7.jsonl`, Vector ships it, and `source:=mcu-h7` is queryable in VictoriaLogs with a resolved `code_name` and a sane `_time`. This is the end-to-end gate, runnable on the dev host.
- **Bandwidth check (sim):** confirm warn/error-gated emission under a representative run stays well under the transport budget; confirm debug/trace is dropped at emit by default.

## 9. Component boundaries
| Unit | Purpose | Lang | Depends on |
|---|---|---|---|
| `src/kalico_log.c` (+ `.h`) | C-owned log ring + `kalico_log_emit` + drain/TX task | C | kalico transport |
| `runtime` log call sites | emit at anomaly detection | Rust (MCU) | `extern "C" kalico_log_emit` |
| `KALICO_MSG_LOG` codec | wire encode/decode | Rust (`kalico-protocol`) | — |
| `FaultCode::code_name` + tables | code/subsystem/event resolution | Rust | — |
| `ClockSyncEstimator::wall_time_at_mcu` | tick → wall time | Rust (`kalico-host-rt`) | clock-sync |
| `RuntimeEvent::McuLog` + decode | host decode | Rust (`kalico-host-rt`) | protocol |
| MCU-log re-emit hook | resolve + stamp + write `mcu-*.jsonl` | Rust (`motion-bridge`, injected) | Stage 2 `logging::writer`, `SessionContext` |

## 10. Implementation staging (for the plan)
1. **Protocol + host decode (no MCU behavior change):** add `KALICO_MSG_LOG` codec + `RuntimeEvent::McuLog` + `wall_time_at_mcu` + `code_name`/tables + the re-emit hook + `mcu-*.jsonl` writer. Unit-testable on the host with synthetic frames. Nothing emits yet.
2. **MCU transport:** `src/kalico_log.c` ring + `kalico_log_emit` + drain/TX task + the C-side schema header. Sim-buildable; emit a test frame from a C DECL_TASK to prove the wire path end-to-end in the playground.
3. **Rust engine call sites:** wire `kalico_log_emit` into the `FaultCode` anomaly paths (warn/error) + level gating + ring-overflow accounting. The "why did it fault" payoff; verified in the playground.
4. **Level control + dedup pass:** runtime min-level, confirm no triple-emit with `FaultEvent`/`output()`.

Each stage is independently testable in the sim playground.

## 11. Open items
- Exact `args` width (2×u32 vs variable) — start at 2, extend if a call site needs more.
- Whether to also re-emit the boot persistent-diag through this structured path (currently `output()` strings) — deferred; could be a stage 5.
- Ring size / drain rate tuning on the real H7 vs sim — confirm on Trident later (sim validates the path; sizing is hardware-specific).
- Whether `mcu-*.jsonl` re-emit should share the host-rust `RotatingJsonlWriter` rotation settings (32 MB × 5) — default yes.
