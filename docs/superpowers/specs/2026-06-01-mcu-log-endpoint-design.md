# MCU Structured-Log Endpoint — Design (Observability Spec #2)

- **Date:** 2026-06-01
- **Branch / worktree:** `observability` (`.worktrees/observability`)
- **Status:** Design — **adversarially reviewed (multi-agent workflow); rework applied**; pending user review before plan/implementation.
- **Depends on:** the host pipeline (Stages 1–3, shipped on this branch) — schema, `events/*.jsonl`, Vector→VictoriaLogs, the `query-logs` skill. This spec adds `source = mcu-h7` / `mcu-f4` to that pipeline.
- **Boundary doc (binding):** `docs/kalico-rewrite/mcu-c-rust-boundary.md`.

> **Review-driven revisions (v2).** A code-grounded review found the v1 timestamping path was built on a non-existent "PushPiecesResponse host-side widening heuristic" (a wrong recon fact). v2 corrections: (1) the **MCU pre-widens** the log tick to `u64` before transmit (the real `arrival_clock` pattern) — `mcu_tick` is `u64` on the wire, no host widener; (2) `wall_time_at_mcu` returns a **fallible wall-clock** (`Option<(OffsetDateTime, bool)>`) backed by a new `(SystemTime, Instant)` anchor — not an `Instant` (which can't be RFC3339-formatted); (3) the C enqueue uses **`irq_save`/`irq_restore`** (not `irq_disable`), matching `diag_ring_push`, because OTG (NVIC prio 1) preempts TIM5 (prio 2); (4) the schema hash is bumped by editing **`schema_def.rs`** (not `messages.rs`) — with a **precondition** to fix the already-stale `PushPiecesResponse` entry; (5) the host hook is a concrete `EventDispatcher` field (mirroring `heartbeat_callback`) with a shared `Arc<RwLock<ClockSyncEstimator>>` and a dedicated `Arc<Mutex<RotatingJsonlWriter>>`. The `Trace`-is-vestigial decision stands, but on the correct grounds (no MCU emitter / no native message), not "dropped on the host."

---

## 1. Problem

The MCU is the one place the observability pipeline can't see. Its diagnostics are scattered across uncorrelated channels: Klipper `output()` strings (free-form → `RuntimeEvent::UnknownOutput`), `StatusHeartbeat` (`0x0083`, 10 Hz, `engine_state`/`fault_code` dropped at decode — `kalico_native.rs:295`), `FaultEvent` (`0x0082`, fire-once, raw numeric code with **no host-side name resolution** — "what is 65228?"; `FaultCode` is a `#[repr(i32)]` enum in `rust/runtime/src/error.rs` with no `code_name()`), and boot-time persistent-diag (`.persistent_diag`/`.bkp_bss`, replayed via `output()` strings). None land in the structured store with a session id, wall-clock time, subsystem, or resolved code name. The Rust motion engine — where anomalies are *raised* (the `raise_*` helpers in `rust/runtime/src/fault_helpers.rs`, called from `tick.rs`/`engine.rs`) — has **zero** logging today.

**Goal:** a structured MCU log the host decodes into the **same schema** as host-py/host-rust (`source = mcu-h7`/`mcu-f4`), stamped with the current `session_id`/`print_id` and a clock-sync-converted RFC3339 `_time`, with `code`/`code_name`/`event`/`subsystem` resolved host-side — flowing into the existing Vector→VictoriaLogs pipeline so MCU faults show up in the same UI, queryable per session.

## 2. Goals / Non-goals

**Goals:** (1) a C-owned MCU log transport both C and the Rust engine emit into, boundary-compliant (§4); (2) host decode → the Stage-1 schema with `code`→`code_name` resolution; (3) real RFC3339 timestamps via the clock-sync; (4) bandwidth-safe (structured coded frames, level-gated at emit); (5) testable entirely in the sim playground, no Trident.

**Non-goals (deferred):** replacing `FaultEvent` (kept as the hard-fault signal) or the boot persistent-diag replay; per-tick wire tracing (infeasible, §6); a rich runtime per-subsystem level map (a single min-level for v1); MCU-side string formatting (host owns templates; wire carries codes + numeric args).

## 3. Decisions

| # | Decision | Choice | Rationale |
|---|---|---|---|
| **Spec correction** | "reuse `RuntimeEvent::Trace`" | **Cannot reuse** | `Trace` is **functionally dead, but not "dropped"**: `events.rs:272` routes it to a *live* `TraceRing` with subscriber machinery (`events.rs:49-102`) — there is simply **no MCU emitter for `kalico_trace` anywhere in `src/` and no kalico-native wire message for it**, so the ring has no producer. `TraceRing` is also a raw-binary high-rate channel with no structured schema (no session/subsystem/event/code, no Vector path), so it is unsuitable regardless. We add a proper message. |
| **A** | Transport | **New kalico-native message `KALICO_MSG_LOG (0x0084)`** on `KALICO_CHANNEL_EVENTS` | First-class, decodable, schema-hash-versioned, beside `FaultEvent`/`StatusHeartbeat`. |
| **B** | Wire format | **Structured codes + numeric args**, host resolves names; `mcu_tick` is **`u64` (MCU pre-widened)** | Tiny frames; bandwidth-safe; matches the schema; eliminates host-side 32-bit wrap inference (§4.1). |
| **C** | Host output | **Separate `events/mcu-h7.jsonl` / `mcu-f4.jsonl`** (dedicated `RotatingJsonlWriter` instance) | Matches the §5 `source`→file model; keeps MCU logs out of the `host-rust` tracing subscriber. |
| **D** | v1 emit sites | The `raise_*` fault helpers in **`fault_helpers.rs`** + a few motion lifecycle events; **warn/error gated in motion** | One canonical emit site; "why did it fault" payoff; bandwidth-safe by default. |
| **E** | Level control | **Runtime-settable min level** (default `warn` in motion); gated **at emit** | Cheapest drop point; a debug session raises it without a reflash. |

**Deploy-lockstep:** the new message bumps `KALICO_SCHEMA_HASH` (computed by `build.rs` from **`schema_def.rs`** — see §4.2), so MCU + host deploy together (the `IdentifyResponse` hash check, `kalico_native.rs:241`, rejects a mismatch). Accepted; the playground rebuilds both.

## 4. Architecture

```
  Rust engine: fault_helpers.rs raise_*()  ──┐
   (called from tick.rs/engine.rs)           │  extern "C" kalico_log_emit(level, subsystem, event, code, a0, a1)
  C foreground/dispatch ─────────────────────┤    captures raw 32-bit timer_read_time(); irq_save; push; irq_restore
                                              ▼
                  ┌──────────── C-OWNED log ring (src/kalico_log.c) ────────────┐
                  │  #[repr(C)] ring in a C linker section (.bss / DTCM on H7)    │
                  │  producers: Rust engine + C (irq_save enqueue, à la           │
                  │  diag_ring_push) ; consumer: the existing runtime_drain task  │
                  └────────────────────────────────────────────────────────────────┘
                                              │  drain extends runtime_drain (~1 kHz DECL_TASK; never ISR):
                                              │  pre-widen each entry's 32-bit tick → u64 via
                                              │  runtime_widened_host_clock(), then transmit
                                              ▼
              kalico_transport_send_frame(KALICO_CHANNEL_EVENTS, KALICO_MSG_LOG, …)
                                              │  wire: mcu_tick(u64) level(u8) subsystem(u8)
                                              ▼        event(u16) code(u16) seq(u16) args[u32;2]
  ─────────────────────────────────────── host ──────────────────────────────────────────
  kalico_native.rs decode 0x0084 → RuntimeEvent::McuLog{…, host_recv: Instant (stamped at decode)}
                                              │
  EventDispatcher (new mcu_log_hook field, mirrors heartbeat_callback) → injected closure:
     • OffsetDateTime = clock.read().wall_time_at_mcu(mcu_tick) → Option<(OffsetDateTime, bool)>   (NEW)
       (clock = Arc<RwLock<ClockSyncEstimator>> shared with the clock-sync thread; bool = time_estimated)
     • fallback: host_recv → OffsetDateTime + time_estimated=true when no clock-sync window
     • resolve subsystem/event names + _msg template; from_u16(code) → code_name
     • stamp session_id/print_id from the global SessionContext
     • write one NDJSON line → dedicated Arc<Mutex<RotatingJsonlWriter>> for events/<source>.jsonl
                                              ▼
                       Vector → VictoriaLogs (unchanged)
```

### 4.1 The C-owned log ring (boundary §B2/§B3)
`src/kalico_log.c` declares `struct kalico_log_entry` (`#[repr(C)]` mirror in Rust) and a fixed-size ring in a C linker section (regular `.bss`; DTCM on H7). Enqueue is a thin `extern "C" kalico_log_emit(...)` that captures the **raw 32-bit `timer_read_time()`** and pushes under an **`irq_save`/`irq_restore`** critical section — **not** `irq_disable`/`irq_enable`: on this firmware OTG_FS is NVIC priority 1 (`usbotg.c:514`), higher-urgency than TIM5 priority 2 (`runtime_tick_h7.c:122`), so OTG preempts TIM5; an unconditional `irq_enable()` inside `kalico_log_emit` called from the TIM5 ISR would unmask interrupts mid-section and let OTG touch `transmit_buf` non-atomically. `irq_save`/`irq_restore` is the idempotent form every multi-context ring push uses (canonical: `diag_ring_push`, `fault_handler.c:335`). Rust **never holds a borrow into the ring** — it calls the C function; no Rust-typed structure crosses the ABI (§B3, the 2026-05-18 SPSC lesson). Multi-producer (ISR engine + C foreground) is fine under `irq_save` (B3 mandates no-Rust-borrow, not single-producer).

`timer_read_time()` is **ISR-safe** here: on H7 it is `DWT->CYCCNT`, a single-cycle read. The "foreground-only" warning at `runtime_tick.c:71` applies to `runtime_host_now_us` (the wrapper that *divides* after the read), not the raw read; under `CONFIG_KALICO_SIM` the software CYCCNT is also ISR-safe.

The consumer **extends the existing `runtime_drain` DECL_TASK** (`runtime_tick.c:438`, woken ~1 kHz, already calls `kalico_transport_send_frame`) — not a new fourth task. The drain **pre-widens** each entry's 32-bit tick to `u64` via `runtime_widened_host_clock()` (`runtime_tick.c:190`, foreground-safe) **before** transmit — the same MCU-side widening `arrival_clock` uses. So the host never widens; it receives a `u64`. 1 ms worst-case drain latency is fine for warn/error.

### 4.2 The wire message (boundary §B4) + schema-hash edit points
`KALICO_MSG_LOG (0x0084)` on `KALICO_CHANNEL_EVENTS`. Payload, fixed-width LE: `mcu_tick: u64` (MCU-pre-widened), `level: u8`, `subsystem: u8`, `event: u16`, `code: u16`, `seq: u16`, `args: [u32; 2]` (`seq` = per-MCU monotonic for host drop detection).

**Adding it touches three places** (the v1 spec wrongly said "edit `messages.rs`, `build.rs` regenerates the header" — `build.rs` reads `schema_def.rs`, *not* `messages.rs`):
1. `rust/kalico-protocol/src/messages.rs`: `MessageKind` variant + `from_u16` + hand-written `Encode`/`Decode` (the working codec).
2. `rust/kalico-protocol/schema_def.rs`: a new `SchemaMessage` entry — **this** is what `build.rs` (`:26,34-37,94-101`) SHA-256s to bump `KALICO_SCHEMA_HASH` and emits the C `#define KALICO_MSG_LOG` into `src/kalico_protocol_schema.h`. A `messages.rs`-only change compiles but leaves the hash + C header stale, defeating deploy-lockstep.
3. The C-side `0x0084` handler / define usage.

**Precondition (already-stale entry):** `schema_def.rs:83-91` lists `PushPiecesResponse` with only `result:i32`, but `messages.rs:246-274` wires `result + arrival_clock:u64 + front_start_time:u64` (20 B) — the hash already doesn't cover the wire. Before adding `0x0084`, update `schema_def.rs`'s `PushPiecesResponse` (add `arrival_clock:u64`, `front_start_time:u64`, bump version) and regenerate, so the lockstep guarantee is real (§10 step 1).

**Fail-loud header gen:** `build.rs:119-136` is best-effort (emits a warning + returns OK if `src/` missing). Change it so that when `src/` **exists** but the write fails, it propagates the error (panic/exit 1) — a silently stale C header violates the seam. Keep the `exists()` guard only for the standalone-crate-publish case, commented.

### 4.3 Host decode + re-emit
- `kalico_native.rs` decodes `0x0084` → `RuntimeEvent::McuLog { mcu_tick: u64, level, subsystem, event, code, seq, args, host_recv: Instant }` (`host_recv` stamped at decode, matching `add_piggyback_sample`, `clock_sync.rs:235`). `McuLogEvent`/the variant is defined in **`kalico-host-rt`** (not motion-bridge) to avoid a cyclic dependency.
- **`ClockSyncEstimator` gains** `wall_time_at_mcu(mcu_ticks: u64) -> Option<(OffsetDateTime, bool)>` (the bool is `time_estimated`): `None` only with zero samples; `Some(_, true)` when extrapolating outside the regression window; `Some(_, false)` inside it. This needs a **new `(SystemTime, Instant)` anchor** captured at estimator epoch — the estimator today holds no Instant↔wall-clock anchor and works in monotonic/seconds space, so a bare `Instant` cannot be RFC3339-formatted. Inverse: `host_secs = anchor_host_time + (mcu_ticks - anchor_mcu_clock) / freq_estimate`, mapped to UTC via the new anchor.
- **Sharing:** `ClockSyncEstimator` lives as a thread-local in `spawn_periodic_clock_sync` (`bridge.rs:139`) — not reachable at dispatch. Place it behind an `Arc<RwLock<ClockSyncEstimator>>` (or `Arc<ArcSwap<sample>>`) shared between the clock-sync thread (writer) and the hook (reader); the hook **captures the Arc and locks inside** — the signature drops the live `&ClockSyncEstimator` borrow.
- **Hook wiring:** `EventDispatcher` (`events.rs:230`) gains an `mcu_log_hook: Option<Box<dyn Fn(McuLogEvent) + Send + Sync>>` field + setter, mirroring the existing `heartbeat_callback`. The bridge supplies the closure, which captures: the shared clock Arc, a **dedicated** `Arc<Mutex<RotatingJsonlWriter>>` for `events/<source>.jsonl` (a *separate* writer instance from `host-rust.jsonl`; `Arc<Mutex>` required for `Send` across the reactor dispatch thread, `reactor.rs:882`), and reads the global `SessionContext` via `logging::context::load_context()`.

### 4.4 Code/name resolution
- `code` is a **sign-wrapped `u16`**: `FaultCode` is `#[repr(i32)]` with negative discriminants encoded via `as_u16` (`error.rs:138`), so the host receives `0xFECC` for `-308`. Add `FaultCode::from_u16` (sign-extend `u16`→`i16`→`i32`, then match) + `code_name(&self) -> &'static str`. The match spans 42 non-contiguous discriminants (`-1..-311`, gaps) — ~50 mechanical lines. Round-trip test (`as_u16`→`from_u16`→`code_name`) in §8.
- `subsystem`/`event` tables are **greenfield** (neither enum exists today). Define in the **`runtime` crate** (it compiles `no_std` for the MCU and f64 for the host, so MCU emit sites and the host resolver share one source): initial subsystems `runtime`/`motion`/`tick`/`endstop`; initial per-subsystem event codes + template strings. `_msg` is composed host-side from the template + args.

## 5. Schema mapping
Per re-emitted record: `_time` (RFC3339 from `wall_time_at_mcu(mcu_tick)`, or `host_recv` with `time_estimated:true` on fallback), `_msg` (template + args), `level`, `source` (`mcu-h7`/`mcu-f4`), `subsystem` (resolved), `session_id`/`print_id` (host global), `target` (`mcu::<subsystem>`), `event` (resolved), `code`/`code_name` (when non-zero), payload (`arg0`/`arg1`, `seq`, `time_estimated`). Identical shape to host records ⇒ `query-logs` recipes + the playground UI work unchanged.

## 6. Constraints / tensions (resolved)
1. **C-owns-buffer, Rust-logs** — single thin `extern "C"` call; one bounded `irq_save` enqueue on the ISR path. Measured in the sim.
2. **Bandwidth** — warn/error at anomaly rate is negligible; debug/trace continuous is infeasible → level-gate at emit (decision E) is mandatory.
3. **Dedup vs `FaultEvent`/`output()`** — additive (structured, tick-stamped, session-correlated); `FaultEvent` stays the hard-fault signal; in-motion warn/error gating avoids triple-emit.
4. **Tick widening (resolved):** the **MCU** pre-widens the 32-bit tick to `u64` in the drain via `runtime_widened_host_clock()` before transmit (the `arrival_clock` pattern). There is **no host-side widening heuristic** (the v1 spec invented one); a bare 32-bit tick from an async ring that can buffer across the ~8.2 s `u32` wrap has no correct host-side reconstruction, which is exactly why widening is done MCU-side.
5. **Schema-hash lockstep** — MCU + host deploy together; bumped via `schema_def.rs` (§4.2).

## 7. Failure modes (fail-loudly)
- **Ring overflow:** drop newest, increment a counter, emit one `observability/log_drops` frame when a slot frees — loss is reported, never silent.
- **TX full** (`kalico_console_write_raw` → −1): existing `diag_record_tx_drop_kalico` counter; drain retries next tick; no blocking.
- **Unknown code/event on host:** `code_name="unknown"`, keep the raw numeric — never drop.
- **Clock-sync not converged / pre-window:** `wall_time_at_mcu` returns `None` or `(_, true)`; the hook stamps `host_recv` (the `Instant` captured at decode) converted to wall-clock, with `time_estimated:true` — never dropped.
- **Build-time header staleness:** `build.rs` fail-loud on write failure when `src/` exists (§4.2).

## 8. Testing (in the playground — no Trident)
- **Unit (host):** `wall_time_at_mcu` inverse + the new `(SystemTime, Instant)` anchor + `None`/`time_estimated` cases; `0x0084` decode incl. `host_recv` stamping; `from_u16`→`code_name` round-trip (incl. sign-wrap, e.g. `-308`↔`0xFECC`); the hook produces a schema-conformant line with a real `_time`.
- **Unit (protocol):** `KALICO_MSG_LOG` `Encode`/`Decode` round-trip; **schema-hash test updated** for the corrected `PushPiecesResponse` + the new message; C `#define` emitted.
- **Sim integration (playground):** inject a `raise_tick_interval_exceeded` in the engine → MCU emits `0x0084` (MCU-widened `u64` tick) → host writes `events/mcu-h7.jsonl` → Vector ships → `source:=mcu-h7` queryable in VictoriaLogs with resolved `code_name` and a sane RFC3339 `_time`. End-to-end gate on the dev host.
- **Bandwidth (sim):** warn/error-gated emission stays well under the transport budget; debug/trace dropped at emit by default.

## 9. Component boundaries
| Unit | Purpose | Lang | Notes |
|---|---|---|---|
| `src/kalico_log.c` (+ `.h`) | C-owned ring + `kalico_log_emit` (`irq_save`) + drain extension in `runtime_drain` (pre-widen + TX) | C | reference: `diag_ring_push` |
| `fault_helpers.rs` `raise_*` | emit at fault-raise (canonical site) | Rust (MCU) | calls `extern "C" kalico_log_emit` |
| `messages.rs` + `schema_def.rs` | wire codec + **hash/header source** | Rust | both edited (§4.2) |
| `FaultCode::from_u16`/`code_name` + subsystem/event tables | resolution | Rust (`runtime`, no_std+host) | sign-wrap aware |
| `ClockSyncEstimator::wall_time_at_mcu` + `(SystemTime,Instant)` anchor | tick → RFC3339 | Rust (`kalico-host-rt`) | `Option<(OffsetDateTime,bool)>` |
| `RuntimeEvent::McuLog` (+ `host_recv`) + decode | host decode | Rust (`kalico-host-rt`) | type defined here (no cyclic dep) |
| `EventDispatcher.mcu_log_hook` field + setter | injection slot | Rust (`kalico-host-rt`) | mirrors `heartbeat_callback` |
| re-emit closure | resolve + stamp + write | Rust (`motion-bridge`) | captures `Arc<RwLock<ClockSyncEstimator>>` + dedicated `Arc<Mutex<RotatingJsonlWriter>>` + `SessionContext` |

## 10. Implementation staging
1. **Protocol + host decode (no MCU behavior change).** **Precondition:** fix the stale `schema_def.rs` `PushPiecesResponse` (+`arrival_clock`,+`front_start_time`, bump version), regenerate, update the schema-hash test. Then: add `KALICO_MSG_LOG` to `messages.rs` (+`from_u16`+codec) **and** `schema_def.rs` (bumps hash + emits C define); `build.rs` fail-loud fix; `RuntimeEvent::McuLog`(+`host_recv`) + decode; `from_u16`/`code_name` + subsystem/event tables; `wall_time_at_mcu` + the `(SystemTime,Instant)` anchor; `EventDispatcher.mcu_log_hook` + setter; the re-emit closure + dedicated `mcu-*.jsonl` writer + `Arc<RwLock<ClockSyncEstimator>>` sharing. Host-unit-testable with synthetic frames; nothing emits yet.
2. **MCU transport.** `src/kalico_log.c` ring + `kalico_log_emit` (`irq_save`) + the `runtime_drain` extension (pre-widen via `runtime_widened_host_clock()` + TX) + the C schema define. Emit a test frame from C to prove the wire path end-to-end in the playground.
3. **Rust engine emit sites.** Wire `kalico_log_emit` into the `raise_*` helpers in `fault_helpers.rs` (one canonical site, not each `tick.rs`/`engine.rs` call site) + level gating + ring-overflow accounting. The "why did it fault" payoff; verified in the playground.
4. **Level control + dedup pass.** Runtime min-level; confirm no triple-emit with `FaultEvent`/`output()`.

Each stage is independently testable in the sim playground.

## 11. Open items
- Exact `args` width (2×u32 vs variable) — start at 2.
- Re-emitting boot persistent-diag through this structured path (currently `output()` strings) — deferred (possible stage 5).
- Ring size / drain cadence tuning on real H7 vs sim — sim validates the path; sizing confirmed on Trident later.
- `mcu-*.jsonl` rotation settings — default to the host-rust `RotatingJsonlWriter` settings (32 MB × 5).
- `cfg`-gating of `code_name`/tables for the no_std MCU build vs f64 host — confirm at implementation (the `runtime` crate already dual-targets).
